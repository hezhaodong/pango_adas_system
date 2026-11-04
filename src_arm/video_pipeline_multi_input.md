# 多路输入 PIO 切换技术文档

## 项目概况

FPGA 工程 `high_speed_test_ov_fmc` 同时接入 FMC 摄像头和 HDMI 两路视频输入，通过 PIO 寄存器动态切换。ARM 端 `high_speed_pcie_eglf5.8` 根据帧来源自动分流。统一输出 1280×720 RGB565。

**为什么需要两路输入**：HDMI 输入画质好但依赖外部源（电脑），FMC 摄像头是板上直连更稳定。实际部署时需要灵活切换。这套方案的设计目标是——切换时不出混合帧、不丢帧通知、不掉 DMA 状态、上层 AI 不串数据。

---

## 1. 数据通路

```
FMC摄像头 → 独立采集/组帧 → FMC异步FIFO ──┐
                                          ├→ PIO选择(0x300) → PCIe DMA → ARM FrameBuffer
HDMI输入  → 独立采集/组帧 → HDMI异步FIFO ──┘
                                              │
                         ┌────────────────────┼───────────────────┐
                         │                    │                   │
                         ▼                    ▼                   ▼
                       显示             车牌检测识别           YOLO检测
```

两路输入各自使用独立的像素时钟域、各自的 `video_crtl` 状态机和各自的 `pcie_tx_fun` DMA 发送引擎。FMC 摄像头有自己的像素时钟（来自摄像头本身），HDMI 的像素时钟由 MS72xx 接收芯片从 TMDS 信号恢复。两路时钟完全不同，所以各自配了一套异步视频 FIFO，数据进入 FIFO 后统一在 PCIe 时钟域（125MHz）读出、组包、DMA 发送。

PCIe DMA 控制器（`ips2l_pcie_dma`）只有一套，不复制。PIO 在 PCIe 时钟域控制当前哪一路 FIFO 的读端口连接到 DMA 控制器。

---

## 2. PIO 寄存器体系

### 2.1 全览

FPGA 的 PIO（Programmed IO）本质是 BAR0 空间内的寄存器。ARM 通过 mmap BAR0 物理地址后，直接用 CPU load/store 指令读写这些寄存器——每次操作都是一条 PCIe Memory Read/Write TLP，约 300ns 往返，无系统调用。

| 地址 | 方向 | 功能 |
|------|------|------|
| `0x000` | ARM写 | 采集启动/停止（`0xFFFFFFE5`/`0xFFFFFF00`） |
| `0x050-0x060` | ARM写 | 4帧 DMA 基地址配置 |
| `0x140` | ARM读 | 帧完成状态（**读清零**） |
| `0x150-0x2d0` | ARM读 | FPGA 调试寄存器（16字节对齐，无副作用） |
| `0x300` | ARM写 | 输入源切换请求 |
| `0x310` | ARM读 | 输入源状态（当前源、忙标志、有效标志、epoch） |

`0x000`、`0x050-0x060` 和 `0x140` 是原 HDMI 工程就有的。`0x300` 和 `0x310` 是多路输入新增的。

### 2.2 输入源控制寄存器 `0x300`

**功能**：ARM 写入目标输入源编号。FPGA 收到后不会立即切换——它先标记"有人请求切换"，然后等当前正在 DMA 的帧全部传完，再把 MUX 切到新源，最后等新源的 `video_crtl` 遇到下一个 VSYNC 上升沿才开始输出新帧。整个过程忙标志（`0x310` bit2）一直为高，切换完成后才清零。

```cpp
// 写 0 → 切 FMC，写 1 → 切 HDMI
void pioWriteReg(unsigned int byteOffset, unsigned int data) {
    if (!m_bar0VirtAddr) return;
    *((volatile unsigned int *)m_bar0VirtAddr + (byteOffset >> 2)) = data;
    __sync_synchronize();  // 阻止编译器/CPU 重排 MMIO 操作顺序
}
```

`__sync_synchronize()` 是一条 ARM 内存屏障指令（DMB）。ARM 的 MMIO 写（`STR` 指令）会被 CPU 内部的 store buffer 延迟发出到 PCIe 总线。不加屏障的话，写 `0x300` 之后立即读 `0x310` 可能读到旧值——因为写还没真正到达 FPGA。`__sync_synchronize()` 强制 CPU 把所有 pending 的 store 全部推出去后才继续执行后面的 load。

### 2.3 输入源状态寄存器 `0x310`

**功能**：ARM 读取 FPGA 当前切换状态。这是**只读寄存器，无副作用**——可以反复读，不会影响 FPGA 状态。

| 位 | 名称 | 含义 |
|----|------|------|
| bit0 | `active` | FPGA **当前实际**输入源：`0`=FMC，`1`=HDMI |
| bit1 | `requested` | ARM **请求的**目标输入源 |
| bit2 | `busy` | 切换忙。1=正在等当前帧 DMA 完成、切 MUX、等新源 VSYNC |
| bit3 | `fmc_valid` | FMC 输入是否有效（检测到像素时钟且收到过 VSYNC） |
| bit4 | `hdmi_valid` | HDMI 输入是否有效（MS72xx 初始化完成且有 TMDS 信号） |
| bit15:8 | `epoch` | 切换代号，每完成一次切换自增 1。8 位可计 256 次，回绕 |

`epoch` 是切换场景的关键字段。ARM 收到帧时不仅要看 `source` 是 FMC 还是 HDMI，还要对 `epoch`。如果 ARM 发起切换后，FPGA 还没切完，此时 `0x140` 报的帧 epoch 和 ARM 期望的不一致——ARM 就知道"这帧还是旧源的"，可以丢弃或特殊处理。

### 2.4 帧完成寄存器 `0x140`

**功能**：ARM 读取帧完成通知。**只能读一次**——读操作触发 FPGA 内部 `pio_rd_en` 信号，`r_wr_frame_done` 被自动清零。第二次读必然是 `frame_done=0`。

```
0x140 = { epoch[7:0], source[0], buffer_index[1:0], frame_done[0] }
        └──新增──┘ └──新增──┘ └──────原有──────┘ └───原有──┘
```

| 位 | 名称 | 含义 |
|----|------|------|
| bit0 | `frame_done` | 1=FPGA 刚完成一帧 DMA 写入 |
| bit2:1 | `buffer_index` | DMA 槽号 0–3 |
| bit3 | `source` | 该帧来源：0=FMC, 1=HDMI |
| bit15:8 | `epoch` | 该帧对应的输入源切换代号 |

新增的 `source` 和 `epoch` 在高位，低三位保持 `frame_done` + `buffer_index` 不变。旧代码只读低三位，完全不受影响。

ARM 解析代码（`pcie_device.cpp:607-631`）：

```cpp
int PcieDevice::readPioStatus(FrameStatus &status)
{
    status = FrameStatus();
    if (!m_bar0VirtAddr) return -1;

    // BAR0 已 mmap 到用户空间，直接指针解引用
    volatile unsigned int *pioBase = (volatile unsigned int *)m_bar0VirtAddr;
    const unsigned int wordOffset = 0x140 >> 2;   // byte 0x140 → word index 0x50
    const uint32_t regVal = pioBase[wordOffset];  // 唯一一次 MMIO 读！
    __sync_synchronize();

    status.frame_done   = (regVal & 0x01) != 0;
    status.buffer_index = (regVal >> 1) & 0x03;
    status.source       = ((regVal >> 3) & 0x01) != 0
                        ? FpgaInputSource::HDMI : FpgaInputSource::FMC;
    status.source_epoch = static_cast<uint8_t>((regVal >> 8) & 0xff);
    status.valid        = status.frame_done;
    return 0;
}
```

**为什么这里必须只读一次**：ARM 采集线程是轮询模式——每 500μs 读一次 `0x140`。如果读两次：第一次读到 `frame_done=1`，第二次（比如为了确认 buffer_index）又读了同一个寄存器，`frame_done` 已经被清零，整个轮询循环就以为没有新帧。所以代码注释强调：0x140 的读操作必须集中在 `readPioStatus()` 这一个函数里，只做一次 MMIO 读。

---

## 3. 安全切换机制

### 3.1 为什么"安全切换"不是简单的 MUX 切换

如果 FPAG 直接切 MUX——比如当前正在传输 FMC 的第 360 行数据，突然 HDMI 数据接上——ARM 会收到一帧上半截 FMC、下半截 HDMI 的"混合帧"。这帧数据没有任何意义，但 `frame_done` 一样会报。AI 识别到这种帧，车牌号串到另一个画面，结果全错。

所以安全切换的核心原则是：**必须在帧边界（frame boundary）切换**。

### 3.2 FPGA 侧

FPGA 切换状态机：

```
空闲 → ARM写0x300 → 标记目标源 → 等当前帧 DMA tx_done
                                  → 切 MUX 到新源 FIFO
                                  → 等新源 video_crtl 捕捉到 VSYNC 上升沿
                                  → 开始从新 FIFO 读数据、DMA 发送
                                  → 完成后报 frame_done (带新源 source 和 epoch)
                                  → busy 清零
```

如果当前正在使用的源突然没了（比如 HDMI 线被拔掉）——FPGA 检测到该源的像素时钟停止或 VSYNC 连续 50ms 不来——就强制切换回另一个有效源。

### 3.3 ARM 侧

`pcie_device.cpp:447-503`，`setFpgaInputSource()` 函数：

```cpp
bool PcieDevice::setFpgaInputSource(FpgaInputSource source)
{
    if (!isOpened() || !m_bar0VirtAddr) return false;

    const unsigned int value = source == FpgaInputSource::HDMI ? 1u : 0u;
    pioWriteReg(0x300, value);
    unsigned int status = pioReadReg(0x310);  // 冲刷 posted write

    // BAR 读流水线有时会返回旧值——再写一次+再读一次确认
    if (((status >> 1) & 0x1u) != value) {
        pioWriteReg(0x300, value);
        status = pioReadReg(0x310);
    }

    // 轮询忙等：最多等 250ms
    // Gen1约100ms切换延迟 + 下一个VSYNC最多约17ms = 250ms留有余量
    unsigned int waitMs = 0;
    while (((((status >> 1) & 0x1u) != value)      // requested 还没变
            || ((status & 0x1u) != value)           // active 还没切
            || ((status >> 2) & 0x1u))              // busy 标志还没清
           && waitMs < 250u) {
        usleep(1000);                                // 每 1ms 轮询一次
        status = pioReadReg(0x310);
        ++waitMs;
    }

    const bool active   = (status & 0x1u) != 0;
    const bool busy     = ((status >> 2) & 0x1u) != 0;
    const bool switched = (active == (value != 0u)) && !busy;

    if (!switched) {
        // 超时：主动把请求回滚到当前活跃源
        // 防止 ARM 以为切了但 FPGA 没切，两边状态不一致
        pioWriteReg(0x300, active ? 1u : 0u);
        (void)pioReadReg(0x310);
    }
    return switched;
}
```

**为什么需要 retry posted write**：ARM 的 store buffer 对于 MMIO 区域可能不保证写入顺序。写 `0x300` 后立即读 `0x310`，可能读到的是"写还在路上"时的旧值。`__sync_synchronize()` 保证写已完成，但如果 PCIe fabric 里有对 BAR0 写的缓冲，ARm 读 `0x310` 时仍然可能看到旧值。所以加一次 retry——再写一次、再读一次——确保 FPGA 确实收到了请求。

**为什么超时 250ms**：Gen1 PCIe 下 DMA 传输 1.84MB 比 Gen2 慢，加上下一个 VSYNC 要等最多 –17ms。250ms 是 Gen1 最慢情况下的安全余量。超时后必须回滚——不然 ARM 认为切了 HDMI 但 FPGA 还在 FMC，后续所有帧的 source 字段都不对，epoch 也对不上，整个系统进入不一致状态。

### 3.4 UI 侧

`mainwindow.cpp:1347-1398`，`toggleFpgaInputSource()` 函数：

```cpp
void MainWindow::toggleFpgaInputSource()
{
    if (!m_device || !m_device->isOpened()) return;

    const FpgaInputSource requested =
        m_fpgaInputSource == FpgaInputSource::FMC
        ? FpgaInputSource::HDMI : FpgaInputSource::FMC;

    // 关键：暂停采集线程
    // 0x140 和 0x310 是两个 PIO 地址，如果采集线程正在轮询 0x140
    // 而 UI 线程同时在读 0x310，两个 MMIO 读会交织
    // FPGA 的 PIO 接口不能同时服务两个地址
    const bool restartCaptureThread = m_capture && m_capture->isRunning();
    if (restartCaptureThread) {
        m_capture->stopCapture();
        if (!m_capture->wait(1000)) return;
    }

    // 执行 PIO 切换
    if (!m_device->setFpgaInputSource(requested)) {
        if (restartCaptureThread) m_capture->start();
        return;
    }

    m_fpgaInputSource = requested;

    // 切换后必须清空所有旧状态：
    // 1. FrameBuffer 清空——旧源残留帧全部丢弃
    // 2. 显示的缓存帧清空——画面不能还显示旧源最后一张图
    // 3. 车牌跟踪历史清空——旧源场景的车牌不能串到新源
    m_frameBuffer->reset();
    m_lastDisplaySrc = nullptr;
    m_cachedDisplayImage = QImage();
    m_stableDetectFrameSeq = -1;
    resetPlateHistory();

    if (restartCaptureThread) m_capture->start();
}
```

---

## 4. 采集线程根据帧来源分流

`frame_capture.cpp` 的采集主循环中，从 `0x140` 获取帧时同步得到帧来源。根据来源分为两条完全不同的处理路径：

```cpp
FrameStatus status;
m_device->readPioStatus(status);  // 读 0x140，一次获取 frame_done + source + epoch

const bool isHdmi = (status.source == FpgaInputSource::HDMI);

if (isHdmi) {
    // ======== HDMI 路径（零拷贝） ========
    // FPGA 已经把帧 DMA 到了 slot N 的物理内存
    // ARM 已通过 dma_mmap_coherent 映射这片内存
    // 所以不需要 memcpy——直接告诉 FrameBuffer 这片内存的指针
    src = dmaUserAddr + status.buffer_index * FRAME_SIZE_RGB565;
    m_buffer->commitMappedFrame(src, m_frameSeq);
    // 耗时：~0ms（传 8 字节指针 + 一次锁操作）
    // 帧率：60fps

} else {
    // ======== FMC 路径（memcpy） ========
    // FMC 帧需要通过 m_rawBuf4 中转（驱动 read() 填充）
    // 需要做 memcpy → 接缝检测修正 → 帧签名校验 → 提交
    src = m_rawBuf4 + status.buffer_index * FRAME_SIZE_RGB565;
    memcpy(dst, src, FRAME_SIZE_RGB565);
    // 耗时：~27ms（memcpy 1.84MB + 接缝修正 + 签名校验）
    // 帧率：~38fps
}
```

**为什么 FMC 不能用零拷贝**：FMC 摄像头有自己的帧数据通路——数据从摄像头进入 FPGA，经过 `cmos_capture_data` 采集模块、RGB565 组帧、FIFO 缓冲，最终由 `pcie_tx_fun` 通过 DMA 写入 ARM 内存。但 FMC 帧的 DMA 目标地址不是通过驱动分配的 coherent buffer，而是由 FPGA 内部 PIO 基地址寄存器配置的。这个地址序列和驱动分配的 DMA ring 不是同一片物理内存。所以要经过 `::read()` 路径从驱动拷出来（这也是为什么 FMC 只能 ≥30ms/帧）。

---

## 5. FMC vs HDMI 处理差异总结

| | HDMI | FMC |
|------|------|------|
| DMA方式 | mmap 零拷贝（指针传递，–0ms） | memcpy 1.84MB（–27ms） |
| 帧率上限 | 60fps（受 FPGA 60Hz 输入限制） | –38fps（受 memcpy 限制） |
| `pcie_tx_fun` VS对齐 | `VS_FRAME_REALIGN=0`（保持连续 720 DMA burst） | `VS_FRAME_REALIGN=1`（每次 VS 重新对齐行计数） |
| 0x140 槽号来源 | 当次完成脉冲 `r_dma_index` | `r_dma_index_d0`（晚一拍） |
| 接缝修正 | 不需要 | 需要（FMC 摄像头存在跨行偏移） |
| 帧签名校验 | 不需要 | 需要（3 次签名采样确认帧完整性） |

---

## 6. 数据结构定义

`pcie_device.h` 中的枚举和帧状态结构体：

```cpp
// 输入端硬件来源
enum class FpgaInputSource : uint8_t {
    FMC  = 0,   // FMC 子卡摄像头
    HDMI = 1    // HDMI 输入
};

// 从 0x140 解析出的帧完成信息
struct FrameStatus {
    bool             valid        = false;  // frame_done 为真
    int              buffer_index = 0;      // DMA 槽号 0~3
    bool             frame_done   = false;  // FPGA 已完成一帧
    FpgaInputSource  source       = FpgaInputSource::FMC;  // 帧来源
    uint8_t          source_epoch = 0;      // 切换代号（用于校验帧来源一致性）
};
```

---

## 7. 操作方法

### EGLFS 全屏快捷键

| 按键 | 功能 |
|------|------|
| `H` | 切换 HDMI/FMC（应用启动默认 HDMI，因此第一次按 H 切到 FMC 摄像头；自动重复事件被忽略） |
| `Q` | 切换 PCIe/WiFi 源 |
| `A` | 车牌识别 |
| `E` | 增强识别 |
| `1` | SCI |
| `2` | FFDNet |
| `3` | Deblur |
| `4` | 头盔检测 |
| `5` | 行人过马路检测 |
| `6` | 自动驾驶辅助 |
| `7` | 斑马线礼让检测 |
| `ESC` | 退出程序 |

H 键实现（`eglfs_video_window.cpp:218-238`）：

```cpp
case Qt::Key_H:
    if (!event->isAutoRepeat() && m_toggleFpgaHandler)
        m_toggleFpgaHandler();  // → MainWindow::toggleFpgaInputSource()
    event->accept();
    return;
```

### Qt 界面

按钮文字动态切换：`FPGA input: FMC` / `FPGA input: HDMI`。

### 6.1 HDMI 启动无帧诊断

ARM 应用发布构建启用了 `ARM_FINAL_FPGA_DIAGNOSTICS`。当采集线程连续
150 ms 没收到 `frame_done` 时，只抓取一次 BAR0 快照并向标准错误输出
`[fpga-source-debug]`、`[fpga-debug]` 和 `[fpga-diagnosis]`。正常视频期间
不会轮询这些调试寄存器，因此不影响稳定帧率。

在板端保存启动日志：

```bash
./camera_capture_v2 2>&1 | tee /mnt/usb/hdmi_startup_debug.log
```

优先查看最后一条 `blocked_at`：

| `blocked_at` | 含义 |
|---|---|
| `source-switch-not-complete` | `0x300` 的 HDMI 请求尚未成为 FPGA 实际输入 |
| `hdmi-not-valid-ms72xx-init-or-pixel-clock` | MS72xx 初始化未完成，或 HDMI 输入没有像素时钟/DE |
| `ms7200-sta-rd-waiting-for-valid-hdmi-timing` | MS7200 I²C 正常，但接收器停在 `STA_RD` 等待有效 HDMI 时序 |
| `capture-start-command-not-latched` | FPGA 没有锁存 ARM 的采集启动命令 |
| `dma-base-addresses-not-fully-configured` | 四帧 DMA 地址没有完整写入 FPGA |
| `input-valid-but-no-deep-debug-register-map` | 当前 FPGA 镜像只有基础状态寄存器，需要烧录带深度调试寄存器的镜像 |
| `pcie-dma-no-completion` | HDMI 像素链路已工作，但 PCIe DMA 没有完成 |

如果断电重启和 FPGA 复位均不能恢复、只有重刷 RK 系统镜像有效，按 `R` 启动
`recover_rk_system_pipeline.sh`。应用会先优雅退出并释放 DMA；脚本随后卸载并明确
重载当前目录的 `pango_pci_driver.ko`，对 `0755:0755` PCIe 端点执行 function reset
或 remove/rescan，再启动应用。恢复前后的内核、模块哈希、PCIe 状态和应用日志保存在
`/mnt/usb/rk_pcie_system_recovery_*.log` 与 `/mnt/usb/camera_after_recovery_*.log`。

---

## 8. 验证清单

- [x] FPGA 两路独立时钟域 + 各自异步 FIFO
- [x] PIO 安全切换（帧边界等待 + 50ms 源丢失超时）
- [x] 0x140 扩展字段（source + epoch）向后兼容旧版 ARM 代码
- [x] ARM 根据 source 分流处理（HDMI mmap 零拷贝 / FMC memcpy）
- [x] 切换时清空 FrameBuffer + 旧帧缓存 + 车牌历史
- [x] H 键自动重复过滤（`!event->isAutoRepeat()`）
- [x] 切换超时 250ms 自动回滚（防止 ARM/FPGA 状态不一致）
- [x] `__sync_synchronize()` 内存屏障保护 MMIO 操作顺序
- [x] PDS 综合、映射、时序检查成功
- [x] 100 个 Verilog 源文件通过 ModelSim lint
- [x] 引脚约束无重复冲突
- [ ] 实机验证应用启动默认显示 HDMI，第一次按 H 切到 FMC 摄像头
- [ ] 实机验证 FMC/HDMI 动态切换（画面平稳过渡、无混合帧）
- [ ] 分别验证两路显示、车牌识别和 YOLO 检测
- [ ] epoch 校验：切换后前几帧 epoch 是否匹配、旧源帧是否被正确丢弃
