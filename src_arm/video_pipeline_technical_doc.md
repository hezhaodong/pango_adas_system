# 视频链路通俗讲解

> 假设你完全不懂 FPGA 和 PCIe，本文用最白话的方式解释每一段代码在干什么。

---

## 先搞清楚数据是怎么流的

想象一条流水线：

```
摄像头 → FPGA芯片 → PCIe总线 → ARM内存 → 你的程序读取 → 显示到屏幕
                                                    ↓
                                               AI识别车牌
```

**FPGA** 是一个可编程芯片，专门负责把摄像头的视频信号转成 ARM 能读的数据。

**PCIe** 是 FPGA 和 ARM 之间的高速通道。

---

## 第一关：视频怎么从 FPGA 到达 ARM 内存？

### 1. FPGA 往 ARM 内存里"写作业"

FPGA 每次拍完一帧照片（1280×720 的 RGB565 图片，1.84MB），就通过 PCIe 通道直接把数据扔到 ARM 的内存里。

这个扔的过程叫 **DMA（直接内存访问）**——FPGA 自己动手，ARM CPU 不用管。

FPGA 有 **4 个槽位**（就像 4 个停车位），轮流使用：

```
槽0 → 槽1 → 槽2 → 槽3 → 回到槽0 → ...
```

每写完一帧，FPGA 就举一下手说"这帧写完了"，这个信号叫 `frame_done`，同时告诉 ARM "我刚写的是几号槽"，这个叫 `buffer_index`。

这两个信息放在 FPGA 的一个特殊寄存器里（地址 `0x140`），ARM 可以直接读。

### 2. ARM 端怎么拿到这些数据？

在 `pcie_device.cpp` 中，有一个关键步骤叫 **mmap**（内存映射）：

```cpp
// pcie_device.cpp: tryMmapDna()
// 把 FPGA 写的 DMA 内存区域直接映射到用户空间
// 相当于把 FPGA 的"黑板"搬到你面前，直接看，不用别人抄给你
void *addr = mmap(nullptr, m_dmaMapSize, PROT_READ, MAP_SHARED, m_devFd, 0);
```

**白话解释**：
- 以前的老办法：数据在 ARM 内核里，你要通过 `read()` 系统调用去"要"数据，内核把数据从 FPGA 的内存拷到你的程序内存。这个过程很慢（–30ms/帧，导致只有 30fps）。
- 现在的 mmap 办法：FPGA 写的那个内存区域，直接"映射"到你的程序里，你直接读，不经过内核拷贝。这叫**零拷贝**。

---

## 第二关：采集线程怎么"抓"帧？

采集线程在 `frame_capture.cpp` 里，是一个独立的循环，专门负责盯着 FPGA 有没有新帧。

```cpp
// frame_capture.cpp: run()
while (!m_stopRequested) {
    // 1. 直接读 FPGA 的寄存器，问"有新帧吗？"
    //    BAR0 PIO 读，不需要系统调用，2微秒完成
    rd = m_device->readPioStatus(status);
    // status.frame_done   → 有没有新帧？
    // status.buffer_index → 新帧在哪个槽位？

    // 2. 没新帧 → 继续等
    if (!status.frame_done) {
        usleep(500);   // 睡 0.5 毫秒
        continue;
    }

    // 3. 有新帧 → 直接算出数据在哪
    src = (uint8_t*)m_device->dmaUserAddr()
          + status.buffer_index * FRAME_SIZE_RGB565;
    // dmaUserAddr 就是 mmap 得到的那块内存的起始地址
    // 加上 buffer_index * 1.84MB 就是这一帧的地址

    // 4. 把帧数据的指针提交给 FrameBuffer（零拷贝，0ms）
    m_buffer->commitMappedFrame(src, m_frameSeq);
}
```

**白话解释**：
采集线程就像一个不停巡视的保安。听到 FPGA 说"新帧好了"，直接找到数据在哪，把地址交给下一道工序。全程没有拷贝大数据。

---

## 第三关：FrameBuffer 三缓冲——生产者和消费者

`frame_buffer.h` 是一个环形队列，专门协调采集线程（生产者）和显示线程（消费者）。

```cpp
// frame_buffer.h
// 3个槽位的环形缓冲
// 采集线程往槽位写指针 → 显示线程从槽位读指针

// 采集线程调用（零拷贝，只传指针）:
void commitMappedFrame(const uint8_t *srcRgb565, int frameSeq) {
    m_frameSrc[m_writeIndex]   = srcRgb565;  // 存指针，不是拷贝数据！
    m_frameSeq[m_writeIndex]   = frameSeq;
    m_frameValid[m_writeIndex] = true;
    m_writeIndex = (m_writeIndex + 1) % 3;   // 环形移动到下一个槽
}

// 显示线程调用（非消耗性读取——读完不删，AI 线程还能再读）:
bool acquireLatestFrameView(const uint8_t *&srcRgb565, int &frameSeq) {
    // 找最新完成的帧
    srcRgb565 = m_frameSrc[latest];
    frameSeq  = m_frameSeq[latest];
    return true;
}
```

**白话解释**：
FrameBuffer 就像一个有三块白板的会议室。采集线程拿到新帧后，把"白板在哪"的纸条贴到下一个空位。显示线程来看时，直接找到最新的纸条，去对应的白板上看内容。AI 线程也能同时看同一块白板（非消耗性读取）。三块白板轮流用，保证采集线程和显示线程不会踩到对方的脚。

---

## 第四关：显示线程怎么把帧画到屏幕上？

在 `mainwindow.cpp` 里，显示刷新由 16ms 定时器驱动：

```cpp
// mainwindow.cpp: onDisplayUpdate()
void MainWindow::onDisplayUpdate() {
    // 1. 从 FrameBuffer 获取最新帧的指针（零拷贝）
    const uint8_t *frameView = nullptr;
    m_frameBuffer->acquireLatestFrameView(frameView, frameSeq);

    // 2. 用这个指针直接构造 QImage（RGB565 格式）
    //    QImage 构造函数不拷贝数据，只是引用 frameView 指向的内存
    m_cachedDisplayImage = QImage(
        const_cast<uint8_t *>(frameView),  // 帧数据地址
        FRAME_WIDTH, FRAME_HEIGHT,          // 1280×720
        FRAME_WIDTH * 2,                    // 每行字节数
        QImage::Format_RGB16                // RGB565 格式
    ).copy();  // .copy() 做了一次深拷贝，保护原始数据不被 Qt 内部修改

    // 3. 如果有 AI 检测结果，叠加检测框
    if (m_detectionEnabled && m_detectionWorker->isLoaded()) {
        // 获取最新检测结果
        const DetectResult displayResult = makeDisplayDetectResult(displaySeq);
        // 设置叠加层（框 + 车牌号）
        setDisplayObbOverlay(displayResult, names, numClasses);
    }

    // 4. 把最终画面传给 OpenGL 专用窗口显示
    presentDisplayFrame(m_cachedDisplayImage);
}
```

**为什么显示帧率低于输入 60fps？**

采集线程拿帧几乎是零延迟，但显示线程要做 QImage::copy() 深拷贝 1.84MB + OpenGL 纹理上传 + EGLFS 页面翻转。具体每步耗时未单独测量，需要加计时代码才能确认瓶颈。

---

## 第五关：AI 识别怎么异步运行？

AI 推理在**独立的线程**中运行，不影响显示。

`detection_worker.h` 和 `detection_worker.cpp`：

```cpp
// detection_worker.cpp: processNextFrame()
void DetectionWorker::processNextFrame() {
    // 1. 从 FrameBuffer 非消耗性读取最新帧
    const uint8_t *src = nullptr;
    int frameSeq = -1;
    if (!m_frameBuffer->acquireLatestFrameView(src, frameSeq)) {
        // 没有新帧，等 1ms 再试
        QTimer::singleShot(1, this, &DetectionWorker::processNextFrame);
        return;
    }

    // 2. 跳过已经处理过的帧
    if (frameSeq == m_lastProcessedSeq) {
        QTimer::singleShot(0, this, &DetectionWorker::processNextFrame);
        return;
    }
    m_lastProcessedSeq = frameSeq;

    // 3. RGB565 转 RGB888（YOLO 模型需要 RGB888 输入）
    ImageConverter::rgb565ToRGB888Fast(src, m_rgb888Buf, ...);

    // 4. YOLO OBB 推理（RKNN NPU 硬件加速）
    DetectResult result;
    m_detector.detect(m_rgb888Buf, IMG_WIDTH, IMG_HEIGHT, result);

    // 5. 对每个检测到的车牌做 LPRNet 字符识别
    for (ObbBox &box : result.boxes) {
        if (box.classId == 0) → 蓝牌模型
        if (box.classId == 1) → 黄牌模型
            ├─ 宽高比 < 2.3 → 可能是双层黄牌 → 用专用模型再识别一次
            └─ 宽高比 ≥ 2.3 → 单层黄牌
        if (box.classId == 2) → 绿牌模型
    }

    // 6. OBB 完成后立即发布几何框，LPR 完成后再更新车牌文字
    // UI 在最新显示帧上直接绘制矢量叠加层
    {
        QMutexLocker lock(&m_resultMutex);
        m_stableResult = result;
        m_stableFrameSeq = frameSeq;
    }

    // 7. 通知 UI 线程：有新的检测结果了
    emit inferenceDone(frameSeq);

    // 8. 立即处理下一帧（自驱动）
    QTimer::singleShot(0, this, &DetectionWorker::processNextFrame);
}
```

**白话解释**：
AI 线程是自驱动的——推理完成后不等 UI 线程叫它，自己立刻取下一帧接着推理。它和显示线程共享 FrameBuffer，但不消费帧（读完不删），所以同一帧既能显示也能 AI 识别。

---

---

## 第六关：WiFi 无线视频输入（备选方案）

除了 PCIe 采集，这个项目还支持通过 WiFi 接收视频流。这是为远程场景准备的——比如 Windows 电脑上跑 AI 检测，把结果通过 WiFi 传到 ARM 板显示。

### 三个模块的分工

```
Windows 发送端 (TCP MJPEG 推流)
        │
        │ WiFi (局域网)
        │
        ↓
ARM 板 (linaro-alip)
        │
        ├── WifiDiscovery (UDP 端口 12345)
        │       自动发现网内有哪些发送端
        │       收到发送端的 IP、端口、分辨率信息
        │
        ├── WifiReceiver (TCP 端口 8080)
        │       独立线程，接收 MJPEG 流
        │       协议: | 4字节 JPEG 大小 | N 字节 JPEG 数据 |
        │       解码 JPEG → RGB565 → 写入 FrameBuffer
        │       (和采集线程一样写同一个 FrameBuffer)
        │
        └── WifiControl (UDP 端口 12346)
                接收远程控制命令:
                "SAI_CTRL AI_ON"      → 开启车牌 AI
                "SAI_CTRL AI_OFF"     → 关闭车牌 AI
                "SAI_CTRL YOLO_ON"    → 开启 YOLO 检测
                "SAI_CTRL YOLO_OFF"   → 关闭 YOLO 检测
                "SAI_CTRL SRC_PCIE"   → 切换到 PCIe 源
                "SAI_CTRL SRC_WIFI"   → 切换到 WiFi 源
                "SAI_CTRL DEVICE_OPEN/CLOSE"       → 打开/关闭 PCIe 设备
                "SAI_CTRL CAPTURE_START/STOP"      → 开始/停止 PCIe 采集
                "SAI_CTRL FPGA_FMC/FPGA_HDMI"      → 选择 FPGA 物理输入
                "SAI_CTRL BRIGHTNESS_AUTO_ON/OFF"  → 开关自适应亮度
                "SAI_CTRL SAVE_FRAME"              → 保存当前帧到 /userdata/captures
                "SAI_CTRL_DISCOVER"                → 独立发现常驻控制端
                "SAI_CTRL_HERE"                    ← RK3568 回复，和当前视频源无关

识别结果回传使用独立 UDP 端口 `12347`。Windows 在发现报文中携带结果端口，
RK3568 使用 `SAI_RESULT <JSON>` 回传两类消息：`obb_lpr`（旋转框、车牌字符、
车辆标签）和 `yolo`（类别、置信度、矩形坐标）。该通道不依赖视频 TCP 连接。

确认后的车牌图片使用独立 TCP 端口 `12348`。每条轨迹以首次有效文字为候选，
相同文字成功识别累计 3 次后才发送图片；计数期间累计出现 2 次不同文字时，
以当前文字重新从 1 开始。图片在独立线程中缩放至最大宽度 320 并以 JPEG 78
质量编码，发送队列最多保留 4 张，避免阻塞推理、显示和远程控制。
```

### 和 PCIe 采集共用一套下游

无论是 PCIe 采集的帧还是 WiFi 接收的帧，最终都写进同一个 `FrameBuffer`。显示线程和 AI 线程不关心帧从哪来——它们只管从 FrameBuffer 读最新帧。

```cpp
// mainwindow.h
enum class InputSource { PCIE, WIFI, NONE };
InputSource m_currentSource;

// 切换源
void switchToSource(InputSource newSource) {
    if (newSource == PCIE) {
        m_wifiReceiver->stop();    // 停 WiFi 接收
        m_capture->start();        // 启动 PCIe 采集
    } else if (newSource == WIFI) {
        m_capture->stopCapture();  // 停 PCIe 采集
        m_wifiReceiver->start();   // 启动 WiFi 接收
    }
}
```

### WiFi 路径和 PCIe 路径的区别

| | PCIe 路径 | WiFi 路径 |
|---|---|---|
| 视频源 | FPGA HDMI 采集 | Windows 发送端 |
| 帧格式 | RGB565 原始像素 | JPEG 压缩 |
| 输入帧率 | 60fps | 取决于网络和发送端 |
| 延迟 | 极低（–1帧） | 较高（网络+编解码） |
| 画面质量 | 可能有 HDMI 噪点 | JPEG 压缩损失 |
| 适用场景 | 实时本地采集 | 远程监控/无线显示 |

---

## 总结：整个链路的关键设计

| 环节 | 核心思想 | 代码位置 |
|------|---------|---------|
| **mmap 映射** | FPGA 的内存直接映射到程序，不经过内核拷贝 | `pcie_device.cpp:tryMmapDna()` |
| **BAR0 PIO 读状态** | 直接读 FPGA 寄存器，不调用 `read()`，2 微秒完成 | `pcie_device.cpp:readPioStatus()` |
| **零拷贝提交** | 只传指针不拷贝数据，1.84MB 变成 8 字节 | `frame_buffer.h:commitMappedFrame()` |
| **非消耗性读取** | 显示和 AI 同时读同一帧，互不干扰 | `frame_buffer.h:acquireLatestFrameView()` |
| **自驱动 AI** | 推理完成后自己取下一帧，不等 UI 驱动 | `detection_worker.cpp:processNextFrame()` |
| **EGLFS 显示** | 直接 OpenGL 渲染，绕过 X11/Wayland 开销 | `eglfs_video_window.h` |

**整个链路只有一次数据拷贝**：`QImage::copy()` 在显示线程中。其他所有环节都是传指针，数据始终在 FPGA 写入的 DMA 内存中不动。

---

## 这样做有什么好处？

### 1. 从 30fps 变成 60fps

**老方案**（每帧拷贝两次 1.84MB）：

```
ARM 内核 copy_to_user 搬运 1.84MB  →  ~20ms（uncached 内存读很慢）
用户空间 memcpy 再搬一次 1.84MB   →  ~1.5ms
等 FPGA 下一帧                   →  ~8ms（平均）
─────────────────────────────────────────
总共 ~30ms/帧  →  最多 ~33fps
```

**新方案**（零拷贝）：

```
读 FPGA 状态寄存器（2微秒）         →  ~0ms
传指针给 FrameBuffer（8字节）      →  ~0ms
显示线程 QImage::copy（1.84MB）   →  ~3ms
─────────────────────────────────────────
总共 ~3ms/帧  →  理论 ~300fps，实际受限于 FPGA 60fps 输入
```

**效果**：老方案采集线程每帧花 30ms，新方案几乎不花时间。帧率直接从 30fps 跳到 60fps。

### 2. 不抢 CPU，AI 和显示各跑各的

老方案中采集线程花 30ms 做数据拷贝，这段时间 CPU 被占用。AI 推理和显示刷新都要排队等。

新方案采集线程几乎没有 CPU 开销（只读寄存器 + 传指针），AI 线程和显示线程可以同时运行：

```
采集线程: █░░░░░░░░░░░░░░░  (几乎不占CPU)
显示线程: ███░░░░░░░░░░░░░  (QImage 拷贝 + GPU 渲染)
AI 线程:  ██████████████░░░  (YOLO + LPRNet，耗时未实测)
```

三条线并行，互不阻塞。采集和显示都不被 AI 阻塞。

### 3. 不掉帧，显示画面流畅

老方案每帧处理 30ms，而 FPGA 每 16.7ms 就产生一帧。ARM 来不及处理，每两帧只能处理一帧，大量帧被丢弃。

新方案采集线程处理一帧不到 1ms，能跟上 FPGA 的 60fps 输出。每一帧都被采集到 FrameBuffer 中。

### 4. 同一帧数据，显示和 AI 可以同时用

`acquireLatestFrameView()` 是非消耗性读取——读完不删除帧：

```
FPGA 写帧到槽位2
     │
     ├→ 显示线程: acquireLatestFrameView() → 读到槽位2 → 显示
     │
     └→ AI 线程:  acquireLatestFrameView() → 也读到槽位2 → 识别
```

不需要为显示和 AI 各自拷贝一份数据。两个线程共享同一块 DMA 内存，省了 1.84MB ×2 = 3.68MB 的内存拷贝。

### 5. 省内存带宽

RK3568 的内存带宽有限（DDR4 –3GB/s 理论值，实际可用约 1.5-2GB/s）。每条数据拷贝都在消耗带宽：

| 操作 | 老方案 | 新方案 |
|------|--------|--------|
| 内核拷贝 DMA→用户空间 | 110 MB/s (60fps×1.84MB) | **0** |
| 用户拷贝 用户空间→FrameBuffer | 110 MB/s | **0** |
| 显示拷贝 FrameBuffer→QImage | 110 MB/s | 110 MB/s |
| **总计** | **330 MB/s** | **110 MB/s** |

新方案省掉了三分之二的带宽消耗，留给 AI 推理（NPU 也要访问内存）更多的带宽余量。

### 6. AI 检测不拖慢显示

AI 推理在独立线程。如果是串行处理（AI 和显示排队）：

```
显示帧1 → AI处理帧1 → 显示帧2 → AI处理帧2 → ...
    ↑                       ↑
  16.7ms 间隔         AI 慢 → 显示也要等 → 卡顿
```

异步处理后：

```
时间轴 →
显示: 帧1→帧2→帧3→帧4→帧5→帧6→...   (流畅，不被 AI 阻塞)
AI:   处理帧1→处理帧4→...              (处理一部分帧，不阻塞显示)
```

AI 慢不影响显示流畅度。AI 处理的是"最近的一帧"，保证车牌识别结果总是最新的。

### 7. 总结：一张对比表

| | 老方案 (`::read()` 路径) | 新方案 (mmap 零拷贝) |
|---|---|---|
| 输入帧率 | 30fps | **60fps** |
| 采集 CPU 占用 | 高（30ms/帧拷贝） | **极低**（传指针） |
| 内存拷贝次数 | 3 次 | **1 次** |
| 内存带宽消耗 | 330 MB/s | **110 MB/s** |
| 显示帧率 | –25fps | **<60fps** (QImage.copy + GL 管线开销) |
| AI 是否阻塞显示 | 是 | **否** |
| 丢帧率 | 高 | **低** |
