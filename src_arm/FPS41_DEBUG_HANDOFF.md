# RK3568 + Pango FPGA：HDMI 视频稳定在 41 FPS 的调试交接清单

## 1. 目标

定位视频链路从约 60 FPS 降到稳定约 41 FPS 的根因。用户反馈：

- 普通重启、重新上电不能恢复原帧率。
- 历史上重新烧写完整 RK3568 系统镜像后可以恢复。
- 当前摄像头/FMC仍有画面；HDMI曾被一次不安全的PCIe恢复操作弄丢，冷启动后重新出现，但帧率仍约41。

本轮只做低开销诊断，不训练模型、不跑AI推理测试、不重新综合FPGA。

## 2. 平台与路径

- ARM：RK3568，Linux 6.1.99，aarch64。
- FPGA PCIe endpoint：`0755:0755`。
- 板端工程：`/mnt/usb/ARM_final1`。
- ARM程序：`camera_capture_v2`。
- 驱动设备：`/dev/pango_pci_driver`。
- 本地工程：`ARM_final1/`。
- FPGA工程：`FPGA_final/`。
- 恢复日志：`/mnt/usb/rk_pcie_system_recovery_*.log`。
- 应用日志：`/mnt/usb/camera_after_recovery_*.log`。

## 3. 已确认事实

1. PCIe没有降速：

   - `LnkSta: Speed 5GT/s, Width x2`
   - `MaxPayload 128 bytes, MaxReadReq 512 bytes`
   - 当前日志没有AER链路错误。

2. 41 FPS不能直接归因于PCIe带宽。

3. 程序HUD同时存在两个指标：

   - `输入 FPS`：采集线程识别到的新DMA帧数量。
   - `显示 FPS`：Qt/EGLFS实际提交显示的帧数量。

   必须先确认用户说的“41”是哪一个，不能混用。

4. 日志反复出现：

   ```text
   Could not queue DRM page flip on screen HDMI1 (Device or resource busy)
   ```

   因此显示端DRM page flip冲突是高优先级嫌疑。如果“输入≈60、显示≈41”，优先查DRM，不查DMA。

5. 一次HDMI切换日志为：

   ```text
   [fpga-source-debug] result=OK target=HDMI wait_ms=26 final=0x703
   active=true requested=true busy=false fmc_valid=0 hdmi_valid=0 epoch=7
   ```

   `result=OK`目前只判断active/requested/busy，没有判断目标源valid，因此该OK可能是假成功。

6. `FrameCapture`的预期高速路径：

   - `tryMmapDma()`成功后，`useMmap=true`。
   - HDMI使用`commitMappedFrame()`，采集线程不复制整帧。
   - 非mmap路径使用驱动`read()`搬运整帧，理论上只能约30～40 FPS。

7. 工程使用了`QT_NO_DEBUG_OUTPUT`，`qDebug()`被编译掉。因此当前日志不会显示：

   - DMA mmap成功/失败。
   - BAR映射细节。

   若增加诊断，应使用低频`qWarning()`，不要打开大量逐帧日志。

## 4. 恢复脚本和驱动的重要风险

不要再使用旧版恢复逻辑执行以下操作：

- `rmmod/insmod pango_pci_driver`
- PCI function reset
- PCI remove/rescan

原因：

1. 旧操作曾导致HDMI分支消失。
2. 驱动日志出现：

   ```text
   sysfs: cannot create duplicate filename '/class/pango_pci_driver'
   ```

3. 驱动源码的`init_pango_cdev_class(struct class *pango_cdev_class)`按值传递指针，存在创建后的class指针没有写回`pci_info._cdev_class`的缺陷，可能导致卸载时sysfs class没有正确销毁。

本地新版`recover_rk_system_pipeline.sh`已改为仅执行PCI驱动`unbind -> bind`，不卸载模块、不复位endpoint。板端是否已同步需要确认。

## 5. 最小诊断步骤（按顺序）

### A. 首先记录HUD两个FPS

让用户原样报告：

```text
输入 xx.x FPS | 显示 yy.y FPS | 当前源 HDMI/FMC
```

判断：

- 输入≈60、显示≈41：直接进入步骤D，查DRM显示端。
- 输入≈41、显示≈41：进入步骤B/C，查采集路径和FPGA计数。

### B. 不改代码，确认DMA mmap是否存在

板端执行：

```sh
for D in /proc/[0-9]*; do E=$(readlink "$D/exe" 2>/dev/null || true); case "$E" in *camera_capture_v2*) echo "PID=${D##*/} EXE=$E"; grep -E "pango_pci_driver|/dev/mem" "$D/maps";; esac; done
```

判断：

- 映射表出现约7.37 MB的`/dev/pango_pci_driver`映射：DMA mmap路径大概率有效。
- 只有设备fd、没有对应mmap，或者进程不存在：高速路径没有建立。
- 注意程序可能显示为`camera_capture_v2 (deleted)`，表示编译覆盖后旧进程仍在运行。

### C. 只增加每秒一次的采集诊断

不要逐帧打印。建议另一个AI在`FrameCapture::run()`中增加：

1. 启动时一次：

   ```text
   [capture-path] mmap=true/false source=...
   ```

2. 每秒一次：

   ```text
   [capture-rate] input=... process=... hw_vs_delta=... hw_frame_done_delta=...
   ```

可用FPGA BAR0只读调试计数器：

- `0x260`：video controller VS计数。
- `0x270`：video controller TX计数。
- `0x280`：DMA request计数。
- `0x290`：DMA done计数。
- `0x2a0`：frame done计数。

限制：

- 每个计数器每秒读取一次即可。
- 不要由外部工具读取`0x140`；它是read-to-clear完成通知，会干扰采集线程。
- 如果`pioReadReg()`为刷新PCIe completion而重复读，同样保持总频率很低。

判断矩阵：

| hw_vs | hw_frame_done | ARM input | 结论 |
|---:|---:|---:|---|
| ≈60 | ≈60 | ≈41 | ARM漏读/合并了完成通知，重点查`0x140`单bit latch和轮询调度 |
| ≈60 | ≈41 | ≈41 | FPGA DMA/行传输吞吐瓶颈 |
| ≈41 | ≈41 | ≈41 | HDMI输入时序或video controller只收到约41帧 |
| ≈60 | ≈60 | ≈60，显示≈41 | DRM/EGLFS显示瓶颈 |

### D. 检查DRM占用，低开销

板端执行：

```sh
fuser -v /dev/dri/card0 /dev/dri/card1 /dev/dri/renderD128 2>/dev/null || true
```

```sh
ps -ef | grep -E "camera_capture_v2|weston|Xorg|kwin|kms|eglfs" | grep -v grep
```

目标：确认是否有两个显示进程、系统服务或旧的`camera_capture_v2 (deleted)`同时占用DRM。

不要在程序运行时重新编译覆盖二进制。正确流程：先终止旧进程，确认DRM和PCI设备都释放，再编译、启动。

### E. 查看当前应用关键日志

```sh
APP=$(ls -1t /mnt/usb/camera_after_recovery_*.log | head -n 1); grep -E "fpga-source|DRM page flip|frame-timeout|dma-integrity|capture-path|capture-rate" "$APP" | tail -n 200
```

### F. 系统镜像差异对比

因为只有“重烧完整系统”曾恢复，下一次出现正常60 FPS时立即保存：

- `uname -a`
- `/proc/cmdline`
- 正在使用的`pango_pci_driver.ko` SHA256和vermagic
- `/boot`下实际DTB SHA256
- `camera_capture_v2` SHA256
- `run_camera.sh` SHA256
- PCIe `lspci -vv -d 0755:0755`

然后与41 FPS状态逐项比较。系统重烧可能恢复的是驱动、DTB、启动参数或应用版本，而不是瞬态硬件状态。

## 6. 取得证据后再做的代码修复

1. 修正`setFpgaInputSource()`：`switched`必须同时检查目标源valid，避免`hdmi_valid=0`仍报告OK。
2. 修正内核驱动class指针生命周期，再考虑允许`rmmod/insmod`。
3. 如果`hw_frame_done≈60`但ARM≈41：把单bit read-to-clear完成通知改为单调递增frame sequence/counter，ARM按计数差消费最新槽，避免多个完成事件合并成一个。
4. 如果只有显示≈41：确保单一DRM owner，检查EGLFS多屏配置和page flip pending逻辑，不修改FPGA/DMA。
5. 如果mmap失败：先修复驱动mmap和DMA映射，不优化UI或FPGA。

## 7. 明确禁止事项

- 不连续读取大量BAR调试寄存器。
- 不从第二个进程读取`0x140`。
- 不重新执行旧版`R`恢复脚本。
- 不在应用运行时覆盖`camera_capture_v2`。
- 不在修复class生命周期前重复卸载/加载Pango驱动。
- 不在明确`输入FPS`和`显示FPS`之前改FPGA时钟或DMA逻辑。

