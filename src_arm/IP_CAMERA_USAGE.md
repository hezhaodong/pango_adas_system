# 手机 IP 摄像头输入（初版）

## 支持范围

- HTTP MJPEG 连续视频流
- IPv4、主机名和非 80 端口
- URL 中的 HTTP Basic Auth 用户名/密码
- 自动缩放到 1280×720，并接入原有显示、YOLO 和车牌识别流程
- 断线后每秒自动重连

初版不支持 HTTPS、RTSP/H.264/H.265 和仅能在网页 JavaScript 播放的地址。

## 启动方法

手机与 RK3568 接入同一局域网，在手机 IP Cam 应用中启动 MJPEG 服务，取得完整的视频流地址，例如：

```text
http://192.168.1.100:8080/video
```

启动 ARM 程序：

```bash
./camera_capture_v2 --ip-camera-url "http://192.168.1.100:8080/video"
```

带 Basic Auth 的地址也可以直接传入：

```bash
./camera_capture_v2 --ip-camera-url "http://user:password@192.168.1.100:8080/video"
```

密码不会出现在状态栏中。建议在手机端选择 1280×720、15～25 FPS、JPEG 质量 60～75。

## 输入源切换

设置 `--ip-camera-url` 只保存手机视频流地址；程序启动后仍然先显示 PCIe。按 `Q` 时按以下顺序切换：

```text
PCIe → WiFi 自定义发送器 → IP Cam → PCIe
```

未设置手机 URL 时，`Q` 保持原来的 PCIe/WiFi 两路切换。

## Windows WiFi 控制端

Windows `wifi_connect_gui.py` 的 RK3568 控制区提供：

- “手机 MJPEG”输入框：填写完整的 `http://...` 视频流地址；
- “应用地址”：通过 `SAI_CTRL IPCAM_URL <url>` 下发地址；
- “切到 IP Cam”：通过 `SAI_CTRL SRC_IPCAM` 切换输入源。

“RK3568 键盘等价控制”区还可远程触发板端 EGLFS 的全部快捷键：
`Q/H/R/A/E/1/2/3/4/5/6/7/Esc`。这些按钮发送 `SAI_CTRL KEY_<键名>`，
并在 ARM 端复用与物理键盘完全相同的处理函数；远程退出前会在 Windows
端要求二次确认。

先点击“应用地址”，收到 RK3568 确认后再点击“切到 IP Cam”。切回 PCIe 或原有 WiFi 输入仍使用同一行的对应按钮。

## 地址检查

先在同一局域网内的浏览器打开视频 URL。如果浏览器直接连续显示视频，通常就是可用的 MJPEG 地址；如果打开的是控制网页，需要在应用说明或网页中找到真正的 `/video`、`/mjpeg` 或 `/mjpegfeed` 地址。
