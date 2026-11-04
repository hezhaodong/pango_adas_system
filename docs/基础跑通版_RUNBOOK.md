# 基础跑通版（BASELINE，2026-10-28 15:00 冻结）

这是**唯一保底可用**的版本：学长的原版 ARM 程序 + 原版风格 RTL 重编位流，
笔记本显示器 2 的画面经 子卡 → FPGA → PCIe → RK3568 → 底板 HDMI OUT → 显示器，
实测 `输入 60.0 FPS / 显示 59.9 FPS`；按 `6` 出学长自带 AI 检测（car 94%/84%/67%，33~37 FPS）。

## 0. 冻结件（缺一不可，已 md5 校验）

| 件 | 位置 | md5 / 大小 |
|---|---|---|
| 位流（BASELINE） | `D:\Users\daaa\high\_work\bit\BASELINE_hdmi_loop_E819B0EF.sbit` | `E819B0EFB79EC59DD7736EF0BB9355DB` / 2,113,704B |
| 烧录 tcl | `D:\Users\daaa\high\_work\flash_baseline.tcl`（写死烧 `C:/hdbuild/proj50/generate_bitstream/hdmi_loop.sbit`） | — |
| 板端 ARM 程序 | `/userdata/high_speed_pcie/camera_capture_v2` | 986,120B（学长原版 = `.bak` 同尺寸） |
| 板端一键启动 | `/userdata/hs_tools/start_demo.sh` | 3,759B |
| 板端兜底重启 | `/userdata/hs_tools/plain_start.sh` | 803B（只重启 app，不动 PCIe） |
| 显示控制 | `D:\Users\daaa\high\_work\setdisp.ps1`（P/Invoke ChangeDisplaySettingsEx） | — |
| 一键总入口 | `D:\Users\daaa\high\_work\RUN_基础跑通版.ps1` | — |

## 1. 硬件前提（缺一不可）

- 板子上电；**CH340 串口线**（COM7）与 **Pango 下载器**（COM13/14）都插好。
- 笔记本 HDMI 线插在**子卡 HDMI IN**；显示器接**底板 HDMI OUT**。
- 笔记本显示设为**扩展**，显示器 2 = `1280×720@60`，**视频用窗口放（1280×720），不要全屏**。

## 2. 一键命令

```powershell
powershell -File D:\Users\daaa\high\_work\RUN_基础跑通版.ps1 -Shot
```

内部严格按这个顺序（多一步都可能坏）：

1. 查 `COM7`；起 `cdt_js.exe -port 65420`（若未起）。
2. **先锁显示**：`setdisp.ps1 -DisplayIndex 2 -Width 1280 -Height 720 -Hz 60`。
3. **再烧位流**：把 BASELINE sbit 拷到 `C:\hdbuild\proj50\generate_bitstream\hdmi_loop.sbit`，
   跑 `cdt_cfg_shell.exe -file D:/Users/daaa/high/_work/flash_baseline.tcl`
   → 必须看到 `The done bit is 1` + `PROGRAM_OK`。
4. 板端 `nohup sh /userdata/hs_tools/start_demo.sh > /tmp/sd.log 2>&1 &`，等 60 秒。
5. 判读：`[capture-rate] input=58~61 source=HDMI` → PASS；
   若 `input<45`，只跑 `/userdata/hs_tools/plain_start.sh`（**只重启 app，绝不动 PCIe**），再判。
6. `-Shot` 时用 `ffmpeg kmsgrab` 抓图并 `rkget.py` 下载到 `_shots\`。

## 3. 期望读数（2026-10-28 14:21 实测）

```
0x110 = 0x0012E1FC      # 1650x750 = 720p60 输入
0x140 = 0x0000030C      # 源=HDMI, frame_done=0
0x2A0 = 每秒 +60        # 帧计数
0x2D0 = 0x00000002      # START 已生效
0x310 = 0x00000303      # active/request = HDMI
0x320 = 0x494D4750      # 指纹
[capture-rate] input=61 source=HDMI processed=61 slot_skips=0 copy_mode=5 proc_avg_us≈2600
```

按 `6` 后：`input≈33`，屏幕出现红色车辆框（`car 94%` 等）。

## 4. 三条纪律（血泪）

1. **不要按 Esc**：EGLFS 下键盘归板端 app，Esc 是它的退出键（一按 app 退 → 显示器黑）。
   退出 Edge 全屏用 **F11**，或先 `Alt+Tab` 切焦点。
2. **视频不要全屏**：全屏会让 Windows 把显示器 2 切 1080p → `input=121`（假高帧）+ 黑屏。
3. app 跑起来后**不要**再 `unbind/bind`、不要写 `0x300` 钉源、不要 `rmmod pango_pci_driver`
   （会 0 帧 / 4fps / 内核 Oops）。

## 5. 故障速查

| 现象 | 处理 |
|---|---|
| 显示器黑、`pgrep -c camera_capture`=0 | `sh /userdata/hs_tools/plain_start.sh`（只重启 app） |
| `input=1~4`、画面仍正确 | 同上（只重启 app），**不要** unbind/bind |
| `input=121` + 拉丝/黑 | 显示器 2 被切成 1080p → 重新锁 `1280x720@60`，关掉全屏播放 |
| 板端 `gcc` 报 ICE / app 随机崩 | 板子内存被 DMA 写坏 → `echo b > /proc/sysrq-trigger` 强制重启，再走第 2 节 |
| JTAG `No devices detected` | 换 USB 口 + 板子断电上电 + 重启 cdt_js |
| `0x2A0` 不涨、`0x310=0x500`（无源） | 干净重启 + 重烧位流（不要手动写 0x300） |

## 6. 证据

- `D:\Users\daaa\high_speed_pcie_one_camer(1)\_shots\live_60fps_1028.jpg`（HUD 输入 60.0 / 显示 59.9）
- `D:\Users\daaa\high_speed_pcie_one_camer(1)\_shots\det_60fps_1028.jpg`（按 6 后 car 94%/84%/67%）
