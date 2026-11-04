# 标准版（ADAS + 60Hz 基础跑通版）— 冻结于 2026-10-28 17:52

以后**一句"跑标准版"**＝本目录 + `RUN_标准版_ADAS60.ps1`。除非明确要求，不再动 RTL / 不再改 app。

## 0. 这一版是什么

| 项目 | 值 |
|---|---|
| 位流 | `BASELINE_hdmi_loop_E819B0EF.sbit`，md5 **`E819B0EFB79EC59DD7736EF0BB9355DB`**（2,113,704 B，720p60 采集 + 调试寄存器，PREP_MODE=0 直通） |
| 板端 app | `camera_capture_v2` = **ADAS 增强版** 1,060,720 B，md5 **`ae8604c98f0e847dc106cbb074b0aeee`**（学长原版逻辑 + ADAS 增量补丁 + 2026-11-01 叠加偏移修复 + 线型固定显示） |
| 原版备份 | 板上 `camera_capture_v2.bak` 986,120 B md5 `d5062c1896e3a350641c40cd91ade7ae`；ADAS 版副本 `camera_capture_v2.adas60` |
| 启动脚本 | 板端 `/userdata/hs_tools/start_60.sh`（md5 `6f72c218c1448d1ffc2d69f69c3590d6`）+ `plain_start.sh`（`d1a970b5…`） |
| 键注入 | 板端 `/userdata/hs_tools/adas_keys.sh`（`ce7bee31…`），自动编译 `/tmp/keyinject` |
| ADAS 源码包 | `adas_20261028.tar.gz`（md5 `206ce366e6bfe9829e49cd2703d9bb7a`），对应 `D:\Users\daaa\high\08-ADAS新增源码\` |
| 实测指标 | 输入 55~61 FPS（`source=HDMI`、`copy_mode=5`、`proc≈2.6~2.9ms`）、显示 46~57 FPS、`0x110=0x0012E1FC`、`0x2D0=2`、`0x2A0` 持续增长 |

## 1. 一键跑（板子断电重来也适用）

```powershell
powershell -File "D:\Users\daaa\high\_work\ADAS_60HZ_2026-10-28\RUN_标准版_ADAS60.ps1" -Shot
# 想跑完自动打开 ADAS 叠加层：加 -AdasOn
```
脚本顺序（**不要增删步骤**）：查 COM7 → 起 cdt_js(65420) → 锁显示器2=1280x720@60 → 烧 E819 位流 → 板端 `start_60.sh` → 读 `capture-rate`（`<45` 自动 `plain_start.sh`）→ 可选截图。

## 2. 先提条件（缺一不可）

1. 板子上电；CH340(COM7) + Pango 下载器(COM13/14) 两根 USB 插好。
2. 笔记本 HDMI 接 **MES-40P-HDMI 子卡 HDMI IN**；底板 HDMI OUT 接显示器。
3. Windows 侧：扩展模式（`DisplaySwitch.exe /extend`），显示器 2 = 1280×720@60（脚本会自动设）。
4. 视频用**窗口（1280×720 左右）**播放，**不要全屏**（全屏会让 Windows 把显示器 2 切 1080p，采集拉丝）。

## 3. 按键表（ADAS 版）

| 键 | 作用 |
|---|---|
| 空格 | ADAS 演示开/关（关闭时画面只显示一行灰字） |
| ↑ | 前进（直行指令） |
| ↓ | 停止 |
| ← | 强制左变道（直接切换车道，不再报警） |
| → | 强制右变道（直接切换车道，不再报警） |
| M | 水平回卷 256px 开关（默认关） |
| 6 | 学长"自动驾驶辅助"（YOLO+3D 检测） |
| 1 | 车牌识别 |
| Esc | ⚠️ 退出 app，别按 |

注入方式（串口）：`sh /userdata/hs_tools/adas_keys.sh space|up|down|left|right|m|6|1`

## 4. 期望读数（判断是否正常）

| 寄存器/日志 | 正常值 |
|---|---|
| `[capture-rate] input=` | 45~61，`source=HDMI`，`copy_mode=5` |
| `0x110` | `0x0012E1FC`（1650×750 = 720p60） |
| `0x2D0` | `0x00000002`（START 生效） |
| `0x2A0` | 每秒 +50~60 持续增长 |
| `0x310` | `0x303` / `0x503` / `0x603`（均表示 HDMI 有源） |
| HUD | `输入 xx FPS / 显示 xx FPS`，ADAS 面板 + 三格车道条 + 灯条 |

## 5. 故障速查

| 现象 | 处理 |
|---|---|
| 画面全黑 / 没画面 | `nohup sh /userdata/hs_tools/plain_start.sh > /tmp/ps.log 2>&1 &`，等 40 秒 |
| `input=1~5`（很卡） | 同上（只重启 app，**不要** unbind/bind） |
| `input=121` + 拉丝 | 显示器 2 被切成 1080p：`setdisp.ps1 -DisplayIndex 2 -Width 1280 -Height 720 -Hz 60` |
| `0x310=0x500/0x700`（源变 FMC/Fiber） | 干净重启板子 + 重烧位流（脚本重跑一次） |
| 板端 `gcc: inner compiler error` / app 段错误 | 内存被写坏：`echo b > /proc/sysrq-trigger` 硬重启 → 重跑脚本 |
| JTAG `No devices detected` | 换 USB 口 + 板子断电 15 秒 + 重启 cdt_js |
| 截图失败 `/tmp/demo_shot.jpg` 不存在 | 用 `-Shot` 时请确认 app 起过（`start_60.sh` 会自己抓一张）；必要时先 `plain_start.sh` 再抓 |

## 6. 纪律（血泪）

1. **绝不 `rmmod pango_pci_driver`**（会内核 Oops，只能断电）。
2. **app 在跑时不要 unbind/bind、不要写 `0x300`**（会锁内核 / 把换源状态机搞挂）。
3. 不按 **Esc**；视频窗口**不要全屏**。
4. 改 app / 编译前先 `pkill -9 -f camera_capture; pkill -9 -f run_camera.sh; sleep 2`（否则内存被 DMA 写坏）。
5. 每次**断电后必须重烧位流**（Flash 启动坏的，位流在 SRAM）。
6. `/tmp` 重启即清；持久脚本都在 `/userdata/hs_tools/`。

## 7. 回滚 / 切换

```sh
# 板上切回纯基线（无 ADAS 叠加）
cp -f /userdata/high_speed_pcie/camera_capture_v2.bak /userdata/high_speed_pcie/camera_capture_v2
# 切回 ADAS 标准版
cp -f /userdata/high_speed_pcie/camera_capture_v2.adas60 /userdata/high_speed_pcie/camera_capture_v2
# 之后都要 plain_start.sh 重启 app
```
纯基线的一键脚本仍在 `D:\Users\daaa\high\_work\BASELINE_60HZ_2026-10-28\RUN_基础跑通版.ps1`（位流相同，只是 app 用原版）。

## 8. 本目录文件 md5

见 `MD5SUMS.txt`。
