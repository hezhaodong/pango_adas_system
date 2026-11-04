# 标准版（ADAS + 60Hz 基础跑通版）

> 冻结时间：2026-10-28 18:0x
> 本包目标：**新工作区里也能直接跑通**。除了 Pango PDS 与板子 USB，不需要本机任何其它残留文件。

---

## 0. 先看这一页（30 秒）

**这是什么**：紫光同创 PG2L50H FPGA + 瑞芯微 RK3568 的软硬协同 ADAS 演示系统。
笔记本 HDMI → MES-40P-HDMI 子卡 → FPGA（720p60 采集、RGB565、异步 FIFO）→ **PCIe Gen2 ×2** → RK3568（NPU 跑 YOLO/车牌/红绿灯 + 我们的 ADAS 决策与叠加）→ 底板 HDMI OUT → 显示器。
实测：**输入约 60 FPS（58~61 抖动）、单帧处理约 4.1 ms（含采集端 256 px 回卷纠偏；纠偏前 2.6~3.3 ms）、显示 46~57 FPS（全功能 ADAS 叠加时约 32 FPS）**。

**一条命令跑通**（板子已上电、两根 USB 已插、显示器接着底板 HDMI OUT）：

```powershell
powershell -NoProfile -File ".\run\RUN_STANDARD.ps1" -Shot
```

看到 `=== PASS (STANDARD ADAS@60Hz): input=59fps ===` 就是成功；截图会存到本包 `shots\standard_<时间>.jpg`。

**先验证环境不烧录**：加 `-DryRun`。

---

## 1. 目录结构

```
high_FINAL_2026-10-28\
├─ README.md                          ← 本文件（先读这个）
├─ run\                                ← 上位机：一键跑通所需的一切
│   ├─ RUN_STANDARD.ps1                一键脚本（-DryRun / -Shot / -AdasOn / -WaitSec N / -Port COM7）
│   ├─ flash_baseline.tcl              JTAG 烧录脚本（内含绝对路径，指向本包 run 目录）
│   ├─ BASELINE_hdmi_loop_E819B0EF.sbit  冻结位流（md5 E819B0EFB79EC59DD7736EF0BB9355DB）
│   ├─ setdisp.ps1 / setdisp_types.cs  Windows 多显示器控制（把显示器 2 锁成 1280x720@60）
│   ├─ rkget.py / rk.py               板端文件下载工具（串口 base64 分块，纯 ctypes 不依赖 pyserial）
│   └─ MD5SUMS.txt                     本包所有文件的 md5
├─ board\                              ← 板端脚本（已经预置在板子 /userdata/hs_tools/ 里，这里留备份）
│   ├─ start_60.sh                     起链路：杀 app→停 lightdm→平台 unbind/bind→setpci→insmod→起 app→抓图
│   ├─ plain_start.sh                 只重启 app（低帧率/黑屏时最有效的急救）
│   ├─ adas_keys.sh                    按键注入（空格/上/下/左/右/M/6/1），自动编译 keyinject
│   ├─ adas_build3.sh                  在板上重编 ADAS 版 app（先还原 pristine 再打补丁）
│   ├─ regread.c / regwrite.c          寄存器读写小工具（gcc 编译，读 BAR0）
│   ├─ allinone.sh / verify.sh         完整 bringup / 一键自检
├─ src_adas\                           ← 我们做的 ADAS 增量源码（插进学长工程，不改他的逻辑）
│   ├─ adas_decision.h（学长原件）adas_bridge/adas_glue/adas_overlay/adas_lane_type
│   ├─ adas_patch.py（幂等插入补丁器）build_on_board.sh  test/test_adas_logic.cpp（PC 自测 67/67）
│   └─ adas_20261028.tar.gz            整体打包（md5 206ce366e6bfe9829e49cd2703d9bb7a）
├─ src_arm\                            ← 学长 ARM 工程快照（Qt/EGLFS + RKNN 推理，可 make 重编）
├─ src_fpga\                           ← FPGA 源码
│   ├─ rtl\                            17 个 .v（hdmi_loop / pio_crtl / video_crtl 引用 / disp_prep / img_proc_pipeline …）
│   └─ fdc\                            管脚约束 hdmi_loop.fdc（V1.1 子卡：pixclk=W19）
├─ src_tools\                          ← keyinject.c / pcie_check.c
├─ docs\                               ← 文档
│   ├─ README_标准版.md                冻结说明：期望读数表、按键表、故障速查、纪律
│   ├─ RUNBOOK_BASELINE.md             基础版 RUNBOOK（纯基线，无 ADAS）
│   ├─ 赛题三_完成度评估与演示视频脚本_2026-10-28.md   完成度自查 + 3 分钟分镜脚本
│   └─ ADAS_60HZ_2026-10-28.md         ADAS 编入 60Hz 版的变更记录
└─ shots\                              ← 证据图（60fps、ADAS 叠加、检测框、十字路口等）
```

---

## 2. 跑之前必须满足的 6 个条件

1. **板子上电**（断电后 FPGA 配置丢失，位流在 SRAM，所以要重烧——脚本会自动烧）。
2. **两根 USB**：CH340 串口（COM7，1,500,000 8N1）+ Pango 下载器（COM13/14）。
3. **Pango PDS 2022.2-SP6.4** 装在 `D:\Pango\PDS_2022.2-SP6.4\`（脚本用里面的 `cdt_js.exe` / `cdt_cfg_shell.exe`）。
4. **接线**：笔记本 HDMI → 子卡 **HDMI IN**；底板 HDMI OUT → 显示器。
5. **Windows**：扩展模式；显示器 2 = 1280×720@60（脚本会自动设，但要求显示器 2 存在）。
6. **视频播放要窗口化**（约 1280×720），**不要全屏**——全屏会让 Windows 把显示器 2 切 1080p，采集会拉丝。

---

## 3. 按键表（板端 app 收键盘事件，需要 USB 键盘或 `adas_keys.sh`）

| 键 | 作用 |
|---|---|
| 空格 | ADAS 演示开 / 关 |
| ↑ | 前进（直行指令） |
| ↓ | 停止 |
| ← | 强制左变道（直接切换车道，不再报警） |
| → | 强制右变道（直接切换车道，不再报警） |
| M | 水平回卷 256px（默认关，切屏时可用） |
| 6 | 学长"自动驾驶辅助"（YOLO + 3D 检测） |
| 1 | 车牌识别 |
| Esc | ⚠️ 直接退出 app，**别按** |

串口注入：`sh /userdata/hs_tools/adas_keys.sh space|up|down|left|right|m|6|1`

---

## 4. 正常时的读数（判断是否真的跑起来了）

| 位置 | 正常值 |
|---|---|
| `/tmp/rc.log` 的 `[capture-rate]` | `input=45~61 source=HDMI copy_mode=5 proc_avg_us≈2600~3300` |
| 寄存器 `0x110` | `0x0012E1FC`（=1650×750，720p60 锁定） |
| 寄存器 `0x2D0` | `0x00000002`（START 已生效） |
| 寄存器 `0x2A0` | 每秒 +50~60，持续增长 |
| 寄存器 `0x310` | `0x303 / 0x503 / 0x603`（都表示 HDMI 有源） |
| HUD | 左上角 `输入 xx FPS / 显示 xx FPS`；ADAS 面板 + 三格车道条 + LED 灯条 |

读寄存器：板上 `/tmp/reg_read 0xf0200000 96`（没编译就 `gcc -O2 -o /tmp/reg_read /userdata/hs_tools/regread.c`）。

---

## 5. 故障速查（按顺序试）

| 现象 | 处理 |
|---|---|
| 全黑 / 没画面 | `nohup sh /userdata/hs_tools/plain_start.sh > /tmp/ps.log 2>&1 &`，等 40 秒 |
| `input=1~5` 很卡 | 同上；**不要** unbind/bind |
| 画面拉丝且 `input=121` | 显示器 2 被切成 1080p：`run\setdisp.ps1 -DisplayIndex 2 -Width 1280 -Height 720 -Hz 60` |
| `0x310=0x500/0x700` | 重跑一次一键脚本（干净重烧 + 重启 app） |
| 板端 `gcc` 报 inner compiler error / app 段错误 | 内存被 DMA 写坏：`echo b > /proc/sysrq-trigger` 硬重启后重跑脚本 |
| JTAG `No devices detected` | 换 USB 口 + 板子断电 15 秒 + 重启 `cdt_js` |
| `FAIL: flash` | 看是否 `PROGRAM_OK` 缺失；确认 `cdt_js` 在 65420、位流路径存在 |
| 截图没出来 | `-Shot` 依赖 python 与板上 `/tmp/demo_shot.jpg`（`start_60.sh` 起 app 时会抓一张） |

---

## 6. 六条纪律（都是踩过的坑）

1. **绝不 `rmmod pango_pci_driver`** —— 会内核 Oops，只能断电。
2. **app 在跑时不要 unbind/bind、不要写 `0x300`** —— 会锁内核 / 把换源状态机搞挂。
3. **不按 Esc**；视频**不要全屏**。
4. 改 app / 编译前先 `pkill -9 -f camera_capture; pkill -9 -f run_camera.sh; sleep 2`。
5. **每次断电都要重烧位流**。
6. `/tmp` 重启即清，持久脚本都在 `/userdata/hs_tools/`。

---

## 7. 如果要重编 / 改代码

### 7.1 ARM（板上直接编，qmake 齐全）
```sh
pkill -9 -f camera_capture; pkill -9 -f run_camera.sh; sleep 2
cd /userdata/hs_tools && tar -xzf adas_20261028.tar.gz -C /userdata/hs_tools/adas_pkg --strip-components=1
sh /userdata/hs_tools/adas_build3.sh          # 会先按 *.adas.bak 还原 pristine 再打补丁再 make
# 产物 /userdata/high_speed_pcie/camera_capture_v2（ADAS 版 1,059,704 B）
```
回滚：`cp /userdata/high_speed_pcie/camera_capture_v2.bak /userdata/high_speed_pcie/camera_capture_v2` + `plain_start.sh`

### 7.2 FPGA（在装有 PDS 的 Windows 上）
```
1) 把 src_fpga\rtl\*.v 与 src_fpga\fdc\hdmi_loop.fdc 放进 PDS 工程目录
2) 工程：C:\hdbuild\proj50\hdmi_loop.pds（若无，可从 src_fpga 重建：new project, device PG2L50H-6FBG484, top=hdmi_loop）
3) 命令行：pds_shell.exe -project C:\hdbuild\proj50\hdmi_loop.pds -work_dir C:\hdbuild\proj50
   流程：compile -top_module hdmi_loop → synthesize → dev_map → pnr → report_timing → gen_bit_stream
4) 产物 generate_bitstream\hdmi_loop.sbit ≈ 2,113,704 B，用 run\flash_baseline.tcl 改路径即可烧
```
> 说明：`disp_prep.v` / `med9_28ce.v` / `img_proc_pipeline` 系列是**预处理 IP**（灰度/去噪/边缘/二值/YCbCr/ROI/帧差）。当前冻结位流中 `u_img_proc` 已综合，但其输入取自 OV5640 DVP 接口；本演示链路只接 HDMI（`VIDEO_SOURCE=1`）且显示通路直通，因此预处理结果只经 PIO 0x200~0x208 暴露、不上屏。若要在 HDMI 通路上启用，需把 `disp_prep` 插回 `v_src_clk` 域后重编位流。

---

## 8. 版本与校验

- 位流：`E819B0EFB79EC59DD7736EF0BB9355DB`
- 板端 app（ADAS 版，2026-11-01 修复叠加偏移 + 线型固定显示后重编）：`ae8604c98f0e847dc106cbb074b0aeee`（1,060,720 B）
- 板端原版备份：`d5062c1896e3a350641c40cd91ade7ae`（986,120 B）
- ADAS 源码包：`206ce366e6bfe9829e49cd2703d9bb7a`
- 全部文件 md5：`run\MD5SUMS.txt`
