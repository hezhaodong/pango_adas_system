# RUN_STANDARD.ps1 -- 最终提交版一键跑通（ADAS + 60Hz 标准版）
# 2026-10-28 frozen. ASCII-only (PowerShell 5.1 parses non-ASCII .ps1 as ANSI).
#
# 用法（在任意目录执行都行，脚本自动定位本包）：
#   powershell -NoProfile -File "<本包>\run\RUN_STANDARD.ps1"
#   powershell -NoProfile -File "...\RUN_STANDARD.ps1" -Shot          # 跑完再抓图
#   powershell -NoProfile -File "...\RUN_STANDARD.ps1" -AdasOn        # 跑完按空格打开 ADAS 叠加
#   powershell -NoProfile -File "...\RUN_STANDARD.ps1" -WaitSec 75    # 加长热机时间
#   powershell -NoProfile -File "...\RUN_STANDARD.ps1" -DryRun        # 只检查依赖，不烧录不启动
#
# 依赖（外部软件，必须在机器上）：
#   - Pango PDS 2022.2-SP6.4  ->  D:\Pango\PDS_2022.2-SP6.4\bin\cdt_js.exe / cdt_cfg_shell.exe
#   - 板子 CH340 串口（默认 COM7，1500000 8N1）+ Pango 下载器
#   - Windows PowerShell 5.1 + python（用于 rkget.py 下载截图；PS 脚本本身不需要 python）
#
# 步骤（顺序固定，别增删）：
#   0 依赖检查  1 串口  2 cdt_js:65420  3 锁显示器2=1280x720@60  3b 先杀板上 app
#   4 烧位流  5 板端 start_60.sh  6 读帧率（<45 自动 plain_start.sh）  7 可选按空格/抓图
#
param(
    [switch]$Shot,
    [switch]$AdasOn,
    [switch]$DryRun,
    [int]$WaitSec = 60,
    [string]$Port = 'COM7'
)
$ErrorActionPreference = 'Continue'
$PKG  = Split-Path $PSScriptRoot -Parent          # <本包>
$RUN  = Join-Path $PKG 'run'
$BIT  = Join-Path $RUN 'BASELINE_hdmi_loop_E819B0EF.sbit'
$TCL  = Join-Path $RUN 'flash_baseline.tcl'
$SHOTDIR = Join-Path $PKG 'shots'
$APP_MD5 = 'ae8604c98f0e847dc106cbb074b0aeee'      # 板端 ADAS 版 app（1,060,720 B，2026-11-01 修复叠加偏移 + 线型固定显示后重编）

function Say($m) { Write-Output $m }
$fail = 0

# ---- 0 dependencies ----
foreach ($p in @($BIT, $TCL, (Join-Path $RUN 'setdisp.ps1'), (Join-Path $RUN 'rkget.py'))) {
    if (Test-Path $p) { Say "OK file $(Split-Path $p -Leaf)" } else { Say "FAIL missing: $p"; $fail++ }
}
$PDS = 'D:\Pango\PDS_2022.2-SP6.4\bin'
foreach ($e in @('cdt_js.exe', 'cdt_cfg_shell.exe')) {
    if (Test-Path (Join-Path $PDS $e)) { Say "OK tool $e" } else { Say "FAIL missing tool: $PDS\$e"; $fail++ }
}
if (-not ($SHOTDIR)) { $fail++ }
if ($fail -gt 0) { Say "FAIL: $fail dependency problem(s) -> fix before running"; exit 2 }
Say "OK dependencies"

if ($DryRun) {
    Say 'DRY RUN: dependencies OK; steps 1-7 skipped.'
    Say "package root = $PKG"
    exit 0
}

# ---- 1 serial ----
if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) {
    Say "FAIL: $Port (CH340) missing -> plug board USB and power on"
    exit 3
}
Say "OK serial $Port"

# ---- 2 cdt_js ----
$up = Test-NetConnection -ComputerName 127.0.0.1 -Port 65420 -InformationLevel Quiet -WarningAction SilentlyContinue
if (-not $up) {
    Start-Process (Join-Path $PDS 'cdt_js.exe') -ArgumentList '-port', '65420'
    Start-Sleep -Seconds 5
    $up = Test-NetConnection -ComputerName 127.0.0.1 -Port 65420 -InformationLevel Quiet -WarningAction SilentlyContinue
}
if ($up) { Say "OK cdt_js 65420" } else { Say "FAIL: cdt_js 65420 not listening"; exit 4 }

# ---- 3 lock Display2 = 1280x720@60 (before flashing: subcard EDID appears only after FPGA config) ----
& (Join-Path $RUN 'setdisp.ps1') -DisplayIndex 2 -Width 1280 -Height 720 -Hz 60 2>&1 |
    Select-Object -Last 1 | ForEach-Object { Say "display2: $_" }
Start-Sleep -Seconds 3

# ---- serial helper ----
$sp = New-Object System.IO.Ports.SerialPort $Port, 1500000, 'None', 8, 'One'
$sp.ReadTimeout = 1500; $sp.NewLine = "`n"
try { $sp.Open() } catch { Say "FAIL: open $Port : $_"; exit 5 }
Start-Sleep -Milliseconds 700; [void]$sp.ReadExisting()

function Brd($cmd, $sec) {
    $sp.Write("`r"); Start-Sleep -Milliseconds 500; [void]$sp.ReadExisting()
    $sp.Write("stty sane`r"); Start-Sleep -Milliseconds 300; [void]$sp.ReadExisting()
    $sp.Write("$cmd`r")
    $deadline = (Get-Date).AddSeconds($sec); $o = ''
    while ((Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 500; $o += $sp.ReadExisting() }
    return ($o -replace "`e\[[0-9;?]*[a-zA-Z]", '') -replace "`r", ''
}

# ---- 3b kill running app BEFORE re-configuring the FPGA (else kernel can wedge) ----
[void](Brd 'pkill -9 -f camera_capture; pkill -9 -f run_camera.sh; sleep 2; echo KILLED' 10)
Say 'OK board app stopped'

# ---- 4 flash frozen bitstream ----
$flog = & (Join-Path $PDS 'cdt_cfg_shell.exe') -file ($TCL -replace '\\', '/') 2>&1
if ($flog -match 'The done bit is 1' -and $flog -match 'PROGRAM_OK') {
    Say ("OK flashed " + (Get-FileHash $BIT -Algorithm MD5).Hash)
} else {
    Say 'FAIL: flash'
    ($flog | Select-Object -Last 10) | ForEach-Object { Say "  $_" }
    exit 6
}
Start-Sleep -Seconds 3

# ---- 5 board pipeline ----
$m = Brd 'md5sum /userdata/high_speed_pcie/camera_capture_v2' 6
if ($m -match $APP_MD5) { Say "OK board app = ADAS build" }
else {
    Say "WARN: board app is not the frozen ADAS build (expected $APP_MD5)."
    Say "      fix on board: cp /userdata/high_speed_pcie/camera_capture_v2.adas60 /userdata/high_speed_pcie/camera_capture_v2"
}
[void](Brd 'rm -f /tmp/sd.log; nohup sh /userdata/hs_tools/start_60.sh > /tmp/sd.log 2>&1 &' 8)
Say "start_60.sh launched, waiting ${WaitSec}s ..."
Start-Sleep -Seconds $WaitSec

$r = Brd 'tail -12 /tmp/sd.log' 10
Say '--- start_60.sh ---'
$r -split "`n" | Where-Object { $_ -match 'capture-rate|0x110|0x120|START60_DONE|snap' } |
    ForEach-Object { Say ("  " + $_.Trim()) }

# ---- 6 verdict ----
function Rate { $o = Brd 'grep -h capture-rate /tmp/rc.log | tail -1' 8; $mm = [regex]::Match($o, 'input=(\d+)'); if ($mm.Success) { [int]$mm.Groups[1].Value } else { -1 } }
$fps = Rate
Say "fps = $fps"
if ($fps -lt 45) {
    Say 'fps low -> plain_start.sh (restart app only; never rebind PCIe while app runs)'
    [void](Brd 'nohup sh /userdata/hs_tools/plain_start.sh > /tmp/ps.log 2>&1 &' 8)
    Start-Sleep -Seconds 35
    $fps = Rate
    Say "fps after plain_start = $fps"
}

# ---- 6b optional ADAS overlay ----
if ($AdasOn) { [void](Brd 'sh /userdata/hs_tools/adas_keys.sh space' 4); Say 'ADAS overlay toggled' }

# ---- 7 optional screenshot ----
for ($i = 0; $i -lt 2; $i++) { $sp.Write([string][char]3); Start-Sleep -Milliseconds 400 }
$sp.Close(); Start-Sleep -Seconds 2

if ($Shot) {
    $ts = Get-Date -Format 'yyyyMMdd_HHmmss'
    $dst = Join-Path $SHOTDIR "standard_$ts.jpg"
    $env:MSYS2_ARG_CONV_EXCL = '*'
    $py = (Get-Command python -ErrorAction SilentlyContinue)
    if ($py) {
        & python (Join-Path $RUN 'rkget.py') /tmp/demo_shot.jpg $dst 2>&1 | Select-Object -Last 1 | ForEach-Object { Say "shot -> $_" }
    } else { Say "skip screenshot: python not found" }
}

if ($fps -ge 45) { Say "=== PASS (STANDARD ADAS@60Hz): input=${fps}fps ==="; exit 0 }
Say "=== FAIL (STANDARD): input=${fps}fps -> see docs/README_standard.md section 5 ==="
exit 7
