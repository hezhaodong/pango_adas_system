#!/bin/sh
# start_60.sh -- 60Hz 稳定链路启动器（不改学长代码，只做启动环境）
# 关键修正（相对 start_demo.sh）：
#   1) 不再跑 pin_hdmi.sh（它每秒整段读 BAR，会读到 0x140 读清 → 每秒吞掉一次帧完成通知）
#   2) 起 app 前把 DSI-1 连接器置 off（板上有第二个 CRTC 在跑 1280x720@69，
#      app 的 logActiveDrmMode() 取所有 active CRTC 的 max refresh → pacer 被顶到 69Hz，
#      与 60Hz 面板不匹配 → flip backlog → 周期性卡顿）
#   3) 显式导出学长脚本里用到的帧率相关环境变量（SWAPINTERVAL / CAPTURE_COPY / CAPTURE_CPU）
set -u
LOG=/tmp/rc.log
HS=/userdata/hs_tools

echo "=== start_60 $(date '+%F %T') ==="

# --- 1. 清场：app / run_camera / pin_hdmi / X ---
pkill -9 -f pin_hdmi.sh 2>/dev/null || true
pkill -9 -f camera_capture 2>/dev/null || true
pkill -9 -f run_camera.sh 2>/dev/null || true
systemctl stop lightdm 2>/dev/null || systemctl stop display-manager 2>/dev/null || true
sleep 2
echo "procs app=$(pgrep -c -f camera_capture) xorg=$(pgrep -c Xorg) pin=$(pgrep -c -f pin_hdmi.sh)"

# --- 2. 关掉多余 CRTC（DSI-1），让 pacer 跟随 HDMI 的 60Hz ---
if [ -w /sys/class/drm/card0-DSI-1/status ]; then
    echo off > /sys/class/drm/card0-DSI-1/status 2>/dev/null || true
fi
sleep 1
echo "crtc-active:"
grep -E 'crtc\[|mode:' /sys/kernel/debug/dri/0/state 2>/dev/null | head -6

# --- 3. 60Hz 启动环境（照抄学长 run_camera_60.sh + 其 1080p120 脚本里的 SWAPINTERVAL） ---
export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_kms
export QT_QPA_EGLFS_KMS_CONFIG=/userdata/kms.json
export QT_QPA_EGLFS_HIDECURSOR=1
export QT_QPA_EGLFS_SWAPINTERVAL=1
unset QT_QPA_EGLFS_ALWAYS_SET_MODE
unset QT_QPA_EGLFS_KMS_ATOMIC
unset QT_QPA_EGLFS_FORCE888
unset HS_POSTSET_HDMI_MODE
unset HS_REQUIRE_DRM_HZ
export CAPTURE_COPY=5
export CAPTURE_CPU=3

# --- 4. 起 app（只有 /dev 缺失才 insmod；绝不 rmmod） ---
cd /userdata/high_speed_pcie || exit 1
if [ ! -e /dev/pango_pci_driver ]; then
    insmod /userdata/high_speed_pcie/pango_pci_driver.ko 2>/dev/null || true
fi

# FPGA 重新烧录后 PCIe 设备要重新枚举，否则 pio 读回全是 0xFFFFFFFF
# （app 的 capture-guard 会因 STOP 确认超时拒绝映射 DMA）。
# 顺序照抄已验证的 start_demo.sh：app 已杀 → unbind → bind → setpci memory-enable。
printf "3c0800000.pcie" > /sys/bus/platform/drivers/rk-pcie/unbind 2>/dev/null || true
sleep 3
printf "3c0800000.pcie" > /sys/bus/platform/drivers/rk-pcie/bind 2>/dev/null || true
sleep 6
setpci -s 0002:21:00.0 4.w=0x0006 2>/dev/null || true
echo "pci cmd=$(setpci -s 0002:21:00.0 4.w 2>/dev/null) dev=$(lspci -nn 2>/dev/null | grep -c 0755)"
if [ ! -x /tmp/reg_read ] && [ -f /userdata/hs_tools/regread.c ]; then
    gcc -O2 -o /tmp/reg_read /userdata/hs_tools/regread.c 2>/dev/null || true
fi
if [ -x /tmp/reg_read ]; then
    echo "bar0 probe: $(/tmp/reg_read 0xf0200000 1 2>/dev/null | head -1)"
fi
: > $LOG
setsid sh ./run_camera.sh < /dev/null > $LOG 2>&1 &
echo "app launched, waiting 55s ..."
sleep 55

# --- 5. 读数（只用 app 自己的 hw_delta，不回读 0x140） ---
echo "--- pacer / kms ---"
grep -h -E 'display-pacer|display-gl|display-kms' $LOG | tail -4
echo "--- rate ---"
grep -h capture-rate $LOG | tail -3
echo "--- regs ---"
if [ -x /tmp/reg_read ]; then
    /tmp/reg_read 0xf0200000 128 2>/dev/null | grep -E '^0x(0A0|0B0|0C0|110|120|2A0) '
else
    echo "reg_read missing"
fi
echo "--- mem ---"
grep -E 'Cma(Total|Allocated|Free)' /proc/meminfo
echo "--- shot ---"
ffmpeg -y -loglevel error -f kmsgrab -i - -frames:v 1 \
    -vf hwdownload,format=bgr0,scale=1000:-1 -q:v 4 /tmp/demo_shot.jpg 2>/dev/null \
    && ls -l /tmp/demo_shot.jpg || echo "kmsgrab failed"
echo "=== START60_DONE ==="
