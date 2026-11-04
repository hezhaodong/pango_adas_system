#!/bin/bash
# 只读采集当前 41fps 系统状态, 不修改任何配置。
# 将来恢复 60fps 后重新运行, diff 两份结果定位根因。

OUT="${1:-/mnt/usb/fps41_system_snapshot_$(date +%Y%m%d_%H%M%S).txt}"
exec > >(tee -a "$OUT") 2>&1

echo "=== uname ==="
uname -a

echo ""
echo "=== cmdline ==="
cat /proc/cmdline

echo ""
echo "=== display manager ==="
systemctl status lightdm 2>/dev/null || echo "lightdm not found"
systemctl status display-manager 2>/dev/null || echo "display-manager not found"

echo ""
echo "=== DRM owners ==="
for D in /dev/dri/card0 /dev/dri/card1 /dev/dri/renderD128; do
    echo "--- $D ---"
    fuser -v "$D" 2>/dev/null || echo "(no owner)"
done

echo ""
echo "=== camera_capture_v2 env ==="
PID=$(pgrep -f camera_capture_v2 | head -1)
if [ -n "$PID" ]; then
    tr '\0' '\n' < "/proc/$PID/environ" | grep -E "^QT_|^EGL|^DISPLAY|^WAYLAND"
else
    echo "process not running"
fi

echo ""
echo "=== KMS / EGLFS config ==="
for F in /userdata/kms.json /etc/kms.json /tmp/kms.json; do
    if [ -f "$F" ]; then
        echo "--- $F ---"
        cat "$F"
    fi
done

echo ""
echo "=== DRM connector state ==="
for C in /sys/class/drm/card*-*; do
    [ -d "$C" ] || continue
    echo "--- $C ---"
    echo "status: $(cat "$C/status" 2>/dev/null)"
    echo "enabled: $(cat "$C/enabled" 2>/dev/null)"
    echo "modes: $(cat "$C/modes" 2>/dev/null)"
done

echo ""
echo "=== DRM debug state ==="
[ -f /sys/kernel/debug/dri/0/state ] && cat /sys/kernel/debug/dri/0/state || echo "debugfs not mounted"

echo ""
echo "=== CPU freq ==="
for P in /sys/devices/system/cpu/cpufreq/policy*; do
    [ -d "$P" ] || continue
    echo "--- $P ---"
    echo "governor: $(cat "$P/scaling_governor" 2>/dev/null)"
    echo "cur_freq: $(cat "$P/scaling_cur_freq" 2>/dev/null)"
    echo "max_freq: $(cat "$P/scaling_max_freq" 2>/dev/null)"
    echo "min_freq: $(cat "$P/scaling_min_freq" 2>/dev/null)"
done

echo ""
echo "=== GPU freq ==="
for P in /sys/class/devfreq/*gpu* /sys/class/devfreq/*mali*; do
    [ -d "$P" ] || continue
    echo "--- $P ---"
    echo "governor: $(cat "$P/governor" 2>/dev/null)"
    echo "cur_freq: $(cat "$P/cur_freq" 2>/dev/null)"
    echo "max_freq: $(cat "$P/max_freq" 2>/dev/null)"
done

echo ""
echo "=== DMC freq ==="
for P in /sys/class/devfreq/*dmc*; do
    [ -d "$P" ] || continue
    echo "--- $P ---"
    echo "governor: $(cat "$P/governor" 2>/dev/null)"
    echo "cur_freq: $(cat "$P/cur_freq" 2>/dev/null)"
done

echo ""
echo "=== SHA256 (binary artifacts) ==="
for F in \
    /usr/lib/aarch64-linux-gnu/libMali.so* \
    /usr/lib/aarch64-linux-gnu/qt5/plugins/egldeviceintegrations/libqeglfs-kms-* \
    /boot/rk3568*.dtb \
    /mnt/usb/ARM_final1/pango_pci_driver.ko \
    /mnt/usb/ARM_final1/camera_capture_v2
do
    for G in $F; do
        [ -f "$G" ] && echo "$(sha256sum "$G")  $G"
    done
done

echo ""
echo "=== done ==="
echo "output=$OUT"
