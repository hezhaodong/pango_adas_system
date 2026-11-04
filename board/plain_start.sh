#!/bin/sh
# plain_start.sh - restart app cleanly WITHOUT touching 0x300 (no source switching)
dmesg -n 1
pkill -9 -f pin_hdmi 2>/dev/null
pkill -9 -f camera_capture 2>/dev/null
pkill -9 -f run_camera.sh 2>/dev/null
sleep 2
systemctl stop lightdm 2>/dev/null
rm -f /tmp/camera_capture_v2.lock
cd /userdata/high_speed_pcie || exit 9
setsid sh ./run_camera.sh > /tmp/rcp.log 2>&1 &
sleep 30
echo "=== REG ==="
/tmp/reg_read 0xf0200000 400 2>/dev/null | grep -E '^0x(110|140|2A0|2D0|310|320) '
echo "=== RATE ==="
grep -hE "capture-rate|display-sched" /tmp/rcp.log | tail -2
grep -hE "recv source|source=" /tmp/rcp.log | tail -2
echo "=== SNAP ==="
ffmpeg -y -loglevel error -f kmsgrab -i - -frames:v 1 -vf "hwdownload,format=bgr0,scale=1000:-1" -q:v 4 /tmp/plain.jpg
ls -l /tmp/plain.jpg
echo "PLAIN_DONE"
