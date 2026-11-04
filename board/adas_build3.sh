#!/bin/sh
# adas_build3.sh -- rebuild with ADAS increment, handling patcher idempotency:
#   if *.adas_bak exist -> restore pristine sources first, then patch fresh
set -x
PKG=/userdata/hs_tools/adas_pkg
APP=/userdata/high_speed_pcie
cd "$APP" || exit 30

[ -f camera_capture_v2 ] && [ ! -f camera_capture_v2.adas_prev ] && cp -p camera_capture_v2 camera_capture_v2.adas_prev

# restore pristine (patcher backups) so that patcher really inserts the NEW version
for b in camera_display.pro.adas_bak mainwindow.h.adas_bak mainwindow.cpp.adas_bak \
         eglfs_video_window.h.adas_bak eglfs_video_window.cpp.adas_bak; do
    if [ -f "$b" ]; then
        orig=$(echo "$b" | sed 's/\.adas_bak$//')
        cp -f "$b" "$orig" && echo "restored $orig from backup"
    fi
done

for f in adas_decision.h adas_bridge.h adas_bridge.cpp adas_lane_type.h adas_lane_type.cpp \
         adas_glue.h adas_glue.cpp adas_overlay.h adas_overlay.cpp adas_patch.py; do
    cp -f "$PKG/$f" ./ || exit 31
done
ls -d "$PKG/test" >/dev/null 2>&1 && cp -rf "$PKG/test" ./ 2>/dev/null

python3 ./adas_patch.py . || exit 32

# verify the new symbols really got inserted
grep -q "adasCheckSignal" mainwindow.h && grep -q "adasCheckSignal" mainwindow.cpp || { echo "PATCH_MISSING_SIGNAL_WATCHDOG"; exit 36; }

rm -f camera_capture_v2 adas_bridge.o adas_glue.o adas_overlay.o adas_lane_type.o \
      eglfs_video_window.o mainwindow.o moc_mainwindow.o
qmake camera_display.pro -o Makefile || exit 33
make -j4 2>&1 | tail -20 || exit 34

if [ -x camera_capture_v2 ]; then
    echo "BINARY_BYTES=$(stat -c %s camera_capture_v2)"
    echo "ADAS_LINKED=$(nm camera_capture_v2 2>/dev/null | grep -ci adas)"
    echo "VEIL_STRINGS=$(strings camera_capture_v2 | grep -c '输入无信号')"
    echo "BUILD3_DONE"
else
    echo "BUILD3_FAIL"
    exit 35
fi
