#!/bin/sh
# adas_keys.sh -- inject a key press into the running camera_capture_v2 (EGLFS)
# usage: sh /userdata/hs_tools/adas_keys.sh space
#        sh /userdata/hs_tools/adas_keys.sh up|down|left|right|enter|esc|a|q
# keyinject.c lives on /userdata so it survives reboot; /tmp binary is rebuilt on demand.
if [ ! -x /tmp/keyinject ]; then
    gcc -O2 -o /tmp/keyinject /userdata/hs_tools/keyinject.c || { echo "BUILD_KEYINJECT_FAIL"; exit 21; }
fi
modprobe uinput 2>/dev/null
exec /tmp/keyinject "$@"
