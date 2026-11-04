#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
APP_BIN="$APP_DIR/camera_capture_v2"
BOARD_KMS_CONFIG=${CAMERA_60_KMS_CONFIG:-/userdata/kms.json}

if [ ! -x "$APP_BIN" ]; then
    echo "Missing executable: $APP_BIN" >&2
    echo "Compile: qmake -o Makefile camera_display.pro && make -B -j4" >&2
    exit 3
fi
if [ ! -r "$BOARD_KMS_CONFIG" ]; then
    echo "Missing proven 60-Hz KMS configuration: $BOARD_KMS_CONFIG" >&2
    exit 2
fi

# Reproduce the previously proven 60-Hz launch environment.  In particular,
# keep the board's /userdata/kms.json instead of forcing a synthetic mode.
export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_kms
export QT_QPA_EGLFS_KMS_CONFIG="$BOARD_KMS_CONFIG"
export QT_QPA_EGLFS_HIDECURSOR=1

# Do not inherit any experimental 80/120-Hz mode forcing from the shell.
unset QT_QPA_EGLFS_ALWAYS_SET_MODE
unset QT_QPA_EGLFS_KMS_ATOMIC
export QT_QPA_EGLFS_SWAPINTERVAL=1
unset QT_QPA_EGLFS_FORCE888
unset HS_POSTSET_HDMI_MODE
unset HS_REQUIRE_DRM_HZ
unset HS_DISPLAY_PACER

echo "Using proven compatibility KMS configuration: $QT_QPA_EGLFS_KMS_CONFIG"
echo "Starting with WiFi endpoint 192.168.250.119:8080"
echo "Display pacer: refresh-deadline mode (prevents DRM flip backlog)"

# run_camera.sh supplies the same -w/-p values, creates XDG_RUNTIME_DIR and
# prevents two processes from controlling PCIe DMA at the same time.  Any
# --ip-camera-url argument is forwarded unchanged.
exec "$APP_DIR/run_camera.sh" "$@"
