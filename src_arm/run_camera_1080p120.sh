#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
KMS_1080P120_CONFIG="$APP_DIR/kms_1080p120.json"
APP_BIN="$APP_DIR/camera_capture_v2"

if [ ! -f "$KMS_1080P120_CONFIG" ]; then
    echo "Missing KMS configuration: $KMS_1080P120_CONFIG" >&2
    exit 1
fi

# Native 1920x1080@120 (EDID-advertised, VIC 0x48/0x49) with 1280x720 content
# size. The 720p camera frame is scaled to the 1080p120 CRTC mode.
if ! grep -Eq '"mode"[[:space:]]*:[[:space:]]*"1920x1080@120"' "$KMS_1080P120_CONFIG"; then
    echo "ERROR: $KMS_1080P120_CONFIG is stale; expected 1920x1080@120." >&2
    exit 3
fi
if ! grep -Eq '"size"[[:space:]]*:[[:space:]]*"1280x720"' "$KMS_1080P120_CONFIG"; then
    echo "ERROR: $KMS_1080P120_CONFIG is stale; expected content size 1280x720." >&2
    exit 3
fi

if [ ! -x "$APP_BIN" ]; then
    echo "Missing executable: $APP_BIN" >&2
    exit 3
fi
if command -v strings >/dev/null 2>&1; then
    if ! strings "$APP_BIN" | grep -q 'sigpipe=ignored' \
        || ! strings "$APP_BIN" | grep -q 'DMA stopped before Qt teardown' \
        || ! strings "$APP_BIN" | grep -q 'debug_2d0='; then
        echo "ERROR: camera_capture_v2 is stale; rebuild before using 1080p120." >&2
        echo "Run: qmake camera_display.pro && make -B -j4" >&2
        exit 3
    fi
fi

# Confirm the connected display advertises 1080p120 before switching.
if command -v modetest >/dev/null 2>&1; then
    DRM_MODES=$(modetest -M rockchip -c 2>/dev/null || true)
    if ! printf '%s\n' "$DRM_MODES" \
        | grep -Eq '#[0-9]+[[:space:]]+1920x1080[[:space:]]+120([.]0+)?'; then
        echo "ERROR: HDMI EDID/DRM does not expose 1920x1080@120." >&2
        exit 2
    fi
fi

export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_kms
export QT_QPA_EGLFS_KMS_CONFIG="$KMS_1080P120_CONFIG"
export QT_QPA_EGLFS_ALWAYS_SET_MODE=1
unset QT_QPA_EGLFS_KMS_ATOMIC
export QT_QPA_EGLFS_SWAPINTERVAL=1
export QT_QPA_EGLFS_FORCE888=1
export QT_QPA_EGLFS_HIDECURSOR=1
export QT_LOGGING_RULES="qt.qpa.*=true"
export QT_LOGGING_TO_CONSOLE=1

export CAPTURE_COPY=5
export CAPTURE_CPU=3
unset HS_POSTSET_HDMI_MODE
export HS_REQUIRE_DRM_HZ=120
unset HS_DISPLAY_PACER

echo "Requesting native HDMI 1920x1080@120 with 720p content scaling"
echo "KMS config: $QT_QPA_EGLFS_KMS_CONFIG"
echo "Camera stream is 720p120; display runs 1080p120 (content scaled to CRTC)."
echo "If display stalls with -EBUSY, Ctrl+C and run ./run_camera_60.sh"

exec "$APP_DIR/run_camera.sh" "$@"
