#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
KMS_120_CONFIG="$APP_DIR/kms_720p120.json"
APP_BIN="$APP_DIR/camera_capture_v2"

if [ ! -f "$KMS_120_CONFIG" ]; then
    echo "Missing KMS configuration: $KMS_120_CONFIG" >&2
    exit 1
fi

# The camera and the monitor both expose native 1280x720@120.  Keep the GBM
# buffer and CRTC mode identical so Qt uses the stable legacy page-flip path;
# 1080p120 plus a scaled 720p atomic plane eventually failed with -EBUSY.
if ! grep -Eq '"mode"[[:space:]]*:[[:space:]]*"1280x720@120"' "$KMS_120_CONFIG" \
    || ! grep -Eq '"format"[[:space:]]*:[[:space:]]*"xrgb8888"' "$KMS_120_CONFIG"; then
    echo "ERROR: $KMS_120_CONFIG is stale; expected native 1280x720@120 xrgb8888." >&2
    exit 3
fi

# Do not silently run an old binary after an interrupted `make`.  The marker
# is compiled into the capture and display pacing fixes used by the 120-fps
# path.
if [ ! -x "$APP_BIN" ]; then
    echo "Missing executable: $APP_BIN" >&2
    exit 3
fi
if command -v strings >/dev/null 2>&1; then
    if ! strings "$APP_BIN" | grep -q 'RGA hardware DMA-to-cache snapshot' \
        || ! strings "$APP_BIN" | grep -q 'deadline_timer=enabled' \
        || ! strings "$APP_BIN" | grep -q 'single_update_pending=enabled' \
        || ! strings "$APP_BIN" | grep -q 'pending_timeout_submissions=suppressed' \
        || ! strings "$APP_BIN" | grep -q 'lastWindowClosed; automatic quit disabled' \
        || ! strings "$APP_BIN" | grep -q '\[display-stall\]' \
        || ! strings "$APP_BIN" | grep -q 'capture continues' \
        || ! strings "$APP_BIN" | grep -q 'frameSwapped_feedback=disabled' \
        || ! strings "$APP_BIN" | grep -q 'fifo_high_water_pixels=' \
        || ! strings "$APP_BIN" | grep -q 'first KMS swap complete before PCIe START'; then
        echo "ERROR: camera_capture_v2 is stale or the build was interrupted." >&2
        echo "Finish: qmake camera_display.pro && make -B -j4" >&2
        exit 3
    fi
fi

# Use the monitor's advertised native 720p120 mode for 720p camera content.
if command -v modetest >/dev/null 2>&1; then
    DRM_MODES=$(modetest -M rockchip -c 2>/dev/null || true)
    if ! printf '%s\n' "$DRM_MODES" \
        | grep -Eq '#[0-9]+[[:space:]]+1280x720[[:space:]]+120([.]0+)?'; then
        echo "ERROR: HDMI output EDID/DRM does not expose 1280x720@120." >&2
        echo "Refusing the unsupported mode to avoid a black screen or EGLFS crash." >&2
        echo "Connect the detected high-refresh HDMI monitor, then run this launcher again." >&2
        exit 2
    fi
fi

# This overrides /userdata/kms.json deliberately. The normal launcher gives
# /userdata priority, which otherwise silently keeps the HDMI output at 60 Hz.
# Do not inherit a stale board/shell value such as eglfs_x11.  Qt must select
# the requested mode while constructing QScreen and the GBM/EGL surface; a
# late CRTC-only mode change does not rebuild Qt's presentation scheduler.
export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_kms
export QT_QPA_EGLFS_KMS_CONFIG="$KMS_120_CONFIG"
export QT_QPA_EGLFS_ALWAYS_SET_MODE=1
# Do not inherit the experimental atomic-scaling path.  On this Rockchip Qt5
# stack it can return -EBUSY after running for a while and leave display flips
# stalled even though capture continues normally.
unset QT_QPA_EGLFS_KMS_ATOMIC
export QT_QPA_EGLFS_SWAPINTERVAL=1
export QT_QPA_EGLFS_FORCE888=1
export QT_LOGGING_RULES="qt.qpa.*=true"
export QT_LOGGING_TO_CONSOLE=1

# RGA reads the physical PCIe DMA slot and writes the cached display ring.
# This removes the measured ~12.1-ms single-A55 uncached copy that limited a
# real 120-fps FPGA stream to about 83 processed/displayed frames per second.
export CAPTURE_COPY=5
export CAPTURE_CPU=3
unset HS_POSTSET_HDMI_MODE
export HS_REQUIRE_DRM_HZ=120
unset HS_DISPLAY_PACER

echo "Requesting native HDMI output 1280x720@120 for true 120 fps content updates"
echo "KMS config: $QT_QPA_EGLFS_KMS_CONFIG"
echo "EGLFS integration: $QT_QPA_EGLFS_INTEGRATION (forced; ignores stale eglfs_x11 environment)"
echo "Qt KMS connector: HDMI1 (kernel DRM name is normally HDMI-A-1)"
echo "Qt must initialize QScreen and EGL directly at 1280x720@120; late DRM mode switching is disabled."
echo "Render buffer: native 1280x720 XRGB8888; DRM atomic scaling is disabled."
echo "CAPTURE_COPY=$CAPTURE_COPY (RGA hardware DMA-to-cache snapshot)"
echo "CAPTURE_CPU=$CAPTURE_CPU (capture on CPU3; CPU0..2 remain available)"
if [ -n "${CAMERA_IPCAM_URL:-}" ]; then
    echo "IP Cam URL: supplied through CAMERA_IPCAM_URL"
else
    echo "IP Cam: pass --ip-camera-url=http://phone-ip:port/video when starting this script"
fi

exec "$APP_DIR/run_camera.sh" "$@"
