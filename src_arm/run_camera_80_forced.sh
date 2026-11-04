#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
FORCED_CONFIG="$APP_DIR/kms_720p80_forced.json"

if [ ! -f "$FORCED_CONFIG" ]; then
    echo "Missing KMS configuration: $FORCED_CONFIG" >&2
    exit 1
fi

# Force a user-defined 720p80 mode that is not advertised by the connected
# monitor. Timings reuse CTA 720p blanking with a 99 MHz pixel clock:
# 99,000,000 / (1650 * 750) = 80.000 Hz.
export QT_QPA_EGLFS_KMS_CONFIG="$FORCED_CONFIG"
export QT_QPA_EGLFS_ALWAYS_SET_MODE=1
export QT_QPA_EGLFS_SWAPINTERVAL=1

echo "WARNING: forcing EDID-unsupported HDMI mode 1280x720@80.000"
echo "The display may report out-of-range or lose signal."
echo "Press Ctrl+C over SSH/serial, then run ./run_camera.sh to restore 60 Hz."

exec "$APP_DIR/run_camera.sh" "$@"
