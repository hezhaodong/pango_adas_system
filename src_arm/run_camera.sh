#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$APP_DIR"

# EGLFS/KMS must be the sole scanout owner.  If Xorg remains active it can
# regain DRM master while HDMI input is idle, replace the camera framebuffer
# with the desktop, and leave Qt incorrectly reporting visible/exposed=true.
# Remember exactly what this launcher changed and restore it on exit.
DISPLAY_MANAGER_STOPPED=0
XORG_PAUSED=0
APP_PID=""
SHUTDOWN_REQUESTED=0
STOP_IN_PROGRESS=0
restore_desktop()
{
    if [ "$XORG_PAUSED" -eq 1 ]; then
        pkill -CONT Xorg 2>/dev/null || true
    fi
    if [ "$DISPLAY_MANAGER_STOPPED" -eq 1 ]; then
        systemctl start display-manager 2>/dev/null || true
    fi
}
stop_camera_child()
{
    [ -n "$APP_PID" ] || return 0
    kill -0 "$APP_PID" 2>/dev/null || return 0
    [ "$STOP_IN_PROGRESS" -eq 0 ] || return 0
    STOP_IN_PROGRESS=1
    kill -TERM "$APP_PID" 2>/dev/null || true
    echo "[camera-shutdown] forwarding TERM to pid=$APP_PID" >&2 || true
    while kill -0 "$APP_PID" 2>/dev/null; do
        CHILD_STATE=$(awk '{print $3}' "/proc/$APP_PID/stat" 2>/dev/null || true)
        [ "$CHILD_STATE" = "Z" ] && break
        sleep 0.1
    done
    STOP_IN_PROGRESS=0
}

launcher_signal()
{
    SHUTDOWN_REQUESTED=1
    stop_camera_child
}

launcher_cleanup()
{
    stop_camera_child
    restore_desktop
}

trap launcher_cleanup EXIT
trap launcher_signal INT TERM HUP
trap '' PIPE

if command -v systemctl >/dev/null 2>&1 \
    && systemctl is-active --quiet display-manager; then
    echo "[drm-owner] stopping display-manager for exclusive EGLFS/KMS" >&2
    systemctl stop display-manager
    DISPLAY_MANAGER_STOPPED=1
elif pgrep -x Xorg >/dev/null 2>&1; then
    echo "[drm-owner] pausing unmanaged Xorg for exclusive EGLFS/KMS" >&2
    pkill -STOP Xorg
    XORG_PAUSED=1
fi

# Allow Xorg's final atomic commit and DRM-master release to complete before
# Qt opens card0 and creates its GBM surface.
sleep 1
if pgrep -x Xorg >/dev/null 2>&1 && [ "$XORG_PAUSED" -eq 0 ]; then
    echo "ERROR: Xorg is still active; refusing shared DRM ownership." >&2
    exit 26
fi

RUNTIME_DIR="/tmp/camera-runtime-$(id -u)"
mkdir -p "$RUNTIME_DIR"
chmod 700 "$RUNTIME_DIR"
export XDG_RUNTIME_DIR="$RUNTIME_DIR"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-eglfs}"
export QT_QPA_EGLFS_INTEGRATION="${QT_QPA_EGLFS_INTEGRATION:-eglfs_kms}"
export QT_QPA_EGLFS_HIDECURSOR="${QT_QPA_EGLFS_HIDECURSOR:-1}"

# Board-proven display configuration.  Keep an explicitly supplied valid
# override, otherwise use /userdata/kms.json exactly like the original video
# chain.  Do not force the temporary HDMI-only 1080p test configuration.
if [ -z "${QT_QPA_EGLFS_KMS_CONFIG:-}" ] && [ -f /userdata/kms.json ]; then
    export QT_QPA_EGLFS_KMS_CONFIG=/userdata/kms.json
fi

# Ignore a stale board-image setting such as /userdata/kms.json. EGLFS will
# auto-detect connectors unless the caller supplies an existing config file.
if [ -n "${QT_QPA_EGLFS_KMS_CONFIG:-}" ] && [ ! -f "$QT_QPA_EGLFS_KMS_CONFIG" ]; then
    unset QT_QPA_EGLFS_KMS_CONFIG
fi

# Avoid two processes controlling the same PCIe DMA engine. The lock is
# automatically released even if the application crashes.
if command -v flock >/dev/null 2>&1; then
    exec 9>/tmp/camera_capture_v2.lock
    if ! flock -n 9; then
        echo "camera_capture_v2 is already running" >&2
        exit 1
    fi
fi

for PROC_DIR in /proc/[0-9]*; do
    [ -r "$PROC_DIR/exe" ] || continue
    PROC_EXE=$(readlink "$PROC_DIR/exe" 2>/dev/null || true)
    case "$PROC_EXE" in
        "$APP_DIR/camera_capture_v2"|"$APP_DIR/camera_capture_v2 (deleted)")
            EXISTING_PID=${PROC_DIR##*/}
            echo "ERROR: camera_capture_v2 already owns the pipeline (pid=$EXISTING_PID)." >&2
            exit 1
            ;;
    esac
done

WIFI_HOST=${CAMERA_WIFI_HOST:-192.168.250.119}
WIFI_PORT=${CAMERA_WIFI_PORT:-8080}
EXIT_LOG=${CAMERA_EXIT_LOG:-/tmp/camera_exit.log}

# Preserve a core for fatal signals.  core_pattern decides its final path;
# this launcher separately records exit status and the kernel tail.
ulimit -c unlimited 2>/dev/null || true

echo "[camera-launch] pid=$$ time=$(date '+%Y-%m-%d %H:%M:%S') app=$APP_DIR/camera_capture_v2" >&2
echo "[camera-launch] platform=$QT_QPA_PLATFORM integration=$QT_QPA_EGLFS_INTEGRATION kms=${QT_QPA_EGLFS_KMS_CONFIG:-auto}" >&2

DRM_RESTART_COUNT=0
while :; do
    set +e
    if [ -n "${CAMERA_IPCAM_URL:-}" ]; then
        echo "IP Cam URL supplied through CAMERA_IPCAM_URL"
        if command -v setsid >/dev/null 2>&1; then
            setsid ./camera_capture_v2 -w "$WIFI_HOST" -p "$WIFI_PORT" \
                --ip-camera-url "$CAMERA_IPCAM_URL" "$@" &
        else
            ./camera_capture_v2 -w "$WIFI_HOST" -p "$WIFI_PORT" \
                --ip-camera-url "$CAMERA_IPCAM_URL" "$@" &
        fi
    else
        if command -v setsid >/dev/null 2>&1; then
            setsid ./camera_capture_v2 -w "$WIFI_HOST" -p "$WIFI_PORT" "$@" &
        else
            ./camera_capture_v2 -w "$WIFI_HOST" -p "$WIFI_PORT" "$@" &
        fi
    fi
    APP_PID=$!
    echo "[camera-launch] child_pid=$APP_PID isolated_session=$(command -v setsid >/dev/null 2>&1 && echo yes || echo no)" >&2
    wait "$APP_PID"
    APP_STATUS=$?
    APP_PID=""
    set -e

    if [ "$SHUTDOWN_REQUESTED" -eq 1 ]; then
        break
    fi

    if [ "$APP_STATUS" -eq 25 ] && [ "$DRM_RESTART_COUNT" -lt 3 ]; then
        DRM_RESTART_COUNT=$((DRM_RESTART_COUNT + 1))
        echo "[camera-restart] DRM page-flip failed; restarting Qt/EGLFS ($DRM_RESTART_COUNT/3)" >&2
        # Process exit closes the old DRM fd.  Wait before reopening card0 so
        # the kernel can retire the abandoned flip and release its FB refs.
        sleep 2
        continue
    fi
    break
done

case "$APP_STATUS" in
    0)   EXIT_REASON="clean-exit-or-QCoreApplication-quit" ;;
    21)  EXIT_REASON="DRM_FD_UNAVAILABLE" ;;
    22)  EXIT_REASON="DRM_MODESET_FAILED" ;;
    23)  EXIT_REASON="DRM_MODE_MISMATCH" ;;
    24)  EXIT_REASON="POSTKMS_NOT_LINUX" ;;
    25)  EXIT_REASON="DRM_PAGE_FLIP_RESTART_EXHAUSTED" ;;
    26)  EXIT_REASON="DRM_OWNER_XORG_STILL_ACTIVE" ;;
    132) EXIT_REASON="SIGILL" ;;
    134) EXIT_REASON="SIGABRT" ;;
    135) EXIT_REASON="SIGBUS" ;;
    136) EXIT_REASON="SIGFPE" ;;
    137) EXIT_REASON="SIGKILL-or-OOM" ;;
    139) EXIT_REASON="SIGSEGV" ;;
    143) EXIT_REASON="SIGTERM" ;;
    *)   EXIT_REASON="nonzero-exit" ;;
esac

{
    echo "=== camera exit $(date '+%Y-%m-%d %H:%M:%S') ==="
    echo "status=$APP_STATUS reason=$EXIT_REASON"
    echo "platform=$QT_QPA_PLATFORM integration=$QT_QPA_EGLFS_INTEGRATION kms=${QT_QPA_EGLFS_KMS_CONFIG:-auto}"
    if [ -r /proc/sys/kernel/core_pattern ]; then
        echo "core_pattern=$(cat /proc/sys/kernel/core_pattern)"
    fi
    echo "--- memory ---"
    if [ -r /proc/meminfo ]; then
        head -n 15 /proc/meminfo
    fi
    echo "--- drm connectors ---"
    for status_file in /sys/class/drm/card*-*/status; do
        [ -r "$status_file" ] || continue
        echo "$status_file=$(cat "$status_file")"
    done
    echo "--- kernel tail ---"
    dmesg 2>/dev/null | tail -n 160 || true
    echo "=== end camera exit ==="
} >>"$EXIT_LOG" 2>&1

echo "[camera-exit] status=$APP_STATUS reason=$EXIT_REASON diagnostics=$EXIT_LOG" >&2
exit "$APP_STATUS"
