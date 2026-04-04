#!/usr/bin/env bash
# Args: sleep_sec, /dev/videoN, затем usb_cam_node (__name:=cam1 ...).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sleep "${1:?sleep sec}"
DEV="$2"
shift 2
if [[ "${UVC_SKIP_LOCK:-0}" != "1" ]]; then
  "$HERE/lock_uvc.sh" "$DEV"
fi
exec /opt/ros/noetic/lib/usb_cam/usb_cam_node "$@"
