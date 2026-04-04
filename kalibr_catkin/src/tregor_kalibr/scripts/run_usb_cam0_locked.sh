#!/usr/bin/env bash
# Arg1: /dev/video* — затем аргументы usb_cam_node от roslaunch.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV="$1"
shift
"$HERE/lock_uvc.sh" "$DEV"
exec /opt/ros/noetic/lib/usb_cam/usb_cam_node "$@"
