#!/usr/bin/env bash
# Фиксированные настройки UVC для калибровки: без автоэкспозиции/АББ (DECXIN OV9281).
# Переменные окружения: UVC_EXPOSURE_ABS (1–10000), UVC_GAIN (16–255), UVC_WB_TEMP (2800–6500).
set -eo pipefail
DEV="${1:?usage: lock_uvc.sh /dev/videoN}"
[[ -e "$DEV" ]] || exit 0

EXPOS="${UVC_EXPOSURE_ABS:-156}"
GAIN="${UVC_GAIN:-100}"
WB="${UVC_WB_TEMP:-4600}"

v4l2-ctl -d "$DEV" -c white_balance_automatic=0 2>/dev/null || true
v4l2-ctl -d "$DEV" -c auto_exposure=1 2>/dev/null || true
v4l2-ctl -d "$DEV" -c exposure_time_absolute="$EXPOS" 2>/dev/null || true
v4l2-ctl -d "$DEV" -c gain="$GAIN" 2>/dev/null || true
v4l2-ctl -d "$DEV" -c white_balance_temperature="$WB" 2>/dev/null || true
v4l2-ctl -d "$DEV" -c backlight_compensation=0 2>/dev/null || true
