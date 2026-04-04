#!/usr/bin/env bash
# DECXIN OV9281: формат, FPS и свет через v4l2-ctl (оба «первых» узла стерео).
# Левая линза по умолчанию /dev/video2, правая /dev/video0 (как ov9281_stereo_record.launch).
#
#   ./uvc_ov9281_modes.sh max-slam          # 1280x800 MJPEG 120 + автоэкспозиция (для live SLAM)
#   ./uvc_ov9281_modes.sh calib-lock        # как lock_uvc: ручная экспозиция (калибровка Kalibr)
#   ./uvc_ov9281_modes.sh auto-light-only   # только auto_exposure=3 + AWB, формат не трогать
#
# Свои устройства: UVC_LEFT=/dev/video2 UVC_RIGHT=/dev/video0 ./uvc_ov9281_modes.sh max-slam
set -euo pipefail

SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
UVC_LEFT="${UVC_LEFT:-/dev/video2}"
UVC_RIGHT="${UVC_RIGHT:-/dev/video0}"
MODE="${1:-}"

usage() {
  sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

both() {
  for d in "$UVC_LEFT" "$UVC_RIGHT"; do
    if [[ ! -e "$d" ]]; then
      echo "Пропуск (нет устройства): $d" >&2
      continue
    fi
    v4l2-ctl -d "$d" "$@"
  done
}

case "$MODE" in
  max-mjpeg-120)
    echo "Установка MJPEG 1280x800 @ 120 FPS на $UVC_LEFT и $UVC_RIGHT"
    both --set-fmt-video=width=1280,height=800,pixelformat=MJPG
    both --set-parm=120
    ;;
  auto-light)
    echo "auto_exposure=Aperture Priority (3), AWB=on"
    both -c auto_exposure=3
    both -c white_balance_automatic=1
    ;;
  max-slam)
    "$SCRIPT" max-mjpeg-120
    "$SCRIPT" auto-light
    echo "Готово. Запуск ORB: CAPTURE_W=1280 CAPTURE_H=800 WIDTH=1280 HEIGHT=800 FPS=120 bash .../run_live_wsl.sh"
    ;;
  calib-800-mjpeg-20)
    echo "800x600 MJPEG 20 (как типичный bag для Kalibr)"
    both v4l2-ctl --set-fmt-video=width=800,height=600,pixelformat=MJPG
    both v4l2-ctl --set-parm=20
    ;;
  calib-lock)
    echo "Ручная экспозиция (как lock_uvc.sh) — для SLAM обычно темнее; для калибровки ок."
    EX="${UVC_EXPOSURE_ABS:-156}"
    GA="${UVC_GAIN:-100}"
    WB="${UVC_WB_TEMP:-4600}"
    for d in "$UVC_LEFT" "$UVC_RIGHT"; do
      [[ -e "$d" ]] || continue
      v4l2-ctl -d "$d" -c white_balance_automatic=0 2>/dev/null || true
      v4l2-ctl -d "$d" -c auto_exposure=1 2>/dev/null || true
      v4l2-ctl -d "$d" -c exposure_time_absolute="$EX" 2>/dev/null || true
      v4l2-ctl -d "$d" -c gain="$GA" 2>/dev/null || true
      v4l2-ctl -d "$d" -c white_balance_temperature="$WB" 2>/dev/null || true
    done
    ;;
  status)
    for d in "$UVC_LEFT" "$UVC_RIGHT"; do
      echo "=== $d ==="
      [[ -e "$d" ]] || { echo "нет"; continue; }
      v4l2-ctl -d "$d" --get-fmt-video 2>/dev/null || true
      v4l2-ctl -d "$d" --get-parm 2>/dev/null || true
      v4l2-ctl -d "$d" -l 2>/dev/null | grep -E 'auto_exposure|exposure_time|gain|white_balance' || true
    done
    ;;
  help | -h | --help | "")
    usage
    ;;
  *)
    echo "Неизвестный режим: $MODE" >&2
    usage >&2
    exit 1
    ;;
esac
