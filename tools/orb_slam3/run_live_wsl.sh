#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TOOLS_DIR="$ROOT/tools/orb_slam3"

CAMCHAIN="${CAMCHAIN:-$ROOT/kalibr_data/my_calib_75mm-camchain.yaml}"
SETTINGS_YAML="${SETTINGS_YAML:-$ROOT/kalibr_data/my_calib_75mm-orbslam3-stereo-rectified.yaml}"
RECTIFY_YAML="${RECTIFY_YAML:-$ROOT/kalibr_data/my_calib_75mm-orbslam3-rectify-input.yaml}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/orb_slam3_live}"

# Индексы V4L2: при video0,video1,video2,video3 — левая линза cam0 = 2, правая cam1 = 0 (как ov9281_stereo_record.launch).
# Не используем DEVICE_LEFT/DEVICE_RIGHT из окружения: в сессии часто остаётся старый DEVICE_RIGHT=/dev/video1.
# Переопределение: ORB_LIVE_LEFT / ORB_LIVE_RIGHT (число или путь /dev/videoN).
ORB_LIVE_LEFT="${ORB_LIVE_LEFT:-2}"
ORB_LIVE_RIGHT="${ORB_LIVE_RIGHT:-0}"
# Смена USB-хаба / порта часто меняет, какой /dev/video* — «левый» в смысле Kalibr cam0.
# Если сразу после инициализации сыпется «Fail to track local map» — поменяйте линзы:
#   ORB_LIVE_SWAP=1 bash .../run_live_wsl.sh
# или явно: ORB_LIVE_LEFT=0 ORB_LIVE_RIGHT=2 (и наоборот).
ORB_LIVE_SWAP="${ORB_LIVE_SWAP:-0}"
if [[ "$ORB_LIVE_SWAP" == "1" ]]; then
  _orb_swap_tmp="$ORB_LIVE_LEFT"
  ORB_LIVE_LEFT="$ORB_LIVE_RIGHT"
  ORB_LIVE_RIGHT="$_orb_swap_tmp"
fi
# Разрешение как в Kalibr camchain (800x600). Макс. MJPEG на OV9281 (перед запуском):
#   bash tools/orb_slam3/uvc_ov9281_modes.sh max-slam
#   CAPTURE_W=1280 CAPTURE_H=800 WIDTH=1280 HEIGHT=800 FPS=120 bash .../run_live_wsl.sh
# Масштаб с 800x600 — приблизительно; для точности пересоберите camchain в том же режиме.
WIDTH="${WIDTH:-800}"
HEIGHT="${HEIGHT:-600}"
FPS="${FPS:-20}"
FOURCC="${FOURCC:-MJPG}"
SHOW="${SHOW:-1}"
VIEWER="${VIEWER:-1}"
# По умолчанию без обработки (избегает «квадратов» MJPEG + постеризации). Вкл. при необходимости:
ORB_LIVE_AUTO_LEVELS="${ORB_LIVE_AUTO_LEVELS:-0}"
ORB_LIVE_AUTO_TAIL="${ORB_LIVE_AUTO_TAIL:-0.02}"
ORB_LIVE_UVC_AUTO="${ORB_LIVE_UVC_AUTO:-0}"

: "${ORB_SLAM3_ROOT:?Set ORB_SLAM3_ROOT to your ORB-SLAM3 checkout}"
: "${ORB_SLAM3_VOCAB:?Set ORB_SLAM3_VOCAB to ORBvoc.txt}"

echo "Stereo: left=$ORB_LIVE_LEFT right=$ORB_LIVE_RIGHT (ORB_LIVE_LEFT/RIGHT)"
if [[ "${ORB_LIVE_SWAP:-0}" == "1" ]]; then
  echo "  ORB_LIVE_SWAP=1: левая/правая линза обменяны местами для этого запуска."
fi

EXPORT_EXTRA=()
if [[ -n "${CAPTURE_W:-}" && -n "${CAPTURE_H:-}" ]]; then
  EXPORT_EXTRA+=(--capture-width "$CAPTURE_W" --capture-height "$CAPTURE_H")
fi

python3 "$TOOLS_DIR/export_orbslam3_config.py" \
  --camchain "$CAMCHAIN" \
  --output-settings "$SETTINGS_YAML" \
  --output-rectify "$RECTIFY_YAML" \
  --fps "$FPS" \
  "${EXPORT_EXTRA[@]}"

cmake -S "$TOOLS_DIR" -B "$BUILD_DIR" -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"
cmake --build "$BUILD_DIR" -j"$(nproc)"

"$BUILD_DIR/orbslam3_stereo_live" \
  --vocab "$ORB_SLAM3_VOCAB" \
  --settings "$SETTINGS_YAML" \
  --rectify "$RECTIFY_YAML" \
  --device-left "$ORB_LIVE_LEFT" \
  --device-right "$ORB_LIVE_RIGHT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --fourcc "$FOURCC" \
  --auto-levels "$ORB_LIVE_AUTO_LEVELS" \
  --auto-levels-tail "$ORB_LIVE_AUTO_TAIL" \
  --uvc-auto "$ORB_LIVE_UVC_AUTO" \
  --viewer "$VIEWER" \
  --show "$SHOW"
