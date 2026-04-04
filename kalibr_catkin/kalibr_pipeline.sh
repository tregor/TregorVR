#!/usr/bin/env bash
# Единый пайплайн: стерео OV9281 → bag → Kalibr (Docker).
# Рабочая область: каталог этого скрипта = catkin ws (kalibr_catkin).
# Данные по умолчанию: ../kalibr_data, ../kalibr_debug, ../kalibr_out (рядом с kalibr_catkin).
#
#   ./kalibr_pipeline.sh help
#   ./kalibr_pipeline.sh build-docker
#   ./kalibr_pipeline.sh cameras
#   ./kalibr_pipeline.sh record [имя | путь.bag]
#   ./kalibr_pipeline.sh record-once [секунды]
#   ./kalibr_pipeline.sh calibrate /path/to.bag
#   ./kalibr_pipeline.sh probe bag.bag [/cam0/image_raw]
#   ./kalibr_pipeline.sh shell
#   ./kalibr_pipeline.sh check
#   ./kalibr_pipeline.sh peek
#
# Переменные: KALIBR_BAG_DIR, KALIBR_DEBUG_DIR, KALIBR_OUT, KALIBR_IMAGE, KALIBR_TARGET,
# MODELS, DURATION_SEC, USE_LZ4, MPLBACKEND, KALIBR_EXTRA_ARGS, KALIBR_DOCKER_GUI
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$WS/.." && pwd)"
export KALIBR_BAG_DIR="${KALIBR_BAG_DIR:-$PROJECT_ROOT/kalibr_data}"
export KALIBR_DEBUG_DIR="${KALIBR_DEBUG_DIR:-$PROJECT_ROOT/kalibr_debug}"
KALIBR_OUT="${KALIBR_OUT:-$PROJECT_ROOT/kalibr_out}"
IMAGE="${KALIBR_IMAGE:-kalibr:noetic}"
TARGET="${KALIBR_TARGET:-$WS/chessboard_template.yaml}"
MODELS="${MODELS:-pinhole-equi pinhole-equi}"

source_ros() {
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
  # shellcheck source=/dev/null
  source "$WS/devel/setup.bash"
}

cmd_help() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

cmd_build_docker() {
  local src="$WS/kalibr-upstream"
  if [[ ! -f "$src/Dockerfile_ros1_20_04" ]]; then
    echo "Cloning Kalibr..."
    rm -rf "$src"
    git clone --depth 1 https://github.com/ethz-asl/kalibr.git "$src"
  fi
  echo "Building $IMAGE (долго)..."
  docker build -t "$IMAGE" -f "$src/Dockerfile_ros1_20_04" "$src"
  echo "Done: $IMAGE"
}

cmd_cameras() {
  source_ros
  mkdir -p "$KALIBR_DEBUG_DIR"
  exec roslaunch tregor_kalibr ov9281_stereo_record.launch "debug_dir:=$KALIBR_DEBUG_DIR" "$@"
}

cmd_record() {
  source_ros
  mkdir -p "$KALIBR_BAG_DIR"
  local raw="${1:-kalibr_stereo_$(date +%Y%m%d_%H%M%S)}"
  local bag_file
  if [[ "$raw" == *.bag ]]; then
    bag_file="$raw"
  else
    bag_file="${KALIBR_BAG_DIR}/${raw%.bag}.bag"
  fi
  mkdir -p "$(dirname "$bag_file")"
  echo "Пишем: $bag_file (в другом терминале должен работать: $0 cameras)"
  echo "Ctrl+C — стоп. DURATION_SEC / USE_LZ4 — см. help в начале файла."
  local extra=()
  [[ -n "${DURATION_SEC:-}" ]] && extra+=(--duration="$DURATION_SEC")
  [[ -n "${USE_LZ4:-}" ]] && extra+=(--lz4)
  exec rosbag record "${extra[@]}" -O "$bag_file" \
    /cam0/image_raw /cam1/image_raw /cam0/camera_info /cam1/camera_info
}

cmd_record_once() {
  source_ros
  local duration="${1:-15}"
  mkdir -p "$KALIBR_BAG_DIR" "$KALIBR_DEBUG_DIR"
  local bag="${KALIBR_BAG_DIR}/kalibr_stereo_$(date +%Y%m%d_%H%M%S).bag"
  echo "Камеры + превью JPEG → $KALIBR_DEBUG_DIR"
  roslaunch tregor_kalibr ov9281_stereo_record.launch "debug_dir:=$KALIBR_DEBUG_DIR" &
  local lpid=$!
  cleanup() { kill "$lpid" 2>/dev/null || true; wait "$lpid" 2>/dev/null || true; }
  trap cleanup EXIT
  sleep 6
  echo "Запись ${duration}s → $bag"
  rosbag record -O "$bag" /cam0/image_raw /cam1/image_raw /cam0/camera_info /cam1/camera_info \
    --duration="$duration"
  echo "Готово: $bag"
}

cmd_calibrate() {
  local bag="${1:?$0 calibrate /path/to/file.bag}"
  [[ -f "$bag" ]] || { echo "Нет файла: $bag" >&2; exit 1; }
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Образ $IMAGE не найден. Сначала: $0 build-docker" >&2
    exit 1
  fi
  local bag_dir bag_file target_dir target_file out_dir
  bag_dir="$(cd "$(dirname "$bag")" && pwd)"
  bag_file="$(basename "$bag")"
  target_dir="$(cd "$(dirname "$TARGET")" && pwd)"
  target_file="$(basename "$TARGET")"
  mkdir -p "$KALIBR_OUT"
  out_dir="$(cd "$KALIBR_OUT" && pwd)"

  echo "IMAGE=$IMAGE BAG=$bag_dir/$bag_file TARGET=$target_dir/$target_file OUT=$out_dir MODELS=$MODELS"

  export MPLBACKEND="${MPLBACKEND:-Agg}"
  local extra_args="${KALIBR_EXTRA_ARGS:-}"
  local docker_gui=()
  if [[ -n "${KALIBR_DOCKER_GUI:-}" ]]; then
    docker_gui+=(-e "DISPLAY=${DISPLAY:-:0}")
    if [[ -d /mnt/wslg/.X11-unix ]]; then
      docker_gui+=(-v /mnt/wslg/.X11-unix:/tmp/.X11-unix:ro)
    elif [[ -d /tmp/.X11-unix ]]; then
      docker_gui+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro)
    fi
  fi

  docker run --rm -it \
    -e MPLBACKEND \
    -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
    "${docker_gui[@]}" \
    -v "${bag_dir}:/data:rw" \
    -v "${target_dir}:/target:ro" \
    -v "${out_dir}:/out" \
    -w /out \
    --entrypoint /bin/bash \
    "$IMAGE" \
    -c "source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras \
        --bag /data/${bag_file} \
        --topics /cam0/image_raw /cam1/image_raw \
        --models ${MODELS} \
        --target /target/${target_file} \
        ${extra_args}"

  local stem="${bag_file%.bag}"
  echo "Артефакты Kalibr (рядом с bag):"
  echo "  $bag_dir/${stem}-camchain.yaml"
  echo "  $bag_dir/${stem}-report-cam.pdf"
  echo "  $bag_dir/${stem}-results-cam.txt"
}

cmd_probe() {
  [[ -n "${1:-}" ]] || {
    echo "usage: $0 probe bag.bag [/cam0/image_raw] [probe.py args...]" >&2
    exit 1
  }
  if [[ $# -eq 1 ]]; then
    exec python3 "$WS/probe_chessboard_bag.py" "$1" /cam0/image_raw
  fi
  exec python3 "$WS/probe_chessboard_bag.py" "$@"
}

cmd_shell() {
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Образ $IMAGE не найден. Сначала: $0 build-docker" >&2
    exit 1
  fi
  mkdir -p "$KALIBR_BAG_DIR" "$KALIBR_OUT"
  local bag_dir target_dir out_dir x=()
  bag_dir="$(cd "$KALIBR_BAG_DIR" && pwd)"
  target_dir="$(cd "$WS" && pwd)"
  out_dir="$(cd "$KALIBR_OUT" && pwd)"
  if [[ -d /mnt/wslg/.X11-unix ]]; then
    x+=(-v /mnt/wslg/.X11-unix:/tmp/.X11-unix)
  elif [[ -d /tmp/.X11-unix ]]; then
    x+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
  fi
  # :rw на /data — kalibr пишет yaml/pdf рядом с .bag
  exec docker run --rm -it \
    -e "DISPLAY=${DISPLAY:-:0}" \
    "${x[@]}" \
    -v "${bag_dir}:/data:rw" \
    -v "${target_dir}:/target:ro" \
    -v "${out_dir}:/out" \
    -w /out \
    --entrypoint /bin/bash \
    "$IMAGE" \
    -c 'source /catkin_ws/devel/setup.bash && exec bash --norc'
}

cmd_check() {
  source_ros
  local d
  d="$(mktemp -d "${TMPDIR:-/tmp}/kalibr_smoke.XXXXXX")"
  roslaunch tregor_kalibr ov9281_stereo_record.launch "debug_dir:=$d" >/dev/null 2>&1 &
  local lpid=$!
  sleep 10
  rostopic list | grep -E 'cam0|cam1' || true
  echo "--- hz /cam0/image_raw ---"
  timeout 5 rostopic hz /cam0/image_raw || true
  kill "$lpid" 2>/dev/null || true
  wait "$lpid" 2>/dev/null || true
  rm -rf "$d"
}

cmd_peek() {
  source_ros
  local d
  d="$(mktemp -d "${TMPDIR:-/tmp}/kalibr_peek.XXXXXX")"
  roslaunch tregor_kalibr ov9281_stereo_record.launch "debug_dir:=$d" >/dev/null 2>&1 &
  local lpid=$!
  sleep 12
  timeout 8 rostopic echo /cam0/image_raw -n1 | head -c 200 || true
  echo
  kill "$lpid" 2>/dev/null || true
  wait "$lpid" 2>/dev/null || true
  rm -rf "$d"
}

main() {
  case "${1:-help}" in
    help | --help | -h) cmd_help ;;
    build-docker) cmd_build_docker ;;
    cameras)
      shift
      cmd_cameras "$@"
      ;;
    record)
      shift
      cmd_record "${1:-}"
      ;;
    record-once)
      shift
      cmd_record_once "${1:-}"
      ;;
    calibrate)
      shift
      [[ -n "${1:-}" ]] || {
        echo "usage: $0 calibrate /path/to/file.bag" >&2
        exit 1
      }
      cmd_calibrate "$1"
      ;;
    probe)
      shift
      cmd_probe "$@"
      ;;
    shell) cmd_shell ;;
    check) cmd_check ;;
    peek) cmd_peek ;;
    *)
      echo "Неизвестная команда: $1" >&2
      cmd_help >&2
      exit 1
      ;;
  esac
}

main "$@"
