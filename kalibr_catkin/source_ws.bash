#!/usr/bin/env bash
# Подключение воркспейса kalibr_catkin в текущую оболочку (WSL/Linux).
# Использование из каталога kalibr_catkin:
#   source ./source_ws.bash
#
# Зачем: roslaunch ищет пакеты в ROS_PACKAGE_PATH. Без сборки/без source devel
# пакет tregor_kalibr не находится — мы добавляем .../kalibr_catkin/src в начало пути.
# Если есть devel/setup.bash (после catkin_make), он тоже подключается.
#
# catkin_make: НЕ ставьте пакет apt «catkin» — конфликт с python3-catkin-pkg.
# Для Noetic: sudo apt install ros-noetic-catkin
# Затем: cd kalibr_catkin && source /opt/ros/noetic/setup.bash && catkin_make

set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
elif [[ -f /opt/ros/melodic/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/melodic/setup.bash
else
  echo "warn: не найден /opt/ros/noetic|melodic/setup.bash — roscore/roslaunch могут не работать" >&2
fi

export ROS_PACKAGE_PATH="${WS}/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"

if [[ -f "${WS}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS}/devel/setup.bash"
else
  echo "warn: нет ${WS}/devel/setup.bash — выполните один раз: source /opt/ros/noetic/setup.bash && cd \"${WS}\" && catkin_make" >&2
  echo "    (без devel roslaunch найдёт launch в src, но узлы вроде stereo_rectify_rtabmap_bridge.py могут не стартовать)" >&2
fi

if command -v rospack >/dev/null 2>&1; then
  echo "tregor_kalibr -> $(rospack find tregor_kalibr 2>/dev/null || echo 'не найден (проверьте src/tregor_kalibr/package.xml)')"
else
  echo "rospack не в PATH — выполните: source /opt/ros/noetic/setup.bash" >&2
fi
