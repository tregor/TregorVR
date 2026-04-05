# Стерео: Kalibr → ORB-SLAM3 (текущее состояние)

Краткая схема того, как устроен конвейер в **TregorVR 3.0** после доведения live SLAM до рабочего состояния.

## Компоненты

| Часть | Назначение |
|--------|------------|
| **Камеры** | DECXIN OV9281 (две линзы по USB/V4L2). Для захвата используются **первые** узлы пары: по умолчанию **левая `/dev/video2` (cam0)**, **правая `/dev/video0` (cam1)** — как в `ov9281_stereo_record.launch`. Узлы `video1` / `video3` — второй интерфейс тех же устройств. |
| **`kalibr_catkin/`** | Catkin-пакет `tregor_kalibr`, launch `ov9281_stereo_record.launch`, скрипты `lock_uvc.sh`, `run_usb_cam0_locked.sh`, `run_usb_cam_delayed.sh`. Скрипт **`kalibr_pipeline.sh`** — запись bag, калибровка в Docker, вспомогательные команды. |
| **`kalibr_data/`** | Результаты калибровки **75 mm baseline**: исходный `my_calib_75mm-camchain.yaml`, текст отчёта `my_calib_75mm-results-cam.txt`, сгенерированные под ORB YAML ректификации и настроек. |
| **`tools/orb_slam3/`** | Экспорт конфигов из camchain (`export_orbslam3_config.py`), бинарник **`orbslam3_stereo_live`** (`live_stereo_rectified.cpp`: V4L2 → ректификация OpenCV → grayscale → `TrackStereo`), **`run_live_wsl.sh`**, **`uvc_ov9281_modes.sh`** (режимы v4l2 для OV9281). |
| **`tools/chessboard_second_screen/`** | Шахматная доска на втором мониторе (шаг клетки 75 mm под шаблон Kalibr). |
| **`tools/stereo_rectify_preview/`** | Оффлайн/превью ректификации по той же геометрии, что и в Kalibr. |

## Поток: калибровка

1. Цель и доска: `kalibr_catkin/chessboard_template.yaml` (согласовано с chessboard_second_screen).
2. Запись: `./kalibr_pipeline.sh record` (или `record-once`) при запущенном `roslaunch` из pipeline; по умолчанию **MJPEG 800×600 @ 20 FPS**, **`uvc_lock:=true`** — фиксированная экспозиция через `lock_uvc.sh` (удобно для стабильных кадров в bag).
3. Калибровка: `./kalibr_pipeline.sh calibrate /path/to.bag` → в выходной области появляется `*-camchain.yaml` и отчёты; актуальная копия для проекта — **`kalibr_data/my_calib_75mm-camchain.yaml`**.

## Поток: live ORB-SLAM3 (WSL/Linux)

1. Собранный **ORB-SLAM3** и словарь: переменные окружения **`ORB_SLAM3_ROOT`**, **`ORB_SLAM3_VOCAB`**.
2. Генерация YAML: `run_live_wsl.sh` вызывает `export_orbslam3_config.py` → обновляются **`kalibr_data/my_calib_75mm-orbslam3-stereo-rectified.yaml`** (настройки SLAM под **rectified stereo**) и **`my_calib_75mm-orbslam3-rectify-input.yaml`** (карты ректификации для runner’а).
3. Запуск: `bash tools/orb_slam3/run_live_wsl.sh` с при необходимости **`ORB_LIVE_LEFT` / `ORB_LIVE_RIGHT`**, **`WIDTH`/`HEIGHT`/`FPS`**, **`CAPTURE_W`/`CAPTURE_H`** (масштаб intrinsics относительно camchain — только осознанно или после новой калибровки).
4. Свет и режим камеры для SLAM (не путать с lock для Kalibr): **`bash tools/orb_slam3/uvc_ov9281_modes.sh max-slam`** — MJPEG 1280×800 @ 120 FPS (если железо тянет), автоэкспозиция и AWB. Для записи под Kalibr можно вернуть **`calib-lock`** / **`calib-800-mjpeg-20`** из того же скрипта.
5. Постобработка кадра по умолчанию **выключена** (`ORB_LIVE_AUTO_LEVELS=0`, `ORB_LIVE_UVC_AUTO=0`), чтобы не усиливать блочность MJPEG.

## Зависимости снаружи репозитория

- **ROS Noetic** + **usb_cam** — для Kalibr-записи.
- **Checkout ORB-SLAM3** — библиотека и заголовки для сборки `orbslam3_stereo_live` (через `ORB_SLAM3_ROOT` в CMake).
- **OpenCV**, **Pangolin**, **Eigen**, **v4l2-utils** (`v4l2-ctl`) — как требует ваша среда сборки ORB-SLAM3 и UVC.

## Файлы данных (зафиксированные имена по умолчанию)

- `kalibr_data/my_calib_75mm-camchain.yaml` — источник истины по внутренним и стерео-параметрам после Kalibr.
- `kalibr_data/my_calib_75mm-orbslam3-stereo-rectified.yaml` — то, что читает ORB-SLAM3 в режиме rectified stereo.
- `kalibr_data/my_calib_75mm-orbslam3-rectify-input.yaml` — R/L карты и метаданные для `live_stereo_rectified.cpp`.

При смене разрешения или камеры: заново **record + calibrate**, затем снова **`run_live_wsl.sh`** (или только `export_orbslam3_config.py` с тем же camchain, если разрешение совпадает).

## RTAB-Map (вариант B — уже ректифицированное стерео)

Тот же пайплин ректификации, что у **ORB live** (`*-orbslam3-rectify-input.yaml`, `fisheye::stereoRectify` / `stereoRectify`), публикуется в ROS для **RTAB-Map**:

| Топик | Содержимое |
|--------|------------|
| `/stereo/left/image_rect`, `/stereo/right/image_rect` | `mono8` (по умолчанию) |
| `/stereo/left/camera_info`, `/stereo/right/camera_info` | `P` из OpenCV, **D = 0**, **R = I** (как для готовых ректифицированных кадров) |

Узел: **`stereo_rectify_rtabmap_bridge.py`** (`tregor_kalibr`). Статический TF **левый → правый** оптический кадр: сдвиг **`+baseline_m`** по оси X (можно выключить `~publish_tf:=false`).

**Запуск только моста** (камеры + ректификация):

```bash
source /opt/ros/noetic/setup.bash
source ~/…/kalibr_catkin/devel/setup.bash
roslaunch tregor_kalibr stereo_rectify_bridge.launch
```

Опции: `start_cameras:=false` если `usb_cam` уже крутится; **`rectify_yaml:=/полный/путь/...-rectify-input.yaml`** если `$(find tregor_kalibr)/../../../kalibr_data/...` не находится (другой layout ws).

**Вместе с RTAB-Map** (нужен `sudo apt install ros-noetic-rtabmap-ros`):

```bash
cd /path/to/TregorVR\ 3.0/kalibr_catkin
source ./source_ws.bash
roslaunch tregor_kalibr rtabmap_ov9281_rectified.launch
```

**Если `roslaunch` пишет, что launch не в пакете:** вы не подключили воркспейс. Сделайте **`source ./source_ws.bash`** из **`kalibr_catkin`** (скрипт добавляет **`src`** в **`ROS_PACKAGE_PATH`** и при наличии — **`devel/setup.bash`**).

**Узлы Python** (`stereo_rectify_rtabmap_bridge.py` и т.д.) попадают в `PATH` только после **`catkin_make`** (появится **`devel/`**). Полный цикл один раз:

```bash
sudo apt install ros-noetic-catkin
source /opt/ros/noetic/setup.bash
cd "/path/to/TregorVR 3.0/kalibr_catkin"
catkin_make
source ./source_ws.bash
```

**Сборка catkin:** не ставьте пакет apt **`catkin`** (конфликт с **`python3-catkin-pkg`**). Для Noetic: **`sudo apt install ros-noetic-catkin`**, затем **`source /opt/ros/noetic/setup.bash`** — появится **`catkin_make`**.

**Запуск launch по абсолютному пути** (если пакет всё ещё не виден):

```bash
roslaunch /mnt/e/Developer/TregorVR\ 3.0/kalibr_catkin/src/tregor_kalibr/launch/rtabmap_ov9281_rectified.launch
```

Если версия `rtabmap_ros` ожидает другие имена топиков — смотрите подписки `rosnode info /rtabmap` и поправьте `<remap>` в **`rtabmap_ov9281_rectified.launch`** или запускайте только **`stereo_rectify_bridge.launch`** и стартуйте `rtabmap` вручную.

После смены калибровки снова сгенерируйте **`export_orbslam3_config.py`** (или скопируйте новый `*-orbslam3-rectify-input.yaml`) и укажите его в **`rectify_yaml`**.

## Версионирование (Git)

Ориентир для `.gitignore`: **в индексе держим** `kalibr_data/` (в т.ч. `*.yaml`, `*-results-cam.txt`), `kalibr_catkin/` за исключением клона **`kalibr-upstream/`** и каталогов сборки Catkin (`build/`, `devel/`, `install/`, `logs/`), исходники **`tools/`**, `main.md`, этот файл. **Не коммитим** апстрим Kalibr (подтягивается `kalibr_pipeline.sh build-docker`), бинарные артефакты сборки, **`*.bag`**, локальные выгрузки **`kalibr_debug/`** и **`kalibr_out/`** в корне репозитория, виртуальные окружения (`.venv/` и т.п.). Подробности и снятие с индекса уже отслеживаемых путей — в шапке **`.gitignore`**.
