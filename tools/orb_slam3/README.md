# ORB-SLAM3 Live Stereo

Инструменты для запуска `ORB-SLAM3` на прямых потоках `cam0/cam1` после Kalibr-калибровки и внешней ректификации.

Общая схема репозитория (Kalibr, данные, вспомогательные tools): см. **`STEREO_VISION_PIPELINE.md`** в корне проекта.

Что здесь:

- `export_orbslam3_config.py` — собирает rectified `ORB-SLAM3` settings YAML из `*-camchain.yaml`.
- `live_stereo_rectified.cpp` — runner: V4L2 → ректификация → grayscale → `TrackStereo(left_rect, right_rect, timestamp)`.
- `run_live_wsl.sh` — экспорт YAML, cmake, запуск бинарника при заданных `ORB_SLAM3_ROOT` и `ORB_SLAM3_VOCAB`.
- `uvc_ov9281_modes.sh` — `v4l2-ctl`: режим **max-slam** (MJPEG 1280×800 @ 120, автоэкспозиция), **calib-lock** / **calib-800-mjpeg-20** под запись для Kalibr. Переменные **`UVC_LEFT` / `UVC_RIGHT`** (по умолчанию `/dev/video2` и `/dev/video0`).

Ожидаемый порядок камер:

- `cam0` = левая
- `cam1` = правая

Разрешение и частота:

- По умолчанию **как в Kalibr camchain** (обычно **800×600**) и **20** FPS — без масштабирования intrinsics. Другой размер — с **новой калибровкой** или осознанно через **`CAPTURE_W` / `CAPTURE_H`**. Максимальный MJPEG-режим камер перед запуском: **`uvc_ov9281_modes.sh max-slam`**, затем например `CAPTURE_W=1280 CAPTURE_H=800 WIDTH=1280 HEIGHT=800 FPS=120` (см. комментарии в `run_live_wsl.sh`).

Опционально (по умолчанию выкл. — иначе на MJPEG часто видны **блочность и «квадраты»**): `ORB_LIVE_AUTO_LEVELS=1`, `ORB_LIVE_UVC_AUTO=1`. Яркость в железе — `lock_uvc.sh` / `v4l2-ctl`.

Устройства V4L2 (две камеры в `v4l2-ctl --list-devices`): для **двух линз** — **первые** узлы разных строк (у DECXIN индексы **`2` и `0`**, как в `ov9281_stereo_record.launch`). Узлы `1` и `3` — второй интерфейс той же камеры.

`run_live_wsl.sh` по умолчанию передаёт **`ORB_LIVE_LEFT=2`**, **`ORB_LIVE_RIGHT=0`** и **не** читает `DEVICE_LEFT`/`DEVICE_RIGHT` (в сессии WSL часто остаётся ошибочный `export DEVICE_RIGHT=/dev/video1`). Свои устройства: `export ORB_LIVE_LEFT=... ORB_LIVE_RIGHT=...`.

Типичный запуск в WSL:

```bash
export ORB_SLAM3_ROOT=~/ORB_SLAM3
export ORB_SLAM3_VOCAB=~/ORB_SLAM3/Vocabulary/ORBvoc.txt
bash "/mnt/e/Developer/TregorVR 3.0/tools/orb_slam3/run_live_wsl.sh"
```

Если нужно только пересобрать YAML:

```bash
python3 "/mnt/e/Developer/TregorVR 3.0/tools/orb_slam3/export_orbslam3_config.py" \
  --camchain "/mnt/e/Developer/TregorVR 3.0/kalibr_data/my_calib_75mm-camchain.yaml"
```

Что логируется в live runner:

- tracking state (`OK`, `LOST`, ...)
- признак `pose_valid`
- текущая `t_cw` при валидном трекинге

Для первого запуска держи сцену текстурной и двигай систему медленно: короткие сдвиги вперёд-назад, влево-вправо и маленькие повороты.
