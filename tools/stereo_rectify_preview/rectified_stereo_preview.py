#!/usr/bin/env python3
"""
Живой тест стерео-ректификации по Kalibr (*-camchain.yaml).

  - Открывает две камеры (V4L2 / индексы), синхронно читает пары кадров.
  - Строит rectification maps через OpenCV (pinhole + equidistant -> cv2.fisheye;
    pinhole + radtan -> cv2.stereoRectify).
  - Показывает склеенное окно (слева | справа) или два окна с горизонтальными линиями.

Ожидание: одна точка сцены на одной строке в обоих видах; вертикали не "плывут" по
высоте; сильная бочка по краям заметно слабее.

Дальше по пайплайну: те же P1/P2 / Q и выпрямленные кадры использовать в ORB-SLAM3
(подавать уже rectified left/right и согласовать intrinsics с новой парой P1,P2 или
отдельным yaml под rectified-поток).

Пример (Linux, как в ov9281_stereo_record.launch):

  pip install -r requirements.txt
  python3 rectified_stereo_preview.py \\
    --camchain /path/to/bag-camchain.yaml \\
    --device0 /dev/video2 --device1 /dev/video0 \\
    --width 800 --height 600

  Как в ROS (вторая камера с задержкой, MJPEG): добавьте
  --second-cam-delay 5 --fourcc mjpeg

  WSL2: select() timeout на /dev/video* часто значит, что USB/V4L в WSL не отдаёт
  кадры — проверьте usbipd и v4l2-ctl, либо запуск с нативного Linux.

Windows: --device0 0 --device1 1
"""
from __future__ import annotations

import argparse
import platform
import struct
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


def _parse_device(s: str):
    if s.isdigit():
        return int(s)
    return s


def load_camchain(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if "cam0" not in data or "cam1" not in data:
        raise ValueError("camchain: ожидаются ключи cam0 и cam1")
    return data


def _intrinsic_matrix(cam: Dict[str, Any]) -> np.ndarray:
    fu, fv, cu, cv = cam["intrinsics"]
    return np.array([[fu, 0.0, cu], [0.0, fv, cv], [0.0, 0.0, 1.0]], dtype=np.float64)


def _dist_vector(cam: Dict[str, Any]) -> np.ndarray:
    coeffs = list(cam["distortion_coeffs"])
    return np.ascontiguousarray(np.array(coeffs, dtype=np.float64).reshape(-1, 1))


def stereo_rectify_from_kalibr(
    chain: Dict[str, Any],
    left_id: str = "cam0",
    right_id: str = "cam1",
    alpha: float = 0.0,
    fisheye_balance: float = 0.0,
) -> Tuple[
    Tuple[int, int],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    str,
]:
    """
    Возвращает: image_size, map1x, map1y, map2x, map2y, P1, P2, Q, R1, R2, mode
    mode in ('fisheye', 'pinhole')
    """
    c0 = chain[left_id]
    c1 = chain[right_id]
    w, h = int(c0["resolution"][0]), int(c0["resolution"][1])
    if c1["resolution"] != [w, h]:
        raise ValueError("Разрешения cam0/cam1 должны совпадать для общей ректификации")

    K0 = np.ascontiguousarray(_intrinsic_matrix(c0))
    K1 = np.ascontiguousarray(_intrinsic_matrix(c1))
    D0 = _dist_vector(c0)
    D1 = _dist_vector(c1)

    if "T_cn_cnm1" not in c1:
        raise ValueError("У cam1 нет T_cn_cnm1 (экстринсики cam0->cam1)")
    T = np.ascontiguousarray(np.asarray(c1["T_cn_cnm1"], dtype=np.float64))
    R = np.ascontiguousarray(T[:3, :3])
    t = np.ascontiguousarray(T[:3, 3:4])

    dmodel0 = c0["distortion_model"]
    dmodel1 = c1["distortion_model"]
    if dmodel0 != dmodel1:
        raise ValueError("Разные distortion_model у пары камер — не поддерживается")

    size = (w, h)

    if dmodel0 == "equidistant":
        flags = cv2.fisheye.CALIB_ZERO_DISPARITY
        r1, r2, p1, p2, q = cv2.fisheye.stereoRectify(
            K0,
            D0,
            K1,
            D1,
            size,
            R,
            t,
            flags=flags,
            balance=fisheye_balance,
            newImageSize=size,
        )
        map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
            K0, D0, r1, p1, size, cv2.CV_16SC2
        )
        map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
            K1, D1, r2, p2, size, cv2.CV_16SC2
        )
        return (w, h), map1x, map1y, map2x, map2y, p1, p2, q, r1, r2, "fisheye"

    if dmodel0 == "radtan":
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            K0,
            D0.ravel(),
            K1,
            D1.ravel(),
            size,
            R,
            t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=alpha,
        )
        map1x, map1y = cv2.initUndistortRectifyMap(K0, D0, r1, p1, size, cv2.CV_16SC2)
        map2x, map2y = cv2.initUndistortRectifyMap(K1, D1, r2, p2, size, cv2.CV_16SC2)
        return (w, h), map1x, map1y, map2x, map2y, p1, p2, q, r1, r2, "pinhole"

    raise ValueError(
        "distortion_model=%r не поддержан (нужны equidistant или radtan)" % dmodel0
    )


def draw_epipolar_grid(
    bgr: np.ndarray, step: int = 40, color: Tuple[int, int, int] = (0, 255, 0)
) -> None:
    h = bgr.shape[0]
    w = bgr.shape[1]
    for y in range(step, h, step):
        cv2.line(bgr, (0, y), (w - 1, y), color, 1, cv2.LINE_AA)


def _running_under_wsl() -> bool:
    try:
        ver = Path("/proc/version").read_text(encoding="utf-8", errors="ignore").lower()
    except OSError:
        return False
    return "microsoft" in ver or "wsl" in ver


def _fourcc_to_str(code: float) -> str:
    if code is None or code < 0:
        return "?"
    try:
        return struct.pack("<I", int(code) & 0xFFFFFFFF).decode("ascii", errors="replace")
    except (struct.error, ValueError):
        return "?"


def _apply_pixel_format(cap: cv2.VideoCapture, pixel_format: str) -> None:
    if pixel_format == "none":
        return
    if pixel_format == "mjpeg":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    elif pixel_format == "yuyv":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    else:
        raise ValueError("pixel_format: %r" % (pixel_format,))


def _configure_capture(
    cap: cv2.VideoCapture,
    width: int,
    height: int,
    fps: int,
    pixel_format: str,
    buffer_size: int,
) -> None:
    # Сначала формат (как usb_cam: MJPEG 800x600), затем размер — иначе часть UVC даёт timeout.
    _apply_pixel_format(cap, pixel_format)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, max(1, buffer_size))


def _open_videocapture(dev, prefer_v4l2: bool) -> cv2.VideoCapture:
    if platform.system() == "Linux" and isinstance(dev, str) and dev.startswith("/dev/"):
        if prefer_v4l2:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if cap.isOpened():
                return cap
            cap.release()
        return cv2.VideoCapture(dev)
    return cv2.VideoCapture(dev)


def _warmup_read(
    cap: cv2.VideoCapture,
    label: str,
    max_wait_sec: float,
    delay_sec: float,
) -> bool:
    """Один read() при V4L2 timeout может занимать ~10 с — ограничиваем общее ожидание."""
    t0 = time.monotonic()
    attempts = 0
    while time.monotonic() - t0 < max_wait_sec:
        ok, frame = cap.read()
        attempts += 1
        if ok and frame is not None and frame.size > 0:
            return True
        time.sleep(delay_sec)
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fc = _fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
    print(
        "Не пришёл кадр с %s за %.1f с (%d read; запрошено %.0fx%.0f, FOURCC=%s)"
        % (label, max_wait_sec, attempts, w, h, fc),
        file=sys.stderr,
    )
    return False


def _release_cap(cap: Optional[cv2.VideoCapture]) -> None:
    if cap is not None:
        cap.release()


def open_stereo_captures(
    device0,
    device1,
    width: int,
    height: int,
    fps: int,
    *,
    pixel_format: str,
    pixel_format_auto_order: List[str],
    prefer_v4l2: bool,
    second_cam_delay_sec: float,
    buffer_size: int,
    warmup_max_wait_sec: float,
    warmup_delay_sec: float,
) -> Tuple[cv2.VideoCapture, cv2.VideoCapture, str]:
    """
    Подбирает pixel format (если auto) и порядок backend. Возвращает (cap0, cap1, format_used).
    """
    if _running_under_wsl():
        print(
            "Замечание: обнаружен WSL. Если видите VIDEOIO(V4L2): select() timeout, "
            "часто USB в WSL2 не отдаёт поток (usbipd, права, или запуск вне WSL).",
            file=sys.stderr,
        )

    modes = pixel_format_auto_order if pixel_format == "auto" else [pixel_format]
    v4l2_first = prefer_v4l2
    backend_order = [True, False] if v4l2_first else [False, True]

    for use_v4l2 in backend_order:
        for fmt in modes:
            cap0 = _open_videocapture(device0, use_v4l2)
            if not cap0.isOpened():
                _release_cap(cap0)
                continue
            _configure_capture(cap0, width, height, fps, fmt, buffer_size)
            if not _warmup_read(cap0, str(device0), warmup_max_wait_sec, warmup_delay_sec):
                _release_cap(cap0)
                continue

            if second_cam_delay_sec > 0:
                time.sleep(second_cam_delay_sec)

            cap1 = _open_videocapture(device1, use_v4l2)
            if not cap1.isOpened():
                _release_cap(cap0)
                _release_cap(cap1)
                continue
            _configure_capture(cap1, width, height, fps, fmt, buffer_size)
            if not _warmup_read(cap1, str(device1), warmup_max_wait_sec, warmup_delay_sec):
                _release_cap(cap0)
                _release_cap(cap1)
                continue

            return cap0, cap1, fmt

    hints = (
        "Попробуйте: --fourcc yuyv или --fourcc auto; --backend any; "
        "--second-cam-delay 5; --buffer-size 4; v4l2-ctl --list-formats-ext -d DEVICE. "
        "На WSL2 без стабильного USB/V4L используйте нативный Linux."
    )
    raise RuntimeError("Не удалось получить кадры с обеих камер. " + hints)


def main() -> int:
    ap = argparse.ArgumentParser(description="Kalibr -> OpenCV stereo rectify preview")
    ap.add_argument(
        "--camchain",
        type=Path,
        required=True,
        help="Путь к *-camchain.yaml от Kalibr",
    )
    ap.add_argument("--device0", default="/dev/video2", help="Левая (cam0), путь или индекс")
    ap.add_argument("--device1", default="/dev/video0", help="Правая (cam1), путь или индекс")
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=600)
    ap.add_argument("--fps", type=int, default=0, help="0 = не задавать")
    ap.add_argument(
        "--fourcc",
        choices=("mjpeg", "yuyv", "none", "auto"),
        default="mjpeg",
        help="Формат V4L2 (как у usb_cam MJPEG); auto перебирает mjpeg, yuyv, none",
    )
    ap.add_argument(
        "--backend",
        choices=("v4l2", "any"),
        default="v4l2",
        help="v4l2: приоритет CAP_V4L2; any: сначала backend по умолчанию",
    )
    ap.add_argument(
        "--second-cam-delay",
        type=float,
        default=0.0,
        metavar="SEC",
        help="Пауза перед открытием второй камеры (в ROS для cam1 часто 5 с)",
    )
    ap.add_argument(
        "--buffer-size",
        type=int,
        default=2,
        help="CAP_PROP_BUFFERSIZE (1 у части UVC даёт зависания)",
    )
    ap.add_argument(
        "--warmup-max-wait",
        type=float,
        default=22.0,
        metavar="SEC",
        help="Макс. время ожидания первого кадра на камеру (read при timeout V4L2 ~10 с)",
    )
    ap.add_argument(
        "--warmup-delay",
        type=float,
        default=0.02,
        help="Пауза после неудачного read при прогреве, с",
    )
    ap.add_argument(
        "--read-fail-max",
        type=int,
        default=80,
        help="Подряд неудачных read в основном цикле перед выходом",
    )
    ap.add_argument(
        "--separate-windows",
        action="store_true",
        help="Два окна вместо одного склеенного",
    )
    ap.add_argument("--grid-step", type=int, default=40, help="Шаг горизонтальных линий, px")
    ap.add_argument(
        "--alpha",
        type=float,
        default=0.0,
        help="cv2.stereoRectify alpha (только radtan): 0=crop, 1=все пиксели",
    )
    ap.add_argument(
        "--fisheye-balance",
        type=float,
        default=0.0,
        help="cv2.fisheye.stereoRectify balance (только equidistant), 0..1",
    )
    ap.add_argument(
        "--dump-npz",
        type=Path,
        default=None,
        help="Файл .npz: map1x,map1y,map2x,map2y,P1,P2,Q,R1,R2,mode,width,height (ORB-SLAM3/C++)",
    )
    ap.add_argument(
        "--dump-only",
        action="store_true",
        help="Только записать --dump-npz из camchain и выйти (камеры не нужны)",
    )
    args = ap.parse_args()

    if not args.camchain.is_file():
        print("Нет файла:", args.camchain, file=sys.stderr)
        return 1

    chain = load_camchain(args.camchain)
    (
        (iw, ih),
        map1x,
        map1y,
        map2x,
        map2y,
        p1,
        p2,
        q,
        r1,
        r2,
        mode,
    ) = stereo_rectify_from_kalibr(
        chain,
        alpha=args.alpha,
        fisheye_balance=args.fisheye_balance,
    )

    if args.width != iw or args.height != ih:
        print(
            "Предупреждение: разрешение захвата %dx%d не совпадает с camchain %dx%d; "
            "масштабируйте bag/калибровку или выставьте --width/--height как в yaml."
            % (args.width, args.height, iw, ih),
            file=sys.stderr,
        )

    if args.dump_npz is not None:
        np.savez(
            args.dump_npz,
            map1x=map1x,
            map1y=map1y,
            map2x=map2x,
            map2y=map2y,
            P1=p1,
            P2=p2,
            Q=q,
            R1=r1,
            R2=r2,
            mode=np.array([mode]),
            width=np.int32(iw),
            height=np.int32(ih),
        )
        print("Сохранено:", args.dump_npz.resolve())

    if args.dump_only:
        if args.dump_npz is None:
            print("С --dump-only укажите --dump-npz", file=sys.stderr)
            return 1
        return 0

    d0 = _parse_device(args.device0)
    d1 = _parse_device(args.device1)
    auto_order: List[str] = ["mjpeg", "yuyv", "none"]
    prefer_v4l2 = args.backend == "v4l2"
    try:
        cap0, cap1, fmt_used = open_stereo_captures(
            d0,
            d1,
            args.width,
            args.height,
            args.fps,
            pixel_format=args.fourcc,
            pixel_format_auto_order=auto_order,
            prefer_v4l2=prefer_v4l2,
            second_cam_delay_sec=args.second_cam_delay,
            buffer_size=args.buffer_size,
            warmup_max_wait_sec=args.warmup_max_wait,
            warmup_delay_sec=args.warmup_delay,
        )
    except RuntimeError as e:
        print(e, file=sys.stderr)
        return 1

    win = "rectified L | R (%s)" % mode
    if not args.separate_windows:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    print("Режим:", mode, "| pixel_format:", fmt_used, "| q — выход")
    print("ORB-SLAM3: подавайте выпрямленные кадры; intrinsics для rectified-потока — из P1/P2 выше (или загрузите их из --dump-npz).")

    try:
        read_fail = 0
        while True:
            ok0, raw0 = cap0.read()
            ok1, raw1 = cap1.read()
            if not ok0 or not ok1 or raw0 is None or raw1 is None:
                read_fail += 1
                if read_fail >= args.read_fail_max:
                    print(
                        "Слишком много неудачных read подряд (%d). "
                        "Проверьте USB/хаб, WSL+usbipd или запуск с нативного Linux."
                        % read_fail,
                        file=sys.stderr,
                    )
                    break
                time.sleep(0.02)
                continue
            read_fail = 0

            if raw0.shape[:2] != (ih, iw) or raw1.shape[:2] != (ih, iw):
                raw0 = cv2.resize(raw0, (iw, ih), interpolation=cv2.INTER_AREA)
                raw1 = cv2.resize(raw1, (iw, ih), interpolation=cv2.INTER_AREA)

            rem0 = cv2.remap(raw0, map1x, map1y, cv2.INTER_LINEAR)
            rem1 = cv2.remap(raw1, map2x, map2y, cv2.INTER_LINEAR)

            if args.separate_windows:
                d0c = rem0.copy()
                d1c = rem1.copy()
                draw_epipolar_grid(d0c, args.grid_step)
                draw_epipolar_grid(d1c, args.grid_step)
                cv2.imshow("rectified cam0 (left)", d0c)
                cv2.imshow("rectified cam1 (right)", d1c)
            else:
                combo = np.hstack((rem0, rem1))
                draw_epipolar_grid(combo, args.grid_step)
                cv2.imshow(win, combo)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        cap0.release()
        cap1.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
