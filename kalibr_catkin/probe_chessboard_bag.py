#!/usr/bin/env python3
"""
Подбор (cols, rows) для Kalibr / OpenCV по rosbag.

Быстрый режим по умолчанию: мало кадров, маленький превью-скан, ~20 типичных досок.

  source /opt/ros/noetic/setup.bash
  python3 probe_chessboard_bag.py mystereo.bag /cam0/image_raw
  python3 probe_chessboard_bag.py mystereo.bag /cam0/image_raw --clahe

Полный перебор 4..12 x 4..12 (долго):
  python3 probe_chessboard_bag.py ... --all-sizes --max-frames 25

Kalibr в Docker: ./kalibr_pipeline.sh calibrate bag.bag
Физический шаг квадрата задаётся только в chessboard_template.yaml (сейчас 75 mm).

OpenCV patternSize = (targetCols, targetRows) = (внутр. углы по горизонтали, по вертикали).
"""
from __future__ import print_function

import argparse
import collections
import os
import sys

import cv2
import rosbag
from cv_bridge import CvBridge, CvBridgeError


def msg_to_gray(bridge, msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        return img
    except CvBridgeError:
        pass
    for enc in ("bgr8", "rgb8"):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if enc == "bgr8" else cv2.COLOR_RGB2GRAY)
        except CvBridgeError:
            continue
    raise ValueError("Unsupported encoding: %s" % msg.encoding)


def try_detect(gray, cols, rows, fast_check=False, filter_quads=False):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast_check:
        flags += cv2.CALIB_CB_FAST_CHECK
    if filter_quads:
        flags += cv2.CALIB_CB_FILTER_QUADS
    ok, _corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    return ok


# Типичные внутренние углы (cols, rows); dict.fromkeys — без дубликатов, порядок сохраняем
FAST_SIZES = list(
    dict.fromkeys(
        [
            (6, 7), (7, 6), (8, 6), (6, 8), (9, 6), (6, 9), (7, 5), (5, 7), (10, 6), (6, 10),
            (9, 7), (7, 9), (11, 8), (8, 11), (10, 7), (7, 10), (8, 7), (7, 8),
            (9, 8), (8, 9), (7, 6), (6, 7), (11, 6), (6, 11), (12, 8), (8, 12),
        ]
    )
)

ALL_SIZES_4_12 = [(c, r) for c in range(4, 13) for r in range(4, 13)]


def resize_for_probe(gray, max_side):
    h, w = gray.shape[:2]
    m = max(h, w)
    if m <= max_side:
        return gray
    scale = max_side / float(m)
    return cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="path to .bag")
    ap.add_argument("topic", help="e.g. /cam0/image_raw")
    ap.add_argument("--max-frames", type=int, default=10, help="кадров для быстрого скана (default: 10)")
    ap.add_argument("--max-side", type=int, default=512, help="длинная сторона при скане, px (default: 512)")
    ap.add_argument("--clahe", action="store_true", help="CLAHE перед детектом")
    ap.add_argument(
        "--all-sizes",
        action="store_true",
        help="все пары 4..12 x 4..12 (медленно); иначе только FAST_SIZES",
    )
    ap.add_argument(
        "--brute",
        action="store_true",
        help="то же что --all-sizes + диапазон 4..13",
    )
    args = ap.parse_args()

    if args.brute:
        sizes = [(c, r) for c in range(4, 14) for r in range(4, 14)]
    elif args.all_sizes:
        sizes = ALL_SIZES_4_12
    else:
        sizes = FAST_SIZES

    if not os.path.isfile(args.bag):
        print("No file:", args.bag, file=sys.stderr)
        sys.exit(1)

    bridge = CvBridge()
    counts = collections.Counter()
    enc_seen = None
    shape_seen = None
    n = 0

    with rosbag.Bag(args.bag, "r") as bag:
        for _topic, msg, _t in bag.read_messages(topics=[args.topic]):
            if n >= args.max_frames:
                break
            n += 1
            enc_seen = msg.encoding
            shape_seen = (msg.width, msg.height)
            try:
                gray = msg_to_gray(bridge, msg)
            except Exception as e:
                print("Frame decode error:", e, file=sys.stderr)
                continue
            if args.clahe:
                gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            gray = resize_for_probe(gray, args.max_side)

            for cols, rows in sizes:
                if try_detect(gray, cols, rows, fast_check=True, filter_quads=False):
                    counts[(cols, rows)] += 1

    print("Encoding:", enc_seen, "size:", shape_seen, "frames:", n, "sizes tested:", len(sizes), "max_side:", args.max_side)

    if not counts:
        print("\nБыстрый скан ничего не нашёл. Один кадр, без FAST_CHECK, те же размеры...")
        with rosbag.Bag(args.bag, "r") as bag:
            for _topic, msg, _t in bag.read_messages(topics=[args.topic]):
                gray = msg_to_gray(bridge, msg)
                if args.clahe:
                    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
                gray = resize_for_probe(gray, args.max_side)
                for cols, rows in sizes:
                    if try_detect(gray, cols, rows, fast_check=False, filter_quads=False):
                        counts[(cols, rows)] += 1
                break

    if not counts and not args.all_sizes and not args.brute:
        print("Running ALL_SIZES 4..12 on", args.max_frames, "frames...")
        sizes = ALL_SIZES_4_12
        n2 = 0
        with rosbag.Bag(args.bag, "r") as bag:
            for _topic, msg, _t in bag.read_messages(topics=[args.topic]):
                if n2 >= args.max_frames:
                    break
                n2 += 1
                try:
                    gray = msg_to_gray(bridge, msg)
                except Exception:
                    continue
                if args.clahe:
                    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
                gray = resize_for_probe(gray, args.max_side)
                for cols, rows in sizes:
                    if try_detect(gray, cols, rows, fast_check=True, filter_quads=False):
                        counts[(cols, rows)] += 1

    if not counts:
        print(
            "\nУглы не найдены. Попробуй: --clahe, увеличь --max-side 640, --max-frames 20, "
            "или --brute (очень долго).\n"
            "AprilGrid надёжнее для fisheye."
        )
        sys.exit(2)

    print("\nУспехи (targetCols x targetRows):")
    for (c, r), k in counts.most_common(10):
        print("  targetCols=%2d  targetRows=%2d  ->  %4d frames" % (c, r, k))

    best = counts.most_common(1)[0][0]
    c, r = best
    print(
        "\nYAML-кандидат:\n"
        "  targetCols: %d\n"
        "  targetRows: %d\n"
        % (c, r)
    )

    print("Kalibr-стиль (FILTER_QUADS), топ-4 кандидата, первый полный кадр:")
    with rosbag.Bag(args.bag, "r") as bag:
        for _topic, msg, _t in bag.read_messages(topics=[args.topic]):
            try:
                gfull = msg_to_gray(bridge, msg)
            except Exception as e:
                print("  decode error:", e)
                break
            if args.clahe:
                gfull = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gfull)
            for (cc, rr), _ in counts.most_common(4):
                ok = try_detect(gfull, cc, rr, fast_check=False, filter_quads=True)
                print("  targetCols=%2d targetRows=%2d  ->  %s" % (cc, rr, "OK" if ok else "нет"))
            break

    print(
        "\nЕсли FILTER_QUADS везде «нет» — см. AprilGrid / отключение filterQuads в Kalibr / доска в центре кадра."
    )


if __name__ == "__main__":
    main()
