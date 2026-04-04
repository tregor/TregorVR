#!/usr/bin/env python3
"""
Полноэкранная шахматная доска на выбранном мониторе (Windows, только stdlib: tkinter + ctypes).

По умолчанию 7×8 квадратов — как для targetCols:6, targetRows:7 в Kalibr.
Физический шаг квадрата для калибровки: 75 mm — см. kalibr_catkin/chessboard_template.yaml (0.075 m).

  python chessboard_second_screen.py --monitor 1
  python chessboard_second_screen.py -m 1 --margin 0.06 --cols 7 --rows 8

Выход: Esc. Без рамки окна при необходимости закройте окно из панели задач.
"""
from __future__ import annotations

import argparse
import ctypes
import sys
import tkinter as tk
from ctypes import wintypes


def _windows_monitors() -> list[tuple[int, int, int, int]]:
    """Список мониторов: (left, top, right, bottom) в пикселях."""
    user32 = ctypes.windll.user32
    monitors: list[tuple[int, int, int, int]] = []

    @ctypes.WINFUNCTYPE(
        ctypes.c_int,
        wintypes.HMONITOR,
        wintypes.HDC,
        ctypes.POINTER(wintypes.RECT),
        wintypes.LPARAM,
    )
    def _cb(_h, _dc, prect, _data):
        r = prect.contents
        monitors.append((r.left, r.top, r.right, r.bottom))
        return 1

    user32.EnumDisplayMonitors(None, None, _cb, 0)
    return monitors


def _snap_window_to_rect(root: tk.Tk, left: int, top: int, width: int, height: int) -> None:
    """Tk fullscreen на Windows часто игнорирует смещение и лезет на монитор 0 — двигаем через WinAPI."""
    user32 = ctypes.windll.user32
    root.update_idletasks()
    hwnd = root.winfo_id()
    GA_ROOT = 2
    root_hwnd = user32.GetAncestor(hwnd, GA_ROOT)
    if root_hwnd:
        hwnd = root_hwnd
    SWP_SHOWWINDOW = 0x0040
    SWP_NOZORDER = 0x0004
    user32.SetWindowPos(hwnd, 0, left, top, width, height, SWP_SHOWWINDOW | SWP_NOZORDER)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Chessboard fullscreen on a chosen display (Windows).")
    p.add_argument(
        "-m",
        "--monitor",
        type=int,
        default=1,
        help="Индекс монитора (0 — основной, 1 — второй). По умолчанию: 1",
    )
    p.add_argument(
        "--margin",
        type=float,
        default=0.06,
        metavar="FRAC",
        help="Отступ от краёв, доля от меньшей стороны (0–0.45). По умолчанию: 0.06",
    )
    p.add_argument(
        "--cols",
        type=int,
        default=7,
        help="Квадратов по горизонтали (6 inner corners Kalibr → 7). По умолчанию: 7",
    )
    p.add_argument(
        "--rows",
        type=int,
        default=8,
        help="Квадратов по вертикали (7 inner corners Kalibr → 8). По умолчанию: 8",
    )
    p.add_argument(
        "--top-left",
        choices=("black", "white"),
        default="black",
        help="Цвет левого верхнего квадрата. По умолчанию: black",
    )
    p.add_argument(
        "--list",
        action="store_true",
        help="Показать индексы и размеры мониторов и выйти",
    )
    return p.parse_args()


def main() -> int:
    if sys.platform != "win32":
        print("Скрипт рассчитан на Windows (перечисление экранов через WinAPI).", file=sys.stderr)
        return 1

    args = parse_args()
    monitors = _windows_monitors()

    if args.list:
        if not monitors:
            print("Мониторы не найдены.", file=sys.stderr)
            return 1
        for i, (l, t, r, b) in enumerate(monitors):
            print(f"{i}: {r - l}x{b - t} at ({l}, {t})")
        return 0

    if args.margin < 0 or args.margin > 0.45:
        print("margin must be in [0, 0.45]", file=sys.stderr)
        return 2
    if args.cols < 1 or args.rows < 1:
        print("cols and rows must be >= 1", file=sys.stderr)
        return 2

    if not monitors:
        print("No displays reported.", file=sys.stderr)
        return 1

    idx = args.monitor
    if idx < 0 or idx >= len(monitors):
        print(
            f"Monitor index {idx} out of range (0..{len(monitors) - 1}).",
            file=sys.stderr,
        )
        return 1

    left, top, right, bottom = monitors[idx]
    w = right - left
    h = bottom - top
    tl_black = args.top_left == "black"

    root = tk.Tk()
    root.title("Chessboard (Esc to quit)")
    # Без рамки — иначе «полный экран» не совпадает с прямоугольником монитора; -fullscreen в Tk на Windows
    # почти всегда открывается на мониторе 0, игнорируя +left+top.
    root.overrideredirect(True)
    root.geometry(f"{w}x{h}+{left}+{top}")

    canvas = tk.Canvas(root, highlightthickness=0, bg="white")
    canvas.pack(fill=tk.BOTH, expand=True)

    def paint() -> None:
        canvas.delete("all")
        cw = max(1, canvas.winfo_width())
        ch = max(1, canvas.winfo_height())
        canvas.create_rectangle(0, 0, cw, ch, fill="white", outline="white")

        m = int(min(cw, ch) * args.margin)
        m = max(0, min(m, min(cw, ch) // 2 - 2))
        iw = max(1, cw - 2 * m)
        ih = max(1, ch - 2 * m)

        cell = min(iw / args.cols, ih / args.rows)
        board_w = cell * args.cols
        board_h = cell * args.rows
        ox = m + (iw - board_w) / 2
        oy = m + (ih - board_h) / 2

        for row in range(args.rows):
            for col in range(args.cols):
                is_black = (row + col) % 2 == 0
                if not tl_black:
                    is_black = not is_black
                color = "#000000" if is_black else "#ffffff"
                x0 = int(round(ox + col * cell))
                y0 = int(round(oy + row * cell))
                x1 = int(round(ox + (col + 1) * cell))
                y1 = int(round(oy + (row + 1) * cell))
                canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=color)

    def on_configure(_event: tk.Event) -> None:
        paint()

    def on_map(_event: tk.Event | None = None) -> None:
        _snap_window_to_rect(root, left, top, w, h)

    canvas.bind("<Configure>", on_configure)
    root.bind("<Map>", on_map)
    root.after(200, lambda: _snap_window_to_rect(root, left, top, w, h))
    root.bind("<Escape>", lambda _e: root.destroy())
    root.protocol("WM_DELETE_WINDOW", root.destroy)

    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
