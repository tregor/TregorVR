#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import importlib.util
from pathlib import Path
from types import ModuleType
from typing import Any, Dict

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _scale_camchain_to_resolution(chain: Dict[str, Any], out_w: int, out_h: int) -> Dict[str, Any]:
    """
    Линейное масштабирование intrinsics под другое разрешение (без новой съёмки).
    Точнее — полная перекалибровка на 1280x720; для OV при смене aspect 800x600→1280x720 — приближение.
    """
    c = copy.deepcopy(chain)
    for cam_id in ("cam0", "cam1"):
        if cam_id not in c:
            continue
        cam = c[cam_id]
        w0, h0 = int(cam["resolution"][0]), int(cam["resolution"][1])
        if w0 == out_w and h0 == out_h:
            continue
        sx = out_w / float(w0)
        sy = out_h / float(h0)
        fu, fv, cu, cv = cam["intrinsics"]
        cam["intrinsics"] = [fu * sx, fv * sy, cu * sx, cv * sy]
        cam["resolution"] = [out_w, out_h]
    return c


def _load_rectify_module() -> ModuleType:
    module_path = _repo_root() / "tools" / "stereo_rectify_preview" / "rectified_stereo_preview.py"
    spec = importlib.util.spec_from_file_location("rectified_stereo_preview", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load rectify helper from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _format_scalar(value: Any) -> str:
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        v = float(value)
        s = format(v, ".15g")
        # ORB-SLAM3 Settings + OpenCV FileStorage: поля вроде Stereo.ThDepth должны быть real, не int.
        if "e" in s.lower() or "." in s:
            return s
        return s + ".0"
    return str(value)


def _opencv_matrix_block(name: str, array: np.ndarray, dt: str | None = None) -> str:
    arr = np.asarray(array)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.ndim != 2:
        raise ValueError(f"{name}: expected 2D array, got shape {arr.shape}")

    if dt is None:
        if np.issubdtype(arr.dtype, np.integer):
            dt = "i"
        elif arr.dtype == np.float32:
            dt = "f"
        else:
            dt = "d"

    flat = ", ".join(_format_scalar(v) for v in arr.reshape(-1))
    return (
        f"{name}: !!opencv-matrix\n"
        f" rows: {arr.shape[0]}\n"
        f" cols: {arr.shape[1]}\n"
        f" dt: {dt}\n"
        f" data: [{flat}]\n"
    )


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _default_output_settings(camchain_path: Path) -> Path:
    stem = camchain_path.stem
    if stem.endswith("-camchain"):
        stem = stem[: -len("-camchain")]
    return camchain_path.with_name(f"{stem}-orbslam3-stereo-rectified.yaml")


def _default_output_rectify(camchain_path: Path) -> Path:
    stem = camchain_path.stem
    if stem.endswith("-camchain"):
        stem = stem[: -len("-camchain")]
    return camchain_path.with_name(f"{stem}-orbslam3-rectify-input.yaml")


def build_settings_yaml(
    *,
    width: int,
    height: int,
    fps: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    baseline_m: float,
    bf: float,
    p1: np.ndarray,
    p2: np.ndarray,
    q: np.ndarray,
    n_features: int,
    scale_factor: float,
    n_levels: int,
    ini_fast: int,
    min_fast: int,
    th_depth: float,
) -> str:
    t_rect = np.eye(4, dtype=np.float64)
    t_rect[0, 3] = baseline_m

    text = [
        "%YAML:1.0",
        "",
        "# Auto-generated from Kalibr camchain for pre-rectified stereo input.",
        "# Left camera = cam0, right camera = cam1.",
        'File.version: "1.0"',
        'Camera.type: "PinHole"',
        "",
        "# Rectified intrinsics from P1/P2.",
        f"Camera1.fx: {_format_scalar(fx)}",
        f"Camera1.fy: {_format_scalar(fy)}",
        f"Camera1.cx: {_format_scalar(cx)}",
        f"Camera1.cy: {_format_scalar(cy)}",
        "Camera1.k1: 0.0",
        "Camera1.k2: 0.0",
        "Camera1.p1: 0.0",
        "Camera1.p2: 0.0",
        "",
        f"Camera2.fx: {_format_scalar(fx)}",
        f"Camera2.fy: {_format_scalar(fy)}",
        f"Camera2.cx: {_format_scalar(cx)}",
        f"Camera2.cy: {_format_scalar(cy)}",
        "Camera2.k1: 0.0",
        "Camera2.k2: 0.0",
        "Camera2.p1: 0.0",
        "Camera2.p2: 0.0",
        "",
        f"Camera.width: {width}",
        f"Camera.height: {height}",
        # ORB-SLAM3 Settings ожидает целый fps (readInt), не вещественный.
        f"Camera.fps: {max(1, int(round(fps)))}",
        "Camera.RGB: 0",
        "",
        f"Stereo.ThDepth: {_format_scalar(th_depth)}",
        f"Camera.bf: {_format_scalar(bf)}",
        f"Stereo.baseline_m: {_format_scalar(baseline_m)}",
        _opencv_matrix_block("Stereo.T_c1_c2", t_rect, dt="f").rstrip(),
        "",
        f"ORBextractor.nFeatures: {n_features}",
        f"ORBextractor.scaleFactor: {_format_scalar(scale_factor)}",
        f"ORBextractor.nLevels: {n_levels}",
        f"ORBextractor.iniThFAST: {ini_fast}",
        f"ORBextractor.minThFAST: {min_fast}",
        "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -1.8",
        "Viewer.ViewpointF: 500.0",
        "Viewer.imageViewScale: 1.0",
        "",
        "# Debug/reference values.",
        _opencv_matrix_block("Rectified.P1", p1).rstrip(),
        _opencv_matrix_block("Rectified.P2", p2).rstrip(),
        _opencv_matrix_block("Rectified.Q", q).rstrip(),
        "",
    ]
    return "\n".join(text)


def build_rectify_yaml(
    *,
    width: int,
    height: int,
    camera_model: str,
    k_left: np.ndarray,
    d_left: np.ndarray,
    k_right: np.ndarray,
    d_right: np.ndarray,
    t_raw: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    q: np.ndarray,
    baseline_m: float,
) -> str:
    text = [
        "%YAML:1.0",
        "",
        "# Auto-generated from Kalibr camchain for the direct-camera ORB-SLAM3 live runner.",
        'File.version: "1.0"',
        f'Camera.model: "{camera_model}"',
        f"Camera.width: {width}",
        f"Camera.height: {height}",
        f"Stereo.baseline_m: {_format_scalar(baseline_m)}",
        "",
        _opencv_matrix_block("LEFT.K", k_left).rstrip(),
        _opencv_matrix_block("LEFT.D", d_left.reshape(1, -1)).rstrip(),
        "",
        _opencv_matrix_block("RIGHT.K", k_right).rstrip(),
        _opencv_matrix_block("RIGHT.D", d_right.reshape(1, -1)).rstrip(),
        "",
        _opencv_matrix_block("Stereo.T_c1_c2_raw", t_raw).rstrip(),
        "",
        _opencv_matrix_block("Rectified.P1", p1).rstrip(),
        _opencv_matrix_block("Rectified.P2", p2).rstrip(),
        _opencv_matrix_block("Rectified.Q", q).rstrip(),
        "",
    ]
    return "\n".join(text)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Generate ORB-SLAM3 rectified stereo config from Kalibr camchain."
    )
    ap.add_argument("--camchain", type=Path, required=True, help="Path to *-camchain.yaml")
    ap.add_argument("--output-settings", type=Path, default=None)
    ap.add_argument("--output-rectify", type=Path, default=None)
    ap.add_argument("--left-id", default="cam0")
    ap.add_argument("--right-id", default="cam1")
    ap.add_argument(
        "--fps",
        type=float,
        default=20.0,
        help="Для YAML и live (как usb_cam по умолчанию); 120 только если стабильно по USB.",
    )
    ap.add_argument("--alpha", type=float, default=0.0)
    ap.add_argument("--fisheye-balance", type=float, default=0.0)
    ap.add_argument("--n-features", type=int, default=1500)
    ap.add_argument("--scale-factor", type=float, default=1.2)
    ap.add_argument("--n-levels", type=int, default=8)
    ap.add_argument("--ini-fast", type=int, default=20)
    ap.add_argument("--min-fast", type=int, default=7)
    ap.add_argument("--th-depth", type=float, default=60.0)
    ap.add_argument(
        "--capture-width",
        type=int,
        default=None,
        help="Иначе разрешение из camchain. Вместе с --capture-height — масштабирование K.",
    )
    ap.add_argument("--capture-height", type=int, default=None)
    args = ap.parse_args()

    if not args.camchain.is_file():
        raise FileNotFoundError(f"No such camchain: {args.camchain}")

    if (args.capture_width is None) ^ (args.capture_height is None):
        raise SystemExit("Задай оба: --capture-width и --capture-height, либо ни одного.")

    rectify = _load_rectify_module()
    chain_raw: Dict[str, Any] = rectify.load_camchain(args.camchain)
    if args.capture_width is not None:
        chain = _scale_camchain_to_resolution(chain_raw, args.capture_width, args.capture_height)
        print(
            "Scaled calibration resolution %s -> [%d, %d]"
            % (chain_raw["cam0"]["resolution"], args.capture_width, args.capture_height)
        )
    else:
        chain = chain_raw
    left = chain[args.left_id]
    right = chain[args.right_id]

    (
        (width, height),
        _map1,
        _map2,
        _map3,
        _map4,
        p1,
        p2,
        q,
        _r1,
        _r2,
        _mode,
    ) = rectify.stereo_rectify_from_kalibr(
        chain,
        left_id=args.left_id,
        right_id=args.right_id,
        alpha=args.alpha,
        fisheye_balance=args.fisheye_balance,
    )

    k_left = np.asarray(rectify._intrinsic_matrix(left), dtype=np.float64)
    d_left = np.asarray(rectify._dist_vector(left), dtype=np.float64).reshape(-1)
    k_right = np.asarray(rectify._intrinsic_matrix(right), dtype=np.float64)
    d_right = np.asarray(rectify._dist_vector(right), dtype=np.float64).reshape(-1)
    t_raw = np.asarray(right["T_cn_cnm1"], dtype=np.float64)

    fx = float(p1[0, 0])
    fy = float(p1[1, 1])
    cx = float(p1[0, 2])
    cy = float(p1[1, 2])
    baseline_m = abs(float(p2[0, 3]) / fx)
    bf = fx * baseline_m

    out_settings = args.output_settings or _default_output_settings(args.camchain)
    out_rectify = args.output_rectify or _default_output_rectify(args.camchain)

    settings_text = build_settings_yaml(
        width=width,
        height=height,
        fps=args.fps,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        baseline_m=baseline_m,
        bf=bf,
        p1=np.asarray(p1, dtype=np.float64),
        p2=np.asarray(p2, dtype=np.float64),
        q=np.asarray(q, dtype=np.float64),
        n_features=args.n_features,
        scale_factor=args.scale_factor,
        n_levels=args.n_levels,
        ini_fast=args.ini_fast,
        min_fast=args.min_fast,
        th_depth=args.th_depth,
    )
    rectify_text = build_rectify_yaml(
        width=width,
        height=height,
        camera_model=str(left["distortion_model"]),
        k_left=k_left,
        d_left=d_left,
        k_right=k_right,
        d_right=d_right,
        t_raw=t_raw,
        p1=np.asarray(p1, dtype=np.float64),
        p2=np.asarray(p2, dtype=np.float64),
        q=np.asarray(q, dtype=np.float64),
        baseline_m=baseline_m,
    )

    _write_text(out_settings, settings_text)
    _write_text(out_rectify, rectify_text)

    print(f"camchain={args.camchain.resolve()}")
    print(f"settings={out_settings.resolve()}")
    print(f"rectify={out_rectify.resolve()}")
    print(
        "rectified fx=%.6f fy=%.6f cx=%.6f cy=%.6f baseline_m=%.9f bf=%.6f"
        % (fx, fy, cx, cy, baseline_m, bf)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
