#!/usr/bin/env python3
import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CalibrationImage:
    path: Path
    image: np.ndarray
    points: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an OpenCV checkerboard calibration against a saved capture session."
    )
    parser.add_argument(
        "session",
        type=Path,
        help="Capture session directory, for example data/camera_calibration/captures/20260519_023511",
    )
    parser.add_argument("--checkerboard-cols", type=int, default=8)
    parser.add_argument("--checkerboard-rows", type=int, default=6)
    parser.add_argument(
        "--square-size",
        type=float,
        default=1.0,
        help="Physical checkerboard square size in your chosen units. Intrinsics are scale-invariant, so 1.0 is acceptable if you only need camera matrix and distortion.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional output file. Defaults to <session>/calibration/intrinsics.json",
    )
    parser.add_argument(
        "--min-images",
        type=int,
        default=10,
        help="Minimum number of usable checkerboard images required before solving.",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        help="Optional limit on the number of usable images to feed into calibrateCamera.",
    )
    return parser.parse_args()


def detect_checkerboard(gray: np.ndarray, pattern: tuple[int, int]):
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            found, corners = cv2.findChessboardCornersSB(gray, pattern)
            return bool(found), corners
        except cv2.error:
            pass

    found, corners = cv2.findChessboardCorners(gray, pattern)
    if found:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return bool(found), corners


def collect_images(session: Path) -> list[Path]:
    image_root = session / "images"
    if not image_root.exists():
        raise FileNotFoundError(f"no images directory found in {session}")
    return sorted(image_root.glob("focus_*/cal_*.jpg"))


def load_calibration_images(paths: list[Path], pattern: tuple[int, int]) -> tuple[list[CalibrationImage], tuple[int, int]]:
    usable: list[CalibrationImage] = []
    image_size: tuple[int, int] | None = None

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        height, width = image.shape[:2]
        if image_size is None:
            image_size = (width, height)
        elif image_size != (width, height):
            raise ValueError(
                f"image size mismatch for {path}: expected {image_size[0]}x{image_size[1]}, got {width}x{height}"
            )

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = detect_checkerboard(gray, pattern)
        if not found or corners is None:
            continue

        if corners.ndim == 2:
            corners = corners[:, np.newaxis, :]
        usable.append(CalibrationImage(path=path, image=image, points=corners.astype(np.float32)))

    if image_size is None:
        raise RuntimeError("no readable calibration images were found")

    return usable, image_size


def build_object_points(pattern: tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = pattern
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid *= float(square_size)
    return grid


def reprojection_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> list[float]:
    errors: list[float] = []
    for objp, imgp, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2)
        observed = imgp.reshape(-1, 2)
        error = cv2.norm(observed, projected, cv2.NORM_L2) / len(projected)
        errors.append(float(error))
    return errors


def main() -> int:
    args = parse_args()
    session = args.session
    pattern = (args.checkerboard_cols, args.checkerboard_rows)
    paths = collect_images(session)
    usable, image_size = load_calibration_images(paths, pattern)

    if len(usable) < args.min_images:
        raise RuntimeError(
            f"only {len(usable)} usable checkerboard images found; need at least {args.min_images}"
        )

    if args.max_images is not None:
        usable = usable[: args.max_images]

    object_points_template = build_object_points(pattern, args.square_size)
    object_points = [object_points_template.copy() for _ in usable]
    image_points = [item.points for item in usable]

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_view_errors = reprojection_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    out_path = args.output or (session / "calibration" / "intrinsics.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    result = {
        "session": str(session),
        "image_count": len(paths),
        "usable_image_count": len(usable),
        "rejected_image_count": len(paths) - len(usable),
        "checkerboard_inner_corners": [args.checkerboard_cols, args.checkerboard_rows],
        "square_size": args.square_size,
        "image_size": [image_size[0], image_size[1]],
        "rms_reprojection_error": float(rms),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.reshape(-1).tolist(),
        "per_view_reprojection_error_px": per_view_errors,
        "mean_reprojection_error_px": float(np.mean(per_view_errors)) if per_view_errors else None,
        "max_reprojection_error_px": float(np.max(per_view_errors)) if per_view_errors else None,
        "usable_images": [str(item.path.relative_to(session)) for item in usable],
        "note": (
            "Intrinsics are invariant to square size scale; extrinsic translations use the provided square_size units."
        ),
    }

    out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(result, indent=2))
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
