#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an mrcal-compatible corners cache from checkerboard images using OpenCV."
    )
    parser.add_argument(
        "session",
        type=Path,
        help="Capture session directory, for example data/camera_calibration/captures/20260519_023511",
    )
    parser.add_argument("--checkerboard-cols", type=int, default=8)
    parser.add_argument("--checkerboard-rows", type=int, default=6)
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional output vnlog file. Defaults to <session>/calibration/mrcal/corners-opencv.vnl",
    )
    parser.add_argument(
        "--image-glob",
        default="focus_*/cal_*.jpg",
        help="Glob under <session>/images used to select calibration images.",
    )
    parser.add_argument(
        "--path-mode",
        choices=("relative", "absolute"),
        default="relative",
        help="How image paths are written into the vnlog cache.",
    )
    return parser.parse_args()


def detect_checkerboard(gray, pattern: tuple[int, int]):
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


def collect_images(session: Path, image_glob: str) -> list[Path]:
    image_root = session / "images"
    if not image_root.exists():
        raise FileNotFoundError(f"no images directory found in {session}")
    paths = sorted(image_root.glob(image_glob))
    if not paths:
        raise RuntimeError(f"no images matched {image_root / image_glob}")
    return paths


def build_corners_cache(
    session: Path,
    *,
    checkerboard_cols: int = 8,
    checkerboard_rows: int = 6,
    image_glob: str = "focus_*/cal_*.jpg",
    output: Path | None = None,
    path_mode: str = "relative",
) -> dict:
    session = session.resolve()
    pattern = (checkerboard_cols, checkerboard_rows)
    paths = collect_images(session, image_glob)

    out_path = output or (session / "calibration" / "mrcal" / "corners-opencv.vnl")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    lines = ["# filename x y level"]
    usable_images: list[str] = []
    rejected_images: list[str] = []

    for path in paths:
        cache_path = str(path.relative_to(session)) if path_mode == "relative" else str(path)
        image = cv2.imread(str(path))
        if image is None:
            lines.append(f"{cache_path} - - -")
            rejected_images.append(str(path.relative_to(session)))
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = detect_checkerboard(gray, pattern)
        if not found or corners is None:
            lines.append(f"{cache_path} - - -")
            rejected_images.append(str(path.relative_to(session)))
            continue

        usable_images.append(str(path.relative_to(session)))
        for x, y in corners.reshape(-1, 2):
            lines.append(f"{cache_path} {x:.6f} {y:.6f} 0")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    return {
        "session": str(session),
        "checkerboard_inner_corners": [checkerboard_cols, checkerboard_rows],
        "image_count": len(paths),
        "usable_image_count": len(usable_images),
        "rejected_image_count": len(rejected_images),
        "output": str(out_path),
        "path_mode": path_mode,
        "usable_images": usable_images,
        "rejected_images": rejected_images,
    }


def summarize_corners_cache(corners_cache: Path, *, checkerboard_cols: int, checkerboard_rows: int) -> dict:
    grid_points = checkerboard_cols * checkerboard_rows
    image_counts: dict[str, int] = {}
    with corners_cache.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 3:
                continue
            image_counts[fields[0]] = image_counts.get(fields[0], 0) + 1

    usable = 0
    rejected = 0
    for count in image_counts.values():
        if count == 1:
            rejected += 1
        elif count == grid_points:
            usable += 1
        else:
            usable += 1

    return {
        "image_count": len(image_counts),
        "usable_image_count": usable,
        "rejected_image_count": rejected,
    }


def main() -> int:
    args = parse_args()
    summary = build_corners_cache(
        args.session,
        checkerboard_cols=args.checkerboard_cols,
        checkerboard_rows=args.checkerboard_rows,
        image_glob=args.image_glob,
        output=args.output,
        path_mode=args.path_mode,
    )

    print(json.dumps(summary, indent=2))
    print(f"wrote {summary['output']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
