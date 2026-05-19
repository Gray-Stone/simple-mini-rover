#!/usr/bin/env python3
import argparse
import math
import json
import subprocess
from pathlib import Path

import cv2
import mrcal
import numpy as np

try:
    from mrcal_corners_cache import build_corners_cache, summarize_corners_cache
except ModuleNotFoundError:
    from tools.mrcal_corners_cache import build_corners_cache, summarize_corners_cache


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run mrcal calibration from a saved capture session, including diagnostic plot generation."
    )
    parser.add_argument(
        "session",
        type=Path,
        help="Capture session directory, for example data/camera_calibration/captures/20260519_023511",
    )
    parser.add_argument("--checkerboard-cols", type=int, default=8)
    parser.add_argument("--checkerboard-rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, required=True, help="Checkerboard square size in meters.")
    parser.add_argument("--lensmodel", default="LENSMODEL_OPENCV5")
    parser.add_argument("--focal", type=float, required=True, help="Initial focal length estimate in pixels.")
    parser.add_argument(
        "--image-glob",
        default="focus_*/cal_*.jpg",
        help="Glob under <session>/images used to select calibration images.",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        help="Output directory. Defaults to <session>/calibration/mrcal",
    )
    parser.add_argument(
        "--corners-cache",
        type=Path,
        help="Optional vnlog path. Defaults to <outdir>/corners.vnl for square boards or <outdir>/corners-opencv.vnl for non-square boards.",
    )
    parser.add_argument(
        "--detector",
        choices=("auto", "opencv", "mrgingham"),
        default="auto",
        help="Corner detector backend. 'auto' selects mrgingham for square boards and OpenCV for non-square boards.",
    )
    parser.add_argument("--jobs", type=int, default=1, help="Forwarded to mrcal-calibrate-cameras.")
    parser.add_argument("--skip-plots", action="store_true", help="Do not generate post-calibration SVG plots.")
    parser.add_argument(
        "--plot-dir",
        type=Path,
        help="Optional plot directory. Defaults to <outdir>/plots",
    )
    parser.add_argument(
        "--worst-observations",
        type=int,
        default=3,
        help="How many worst board observations to export as SVGs.",
    )
    parser.add_argument(
        "--uncertainty-distances",
        type=float,
        nargs="*",
        default=(0.5, 1.0, 2.0),
        help="Finite distances in meters for projection-uncertainty heatmaps.",
    )
    return parser.parse_args()


def run_command(args: list[str], *, cwd: Path) -> None:
    print(f"+ {' '.join(args)}", flush=True)
    subprocess.run(args, cwd=str(cwd), check=True)


def valid_region_area_fraction(region: np.ndarray, imagersize: np.ndarray) -> float | None:
    if region is None or len(region) < 3:
        return None
    area = cv2.contourArea(region.astype(np.float32))
    image_area = float(imagersize[0] * imagersize[1])
    if image_area <= 0:
        return None
    return float(area / image_area)


def scalar_uncertainty(model: mrcal.cameramodel, q: np.ndarray, distance_m: float | None) -> float:
    lensmodel, intrinsics_data = model.intrinsics()
    ray = mrcal.unproject(q, lensmodel, intrinsics_data, normalize=True)
    if distance_m is None:
        covariance = mrcal.projection_uncertainty(ray, model, atinfinity=True)
    else:
        covariance = mrcal.projection_uncertainty(ray * distance_m, model)
    return float(mrcal.worst_direction_stdev(covariance))


def format_distance_label(distance: float) -> str:
    if float(distance).is_integer():
        return str(int(distance))
    return str(distance).replace(".", "p")


def compute_analysis(model_path: Path) -> dict:
    model = mrcal.cameramodel(str(model_path))
    lensmodel, intrinsics_data = model.intrinsics()
    imagersize = np.asarray(model.imagersize(), dtype=float)
    optimization_inputs = model.optimization_inputs()

    observations = optimization_inputs["observations_board"]
    imagepaths = optimization_inputs["imagepaths"]
    residuals = mrcal.residuals_chessboard(optimization_inputs)
    residual_magnitudes = np.linalg.norm(residuals, axis=1)
    residual_component_rms = float(math.sqrt(np.mean(residuals * residuals)))

    valid_mask = observations[..., 2] > 0
    valid_points = observations[..., :2][valid_mask]
    coverage = {}
    if valid_points.size:
        hull = cv2.convexHull(valid_points.astype(np.float32))
        coverage = {
            "xmin_px": float(valid_points[:, 0].min()),
            "xmax_px": float(valid_points[:, 0].max()),
            "ymin_px": float(valid_points[:, 1].min()),
            "ymax_px": float(valid_points[:, 1].max()),
            "convex_hull_area_fraction": float(
                cv2.contourArea(hull) / float(imagersize[0] * imagersize[1])
            ),
        }

    per_image = []
    offset = 0
    for path, mask in zip(imagepaths, valid_mask):
        count = int(mask.sum())
        current = residual_magnitudes[offset : offset + count]
        offset += count
        per_image.append(
            {
                "image": str(path),
                "point_count": count,
                "mean_residual_px": float(current.mean()) if count else None,
                "rms_residual_px": float(math.sqrt(np.mean(current * current))) if count else None,
                "max_residual_px": float(current.max()) if count else None,
            }
        )

    per_image_sorted = sorted(
        per_image,
        key=lambda item: (-1.0 if item["rms_residual_px"] is None else item["rms_residual_px"]),
        reverse=True,
    )

    center_q = np.array((imagersize[0] / 2.0, imagersize[1] / 2.0))
    if valid_points.size:
        centroid_q = valid_points.mean(axis=0)
    else:
        centroid_q = center_q

    region = model.valid_intrinsics_region()
    uncertainty_summary = {
        "center_px": [float(center_q[0]), float(center_q[1])],
        "observation_centroid_px": [float(centroid_q[0]), float(centroid_q[1])],
        "worst_direction_stdev_px": {
            "center": {
                "0.5m": scalar_uncertainty(model, center_q, 0.5),
                "1.0m": scalar_uncertainty(model, center_q, 1.0),
                "2.0m": scalar_uncertainty(model, center_q, 2.0),
                "infinity": scalar_uncertainty(model, center_q, None),
            },
            "observation_centroid": {
                "0.5m": scalar_uncertainty(model, centroid_q, 0.5),
                "1.0m": scalar_uncertainty(model, centroid_q, 1.0),
                "2.0m": scalar_uncertainty(model, centroid_q, 2.0),
                "infinity": scalar_uncertainty(model, centroid_q, None),
            },
        },
    }

    return {
        "lensmodel": lensmodel,
        "imagersize_px": [int(imagersize[0]), int(imagersize[1])],
        "intrinsics_parameters": {
            "fx_px": float(intrinsics_data[0]),
            "fy_px": float(intrinsics_data[1]),
            "cx_px": float(intrinsics_data[2]),
            "cy_px": float(intrinsics_data[3]),
            "distortion": [float(x) for x in intrinsics_data[4:]],
        },
        "calibration_inputs": {
            "board_observation_count": int(len(imagepaths)),
            "valid_corner_count": int(valid_mask.sum()),
            "calibration_object_spacing_m": float(optimization_inputs["calibration_object_spacing"]),
            "calobject_warp": [float(x) for x in optimization_inputs["calobject_warp"]],
        },
        "residual_statistics_px": {
            "mean_magnitude": float(residual_magnitudes.mean()),
            "median_magnitude": float(np.median(residual_magnitudes)),
            "rms_magnitude": float(math.sqrt(np.mean(residual_magnitudes * residual_magnitudes))),
            "solver_rms_component": residual_component_rms,
            "p95_magnitude": float(np.percentile(residual_magnitudes, 95.0)),
            "max_magnitude": float(residual_magnitudes.max()),
        },
        "worst_images_by_rms": per_image_sorted[:5],
        "observation_coverage": coverage,
        "valid_intrinsics_region": {
            "vertex_count": int(len(region)) if region is not None else 0,
            "area_fraction": valid_region_area_fraction(region, imagersize),
        },
        "projection_uncertainty": uncertainty_summary,
    }


def write_analysis_files(outdir: Path, analysis: dict) -> tuple[Path, Path]:
    json_path = outdir / "analysis.json"
    md_path = outdir / "analysis.md"
    json_path.write_text(json.dumps(analysis, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# Calibration analysis",
        "",
        f"- Lens model: `{analysis['lensmodel']}`",
        f"- Image size: `{analysis['imagersize_px'][0]} x {analysis['imagersize_px'][1]}`",
        f"- Board observations used: `{analysis['calibration_inputs']['board_observation_count']}`",
        f"- Valid corners used: `{analysis['calibration_inputs']['valid_corner_count']}`",
        f"- Solver-style RMS: `{analysis['residual_statistics_px']['solver_rms_component']:.3f} px`",
        f"- Residual magnitude RMS: `{analysis['residual_statistics_px']['rms_magnitude']:.3f} px`",
        f"- Residual magnitude p95: `{analysis['residual_statistics_px']['p95_magnitude']:.3f} px`",
        f"- Residual magnitude max: `{analysis['residual_statistics_px']['max_magnitude']:.3f} px`",
        "",
        "## Intrinsics",
        "",
        f"- `fx={analysis['intrinsics_parameters']['fx_px']:.3f} px`",
        f"- `fy={analysis['intrinsics_parameters']['fy_px']:.3f} px`",
        f"- `cx={analysis['intrinsics_parameters']['cx_px']:.3f} px`",
        f"- `cy={analysis['intrinsics_parameters']['cy_px']:.3f} px`",
        f"- Distortion: `{analysis['intrinsics_parameters']['distortion']}`",
        "",
        "## Coverage",
        "",
    ]

    coverage = analysis["observation_coverage"]
    if coverage:
        lines.extend(
            [
                f"- Observed x-range: `{coverage['xmin_px']:.1f} .. {coverage['xmax_px']:.1f} px`",
                f"- Observed y-range: `{coverage['ymin_px']:.1f} .. {coverage['ymax_px']:.1f} px`",
                f"- Observed convex-hull area: `{coverage['convex_hull_area_fraction'] * 100.0:.1f}%` of imager",
            ]
        )
    else:
        lines.append("- No valid observation coverage data")

    lines.extend(["", "## Worst Images By RMS", ""])
    for item in analysis["worst_images_by_rms"]:
        lines.append(
            f"- `{Path(item['image']).name}`: rms `{item['rms_residual_px']:.3f} px`, max `{item['max_residual_px']:.3f} px`, points `{item['point_count']}`"
        )

    uncertainty = analysis["projection_uncertainty"]["worst_direction_stdev_px"]
    lines.extend(["", "## Projection Uncertainty", ""])
    lines.append(
        f"- Center ray: `0.5m={uncertainty['center']['0.5m']:.4f}px`, `1.0m={uncertainty['center']['1.0m']:.4f}px`, `2.0m={uncertainty['center']['2.0m']:.4f}px`, `inf={uncertainty['center']['infinity']:.4f}px`"
    )
    lines.append(
        f"- Observation-centroid ray: `0.5m={uncertainty['observation_centroid']['0.5m']:.4f}px`, `1.0m={uncertainty['observation_centroid']['1.0m']:.4f}px`, `2.0m={uncertainty['observation_centroid']['2.0m']:.4f}px`, `inf={uncertainty['observation_centroid']['infinity']:.4f}px`"
    )

    valid_region = analysis["valid_intrinsics_region"]
    lines.extend(["", "## Valid Intrinsics Region", ""])
    if valid_region["area_fraction"] is None:
        lines.append("- Region is empty")
    else:
        lines.append(
            f"- Area fraction: `{valid_region['area_fraction'] * 100.0:.1f}%` with `{valid_region['vertex_count']}` vertices"
        )

    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, md_path


def generate_plots(
    model_path: Path,
    plot_dir: Path,
    worst_observations: int,
    uncertainty_distances: list[float],
    cwd: Path,
) -> list[str]:
    plot_dir.mkdir(parents=True, exist_ok=True)
    model = mrcal.cameramodel(str(model_path))

    outputs = [
        plot_dir / "residual_vectorfield.svg",
        plot_dir / "residual_directions.svg",
        plot_dir / "residual_regional.abs_mean_.svg",
        plot_dir / "residual_regional.stdev.svg",
        plot_dir / "residual_regional.count.svg",
        plot_dir / "residual_magnitudes.svg",
        plot_dir / "residual_histogram.svg",
        plot_dir / "distortion_heatmap.svg",
        plot_dir / "distortion_vectorfield.svg",
        plot_dir / "distortion_radial.svg",
        plot_dir / "geometry.svg",
        plot_dir / "projection_uncertainty_infinity.svg",
    ]

    region = model.valid_intrinsics_region()
    if region is not None and len(region) >= 3:
        outputs.append(plot_dir / "valid_intrinsics_region.svg")

    run_command(
        [
            "mrcal-show-residuals",
            "--vectorfield",
            "--valid-intrinsics-region",
            "--vectorscale",
            "50",
            "--hardcopy",
            str(plot_dir / "residual_vectorfield.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-residuals",
            "--directions",
            "--valid-intrinsics-region",
            "--unset",
            "key",
            "--hardcopy",
            str(plot_dir / "residual_directions.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-residuals",
            "--regional",
            "--gridn",
            "20",
            "12",
            "--hardcopy",
            str(plot_dir / "residual_regional.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-distortion-off-pinhole",
            "--hardcopy",
            str(plot_dir / "distortion_heatmap.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-distortion-off-pinhole",
            "--vectorfield",
            "--vectorscale",
            "20",
            "--hardcopy",
            str(plot_dir / "distortion_vectorfield.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-distortion-off-pinhole",
            "--radial",
            "--show-fisheye-projections",
            "--hardcopy",
            str(plot_dir / "distortion_radial.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-geometry",
            "--show-calobjects-thiscamera",
            "--hardcopy",
            str(plot_dir / "geometry.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    if region is not None and len(region) >= 3:
        run_command(
            [
                "mrcal-show-valid-intrinsics-region",
                "--hardcopy",
                str(plot_dir / "valid_intrinsics_region.svg"),
                str(model_path),
            ],
            cwd=cwd,
        )
    for observation_index in range(worst_observations):
        worst_path = plot_dir / f"worst_{observation_index:02d}.svg"
        outputs.append(worst_path)
        run_command(
            [
                "mrcal-show-residuals-board-observation",
                "--from-worst",
                "--vectorscale",
                "100",
                "--circlescale",
                "0.5",
                "--set",
                "cbrange [0:3]",
                "--hardcopy",
                str(worst_path),
                str(model_path),
                str(observation_index),
            ],
            cwd=cwd,
        )
    for distance in uncertainty_distances:
        label = format_distance_label(distance)
        uncertainty_path = plot_dir / f"projection_uncertainty_{label}m.svg"
        outputs.append(uncertainty_path)
        run_command(
            [
                "mrcal-show-projection-uncertainty",
                "--observations",
                "--valid-intrinsics-region",
                "--distance",
                str(distance),
                "--hardcopy",
                str(uncertainty_path),
                str(model_path),
            ],
            cwd=cwd,
        )
    run_command(
        [
            "mrcal-show-projection-uncertainty",
            "--observations",
            "--valid-intrinsics-region",
            "--hardcopy",
            str(plot_dir / "projection_uncertainty_infinity.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    for location, label in (("center", "center"), ("centroid", "centroid")):
        uncertainty_path = plot_dir / f"projection_uncertainty_vs_distance_{label}.svg"
        outputs.append(uncertainty_path)
        run_command(
            [
                "mrcal-show-projection-uncertainty",
                "--vs-distance-at",
                location,
                "--hardcopy",
                str(uncertainty_path),
                str(model_path),
            ],
            cwd=cwd,
        )
    run_command(
        [
            "mrcal-show-residuals",
            "--magnitudes",
            "--valid-intrinsics-region",
            "--set",
            "cbrange [0:3]",
            "--hardcopy",
            str(plot_dir / "residual_magnitudes.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )
    run_command(
        [
            "mrcal-show-residuals",
            "--histogram",
            "--set",
            "xrange [-3:3]",
            "--unset",
            "key",
            "--binwidth",
            "0.05",
            "--hardcopy",
            str(plot_dir / "residual_histogram.svg"),
            str(model_path),
        ],
        cwd=cwd,
    )

    return [str(path) for path in outputs]


def main() -> int:
    args = parse_args()
    session = args.session.resolve()
    outdir = (args.outdir or (session / "calibration" / "mrcal")).resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    use_mrgingham = (
        args.detector == "mrgingham"
        or (args.detector == "auto" and args.checkerboard_cols == args.checkerboard_rows)
    )
    if use_mrgingham and args.checkerboard_cols != args.checkerboard_rows:
        raise RuntimeError("mrgingham cannot be used with a non-square checkerboard in the installed local mrcal build")

    default_cache_name = "corners.vnl" if use_mrgingham else "corners-opencv.vnl"
    corners_cache = (args.corners_cache or (outdir / default_cache_name)).resolve()
    plot_dir = (args.plot_dir or (outdir / "plots")).resolve()
    image_pattern = str(Path("images") / args.image_glob)

    cache_summary: dict
    if use_mrgingham:
        print(f"+ using native mrgingham corner detection via mrcal", flush=True)
        cache_summary = {
            "output": str(corners_cache),
            "path_mode": "relative",
        }
    else:
        print(f"+ building OpenCV corners cache at {corners_cache}", flush=True)
        cache_summary = build_corners_cache(
            session,
            checkerboard_cols=args.checkerboard_cols,
            checkerboard_rows=args.checkerboard_rows,
            image_glob=args.image_glob,
            output=corners_cache,
            path_mode="relative",
        )

    calibrate_command = [
        "mrcal-calibrate-cameras",
        "--corners-cache",
        str(corners_cache),
        "--lensmodel",
        args.lensmodel,
        "--focal",
        str(args.focal),
        "--object-spacing",
        str(args.square_size),
        "--object-width-n",
        str(args.checkerboard_cols),
        "--object-height-n",
        str(args.checkerboard_rows),
        "--jobs",
        str(args.jobs),
        "--outdir",
        str(outdir),
    ]
    if not use_mrgingham:
        calibrate_command.extend(
            [
                "--image-path-prefix",
                str(session),
            ]
        )
    calibrate_command.append(image_pattern)
    run_command(calibrate_command, cwd=session)

    if use_mrgingham:
        cache_summary.update(
            summarize_corners_cache(
                corners_cache,
                checkerboard_cols=args.checkerboard_cols,
                checkerboard_rows=args.checkerboard_rows,
            )
        )

    model_paths = sorted(outdir.glob("*.cameramodel"))
    if len(model_paths) != 1:
        raise RuntimeError(f"expected exactly 1 cameramodel in {outdir}, found {len(model_paths)}")
    model_path = model_paths[0]

    plot_outputs: list[str] = []
    if not args.skip_plots:
        plot_outputs = generate_plots(
            model_path,
            plot_dir,
            args.worst_observations,
            list(args.uncertainty_distances),
            session,
        )

    analysis = compute_analysis(model_path)
    analysis_json_path, analysis_md_path = write_analysis_files(outdir, analysis)

    summary = {
        "session": str(session),
        "corners_cache": cache_summary["output"],
        "detector": "mrgingham" if use_mrgingham else "opencv",
        "usable_image_count": cache_summary["usable_image_count"],
        "rejected_image_count": cache_summary["rejected_image_count"],
        "model": str(model_path),
        "plot_dir": str(plot_dir),
        "plots": plot_outputs,
        "analysis_json": str(analysis_json_path),
        "analysis_md": str(analysis_md_path),
    }

    summary_path = outdir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"wrote {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
