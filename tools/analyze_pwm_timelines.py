#!/usr/bin/env python3
"""Analyze continuous PWM timeline logs into motion calibration tables."""

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


def median(values: list[float | None]) -> float | None:
    finite = [v for v in values if v is not None and math.isfinite(v)]
    if not finite:
        return None
    return statistics.median(finite)


def pose_from_tag_record(record: dict[str, Any]) -> dict[str, Any] | None:
    pose = record.get("record", {}).get("pose")
    if not pose:
        return None
    euler = pose.get("euler_xyz_deg") or [None, None, None]
    return {
        "t": record["t_s"],
        "x": pose.get("lateral_m"),
        "z": pose.get("range_m"),
        "yaw": euler[2],
        "rmse": pose.get("reprojection_rmse_px"),
    }


def summarize_poses(poses: list[dict[str, Any]]) -> dict[str, float | None] | None:
    if len(poses) < 2:
        return None
    return {key: median([pose.get(key) for pose in poses]) for key in ["x", "z", "yaw", "rmse"]}


def pose_window(
    tags: list[dict[str, Any]],
    start_t: float,
    end_t: float,
) -> tuple[dict[str, float | None] | None, int]:
    poses = [pose for pose in tags if start_t <= pose["t"] <= end_t]
    return summarize_poses(poses), len(poses)


def yaw_delta_deg(before: float | None, after: float | None) -> float | None:
    if before is None or after is None:
        return None
    delta = after - before
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def load_records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def run_summary(path: Path, records: list[dict[str, Any]]) -> dict[str, Any]:
    start = next(
        (record for record in records if record.get("kind") == "run" and record.get("event") == "start"),
        {},
    )
    args = start.get("args", {})
    return {
        "file": path.name,
        "commands": sum(
            1
            for record in records
            if record.get("kind") == "command"
            and record.get("event") == "start"
            and "left_milli" in record
        ),
        "tags": sum(1 for record in records if record.get("kind") == "tag"),
        "complete": any(
            record.get("kind") == "run" and record.get("event") == "complete" for record in records
        ),
        "exposure": args.get("exposure_time_absolute"),
    }


def command_rows(path: Path, records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    tags = [
        pose
        for record in records
        if record.get("kind") == "tag"
        for pose in [pose_from_tag_record(record)]
        if pose is not None
    ]
    commands = [
        record
        for record in records
        if record.get("kind") == "command"
        and record.get("event") == "start"
        and "left_milli" in record
    ]
    settle_end = {
        record["seq"]: record
        for record in records
        if record.get("kind") == "settle" and record.get("event") == "end"
    }

    rows = []
    for command in commands:
        if command["left_milli"] != command["right_milli"]:
            continue

        before, before_count = pose_window(tags, command["t_s"] - 0.45, command["t_s"])
        end = settle_end.get(command["seq"])
        if before is None or end is None:
            continue
        after, after_count = pose_window(tags, end["t_s"] - 0.45, end["t_s"] + 0.15)
        if after is None:
            continue

        pwm = int(command["left_milli"])
        dz_mm = (after["z"] - before["z"]) * 1000
        dx_mm = (after["x"] - before["x"]) * 1000
        axial_mm = -math.copysign(1, pwm) * dz_mm
        rows.append(
            {
                "file": path.name,
                "seq": command["seq"],
                "index": command.get("index"),
                "pwm": pwm,
                "abs_pwm": abs(pwm),
                "sign": 1 if pwm > 0 else -1,
                "duration_ms": int(command["duration_ms"]),
                "axial_mm": axial_mm,
                "dz_mm": dz_mm,
                "dx_mm": dx_mm,
                "yaw_deg": yaw_delta_deg(before["yaw"], after["yaw"]),
                "before_count": before_count,
                "after_count": after_count,
            }
        )
    return rows


def linear_fit(rows: list[dict[str, Any]]) -> tuple[float, float, float, float] | None:
    xs = [row["duration_ms"] for row in rows]
    ys = [row["axial_mm"] for row in rows]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom == 0:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
    intercept = mean_y - slope * mean_x
    rms = math.sqrt(sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys)) / len(xs))
    startup_ms = -intercept / slope if slope else float("nan")
    return slope, intercept, rms, startup_ms


def format_value(value: float | None, width: int = 7, precision: int = 1) -> str:
    if value is None:
        return " " * (width - 3) + "nan"
    return f"{value:{width}.{precision}f}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze PWM timeline JSONL files.")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("data/minimal_motion_specs/pwm_timelines"),
        help="Directory containing *_timeline.jsonl files.",
    )
    parser.add_argument(
        "--since",
        default="20260523_024520",
        help="Only include timeline files whose name is >= this prefix.",
    )
    parser.add_argument("--min-abs-pwm", type=int, default=250)
    parser.add_argument("--targets-mm", default="50,100,150,200")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    paths = [
        path
        for path in sorted(args.root.glob("*_timeline.jsonl"))
        if path.name >= args.since
    ]
    targets = [float(item) for item in args.targets_mm.split(",") if item.strip()]

    summaries = []
    rows = []
    for path in paths:
        records = load_records(path)
        summaries.append(run_summary(path, records))
        rows.extend(command_rows(path, records))

    clean_rows = [
        row
        for row in rows
        if row["before_count"] >= 2
        and row["after_count"] >= 2
        and row["axial_mm"] > 0
        and row["axial_mm"] < 700
    ]

    print("Runs")
    print("file commands tags complete exposure")
    for summary in summaries:
        print(
            f"{summary['file']} {summary['commands']:3d} {summary['tags']:4d} "
            f"{str(summary['complete']):>5s} {summary['exposure']}"
        )
    print(f"\nRows raw={len(rows)} clean={len(clean_rows)}")

    by_fit: dict[tuple[int, int], list[dict[str, Any]]] = defaultdict(list)
    by_table: dict[tuple[int, int, int], list[dict[str, Any]]] = defaultdict(list)
    for row in clean_rows:
        if row["abs_pwm"] < args.min_abs_pwm:
            continue
        by_fit[(row["abs_pwm"], row["sign"])].append(row)
        by_table[(row["abs_pwm"], row["duration_ms"], row["sign"])].append(row)

    print("\nFit: axial_mm = slope_mm_per_ms * duration_ms + intercept_mm")
    print("pwm dir n dur_range speed_mm_s intercept startup_ms rms_mm med_dx_mm med_abs_yaw")
    fit_by_pwm: dict[int, list[tuple[float, float, float, float]]] = defaultdict(list)
    for (abs_pwm, sign), group in sorted(by_fit.items()):
        if len(group) < 4:
            continue
        fit = linear_fit(group)
        if fit is None:
            continue
        slope, intercept, rms, startup_ms = fit
        fit_by_pwm[abs_pwm].append(fit)
        direction = "fwd" if sign > 0 else "rev"
        yaw = median([abs(row["yaw_deg"] or 0.0) for row in group])
        dx = median([row["dx_mm"] for row in group])
        print(
            f"{abs_pwm:3d} {direction:3s} {len(group):2d} "
            f"{min(row['duration_ms'] for row in group):4d}-{max(row['duration_ms'] for row in group):4d} "
            f"{slope * 1000:10.1f} {intercept:9.1f} {startup_ms:10.1f} "
            f"{rms:6.1f} {dx:9.1f} {yaw:11.2f}"
        )

    print("\nMedian axial distance by command")
    print("pwm dur fwd_mm rev_mm fwd_dx rev_dx")
    table_keys = sorted({(key[0], key[1]) for key in by_table})
    for abs_pwm, duration_ms in table_keys:
        fwd = by_table.get((abs_pwm, duration_ms, 1), [])
        rev = by_table.get((abs_pwm, duration_ms, -1), [])
        print(
            f"{abs_pwm:3d} {duration_ms:4d} "
            f"{format_value(median([row['axial_mm'] for row in fwd]))} "
            f"{format_value(median([row['axial_mm'] for row in rev]))} "
            f"{format_value(median([row['dx_mm'] for row in fwd]))} "
            f"{format_value(median([row['dx_mm'] for row in rev]))}"
        )

    print("\nEstimated duration ms from average forward/reverse fit")
    print("pwm " + " ".join(f"{int(target):>8d}mm" for target in targets))
    for abs_pwm in sorted(fit_by_pwm):
        fits = fit_by_pwm[abs_pwm]
        slope = sum(fit[0] for fit in fits) / len(fits)
        intercept = sum(fit[1] for fit in fits) / len(fits)
        durations = [(target - intercept) / slope for target in targets]
        print(f"{abs_pwm:3d} " + " ".join(f"{duration:10.0f}" for duration in durations))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
