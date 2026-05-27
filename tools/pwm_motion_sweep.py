#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import math
import statistics
import subprocess
import time
from pathlib import Path


def parse_csv_ints(text: str) -> list[int]:
    values = []
    for part in text.split(","):
        part = part.strip()
        if part:
            values.append(int(part))
    if not values:
        raise argparse.ArgumentTypeError("expected at least one integer")
    return values


def summarize_pose_jsonl(path: Path, name: str) -> dict:
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("matched_expected") and row.get("pose"):
            rows.append(row)

    if not rows:
        return {"name": name, "matched": 0}

    xs = [row["pose"]["tvec_m"][0] for row in rows]
    ys = [row["pose"]["tvec_m"][1] for row in rows]
    zs = [row["pose"]["tvec_m"][2] for row in rows]
    rms = [row["pose"]["reprojection_rmse_px"] for row in rows]
    edges = [row["detections"][0]["mean_edge_px"] for row in rows if row.get("detections")]
    x_m = statistics.median(xs)
    y_m = statistics.median(ys)
    z_m = statistics.median(zs)
    return {
        "name": name,
        "matched": len(rows),
        "median_tvec_m": [x_m, y_m, z_m],
        "bearing_deg": math.degrees(math.atan2(x_m, z_m)),
        "median_rmse_px": statistics.median(rms),
        "median_edge_px": statistics.median(edges),
    }


def run_command(command: list[str], stdout_path: Path, stderr_path: Path, timeout_s: float) -> None:
    proc = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_s,
    )
    stdout_path.write_text(proc.stdout or "")
    stderr_path.write_text(proc.stderr or "")
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(command)}")


def capture_pose(args: argparse.Namespace, root: Path, name: str) -> dict:
    jsonl_path = root / f"{name}.jsonl"
    stderr_path = root / f"{name}.stderr"
    image_path = root / f"{name}.jpg"
    command = [
        "python3",
        "tools/apriltag_pose.py",
        "--frames",
        str(args.frames),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--focus-absolute",
        str(args.focus_absolute),
        "--jsonl",
        "--save-debug",
        str(image_path),
    ]
    if args.low_light_preset:
        command.append("--low-light-preset")

    try:
        run_command(command, jsonl_path, stderr_path, timeout_s=args.pose_timeout_s)
    except RuntimeError as exc:
        return {"name": name, "matched": 0, "error": str(exc)}
    return summarize_pose_jsonl(jsonl_path, name)


def run_pwm(args: argparse.Namespace, root: Path, name: str, seq: int, milli: int, duration_ms: int) -> None:
    read_seconds = max(args.min_read_s, duration_ms / 1000.0 + args.read_tail_s)
    command = [
        "python3",
        "tools/minimal_rover_serial.py",
        "--port",
        args.port,
        "--seq",
        str(seq),
        "--read-seconds",
        f"{read_seconds:.3f}",
        "pwm",
        "--milli",
        str(milli),
        "--duration-ms",
        str(duration_ms),
    ]
    run_command(command, root / f"{name}.txt", root / f"{name}.stderr", timeout_s=read_seconds + 4.0)
    time.sleep(args.settle_s)


def run_pair(args: argparse.Namespace, root: Path, seq: int, pwm: int, duration_ms: int, repeat: int) -> dict:
    prefix = f"pwm{pwm:04d}_t{duration_ms:04d}_r{repeat:02d}"
    before = capture_pose(args, root, f"{prefix}_00_before")
    if not before.get("matched"):
        return {"pwm_milli": pwm, "duration_ms": duration_ms, "repeat": repeat, "status": "no_start_pose"}

    run_pwm(args, root, f"{prefix}_01_forward_cmd", seq, pwm, duration_ms)
    after_forward = capture_pose(args, root, f"{prefix}_02_after_forward")

    run_pwm(args, root, f"{prefix}_03_reverse_cmd", seq + 1, -pwm, duration_ms)
    after_reverse = capture_pose(args, root, f"{prefix}_04_after_reverse")

    status = "ok"
    if not after_forward.get("matched") or not after_reverse.get("matched"):
        status = "pose_failed"

    result = {
        "pwm_milli": pwm,
        "duration_ms": duration_ms,
        "repeat": repeat,
        "status": status,
        "before": before,
        "after_forward": after_forward,
        "after_reverse": after_reverse,
    }
    if after_forward.get("matched"):
        result["forward_delta_m"] = before["median_tvec_m"][2] - after_forward["median_tvec_m"][2]
        result["forward_bearing_delta_deg"] = after_forward["bearing_deg"] - before["bearing_deg"]
    if after_reverse.get("matched"):
        result["return_error_m"] = after_reverse["median_tvec_m"][2] - before["median_tvec_m"][2]
    if after_forward.get("matched") and after_reverse.get("matched"):
        result["reverse_delta_m"] = after_reverse["median_tvec_m"][2] - after_forward["median_tvec_m"][2]
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sweep raw PWM pulses and measure AprilTag range deltas.")
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--pwm", type=parse_csv_ints, default=[160, 180, 200, 220, 240])
    parser.add_argument("--duration-ms", type=parse_csv_ints, default=[200, 300, 400, 500])
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--seq-start", type=int, default=300)
    parser.add_argument("--frames", type=int, default=25)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--focus-absolute", type=int, default=350)
    parser.add_argument("--low-light-preset", action="store_true")
    parser.add_argument("--settle-s", type=float, default=0.25)
    parser.add_argument("--min-read-s", type=float, default=0.7)
    parser.add_argument("--read-tail-s", type=float, default=0.45)
    parser.add_argument("--pose-timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--continue-on-pose-failure",
        action="store_true",
        help="Keep sweeping after a missing or failed AprilTag pose capture.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.repeat < 1:
        raise SystemExit("--repeat must be >= 1")

    if args.out_dir is None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.out_dir = Path("data/minimal_motion_specs") / f"{stamp}_pwm_sweep"
    args.out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    summary_path = args.out_dir / "summary.json"
    seq = args.seq_start
    for pwm in args.pwm:
        for duration_ms in args.duration_ms:
            for repeat in range(1, args.repeat + 1):
                result = run_pair(args, args.out_dir, seq, pwm, duration_ms, repeat)
                results.append(result)
                summary_path.write_text(json.dumps(results, indent=2))
                print(json.dumps(result, separators=(",", ":")), flush=True)
                seq += 2
                if result["status"] != "ok" and not args.continue_on_pose_failure:
                    print(f"summary_path={summary_path}")
                    return 1

    print(f"summary_path={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
