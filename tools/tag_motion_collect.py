#!/usr/bin/env python3
import argparse
import csv
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


SCHEMA_VERSION = "wave_rover.tag_motion_collection/v1"
DEFAULT_OUTPUT_ROOT = Path("data/tag_motion_batches")


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect structured AprilTag + IMU motion datasets in repeatable batches. "
            "Defaults favor the most useful current dataset: linear and turn pairs together."
        )
    )
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--model", type=Path, help="Optional mrcal model path.")
    parser.add_argument("--family", default="tag16h5")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.034)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--autofocus", action="store_true")
    parser.add_argument("--focus-absolute", type=int, default=350)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--program",
        choices=("linear", "rot", "both"),
        default="both",
        help="Which structured motion set to collect. Default is both because it best constrains later fitting.",
    )
    parser.add_argument("--linear-pwm", type=float, default=0.16)
    parser.add_argument("--linear-duration", type=float, default=0.35)
    parser.add_argument(
        "--linear-repeats",
        type=int,
        default=4,
        help="Number of forward/backward pairs to collect.",
    )
    parser.add_argument("--rot-pwm", type=float, default=0.35)
    parser.add_argument("--rot-duration", type=float, default=0.10)
    parser.add_argument(
        "--rot-repeats",
        type=int,
        default=4,
        help="Number of CCW/CW pairs to collect.",
    )
    parser.add_argument("--pre-seconds", type=float, default=1.5)
    parser.add_argument("--post-seconds", type=float, default=1.5)
    parser.add_argument("--sample-period", type=float, default=0.08)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=0.75,
        help="Extra pause between runs after the probe's own stop burst.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    parser.add_argument(
        "--target-pose",
        type=Path,
        default=Path("config/auto_docking/docked_tag_pose.json"),
    )
    parser.add_argument(
        "--name",
        default="batch",
        help="Short batch label appended to the timestamped output directory.",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="Freeform note stored in the batch metadata for later analysis.",
    )
    parser.add_argument(
        "--stop-on-loss",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Stop the batch when pre/post tag visibility drops below the configured minimums.",
    )
    parser.add_argument("--min-pre-detected", type=int, default=2)
    parser.add_argument("--min-post-detected", type=int, default=2)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned run sequence without executing probes.",
    )
    return parser.parse_args()


def build_plan(args: argparse.Namespace) -> list[dict]:
    plan: list[dict] = []
    run_index = 1

    def add_run(group: str, repeat_index: int, mode: str, pwm: float, duration_s: float, label: str):
        nonlocal run_index
        plan.append(
            {
                "run_index": run_index,
                "group": group,
                "repeat_index": repeat_index,
                "mode": mode,
                "pwm": pwm,
                "duration_s": duration_s,
                "label": label,
            }
        )
        run_index += 1

    if args.program in ("linear", "both"):
        for repeat_index in range(1, args.linear_repeats + 1):
            add_run("linear", repeat_index, "forward", args.linear_pwm, args.linear_duration, "linear_forward")
            add_run("linear", repeat_index, "backward", args.linear_pwm, args.linear_duration, "linear_backward")

    if args.program in ("rot", "both"):
        for repeat_index in range(1, args.rot_repeats + 1):
            add_run("rot", repeat_index, "turn-ccw", args.rot_pwm, args.rot_duration, "rot_ccw")
            add_run("rot", repeat_index, "turn-cw", args.rot_pwm, args.rot_duration, "rot_cw")

    return plan


def make_batch_id(args: argparse.Namespace) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_{args.program}_{args.name}"


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def append_jsonl(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, separators=(",", ":")) + "\n")


def append_summary_csv(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def batch_metadata(args: argparse.Namespace, batch_id: str, batch_dir: Path, plan: list[dict]) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "batch_id": batch_id,
        "created_at": now_utc(),
        "cwd": str(Path.cwd()),
        "batch_dir": str(batch_dir),
        "notes": args.notes,
        "program": args.program,
        "target_pose": str(args.target_pose),
        "camera": {
            "device": args.camera,
            "model": str(args.model) if args.model else None,
            "family": args.family,
            "id": args.id,
            "tag_size_m": args.tag_size,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "fourcc": args.fourcc,
            "autofocus": args.autofocus,
            "focus_absolute": args.focus_absolute,
        },
        "serial": {
            "port": args.port,
            "baud": args.baud,
            "timeout": args.timeout,
        },
        "timing": {
            "pre_seconds": args.pre_seconds,
            "post_seconds": args.post_seconds,
            "sample_period": args.sample_period,
            "command_period": args.command_period,
            "settle_seconds": args.settle_seconds,
        },
        "defaults": {
            "linear_pwm": args.linear_pwm,
            "linear_duration": args.linear_duration,
            "linear_repeats": args.linear_repeats,
            "rot_pwm": args.rot_pwm,
            "rot_duration": args.rot_duration,
            "rot_repeats": args.rot_repeats,
        },
        "plan": plan,
    }


def build_probe_command(
    args: argparse.Namespace,
    batch_id: str,
    batch_dir: Path,
    spec: dict,
) -> tuple[list[str], Path]:
    run_dir = batch_dir / "runs" / (
        f"{spec['run_index']:03d}_{spec['mode']}_r{spec['repeat_index']:02d}"
    )
    probe_script = Path(__file__).with_name("tag_motion_probe.py").resolve()
    cmd = [
        sys.executable,
        str(probe_script),
        "--camera",
        str(args.camera),
        "--family",
        args.family,
        "--id",
        str(args.id),
        "--tag-size",
        str(args.tag_size),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--fps",
        str(args.fps),
        "--fourcc",
        args.fourcc,
        "--focus-absolute",
        str(args.focus_absolute),
        "--port",
        str(args.port),
        "--baud",
        str(args.baud),
        "--timeout",
        str(args.timeout),
        "--mode",
        spec["mode"],
        "--pwm",
        str(spec["pwm"]),
        "--duration",
        str(spec["duration_s"]),
        "--pre-seconds",
        str(args.pre_seconds),
        "--post-seconds",
        str(args.post_seconds),
        "--sample-period",
        str(args.sample_period),
        "--command-period",
        str(args.command_period),
        "--target-pose",
        str(args.target_pose),
        "--name",
        spec["label"],
        "--output-dir",
        str(run_dir),
        "--batch-id",
        batch_id,
        "--sequence-name",
        args.program,
        "--run-index",
        str(spec["run_index"]),
    ]
    if args.model is not None:
        cmd.extend(["--model", str(args.model)])
    if args.autofocus:
        cmd.append("--autofocus")
    return cmd, run_dir


def summarize_run(batch_id: str, session_path: Path, spec: dict) -> dict:
    session = json.loads(session_path.read_text())
    pre = session["pre_summary"]
    motion = session["motion_summary"]
    post = session["post_summary"]
    derived = session["derived"]
    delta = derived.get("post_minus_pre_translation_m") or [None, None, None]
    pre_tag = pre.get("tag_translation_median_m") or [None, None, None]
    post_tag = post.get("tag_translation_median_m") or [None, None, None]
    return {
        "batch_id": batch_id,
        "run_index": spec["run_index"],
        "group": spec["group"],
        "repeat_index": spec["repeat_index"],
        "mode": spec["mode"],
        "pwm": spec["pwm"],
        "duration_s": spec["duration_s"],
        "run_dir": str(session_path.parent),
        "captured_at": session.get("captured_at"),
        "pre_tag_detected_count": pre.get("tag_detected_count"),
        "motion_tag_detected_count": motion.get("tag_detected_count"),
        "post_tag_detected_count": post.get("tag_detected_count"),
        "pre_tag_x_m": pre_tag[0],
        "pre_tag_y_m": pre_tag[1],
        "pre_tag_z_m": pre_tag[2],
        "post_tag_x_m": post_tag[0],
        "post_tag_y_m": post_tag[1],
        "post_tag_z_m": post_tag[2],
        "delta_tag_x_m": delta[0],
        "delta_tag_y_m": delta[1],
        "delta_tag_z_m": delta[2],
        "motion_yaw_deg_mean": motion.get("yaw_deg_mean"),
        "motion_gyro_integrated_deg": motion.get("gyro_integrated_deg"),
        "motion_rmse_px_mean": motion.get("tag_reprojection_rmse_px_mean"),
    }


def run_probe(cmd: list[str], run_dir: Path) -> subprocess.CompletedProcess:
    run_dir.mkdir(parents=True, exist_ok=True)
    write_json(
        run_dir / "probe_command.json",
        {
            "invoked_at": now_utc(),
            "argv": cmd,
        },
    )
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    (run_dir / "probe_stdout.txt").write_text(result.stdout)
    (run_dir / "probe_stderr.txt").write_text(result.stderr)
    return result


def main() -> int:
    args = parse_args()
    plan = build_plan(args)
    if not plan:
        print("No runs planned.", file=sys.stderr)
        return 2

    batch_id = make_batch_id(args)
    batch_dir = args.output_root / batch_id
    meta = batch_metadata(args, batch_id, batch_dir, plan)
    write_json(batch_dir / "batch.json", meta)

    print(f"batch={batch_dir}")
    for spec in plan:
        print(
            f"plan run={spec['run_index']:02d} "
            f"group={spec['group']} repeat={spec['repeat_index']} "
            f"mode={spec['mode']} pwm={spec['pwm']:.3f} duration={spec['duration_s']:.3f}"
        )
    if args.dry_run:
        return 0

    batch_index_path = args.output_root / "collection_index.jsonl"
    append_jsonl(
        batch_index_path,
        {
            "schema_version": SCHEMA_VERSION,
            "record_type": "batch_start",
            "recorded_at": now_utc(),
            "batch_id": batch_id,
            "batch_dir": str(batch_dir),
            "program": args.program,
            "notes": args.notes,
        },
    )

    failure_reason = None
    completed = 0
    for spec in plan:
        cmd, run_dir = build_probe_command(args, batch_id, batch_dir, spec)
        print(
            f"running {spec['run_index']:02d}/{len(plan)} "
            f"{spec['mode']} pwm={spec['pwm']:.3f} duration={spec['duration_s']:.3f}"
        )
        result = run_probe(cmd, run_dir)
        session_path = run_dir / "session.json"
        if result.returncode != 0 or not session_path.exists():
            failure_reason = (
                f"probe_failed run_index={spec['run_index']} returncode={result.returncode}"
            )
            append_jsonl(
                batch_dir / "runs.jsonl",
                {
                    "record_type": "run_failed",
                    "recorded_at": now_utc(),
                    "run_index": spec["run_index"],
                    "mode": spec["mode"],
                    "run_dir": str(run_dir),
                    "returncode": result.returncode,
                },
            )
            break

        summary = summarize_run(batch_id, session_path, spec)
        append_jsonl(
            batch_dir / "runs.jsonl",
            {
                "record_type": "run_complete",
                "recorded_at": now_utc(),
                **summary,
            },
        )
        append_summary_csv(batch_dir / "summary.csv", summary)
        completed += 1

        print(
            f"saved={summary['run_dir']} "
            f"delta_z={summary['delta_tag_z_m']} "
            f"pre_tag={summary['pre_tag_detected_count']} "
            f"post_tag={summary['post_tag_detected_count']}"
        )

        if args.stop_on_loss:
            pre_count = int(summary["pre_tag_detected_count"] or 0)
            post_count = int(summary["post_tag_detected_count"] or 0)
            if pre_count < args.min_pre_detected or post_count < args.min_post_detected:
                failure_reason = (
                    f"tag_visibility_drop run_index={spec['run_index']} "
                    f"pre={pre_count} post={post_count}"
                )
                print(f"stopping: {failure_reason}", file=sys.stderr)
                break

        if args.settle_seconds > 0.0:
            time.sleep(args.settle_seconds)

    completion = {
        "schema_version": SCHEMA_VERSION,
        "record_type": "batch_end",
        "recorded_at": now_utc(),
        "batch_id": batch_id,
        "batch_dir": str(batch_dir),
        "planned_runs": len(plan),
        "completed_runs": completed,
        "stopped_early": failure_reason is not None,
        "failure_reason": failure_reason,
    }
    append_jsonl(batch_dir / "runs.jsonl", completion)
    append_jsonl(batch_index_path, completion)
    write_json(batch_dir / "completion.json", completion)
    return 0 if failure_reason is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
