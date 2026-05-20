#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial import SerialException


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 1
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.setDTR(False)
    ser.setRTS(False)
    return ser


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def clamp_pwm(value: float) -> float:
    return max(-0.5, min(0.5, value))


def send_json(ser: serial.Serial, payload: dict) -> None:
    msg = json.dumps(payload, separators=(",", ":")) + "\n"
    ser.write(msg.encode("utf-8"))
    ser.flush()


def wait_until_ready(ser: serial.Serial, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    quiet_deadline = time.monotonic() + 1.0
    saw_boot_output = False
    while time.monotonic() < deadline:
        try:
            raw = ser.readline()
        except SerialException:
            return
        if not raw:
            if not saw_boot_output and time.monotonic() >= quiet_deadline:
                return
            continue
        text = raw.decode("utf-8", "replace").strip()
        if not text:
            continue
        saw_boot_output = True
        if "UGV started." in text:
            return


def drain_serial(ser: serial.Serial, seconds: float) -> list[dict]:
    deadline = time.monotonic() + seconds
    packets = []
    while time.monotonic() < deadline:
        try:
            raw = ser.readline()
        except SerialException:
            break
        if not raw:
            continue
        text = raw.decode("utf-8", "replace").strip()
        if not text:
            continue
        try:
            packets.append(json.loads(text))
        except json.JSONDecodeError:
            packets.append({"_raw": text})
    return packets


def numeric_items(packet: dict) -> dict[str, float]:
    values = {}
    for key, value in packet.items():
        if isinstance(value, bool):
            continue
        if isinstance(value, (int, float)):
            values[key] = float(value)
    return values


def lower_numeric_map(packet: dict) -> dict[str, float]:
    return {key.lower(): value for key, value in numeric_items(packet).items()}


def score_imu_packet(packet: dict) -> int:
    lower = lower_numeric_map(packet)
    score = 0
    if {"r", "p", "y"} <= lower.keys():
        score += 6
    if {"roll", "pitch", "yaw"} <= lower.keys():
        score += 6
    for key in ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz", "yaw", "heading"):
        if key in lower:
            score += 1
    return score


def score_chassis_packet(packet: dict) -> int:
    lower = lower_numeric_map(packet)
    score = 0
    for key in ("v", "voltage", "l", "r", "y", "yaw", "p", "pitch", "roll", "t", "temp"):
        if key in lower:
            score += 1
    return score


def extract_yaw_deg(packet: dict | None) -> tuple[float | None, str | None]:
    if not packet:
        return None, None
    lower = lower_numeric_map(packet)
    if {"roll", "pitch", "yaw"} <= lower.keys():
        return lower["yaw"], "yaw"
    if {"r", "p", "y"} <= lower.keys():
        return lower["y"], "y"
    for key in ("heading", "heading_deg", "yaw"):
        if key in lower:
            return lower[key], key
    return None, None


def extract_gyro_z_dps(packet: dict | None) -> tuple[float | None, str | None]:
    if not packet:
        return None, None
    lower = lower_numeric_map(packet)
    for key in ("gz", "gyro_z", "gyro_z_dps", "wz"):
        if key in lower:
            return lower[key], key
    return None, None


def extract_voltage(packet: dict | None) -> float | None:
    if not packet:
        return None
    lower = lower_numeric_map(packet)
    for key in ("v", "voltage", "battery", "battery_v"):
        if key in lower:
            return lower[key]
    return None


def select_best_packet(packets: list[dict], scorer) -> dict | None:
    best = None
    best_score = 0
    for packet in packets:
        if not isinstance(packet, dict):
            continue
        score = scorer(packet)
        if score > best_score:
            best = packet
            best_score = score
    return best


def wrap_deg(value: float) -> float:
    wrapped = math.fmod(value + 180.0, 360.0)
    if wrapped < 0:
        wrapped += 360.0
    return wrapped - 180.0


def unwrap_deg(values: list[float]) -> list[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    for value in values[1:]:
        step = wrap_deg(value - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + step)
    return unwrapped


def mean_or_none(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def stddev_or_none(values: list[float]) -> float | None:
    if len(values) < 2:
        return None
    return statistics.pstdev(values)


def span_or_none(values: list[float]) -> float | None:
    return max(values) - min(values) if values else None


def integrate_trapezoid(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 2:
        return None
    area = 0.0
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        area += 0.5 * (v0 + v1) * dt
    return area


def motion_command(mode: str, pwm: float) -> tuple[float, float]:
    pwm = clamp_pwm(pwm)
    if mode == "rest":
        return 0.0, 0.0
    if mode == "forward":
        return pwm, pwm
    if mode == "backward":
        return -pwm, -pwm
    if mode in ("turn-left", "turn-ccw"):
        return -pwm, pwm
    if mode in ("turn-right", "turn-cw"):
        return pwm, -pwm
    raise ValueError(f"Unsupported mode: {mode}")


def read_cycle_packets(
    ser: serial.Serial,
    read_window_s: float,
    request_chassis: bool,
) -> tuple[list[dict], dict | None, dict | None]:
    send_json(ser, {"T": 126})
    if request_chassis:
        send_json(ser, {"T": 130})

    packets = drain_serial(ser, read_window_s)
    imu_packet = select_best_packet(packets, score_imu_packet)
    chassis_packet = select_best_packet(packets, score_chassis_packet)
    return packets, imu_packet, chassis_packet


def collect_phase_samples(
    ser: serial.Serial,
    phase_name: str,
    duration_s: float,
    sample_period_s: float,
    command_period_s: float,
    command_lr: tuple[float, float],
    chassis_every: int,
    run_dir: Path,
) -> list[dict]:
    samples = []
    phase_start = time.monotonic()
    next_command_time = phase_start
    iteration = 0

    while True:
        now = time.monotonic()
        elapsed = now - phase_start
        if elapsed >= duration_s:
            break

        if now >= next_command_time:
            send_json(ser, {"T": 1, "L": command_lr[0], "R": command_lr[1]})
            next_command_time += command_period_s

        request_chassis = chassis_every > 0 and (iteration % chassis_every == 0)
        read_window = max(0.02, sample_period_s * 0.75)
        packets, imu_packet, chassis_packet = read_cycle_packets(ser, read_window, request_chassis)
        yaw_deg, yaw_key = extract_yaw_deg(imu_packet) if imu_packet else (None, None)
        if yaw_deg is None:
            yaw_deg, yaw_key = extract_yaw_deg(chassis_packet)
        gyro_z_dps, gyro_key = extract_gyro_z_dps(imu_packet) if imu_packet else (None, None)
        voltage_v = extract_voltage(chassis_packet)

        sample = {
            "phase": phase_name,
            "t_rel_s": round(time.monotonic() - phase_start, 4),
            "timestamp_utc": now_utc(),
            "command": {"L": command_lr[0], "R": command_lr[1]},
            "imu_packet": imu_packet,
            "chassis_packet": chassis_packet,
            "raw_packets": packets,
            "derived": {
                "yaw_deg": yaw_deg,
                "yaw_key": yaw_key,
                "gyro_z_dps": gyro_z_dps,
                "gyro_z_key": gyro_key,
                "voltage_v": voltage_v,
            },
        }
        samples.append(sample)
        iteration += 1

        remaining = sample_period_s - (time.monotonic() - now)
        if remaining > 0:
            time.sleep(remaining)

    write_jsonl(run_dir / f"{phase_name}.jsonl", samples)
    return samples


def write_jsonl(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def send_stop_burst(ser: serial.Serial, count: int, pause_s: float) -> None:
    for _ in range(count):
        try:
            send_json(ser, {"T": 1, "L": 0.0, "R": 0.0})
        except SerialException:
            return
        time.sleep(pause_s)


def summarize_run(
    mode: str,
    pwm: float,
    duration_s: float,
    samples: list[dict],
) -> dict:
    yaw_points = []
    gyro_points = []
    phase_counts = {"pre": 0, "motion": 0, "post": 0}
    voltages = []
    yaw_keys = []
    gyro_keys = []
    imu_packet_count = 0
    chassis_packet_count = 0

    for sample in samples:
        phase = sample["phase"]
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
        derived = sample["derived"]
        if sample["imu_packet"] is not None:
            imu_packet_count += 1
        if sample["chassis_packet"] is not None:
            chassis_packet_count += 1
        if derived["yaw_deg"] is not None:
            yaw_points.append((sample["phase"], sample["t_rel_s"], derived["yaw_deg"]))
        if derived["yaw_key"] is not None:
            yaw_keys.append(derived["yaw_key"])
        if derived["gyro_z_dps"] is not None:
            gyro_points.append((sample["phase"], sample["t_rel_s"], derived["gyro_z_dps"]))
        if derived["gyro_z_key"] is not None:
            gyro_keys.append(derived["gyro_z_key"])
        if derived["voltage_v"] is not None:
            voltages.append(derived["voltage_v"])

    pre_yaws = [value for phase, _, value in yaw_points if phase == "pre"]
    motion_yaws = [value for phase, _, value in yaw_points if phase == "motion"]
    post_yaws = [value for phase, _, value in yaw_points if phase == "post"]
    pre_gyros = [value for phase, _, value in gyro_points if phase == "pre"]
    motion_gyros = [value for phase, _, value in gyro_points if phase == "motion"]
    post_gyros = [value for phase, _, value in gyro_points if phase == "post"]

    unwrapped_motion = unwrap_deg(motion_yaws)
    yaw_delta_motion = None
    if len(unwrapped_motion) >= 2:
        yaw_delta_motion = unwrapped_motion[-1] - unwrapped_motion[0]

    gyro_motion_integral = integrate_trapezoid(
        [(t_rel, value) for phase, t_rel, value in gyro_points if phase == "motion"]
    )

    summary = {
        "mode": mode,
        "pwm": pwm,
        "duration_s": duration_s,
        "samples_total": len(samples),
        "phase_counts": phase_counts,
        "imu_packet_count": imu_packet_count,
        "chassis_packet_count": chassis_packet_count,
        "yaw_key_detected": statistics.mode(yaw_keys) if yaw_keys else None,
        "gyro_z_key_detected": statistics.mode(gyro_keys) if gyro_keys else None,
        "pre_yaw_span_deg": span_or_none(pre_yaws),
        "post_yaw_span_deg": span_or_none(post_yaws),
        "pre_gyro_bias_dps": mean_or_none(pre_gyros),
        "pre_gyro_noise_dps": stddev_or_none(pre_gyros),
        "motion_yaw_delta_deg": yaw_delta_motion,
        "motion_effective_yaw_rate_dps": (yaw_delta_motion / duration_s) if yaw_delta_motion is not None else None,
        "motion_gyro_integrated_yaw_deg": gyro_motion_integral,
        "motion_mean_abs_gyro_z_dps": mean_or_none([abs(v) for v in motion_gyros]),
        "motion_peak_abs_gyro_z_dps": max((abs(v) for v in motion_gyros), default=None),
        "post_gyro_bias_dps": mean_or_none(post_gyros),
        "voltage_v_mean": mean_or_none(voltages),
        "voltage_v_min": min(voltages) if voltages else None,
    }
    if summary["motion_yaw_delta_deg"] is not None and summary["motion_gyro_integrated_yaw_deg"] is not None:
        summary["motion_yaw_vs_gyro_error_deg"] = (
            summary["motion_yaw_delta_deg"] - summary["motion_gyro_integrated_yaw_deg"]
        )
    else:
        summary["motion_yaw_vs_gyro_error_deg"] = None
    return summary


def print_summary(run_name: str, command_lr: tuple[float, float], summary: dict) -> None:
    print(f"{run_name}: mode={summary['mode']} pwm={summary['pwm']:.3f} L={command_lr[0]:.3f} R={command_lr[1]:.3f}")
    print(
        f"  samples={summary['samples_total']} phase_counts={summary['phase_counts']}"
        f" imu_packets={summary['imu_packet_count']} chassis_packets={summary['chassis_packet_count']}"
    )
    print(
        f"  detected yaw key={summary['yaw_key_detected'] or 'n/a'}"
        f", gyro_z key={summary['gyro_z_key_detected'] or 'n/a'}"
    )
    print(
        "  rest yaw span="
        f"{format_float(summary['pre_yaw_span_deg'])} deg"
        f", rest gyro bias={format_float(summary['pre_gyro_bias_dps'])} dps"
        f", rest gyro noise={format_float(summary['pre_gyro_noise_dps'])} dps"
    )
    print(
        "  motion yaw delta="
        f"{format_float(summary['motion_yaw_delta_deg'])} deg"
        f", effective yaw rate={format_float(summary['motion_effective_yaw_rate_dps'])} dps"
        f", gyro-integrated yaw={format_float(summary['motion_gyro_integrated_yaw_deg'])} deg"
    )
    print(
        "  motion |gyro_z| mean="
        f"{format_float(summary['motion_mean_abs_gyro_z_dps'])} dps"
        f", peak={format_float(summary['motion_peak_abs_gyro_z_dps'])} dps"
        f", yaw-vs-gyro error={format_float(summary['motion_yaw_vs_gyro_error_deg'])} deg"
    )
    print(
        "  voltage mean="
        f"{format_float(summary['voltage_v_mean'])} V"
        f", min={format_float(summary['voltage_v_min'])} V"
        f", post gyro bias={format_float(summary['post_gyro_bias_dps'])} dps"
    )


def format_float(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def parse_pwm_list(text: str | None, default_pwm: float) -> list[float]:
    if not text:
        return [default_pwm]
    values = []
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        values.append(clamp_pwm(float(chunk)))
    return values or [default_pwm]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a conservative motion pulse and log IMU/chassis feedback for WAVE ROVER. "
            "Body-frame convention: +X forward, +Z up, right-hand; positive yaw/omega_z is CCW (left turn)."
        )
    )
    parser.add_argument(
        "mode",
        choices=("rest", "forward", "backward", "turn-left", "turn-right", "turn-ccw", "turn-cw"),
        help="Motion primitive to probe. `turn-ccw` is positive yaw; `turn-cw` is negative yaw.",
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.03)
    parser.add_argument("--wait", type=float, default=3.0, help="Seconds to wait for quiet boot output.")
    parser.add_argument("--pwm", type=float, default=0.18, help="Base T=1 left/right magnitude in [-0.5,0.5].")
    parser.add_argument(
        "--pwm-list",
        help="Comma-separated sweep of PWM magnitudes, for example 0.12,0.16,0.20.",
    )
    parser.add_argument("--duration", type=float, default=1.5, help="Motion pulse duration in seconds.")
    parser.add_argument("--pre-seconds", type=float, default=2.0, help="Stationary sampling before motion.")
    parser.add_argument("--post-seconds", type=float, default=2.0, help="Stationary sampling after motion.")
    parser.add_argument("--sample-period", type=float, default=0.10, help="Telemetry polling period in seconds.")
    parser.add_argument("--command-period", type=float, default=0.10, help="Repeat T=1 command period in seconds.")
    parser.add_argument(
        "--chassis-every",
        type=int,
        default=5,
        help="Request T=130 every N telemetry cycles. Use 0 to disable chassis polling.",
    )
    parser.add_argument("--cooldown", type=float, default=1.5, help="Rest between sweep runs in seconds.")
    parser.add_argument("--label", default="", help="Optional label appended to the output folder name.")
    parser.add_argument(
        "--out-dir",
        default="data/motion_probes",
        help="Base directory for logs and summaries.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pwm_values = parse_pwm_list(args.pwm_list, args.pwm)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{args.label}" if args.label else ""
    session_dir = Path(args.out_dir) / f"{timestamp}_{args.mode}{suffix}"
    session_dir.mkdir(parents=True, exist_ok=True)

    session_info = {
        "created_utc": now_utc(),
        "mode": args.mode,
        "port": args.port,
        "baud": args.baud,
        "timeout": args.timeout,
        "wait": args.wait,
        "pwm_values": pwm_values,
        "duration_s": args.duration,
        "pre_seconds": args.pre_seconds,
        "post_seconds": args.post_seconds,
        "sample_period_s": args.sample_period,
        "command_period_s": args.command_period,
        "chassis_every": args.chassis_every,
        "cooldown_s": args.cooldown,
    }
    (session_dir / "session.json").write_text(json.dumps(session_info, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    summaries = []
    try:
        with open_serial(args.port, args.baud, args.timeout) as ser:
            wait_until_ready(ser, args.wait)
            send_json(ser, {"T": 143, "cmd": 0})
            send_json(ser, {"T": 131, "cmd": 1})
            drain_serial(ser, 0.3)

            for index, pwm in enumerate(pwm_values, start=1):
                run_name = f"run_{index:02d}"
                run_dir = session_dir / run_name
                run_dir.mkdir(parents=True, exist_ok=True)
                command_lr = motion_command(args.mode, pwm)
                all_samples = []

                print(f"Starting {run_name} with mode={args.mode} pwm={pwm:.3f}", file=sys.stderr)
                all_samples.extend(
                    collect_phase_samples(
                        ser,
                        "pre",
                        args.pre_seconds,
                        args.sample_period,
                        args.command_period,
                        (0.0, 0.0),
                        args.chassis_every,
                        run_dir,
                    )
                )
                all_samples.extend(
                    collect_phase_samples(
                        ser,
                        "motion",
                        args.duration,
                        args.sample_period,
                        args.command_period,
                        command_lr,
                        args.chassis_every,
                        run_dir,
                    )
                )
                send_stop_burst(ser, count=3, pause_s=0.05)
                all_samples.extend(
                    collect_phase_samples(
                        ser,
                        "post",
                        args.post_seconds,
                        args.sample_period,
                        args.command_period,
                        (0.0, 0.0),
                        args.chassis_every,
                        run_dir,
                    )
                )
                send_stop_burst(ser, count=5, pause_s=0.05)

                write_jsonl(run_dir / "all_samples.jsonl", all_samples)
                summary = summarize_run(args.mode, pwm, args.duration, all_samples)
                summary["run_name"] = run_name
                summary["command"] = {"L": command_lr[0], "R": command_lr[1]}
                (run_dir / "summary.json").write_text(
                    json.dumps(summary, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                print_summary(run_name, command_lr, summary)
                summaries.append(summary)
                if index != len(pwm_values):
                    time.sleep(args.cooldown)
    except KeyboardInterrupt:
        print("Interrupted; stop burst sent.", file=sys.stderr)
        return 130
    except SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            with open_serial(args.port, args.baud, args.timeout) as ser:
                send_json(ser, {"T": 131, "cmd": 0})
                send_stop_burst(ser, count=5, pause_s=0.05)
        except Exception:
            pass

    (session_dir / "summaries.json").write_text(json.dumps(summaries, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Logs written to {session_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
