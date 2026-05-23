#!/usr/bin/env python3
import argparse
import json
import math
import statistics
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MODE_COLORS = {
    "forward": "#1f77b4",
    "backward": "#2ca02c",
    "turn-ccw": "#d62728",
    "turn-cw": "#ff7f0e",
}


@dataclass
class RunRecord:
    batch_id: str
    run_id: str
    mode: str
    x_pwm: float
    z_pwm: float
    duration_s: float
    pre_count: int
    post_count: int
    pre_t: list[float] | None
    post_t: list[float] | None
    delta_t: list[float] | None
    pre_tag_from_camera: list[list[float]] | None
    post_tag_from_camera: list[list[float]] | None
    pre_euler: list[float] | None
    post_euler: list[float] | None
    delta_euler: list[float] | None
    gyro_integrated_deg: float | None
    motion_rmse_px: float | None

    @property
    def effort(self) -> float:
        return abs(self.x_pwm if self.mode in ("forward", "backward") else self.z_pwm) * self.duration_s

    @property
    def signed_linear_z(self) -> float | None:
        if self.delta_t is None or self.mode not in ("forward", "backward"):
            return None
        sign = -1.0 if self.mode == "forward" else 1.0
        return sign * self.delta_t[2]

    @property
    def signed_linear_x(self) -> float | None:
        if self.delta_t is None or self.mode not in ("forward", "backward"):
            return None
        sign = -1.0 if self.mode == "forward" else 1.0
        return sign * self.delta_t[0]

    @property
    def signed_linear_y(self) -> float | None:
        if self.delta_t is None or self.mode not in ("forward", "backward"):
            return None
        sign = -1.0 if self.mode == "forward" else 1.0
        return sign * self.delta_t[1]

    def camera_position_in_tag(self, phase: str) -> np.ndarray | None:
        matrix = self.pre_tag_from_camera if phase == "pre" else self.post_tag_from_camera
        if matrix is None:
            return None
        arr = np.array(matrix, dtype=np.float64)
        return arr[:3, 3]

    def camera_forward_in_tag(self, phase: str) -> np.ndarray | None:
        matrix = self.pre_tag_from_camera if phase == "pre" else self.post_tag_from_camera
        if matrix is None:
            return None
        arr = np.array(matrix, dtype=np.float64)
        return arr[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze structured tag-motion batches and generate top-view plus "
            "diagnostic plots for linear and turn calibration behavior."
        )
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        default=Path("data/tag_motion_batches"),
        help="Batch root containing per-run session.json files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Optional output directory. Defaults to data/tag_motion_analysis/<timestamp>_all_batches.",
    )
    return parser.parse_args()


def load_records(root: Path) -> list[RunRecord]:
    records: list[RunRecord] = []
    for session_path in sorted(root.glob("*/runs/*/session.json")):
        payload = json.loads(session_path.read_text())
        command = payload["command"]
        pre = payload["pre_summary"]
        post = payload["post_summary"]
        motion = payload["motion_summary"]

        pre_t = pre.get("tag_translation_median_m")
        post_t = post.get("tag_translation_median_m")
        delta_t = None
        if pre_t and post_t:
            delta_t = [float(post_t[i] - pre_t[i]) for i in range(3)]

        pre_euler = pre.get("tag_euler_xyz_deg_median")
        post_euler = post.get("tag_euler_xyz_deg_median")
        delta_euler = None
        if pre_euler and post_euler:
            delta_euler = [float(post_euler[i] - pre_euler[i]) for i in range(3)]

        pre_pose = representative_pose(payload["samples"]["pre"], pre.get("tag_translation_median_m"))
        post_pose = representative_pose(payload["samples"]["post"], post.get("tag_translation_median_m"))

        records.append(
            RunRecord(
                batch_id=session_path.parts[-4],
                run_id=session_path.parent.name,
                mode=command["mode"],
                x_pwm=float(command["x_pwm"]),
                z_pwm=float(command["z_pwm"]),
                duration_s=float(command["duration_s"]),
                pre_count=int(pre.get("tag_detected_count") or 0),
                post_count=int(post.get("tag_detected_count") or 0),
                pre_t=pre_t,
                post_t=post_t,
                delta_t=delta_t,
                pre_tag_from_camera=pre_pose.get("tag_from_camera") if pre_pose else None,
                post_tag_from_camera=post_pose.get("tag_from_camera") if post_pose else None,
                pre_euler=pre_euler,
                post_euler=post_euler,
                delta_euler=delta_euler,
                gyro_integrated_deg=motion.get("gyro_integrated_deg"),
                motion_rmse_px=motion.get("tag_reprojection_rmse_px_mean"),
            )
        )
    return records


def representative_pose(samples: list[dict], median_translation: list[float] | None) -> dict | None:
    detected = [sample["tag_pose"] for sample in samples if sample.get("tag_pose")]
    if not detected:
        return None
    if not median_translation:
        return detected[len(detected) // 2]
    return min(
        detected,
        key=lambda pose: sum(
            (float(pose["tvec_m"][i]) - float(median_translation[i])) ** 2 for i in range(3)
        ),
    )


def usable_records(records: list[RunRecord]) -> list[RunRecord]:
    return [r for r in records if r.pre_t is not None and r.post_t is not None]


def linear_records(records: list[RunRecord]) -> list[RunRecord]:
    return [r for r in usable_records(records) if r.mode in ("forward", "backward")]


def turn_records(records: list[RunRecord]) -> list[RunRecord]:
    return [r for r in usable_records(records) if r.mode in ("turn-ccw", "turn-cw")]


def set_equal_axes(ax, x_values: list[float], y_values: list[float], pad: float = 0.02) -> None:
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    span = max(x_max - x_min, y_max - y_min)
    half = 0.5 * span + pad
    x_center = 0.5 * (x_min + x_max)
    y_center = 0.5 * (y_min + y_max)
    ax.set_xlim(x_center - half, x_center + half)
    ax.set_ylim(y_center - half, y_center + half)


def plot_top_view(records: list[RunRecord], output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 8))
    xs: list[float] = [0.0]
    zs: list[float] = [0.0]

    for mode in ("forward", "backward", "turn-ccw", "turn-cw"):
        subset = [r for r in records if r.mode == mode]
        for idx, record in enumerate(subset):
            pre_pos = record.camera_position_in_tag("pre")
            post_pos = record.camera_position_in_tag("post")
            pre_forward = record.camera_forward_in_tag("pre")
            post_forward = record.camera_forward_in_tag("post")
            if pre_pos is None or post_pos is None:
                continue
            x0, z0 = float(pre_pos[0]), float(pre_pos[2])
            x1, z1 = float(post_pos[0]), float(post_pos[2])
            xs.extend([x0, x1])
            zs.extend([z0, z1])
            ax.plot(
                [x0, x1],
                [z0, z1],
                color=MODE_COLORS[mode],
                alpha=0.55,
                linewidth=1.5,
                label=mode if idx == 0 else None,
            )
            ax.scatter([x0], [z0], color=MODE_COLORS[mode], s=10, alpha=0.45)
            ax.scatter([x1], [z1], color=MODE_COLORS[mode], s=20, alpha=0.80, marker="x")
            if pre_forward is not None:
                dx0, dz0 = normalize_floor_vector(prefer_camera_heading(pre_forward))
                if dx0 is not None and dz0 is not None:
                    ax.arrow(
                        x0,
                        z0,
                        0.06 * dx0,
                        0.06 * dz0,
                        color=MODE_COLORS[mode],
                        width=0.0015,
                        alpha=0.45,
                        length_includes_head=True,
                        head_width=0.015,
                    )
            if post_forward is not None:
                dx1, dz1 = normalize_floor_vector(prefer_camera_heading(post_forward))
                if dx1 is not None and dz1 is not None:
                    ax.arrow(
                        x1,
                        z1,
                        0.06 * dx1,
                        0.06 * dz1,
                        color=MODE_COLORS[mode],
                        width=0.0015,
                        alpha=0.80,
                        length_includes_head=True,
                        head_width=0.015,
                    )

    ax.scatter([0.0], [0.0], color="black", s=60, marker="*", label="tag origin")
    ax.plot([-0.12, 0.12], [0.0, 0.0], color="black", linewidth=2.0, alpha=0.6, label="tag plane")
    ax.axvline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.axhline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.set_title("Top View Floor Layout: Camera Poses in Tag Frame")
    ax.set_xlabel("tag +X / m")
    ax.set_ylabel("tag +Z / m")
    ax.grid(True, alpha=0.25)
    ax.invert_yaxis()
    set_equal_axes(ax, xs, zs)
    ax.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "top_view_segments.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def normalize_floor_vector(vector: np.ndarray) -> tuple[float | None, float | None]:
    vx = float(vector[0])
    vz = float(vector[2])
    norm = math.hypot(vx, vz)
    if norm < 1e-9:
        return None, None
    return vx / norm, vz / norm


def prefer_camera_heading(forward_axis_in_tag: np.ndarray) -> np.ndarray:
    if forward_axis_in_tag[2] > 0.0:
        return -forward_axis_in_tag
    return forward_axis_in_tag


def plot_lateral_bias(records: list[RunRecord], output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 6))
    xs = []
    ys = []

    for mode in ("forward", "backward"):
        subset = [r for r in records if r.mode == mode and r.signed_linear_z is not None]
        z = [r.signed_linear_z for r in subset]
        x = [r.signed_linear_x for r in subset]
        xs.extend(z)
        ys.extend(x)
        ax.scatter(z, x, color=MODE_COLORS[mode], alpha=0.75, label=mode)

    if xs and ys:
        coeff = np.polyfit(xs, ys, deg=1)
        x_line = np.linspace(min(xs), max(xs), 100)
        ax.plot(x_line, coeff[0] * x_line + coeff[1], color="black", linewidth=1.2, label="fit")

    ax.axhline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.axvline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.set_title("Lateral Bias: Signed Forward Motion vs Signed X Drift")
    ax.set_xlabel("signed camera Z change / m")
    ax.set_ylabel("signed camera X change / m")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "linear_lateral_bias.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_vertical_bias(records: list[RunRecord], output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 6))
    xs = []
    ys = []

    for mode in ("forward", "backward"):
        subset = [r for r in records if r.mode == mode and r.signed_linear_z is not None]
        z = [r.signed_linear_z for r in subset]
        y = [r.signed_linear_y for r in subset]
        xs.extend(z)
        ys.extend(y)
        ax.scatter(z, y, color=MODE_COLORS[mode], alpha=0.75, label=mode)

    if xs and ys:
        coeff = np.polyfit(xs, ys, deg=1)
        x_line = np.linspace(min(xs), max(xs), 100)
        ax.plot(x_line, coeff[0] * x_line + coeff[1], color="black", linewidth=1.2, label="fit")

    ax.axhline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.axvline(0.0, color="#bbbbbb", linewidth=0.8)
    ax.set_title("Vertical Bias: Signed Forward Motion vs Signed Y Drift")
    ax.set_xlabel("signed camera Z change / m")
    ax.set_ylabel("signed camera Y change / m")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "linear_vertical_bias.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_linear_response(records: list[RunRecord], output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 6))
    xs = []
    ys = []
    for mode in ("forward", "backward"):
        subset = [r for r in records if r.mode == mode and r.signed_linear_z is not None]
        effort = [r.effort for r in subset]
        signed_z = [r.signed_linear_z for r in subset]
        xs.extend(effort)
        ys.extend(signed_z)
        ax.scatter(effort, signed_z, color=MODE_COLORS[mode], alpha=0.75, label=mode)

    if xs and ys:
        slope = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)
        x_line = np.linspace(0.0, max(xs), 100)
        ax.plot(x_line, slope * x_line, color="black", linewidth=1.2, label=f"origin fit {slope:.2f} m/(PWM*s)")

    ax.set_title("Linear Response: Pulse Effort vs Signed Camera Z Change")
    ax.set_xlabel("|PWM| * duration / PWM*s")
    ax.set_ylabel("signed camera Z change / m")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "linear_response.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_turn_mismatch(records: list[RunRecord], output_dir: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 6))
    x_values = []
    y_values = []

    for mode in ("turn-ccw", "turn-cw"):
        subset = [
            r
            for r in records
            if r.mode == mode and r.delta_euler is not None and r.gyro_integrated_deg is not None
        ]
        gyro = [abs(r.gyro_integrated_deg) for r in subset]
        tag_yaw = [abs(r.delta_euler[2]) for r in subset]
        x_values.extend(gyro)
        y_values.extend(tag_yaw)
        ax.scatter(gyro, tag_yaw, color=MODE_COLORS[mode], alpha=0.75, label=mode)

    if x_values:
        upper = max(max(x_values), max(y_values)) * 1.1
        ax.plot([0.0, upper], [0.0, upper], color="#888888", linestyle="--", linewidth=1.0, label="1:1")
        ax.set_xlim(0.0, upper)
        ax.set_ylim(0.0, upper)

    ax.set_title("Turn Calibration Issue: IMU Turn Magnitude vs Settled Tag Yaw")
    ax.set_xlabel("|gyro-integrated turn| / deg")
    ax.set_ylabel("|post-pre tag yaw| / deg")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "turn_imu_vs_tag_yaw.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_summary(records: list[RunRecord], output_dir: Path) -> Path:
    linear = [r for r in records if r.mode in ("forward", "backward") and r.signed_linear_z is not None]
    turn = [r for r in records if r.mode in ("turn-ccw", "turn-cw")]

    xz_ratios = [r.signed_linear_x / r.signed_linear_z for r in linear if abs(r.signed_linear_z) > 1e-9]
    yz_ratios = [r.signed_linear_y / r.signed_linear_z for r in linear if abs(r.signed_linear_z) > 1e-9]

    slope = None
    if linear:
        xs = [r.effort for r in linear]
        ys = [r.signed_linear_z for r in linear]
        slope = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)

    payload = {
        "generated_at": datetime.utcnow().replace(microsecond=0).isoformat() + "Z",
        "record_count": len(records),
        "linear_count": len(linear),
        "turn_count": len(turn),
        "camera_forward_axis_estimate_in_camera_frame": {
            "x_over_z_mean": statistics.fmean(xz_ratios) if xz_ratios else None,
            "y_over_z_mean": statistics.fmean(yz_ratios) if yz_ratios else None,
            "yaw_skew_deg_estimate": math.degrees(math.atan(statistics.fmean(xz_ratios))) if xz_ratios else None,
            "pitch_skew_deg_estimate": math.degrees(math.atan(statistics.fmean(yz_ratios))) if yz_ratios else None,
        },
        "linear_origin_fit_m_per_pwm_s": slope,
    }
    path = output_dir / "summary.json"
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return path


def main() -> int:
    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path("data/tag_motion_analysis") / f"{timestamp}_all_batches"
    output_dir.mkdir(parents=True, exist_ok=True)

    records = usable_records(load_records(args.input_root))
    if not records:
        raise SystemExit("no usable session.json files found")

    outputs = [
        plot_top_view(records, output_dir),
        plot_lateral_bias(linear_records(records), output_dir),
        plot_vertical_bias(linear_records(records), output_dir),
        plot_linear_response(linear_records(records), output_dir),
        plot_turn_mismatch(turn_records(records), output_dir),
        write_summary(records, output_dir),
    ]

    for path in outputs:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
