"""Generate application-demo data, plots and an animation-ready trajectory."""

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .experiment import Config, run_experiment

COLORS = ["#2784cc", "#dc6830", "#3b9b75", "#9855b0", "#b38b24", "#e14c78"]


def summary(history, config):
    return {
        "physics_contact_steps": int(history["contact_count"][-1]),
        "maximum_height_error_m": float(history["height_error"].max()),
        "maximum_actual_xy_speed_m_s": float(
            np.linalg.norm(history["actual_velocity"][:, :, :2], axis=2).max()
        ),
        "model": "MuJoCo 6-DoF quadrotors; first-order consensus references and centralized QP",
        "config": asdict(config),
        "minimum_physical_distance_m": float(history["minimum_physics_distance"][-1]),
        "minimum_algebraic_connectivity": float(history["connectivity"].min()),
        "maximum_component_speed_m_s": float(np.abs(history["velocity"]).max()),
        "maximum_vector_speed_m_s": float(np.linalg.norm(history["velocity"], axis=2).max()),
        "final_absolute_target_rms_m": float(history["absolute_error"][-1]),
        "final_formation_rms_m": float(history["error"][-1]),
        "pre_join_formation_rms_m": float(history["error"][history["time"] < config.join_time][-1]),
        "pre_leave_formation_rms_m": float(
            history["error"][history["time"] < config.leave_time][-1]
        ),
        "minimum_qp_residual": float(history["qp_residual"].min()),
        "membership_counts": [5, 6, 5],
        "observed_safety_satisfied": bool(
            history["minimum_physics_distance"][-1] >= config.minimum_distance
        ),
        "observed_connectivity_satisfied": bool(np.all(history["connectivity"] > 1e-8)),
        "caveat": "Observed discrete-time performance, not a proof or hardware validation.",
    }


def make_plots(history, config, output):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    curves = [
        ("error", "Formation error [m]"),
        ("min_distance", "Minimum physical distance [m]"),
        ("connectivity", "Algebraic connectivity λ₂"),
    ]
    for ax, (key, label) in zip(axes, curves):
        ax.plot(history["time"], history[key], color="#2784cc", lw=2)
        ax.set_ylabel(label)
        if key == "error":
            ax.lines[-1].set_label("Shape RMS (centroid aligned)")
            ax.plot(
                history["time"],
                history["absolute_error"],
                color="#888888",
                ls="--",
                label="Absolute target RMS",
            )
            ax.legend(loc="upper right")
        ax.grid(alpha=0.2)
        for t, event in [
            (config.join_time, "Agent 6 joins"),
            (config.leave_time, "Agent 3 leaves"),
        ]:
            ax.axvline(t, color="#777777", ls="--", lw=1)
            if ax is axes[0]:
                ax.text(t + 0.3, ax.get_ylim()[1] * 0.9, event, fontsize=9)
    axes[1].axhline(config.minimum_distance, color="#dc6830", ls=":", label="Safety threshold")
    axes[1].legend(loc="upper right")
    axes[2].axhline(0, color="#dc6830", ls=":")
    axes[2].set_xlabel("Simulation time [s]")
    fig.suptitle(
        "Open-team formation: five → six → five agents\nMuJoCo flight with bounded velocity references + safety/connectivity filter"
    )
    fig.savefig(output / "metrics.png", dpi=180)
    fig.savefig(output / "metrics.pdf")
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5), constrained_layout=True)
    for ax, t, title in zip(
        axes, [9.98, 21.98, config.duration], ["Five agents", "Six agents", "Five after departure"]
    ):
        index = int(np.argmin(np.abs(history["time"] - t)))
        members = np.flatnonzero(history["membership"][index])
        points = history["positions"][index]
        for i in range(6):
            ax.plot(
                history["positions"][: index + 1, i, 0],
                history["positions"][: index + 1, i, 1],
                color=COLORS[i],
                lw=0.8,
                alpha=0.35,
            )
            ax.scatter(*points[i], color=COLORS[i] if i in members else "#bbbbbb", s=60)
            ax.text(points[i, 0] + 0.08, points[i, 1] + 0.08, str(i + 1), fontsize=9)
        for a, i in enumerate(members):
            for j in members[a + 1 :]:
                if np.linalg.norm(points[i] - points[j]) < config.sensing_radius:
                    ax.plot(points[[i, j], 0], points[[i, j], 1], color="#333333", alpha=0.4, lw=1)
        ax.scatter(*history["targets"][index, members].T, marker="x", color="#333333", s=25)
        ax.set(
            title=f"{title}\nt = {t:.1f} s",
            xlabel="x [m]",
            ylabel="y [m]",
            xlim=(-1.6, 2.8),
            ylim=(-1.6, 2.5),
            aspect="equal",
        )
        ax.grid(alpha=0.2)
    fig.savefig(output / "formations.png", dpi=180)
    plt.close(fig)


def generate(output, config, viewer=False, environment="empty"):
    from .viewer import run_viewer

    output.mkdir(parents=True, exist_ok=True)
    history = run_viewer(config, environment) if viewer else run_experiment(config, environment)
    np.savez_compressed(output / "trajectory.npz", **history)
    result = summary(history, config)
    (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "metrics.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "active_agents",
                "formation_rms_m",
                "min_physical_distance_m",
                "algebraic_connectivity",
                "max_component_speed_m_s",
                "qp_residual",
            ]
        )
        writer.writerows(
            zip(
                history["time"],
                history["membership"].sum(axis=1),
                history["error"],
                history["min_distance"],
                history["connectivity"],
                np.abs(history["velocity"]).max(axis=(1, 2)),
                history["qp_residual"],
            )
        )
    # 25 Hz trajectory samples; exactly the measured values, no simulated JS dynamics.
    indices = np.arange(0, len(history["time"]), 2)
    data = {
        key: history[key][indices].tolist()
        for key in (
            "time",
            "positions",
            "targets",
            "membership",
            "error",
            "min_distance",
            "connectivity",
        )
    }
    data["config"] = asdict(config)
    (output / "trajectory.json").write_text(json.dumps(data, separators=(",", ":")))
    make_plots(history, config, output)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("demo-results"))
    parser.add_argument("--viewer", action="store_true", help="Run the live MuJoCo viewer")
    parser.add_argument("--environment", default="empty")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    print(
        json.dumps(
            generate(args.output, Config(seed=args.seed), args.viewer, args.environment), indent=2
        )
    )


if __name__ == "__main__":
    main()
