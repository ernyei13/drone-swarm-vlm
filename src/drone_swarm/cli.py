"""Command-line viewer, headless demo, and scene export."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from .environment import available_environments
from .simulation import SwarmSimulation


def positive_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("duration must be finite and positive")
    return seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", default="empty", help="bundled name or MJCF XML path")
    parser.add_argument("--list-environments", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=positive_seconds, default=20.0)
    parser.add_argument("--demo", choices=["hover", "formation"], default="hover")
    parser.add_argument("--export", type=Path, help="export composed MJCF scene and exit")
    args = parser.parse_args()
    if args.list_environments:
        print("\n".join(available_environments()))
        return
    try:
        sim = SwarmSimulation(args.environment)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.export:
        # MjSpec retains native asset paths; XML references original environment assets.
        args.export.parent.mkdir(parents=True, exist_ok=True)
        args.export.write_text(sim.spec.to_xml())
        print(args.export.resolve())
        return
    initial_targets = sim.targets.copy()

    def step():
        if args.demo == "formation":
            phase = max(0, sim.data.time - 3) * 0.3
            offset = np.array([0.6 * math.sin(phase), 0.6 * (1 - math.cos(phase)), 0])
            sim.set_targets(initial_targets + offset)
        sim.step()

    if args.headless:
        while sim.data.time < args.duration:
            step()
    else:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            viewer.cam.distance = 5
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.lookat[:] = [0, 0, 0.8]
            while viewer.is_running() and sim.data.time < args.duration:
                start = time.monotonic()
                step()
                viewer.sync()
                time.sleep(max(0, sim.model.opt.timestep - (time.monotonic() - start)))
    print(
        json.dumps(
            {
                "time": sim.data.time,
                "positions": sim.positions.tolist(),
                "targets": sim.targets.tolist(),
            }
        )
    )


if __name__ == "__main__":
    main()
