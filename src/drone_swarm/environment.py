"""Load native MJCF environments, including their relative assets and includes."""

from importlib.resources import files
from pathlib import Path

import mujoco


def available_environments() -> list[str]:
    return sorted(
        p.stem
        for p in files("drone_swarm").joinpath("environments").iterdir()
        if p.name.endswith(".xml")
    )


def load_environment(source: str | Path) -> mujoco.MjSpec:
    path = Path(source).expanduser()
    if not path.is_file():
        if str(source) not in available_environments():
            raise ValueError(
                f"Unknown environment {source!r}; choose {available_environments()} "
                "or supply an existing MJCF XML path"
            )
        path = Path(str(files("drone_swarm").joinpath("environments", f"{source}.xml")))
    spec = mujoco.MjSpec.from_file(str(path.resolve()))
    # Keep the demo physics consistent across environments.
    spec.option.timestep = 0.002
    spec.option.gravity[:] = [0, 0, -9.81]
    return spec
