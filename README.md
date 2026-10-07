# Drone swarm VLM

A simple MuJoCo simulation with three small quadrotors, closed-loop hover and
formation flight, and native MJCF environment loading. Python 3.10+; no ROS,
Gazebo, ArduPilot, or GPU required for physics.

## Quick start

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
drone-swarm --list-environments
drone-swarm --headless --environment warehouse --duration 10
```

Interactive viewer on Linux / Windows:

```bash
drone-swarm --environment warehouse --demo formation --duration 60
```

On **macOS**, the viewer must run through MuJoCo's `mjpython` launcher:

```bash
mjpython -m drone_swarm --environment warehouse --demo formation --duration 60
```

The viewer supports mouse orbit / zoom and MuJoCo's built-in controls. Headless
runs print final positions and targets as JSON. `--duration` is simulation seconds.

## Environments

Two bundled environments: `empty` (ground plane) and `warehouse` (shelves and a
crate). Supply your own native MuJoCo XML, with relative includes and mesh assets:

```bash
drone-swarm --environment /absolute/path/to/scene.xml --headless
# Export the environment with all three drones for inspection:
drone-swarm --environment warehouse --export scene.xml
```

See [environment authoring](docs/environments.md) for the loading contract.
Gazebo `.world` / SDF, USD and arbitrary 3D formats need conversion to MJCF first.

## Python pipeline

```python
from drone_swarm.simulation import SwarmSimulation

sim = SwarmSimulation("warehouse")
sim.set_targets([[-0.8, -0.8, 1.2], [-0.8, 0.8, 1.5], [0.8, 0, 1.2]])
for _ in range(5000):
    sim.step()
positions = sim.positions              # (3, 3) copy, world metres
rgb = sim.render("drone0_camera")      # uint8 RGB; requires an OpenGL context
sim.reset()                           # reset physics, preserve assigned targets
```

Each drone has a forward camera (`drone0_camera` through `drone2_camera`). This
is the extension point for a future VLM: render images, infer tasks, then assign
position targets. **No VLM model or autonomous planner is implemented yet.**

## Project layout

- `src/drone_swarm/`: simulator, quadrotor model, environment loader, CLI.
- `src/drone_swarm/environments/`: bundled native MJCF scenes.
- `tests/`: takeoff, hover, independent targets, reset and custom asset loading.
- `legacy/`: archived original ROS / Gazebo code and setup notes.

## Validation

```bash
pytest
ruff check src tests
ruff format --check src tests
python -m build
```

CI runs these checks plus a headless warehouse demo on Python 3.10 and 3.12.
MuJoCo 3.15+ is required for the scene composition API used here.

## Simulation scope

Drones weigh 250 g with roughly 24 cm rotor-to-rotor extent. Each has a free joint,
four bounded thrust actuators and alternating rotor yaw torque. A position / attitude
controller allocates force and torque to the motors; flight is integrated by MuJoCo.
This is a demonstration model, without motor lag, aerodynamic calibration, battery,
flight firmware, obstacle avoidance or task allocation. Keep spawn areas clear and
choose reachable targets. Environments share a 2 ms timestep and Earth gravity.

MuJoCo installation and macOS viewer guidance: [official Python documentation](https://mujoco.readthedocs.io/en/stable/python.html).
