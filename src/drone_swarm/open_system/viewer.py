"""Live MuJoCo viewer for the measured join/leave experiment."""

import time

import mujoco
import mujoco.viewer
import numpy as np

from .experiment import OpenSwarmDemo, adjacency


def draw_graph(viewer, demo):
    """Draw only current active sensing edges, plus desired formation slots."""
    scene = viewer.user_scn
    scene.ngeom = 0
    positions = demo.sim.positions
    members = demo.members
    graph = adjacency(positions[members, :2], demo.config.sensing_radius)
    for a, i in enumerate(members):
        for b in range(a + 1, len(members)):
            if graph[a, b]:
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_LINE,
                    np.zeros(3),
                    np.zeros(3),
                    np.eye(3).ravel(),
                    np.array([0.5, 0.8, 1, 0.6]),
                )
                mujoco.mjv_connector(
                    geom, mujoco.mjtGeom.mjGEOM_LINE, 2, positions[i], positions[members[b]]
                )
                scene.ngeom += 1
    for i in members:
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.025] * 3),
            np.r_[demo.targets[i], 1.2],
            np.eye(3).ravel(),
            np.array([1, 1, 1, 0.5]),
        )
        scene.ngeom += 1


def run_viewer(config=None, environment="empty"):
    demo = OpenSwarmDemo(config, environment)
    print("Five agents form a pentagon; pink agent 6 waits nearby.", flush=True)
    print("At 10 s: agent 6 joins. At 22 s: green agent 3 departs to the parking area.", flush=True)
    print(
        "Blue lines: sensing links. White dots: desired slots. Close the viewer to stop.",
        flush=True,
    )
    with mujoco.viewer.launch_passive(demo.sim.model, demo.sim.data) as viewer:
        viewer.cam.distance = 5.5
        viewer.cam.azimuth = 110
        viewer.cam.elevation = -45
        viewer.cam.lookat[:] = [0.4, 0.2, 1.0]
        while viewer.is_running() and demo.time < demo.config.duration - 1e-8:
            start = time.monotonic()
            demo.tick()
            with viewer.lock():
                draw_graph(viewer, demo)
            viewer.sync()
            time.sleep(max(0, demo.config.dt - (time.monotonic() - start)))
    demo.record()
    return demo.results()
