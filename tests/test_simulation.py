import mujoco
import numpy as np
import pytest

from drone_swarm.environment import available_environments
from drone_swarm.simulation import SwarmSimulation


@pytest.mark.parametrize("environment", available_environments())
def test_three_drones_take_off_and_hover(environment):
    sim = SwarmSimulation(environment)
    assert sim.model.nu == 12
    assert sim.model.nq == 21
    assert np.allclose(sim.model.body_mass[sim.body_ids], 0.25)
    for _ in range(4000):
        sim.step()
    assert np.allclose(sim.positions, sim.targets, atol=0.02)
    assert sim.data.ncon == 0


def test_independent_targets_and_reset():
    sim = SwarmSimulation()
    initial = sim.positions
    targets = sim.targets.copy()
    targets += [[0.3, 0.2, 0.1], [-0.2, 0, 0.4], [0, -0.3, 0.2]]
    sim.set_targets(targets)
    for _ in range(5000):
        sim.step()
    assert np.allclose(sim.positions, targets, atol=0.03)
    assert np.max(np.abs(sim.data.qvel)) < 0.02
    assert np.allclose(sim.reset(), initial)
    assert sim.data.time == 0


def test_bad_targets_and_environment():
    sim = SwarmSimulation()
    for value in ([[1, 2, 3]], np.full((3, 3), np.nan)):
        with pytest.raises(ValueError):
            sim.set_targets(value)
    with pytest.raises(ValueError, match="Unknown environment"):
        SwarmSimulation("missing")


def test_custom_environment_relative_include_and_mesh(tmp_path):
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes/tetra.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n"
    )
    (tmp_path / "props.xml").write_text(
        '<mujoco><worldbody><geom name="prop" type="mesh" mesh="tetra" pos="4 4 0"/>'
        "</worldbody></mujoco>"
    )
    path = tmp_path / "scene.xml"
    path.write_text(
        '<mujoco><compiler meshdir="meshes"/><asset><mesh name="tetra" '
        'file="tetra.obj"/></asset><include file="props.xml"/></mujoco>'
    )
    sim = SwarmSimulation(path)
    assert sim.model.geom("prop").id >= 0
    exported = tmp_path / "composed.xml"
    exported.write_text(sim.spec.to_xml())
    model = mujoco.MjModel.from_xml_path(str(exported))
    assert model.nu == 12
