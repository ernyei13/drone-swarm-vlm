import numpy as np
import pytest

pytest.importorskip("scipy")
from drone_swarm.open_system.experiment import (
    Config,
    adjacency,
    algebraic_connectivity,
    run_experiment,
    spanning_tree,
)


def test_connectivity_detects_articulation_point_removal():
    points = np.array([[0, 0], [1, 0], [2, 0]])
    assert algebraic_connectivity(adjacency(points, 1.1)) > 0
    assert algebraic_connectivity(adjacency(points[[0, 2]], 1.1)) == 0
    with pytest.raises(ValueError, match="connected"):
        spanning_tree(points, [0, 2], 1.1)


def test_join_leave_preserves_observed_constraints():
    c = Config()
    h = run_experiment(c)
    assert np.array_equal(
        h["membership"].sum(axis=1),
        np.where(h["time"] < c.join_time, 5, np.where(h["time"] < c.leave_time, 6, 5)),
    )
    assert h["minimum_physics_distance"][-1] >= c.minimum_distance
    assert h["contact_count"][-1] == 0
    assert h["height_error"].max() < 0.02
    assert np.max(h["motor_thrust"]) <= 2.0
    assert h["connectivity"].min() > 0.01
    assert np.abs(h["velocity"]).max() <= c.max_component_speed + 1e-8
    assert h["qp_residual"].min() > -1e-7
    assert h["error"][-1] < 0.06
    assert not h["membership"][-1, 2]
    assert h["membership"][-1, 5]
    assert np.linalg.norm(h["positions"][-1, 2] - [2.4, 1.8]) < 0.02
