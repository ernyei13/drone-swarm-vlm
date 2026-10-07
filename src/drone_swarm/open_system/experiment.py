"""MuJoCo open-team flight with consensus and constrained velocity references.

The nominal controller uses current sensing neighbours and relative desired offsets.
A global QP constrains pairwise safety and retained-tree connectivity in the
first-order reference model. Actual quadrotor motion is integrated by MuJoCo.
These reference constraints are not a guarantee for the higher-order flight dynamics. This is a baseline experiment, not a decentralized safety controller
or an implementation of the algorithms cited in the thesis proposal.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment, minimize

from ..simulation import SwarmSimulation


@dataclass(frozen=True)
class Config:
    dt: float = 0.02
    duration: float = 36.0
    join_time: float = 10.0
    leave_time: float = 22.0
    radius: float = 1.0
    sensing_radius: float = 1.65
    minimum_distance: float = 0.35
    safety_buffer: float = 0.15
    max_component_speed: float = 0.35
    barrier_gain: float = 1.5
    seed: int = 7


def adjacency(positions, radius):
    distances = np.linalg.norm(positions[:, None] - positions[None, :], axis=-1)
    graph = (distances < radius).astype(float)
    np.fill_diagonal(graph, 0)
    return graph


def algebraic_connectivity(graph):
    if len(graph) < 2:
        return 0.0
    laplacian = np.diag(graph.sum(axis=1)) - graph
    return max(0.0, float(np.linalg.eigvalsh(laplacian)[1]))


def spanning_tree(positions, members, radius, targets=None):
    """Retain an initially feasible tree; refuse disconnected membership changes."""
    visited = {int(members[0])}
    edges = []
    while len(visited) < len(members):
        candidates = [
            (np.linalg.norm(positions[i] - positions[j]), i, int(j))
            for i in visited
            for j in members
            if j not in visited
            and (targets is None or np.linalg.norm(targets[i] - targets[j]) < radius)
        ]
        if not candidates:
            raise ValueError("Membership switch has no formation-compatible connected tree")
        distance, i, j = min(candidates)
        if distance >= radius:
            raise ValueError("Membership switch has no feasible connected spanning tree")
        edges.append((i, j))
        visited.add(j)
    return edges


def assign_targets(positions, members, radius):
    angles = np.pi / 2 + np.arange(len(members)) * 2 * np.pi / len(members)
    slots = radius * np.column_stack((np.cos(angles), np.sin(angles)))
    costs = np.linalg.norm(positions[members, None, :] - slots[None, :, :], axis=-1)
    rows, cols = linear_sum_assignment(costs)
    targets = positions.copy()
    targets[members[rows]] = slots[cols]
    return targets


def nominal_velocity(positions, targets, members, sensing_radius):
    velocity = np.zeros_like(positions)
    graph = adjacency(positions[members], sensing_radius)
    errors = positions[members] - targets[members]
    velocity[members] = -0.4 * (np.diag(graph.sum(axis=1)) - graph) @ errors
    # One leader pins translation; relative consensus controls the rest.
    velocity[members[0]] -= 0.8 * errors[0]
    outsiders = np.setdiff1d(np.arange(len(positions)), members)
    velocity[outsiders] = -0.8 * (positions[outsiders] - targets[outsiders])
    return velocity


def constrained_velocity(positions, nominal, tree, config):
    n = positions.size
    rows, lower = [], []
    # Physical collision checks include parked / departing agents, not just members.
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            delta = positions[i] - positions[j]
            row = np.zeros((len(positions), 2))
            row[i], row[j] = 2 * delta, -2 * delta
            rows.append(row.ravel())
            lower.append(
                -config.barrier_gain
                * (delta @ delta - (config.minimum_distance + config.safety_buffer) ** 2)
            )
    for i, j in tree:
        delta = positions[i] - positions[j]
        row = np.zeros((len(positions), 2))
        row[i], row[j] = -2 * delta, 2 * delta
        rows.append(row.ravel())
        # A margin reduces risk of discretization crossing the sensing threshold.
        lower.append(-config.barrier_gain * ((config.sensing_radius - 0.05) ** 2 - delta @ delta))
    matrix, bound = np.array(rows), np.array(lower)
    reference = nominal.ravel()
    speed = config.max_component_speed
    result = minimize(
        lambda u: 0.5 * np.sum((u - reference) ** 2),
        np.clip(reference, -speed, speed),
        jac=lambda u: u - reference,
        bounds=[(-speed, speed)] * n,
        constraints={"type": "ineq", "fun": lambda u: matrix @ u - bound, "jac": lambda u: matrix},
        method="SLSQP",
        options={"ftol": 1e-9, "maxiter": 100},
    )
    residual = float(np.min(matrix @ result.x - bound))
    if not result.success or residual < -1e-7:
        raise RuntimeError(f"Safety QP infeasible or failed: {result.message}; residual={residual}")
    return result.x.reshape(positions.shape), residual


class OpenSwarmDemo:
    """Six physical quadrotors; membership changes affect coordination, not physics."""

    def __init__(self, config=None, environment="empty"):
        self.config = Config() if config is None else config
        c = self.config
        if not (0 < c.join_time < c.leave_time < c.duration):
            raise ValueError("Require 0 < join_time < leave_time < duration")
        if c.dt <= 0 or c.max_component_speed <= 0:
            raise ValueError("dt and speed must be positive")
        self.members = np.arange(5)
        xy = assign_targets(np.zeros((6, 2)), self.members, c.radius)
        xy[:5] += np.random.default_rng(c.seed).uniform(-0.12, 0.12, (5, 2))
        xy[5] = [1.7, -0.8]
        self.sim = SwarmSimulation(environment, positions=np.column_stack((xy, np.full(6, 0.15))))
        self.substeps = round(c.dt / self.sim.model.opt.timestep)
        if self.substeps < 1 or not np.isclose(self.substeps * self.sim.model.opt.timestep, c.dt):
            raise ValueError("Coordination dt must be a multiple of the physics timestep")
        # Stabilize at flight height before starting the measured coordination run.
        for _ in range(round(4.0 / self.sim.model.opt.timestep)):
            self.sim.step()
        self.start_time = self.sim.data.time
        self.sim.horizontal_velocity_control = True
        self.targets = assign_targets(self.sim.positions[:, :2], self.members, c.radius)
        self.reference = self.sim.positions.copy()
        self.tree = spanning_tree(
            self.reference[:, :2], self.members, c.sensing_radius - 0.05, self.targets
        )
        self.joined = self.left = False
        self.last_velocity = np.zeros((6, 2))
        self.last_residual = 0.0
        self.contact_count = 0
        self.minimum_physics_distance = float("inf")
        self.history = {
            key: []
            for key in (
                "time",
                "positions",
                "targets",
                "membership",
                "error",
                "absolute_error",
                "min_distance",
                "connectivity",
                "velocity",
                "qp_residual",
                "actual_velocity",
                "contact_count",
                "minimum_physics_distance",
                "height_error",
                "motor_thrust",
            )
        }

    @property
    def time(self):
        return round(self.sim.data.time - self.start_time, 10)

    def tick(self):
        c = self.config
        positions = self.sim.positions[:, :2]
        if not self.joined and self.time >= c.join_time - 1e-8:
            self.members = np.arange(6)
            self.targets = assign_targets(positions, self.members, c.radius)
            self.tree = spanning_tree(
                positions, self.members, c.sensing_radius - 0.05, self.targets
            )
            self.joined = True
        if not self.left and self.time >= c.leave_time - 1e-8:
            self.members = np.array([0, 1, 3, 4, 5])
            self.targets = assign_targets(positions, self.members, c.radius)
            self.targets[2] = [2.4, 1.8]
            self.tree = spanning_tree(
                positions, self.members, c.sensing_radius - 0.05, self.targets
            )
            self.left = True
        nominal = nominal_velocity(positions, self.targets, self.members, c.sensing_radius)
        self.last_velocity, self.last_residual = constrained_velocity(
            positions, nominal, self.tree, c
        )
        self.record()
        for _ in range(self.substeps):
            self.reference[:, :2] += self.sim.model.opt.timestep * self.last_velocity
            self.reference[:, 2] = 1.2
            self.sim.set_targets(self.reference)
            self.sim.target_velocities[:, :2] = self.last_velocity
            self.sim.step()
            self.contact_count += int(self.sim.data.ncon > 0)
            p = self.sim.positions
            distances = np.linalg.norm(p[:, None] - p[None, :], axis=-1)
            self.minimum_physics_distance = min(
                self.minimum_physics_distance, float(distances[np.triu_indices(6, 1)].min())
            )

    def record(self):
        points = self.sim.positions
        delta = points[:, None] - points[None, :]
        distances = np.linalg.norm(delta, axis=-1)
        mask = np.zeros(6, dtype=bool)
        mask[self.members] = True
        velocity = np.array(
            [self.sim.data.qvel[int(j.dofadr[0]) : int(j.dofadr[0]) + 3] for j in self.sim.joints]
        )
        errors = points[self.members, :2] - self.targets[self.members]
        shape_error = errors - errors.mean(axis=0)
        values = (
            self.time,
            points[:, :2].copy(),
            self.targets.copy(),
            mask,
            float(np.sqrt(np.mean(np.sum(shape_error**2, axis=1)))),
            float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
            float(distances[np.triu_indices(6, 1)].min()),
            algebraic_connectivity(adjacency(points[self.members, :2], self.config.sensing_radius)),
            self.last_velocity.copy(),
            self.last_residual,
            velocity.copy(),
            self.contact_count,
            self.minimum_physics_distance,
            float(np.max(np.abs(points[:, 2] - 1.2))),
            self.sim.data.ctrl[self.sim.motors].copy(),
        )
        for key, value in zip(self.history, values):
            self.history[key].append(value)

    def results(self):
        return {key: np.array(value) for key, value in self.history.items()}


def run_experiment(config=None, environment="empty"):
    demo = OpenSwarmDemo(config, environment)
    while demo.time < demo.config.duration - 1e-8:
        demo.tick()
    demo.record()
    return demo.results()
