"""Closed-loop swarm flight with physically applied rotor thrust."""

import mujoco
import numpy as np

from .environment import load_environment
from .model import ROTOR_POSITIONS, add_swarm

DEFAULT_POSITIONS = np.array([[-0.8, -0.8, 0.15], [-0.8, 0.8, 0.15], [0.8, 0, 0.15]])


class SwarmSimulation:
    def __init__(self, environment="empty"):
        self.spec = load_environment(environment)
        add_swarm(self.spec, DEFAULT_POSITIONS)
        self.model = self.spec.compile()
        self.data = mujoco.MjData(self.model)
        self.body_ids = [self.model.body(f"drone{i}_body").id for i in range(3)]
        self.joints = [self.model.joint(f"drone{i}_joint") for i in range(3)]
        self.motors = [
            [self.model.actuator(f"drone{i}_motor{j}").id for j in range(4)] for i in range(3)
        ]
        self.targets = DEFAULT_POSITIONS.copy()
        self.targets[:, 2] = 1.2
        self.allocation = np.array(
            [
                [1] * 4,
                [y for x, y in ROTOR_POSITIONS],
                [-x for x, y in ROTOR_POSITIONS],
                [0.015 * (-1) ** i for i in range(4)],
            ]
        )
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self.positions

    @property
    def positions(self):
        return self.data.xpos[self.body_ids].copy()

    def set_targets(self, positions):
        targets = np.asarray(positions, dtype=float)
        if targets.shape != (3, 3) or not np.isfinite(targets).all():
            raise ValueError("Targets must be a finite (3, 3) array in world metres")
        self.targets = targets.copy()

    def step(self):
        for i, (body, joint) in enumerate(zip(self.body_ids, self.joints)):
            dof = int(joint.dofadr[0])
            velocity = self.data.qvel[dof : dof + 3]
            omega = self.data.qvel[dof + 3 : dof + 6]
            rotation = self.data.xmat[body].reshape(3, 3)
            acceleration = np.clip(
                4 * (self.targets[i] - self.data.xpos[body]) - 3 * velocity, -4, 4
            )
            force = self.model.body_mass[body] * (acceleration + [0, 0, 9.81])
            z_axis = force / np.linalg.norm(force)
            y_axis = np.cross(z_axis, [1, 0, 0])
            y_axis /= np.linalg.norm(y_axis)
            desired = np.column_stack((np.cross(y_axis, z_axis), y_axis, z_axis))
            error_matrix = desired.T @ rotation - rotation.T @ desired
            error = 0.5 * np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])
            inertia = self.model.body_inertia[body]
            torque = inertia * (-100 * error - 20 * omega)
            thrust = max(0, force @ rotation[:, 2])
            self.data.ctrl[self.motors[i]] = np.clip(
                np.linalg.solve(self.allocation, np.r_[thrust, torque]), 0, 2
            )
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError("Simulation became non-finite")
        return self.positions

    def render(self, camera="drone0_camera", width=640, height=480):
        with mujoco.Renderer(self.model, width=width, height=height) as renderer:
            renderer.update_scene(self.data, camera=camera)
            return renderer.render().copy()
