"""A 250 g, 24 cm quadrotor with four thrust actuators and a forward camera."""

import mujoco

ROTOR_POSITIONS = ((0.085, 0.085), (-0.085, 0.085), (-0.085, -0.085), (0.085, -0.085))
COLORS = (
    (0.2, 0.7, 1, 1),
    (1, 0.5, 0.2, 1),
    (0.4, 0.9, 0.4, 1),
    (0.7, 0.3, 0.9, 1),
    (1, 0.85, 0.2, 1),
    (1, 0.25, 0.5, 1),
)


def drone_spec(color: tuple[float, ...]) -> mujoco.MjSpec:
    rotors = ""
    motors = ""
    for i, (x, y) in enumerate(ROTOR_POSITIONS):
        rotors += f'''<geom type="capsule" fromto="0 0 0 {x} {y} 0" size="0.008" mass="0.005"/>
        <geom type="cylinder" pos="{x} {y} 0.015" size="0.035 0.003" mass="0.005"/>
        <site name="rotor{i}" pos="{x} {y} 0" size="0.006"/>'''
        motors += f"""<general name="motor{i}" site="rotor{i}" gear="0 0 1 0 0 {0.015 * (-1) ** i}"
            ctrllimited="true" ctrlrange="0 2"/>"""
    return mujoco.MjSpec.from_string(f'''<mujoco><worldbody><body name="body">
      <freejoint name="joint"/>
      <geom type="box" size="0.04 0.03 0.015" mass="0.21" rgba="{" ".join(map(str, color))}"/>
      {rotors}
      <camera name="camera" pos="0.045 0 0" xyaxes="0 -1 0 0 0 1" fovy="80"/>
    </body></worldbody><actuator>{motors}</actuator></mujoco>''')


def add_swarm(spec: mujoco.MjSpec, positions) -> None:
    for i, position in enumerate(positions):
        frame = spec.worldbody.add_frame(pos=position)
        spec.attach(drone_spec(COLORS[i % len(COLORS)]), prefix=f"drone{i}_", frame=frame)
