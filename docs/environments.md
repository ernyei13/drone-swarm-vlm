# Environment loading pipeline

1. Resolve a bundled name (`empty`, `warehouse`) or an existing XML file path.
2. Parse the environment with `mujoco.MjSpec.from_file`; MuJoCo resolves includes,
   meshes, textures and compiler asset directories relative to that scene.
3. Set the demo timestep to 0.002 seconds and gravity to (0, 0, -9.81).
4. Attach three independent drone specs with `drone0_`, `drone1_`, `drone2_` prefixes.
5. Compile the composed spec and initialize physics and flight controllers.

Loading happens when constructing `SwarmSimulation`. To change environments,
construct a new simulator; runtime hot reload is not implemented.

## Scene contract

Use a complete `<mujoco>` MJCF root. Supply your own ground, lights and scenery.
Reserve `drone0_`, `drone1_` and `drone2_` names for the swarm. A passive scene is
recommended; other actuators are preserved, but this demo only controls drone motors.

Spawn centres in metres: (-0.8, -0.8, 0.15), (-0.8, 0.8, 0.15), (0.8, 0, 0.15).
Their initial hover targets use the same XY and Z=1.2. Clear the takeoff columns.
A custom floor elevation or obstacles may require changing spawn positions in
`simulation.py`; this pipeline does not infer collision-free spawn points.

Minimal scene:

```xml
<mujoco model="my_environment">
  <worldbody>
    <light pos="0 0 5"/>
    <geom type="plane" size="10 10 0.1" rgba="0.2 0.3 0.4 1"/>
    <geom name="obstacle" type="box" pos="3 0 0.5" size="0.4 0.4 0.5"/>
  </worldbody>
</mujoco>
```

`--export` writes the composed XML. External assets remain referenced rather than
copied into a portable bundle; keep the original assets available. Native MuJoCo
parse / compile errors identify malformed scenes or missing assets.

The camera method uses offscreen OpenGL rendering. Headless physics does not
need OpenGL; on servers configure a supported EGL / OSMesa backend for images.
