#!/usr/bin/env python3
import os
import time
import subprocess

# Path to the SDF model
MODEL_PATH = os.path.abspath("../src/simulation/models/drone/model.sdf")

# Number of drones to spawn
NUM_DRONES = 5

# Gazebo spawn command template
SPAWN_CMD = "ros2 run gazebo_ros spawn_entity.py -file {model} -entity {name} -x {x} -y {y} -z {z}"

# Spawn drones in a line
for i in range(NUM_DRONES):
    name = f"drone_{i+1}"
    x = i * 2.0
    y = 0.0
    z = 1.0
    cmd = SPAWN_CMD.format(model=MODEL_PATH, name=name, x=x, y=y, z=z)
    print(f"Spawning {name} at ({x}, {y}, {z})")
    subprocess.run(cmd, shell=True)
    time.sleep(1)

print("All drones spawned.")
