#!/usr/bin/env python3
import os
import time
import subprocess

# Path to the SDF model
MODEL_PATH = os.path.abspath("src/simulation_local/models/drone/model.sdf")

# Number of drones to spawn
NUM_DRONES = 1

# Gazebo spawn command template
SPAWN_CMD = "ros2 run ros_gz_sim create -file {model} -name {name} -x {x} -y {y} -z {z} --verbose"

# Spawn drones in a line
for i in range(NUM_DRONES):
    name = f"drone_{i+1}"
    x = 0.0
    y = 0.0
    z = 1.0
    cmd = SPAWN_CMD.format(model=MODEL_PATH, name=name, x=x, y=y, z=z)
    print(f"Spawning {name} at ({x}, {y}, {z})")
    subprocess.run(cmd, shell=True)
    time.sleep(1)

print("Drone spawned.")
