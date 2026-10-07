# Simplified Drone Swarm Simulation

This guide provides simplified instructions to build and run a minimal simulation with a single drone.

## Prerequisites

- ROS 2 Jazzy
- Gazebo Garden
- Colcon
- pynput (`pip install pynput`)

## Ardupilot

The latest version (4.3.6 at the time of writing) used in this tutorial. You can use steps below to install it:

```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init –recursive
alias waf="$PWD/modules/waf/waf-light"
waf configure --board=sitl
waf all
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

## Ardupilot_gazebo

```
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j2
sudo make install
```

After installation of this plugin, you need to add lines below inside your bash (bashrc, zshrc, ...):

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```

## Build the Workspace

1.  Open a terminal in the root of the project (`/home/zernyei/dev/drone-swarm-vlm`).
2.  Source your ROS 2 installation. This is a crucial step.
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
3.  **Rebuild the project** to install the new controller nodes:
    ```bash
    colcon build
    ```

## Keyboard Control

1.  In a new terminal, source the local workspace setup file:
    ```bash
    source install/setup.bash
    ```
2.  Launch the simulation for keyboard control:
    ```bash
    ros2 launch simulation_local keyboard_control.launch.py
    ```
    This will open Gazebo and a new terminal window for keyboard input.

3.  In another new terminal, source the local workspace setup file again:
    ```bash
    source install/setup.bash
    ```
4.  Run the script to spawn a single drone:
    ```bash
    python3 scripts/spawn_single_drone.py
    ```

5.  Click on the new terminal window that opened for keyboard control and use the following keys:

    -   **W / S**: Move forward / backward
    -   **A / D**: Move left / right
    -   **Q / E**: Move up / down
    -   **Z / C**: Rotate left / right
    -   **Esc**: Stop the drone and exit the keyboard control node.