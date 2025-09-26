# Simplified Drone Swarm Simulation

This guide provides simplified instructions to build and run a minimal simulation with a single drone.

## Prerequisites

- ROS 2 Jazzy
- Gazebo Garden
- Colcon
- pynput (`pip install pynput`)

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

## Automated Hover

1.  In a new terminal, source the local workspace setup file:
    ```bash
    source install/setup.bash
    ```
2.  Launch the simplified simulation with an empty world and the hover controller:
    ```bash
    ros2 launch simulation_local simple_simulation.launch.py
    ```

3.  In another new terminal, source the local workspace setup file again:
    ```bash
    source install/setup.bash
    ```
4.  Run the script to spawn a single drone:
    ```bash
    python3 scripts/spawn_single_drone.py
    ```

Once the drone is spawned, it should automatically hover in the air.

## Manual Control with `ros2 topic pub`

For manual control, you will launch the simulation without the automatic hover node and then publish velocity commands from the command line.

1.  In a new terminal, source the local workspace setup file:
    ```bash
    source install/setup.bash
    ```
2.  Launch the simulation for manual control:
    ```bash
    ros2 launch simulation_local manual_control.launch.py
    ```

3.  In another new terminal, source the local workspace setup file again:
    ```bash
    source install/setup.bash
    ```
4.  Run the script to spawn a single drone:
    ```bash
    python3 scripts/spawn_single_drone.py
    ```

5.  In a third terminal (after sourcing `install/setup.bash`), you can now send control commands. Here are some examples:

    **Hover (send this continuously to maintain altitude):**
    ```bash
    ros2 topic pub --rate 10 /model/x500_quadcopter/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 9.81}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```
