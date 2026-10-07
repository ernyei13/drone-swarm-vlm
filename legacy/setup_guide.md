# Development Environment Setup Guide

This guide details the steps to set up a development environment on a fresh Ubuntu 24.04 installation for the drone swarm project.

## 1. System Dependencies

First, update your package lists and install `git`:

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y git
```

## 2. ROS 2 Jazzy Installation

These instructions are based on the official ROS 2 documentation.

### Set Locale

Ensure your system supports UTF-8:

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 Repositories

Add the ROS 2 package repositories to your system.

First, enable the Ubuntu Universe repository:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now, add the ROS 2 GPG key and repository to your sources list:

```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2, Gazebo, and RViz

Update your package lists and install the recommended desktop version of ROS 2 Jazzy. This installation includes:
- ROS 2 libraries and tools
- **Gazebo:** for running the simulation.
- **RViz:** for visualizing sensor data and robot models.
- Other development tools.

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y ros-jazzy-desktop
```

### Set Up Environment

Source the setup file to make the ROS 2 commands available in your terminal session:

```bash
source /opt/ros/jazzy/setup.bash
```

To have this done automatically in every new terminal session, add this command to your `.bashrc` file:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 3. Python Dependencies

Install the necessary Python libraries for the VLM.

### PyTorch

For PyTorch, it is highly recommended to use the official command from the PyTorch website to ensure compatibility with your system's hardware (CPU or CUDA).

Please visit the [PyTorch "Get Started" page](https://pytorch.org/get-started/locally/) to generate the correct `pip` command.

### TensorFlow

Install TensorFlow using pip:

```bash
pip install tensorflow
```

## 4. Development Environment

This section details how to set up a development environment with VS Code.

### Python Development Tools

Install `python3-pip` and `python3-venv` to manage Python packages and create virtual environments:

```bash
sudo apt install -y python3-pip python3-venv
```

### Visual Studio Code

Install Visual Studio Code, a popular code editor for ROS and Python development.

First, install the Microsoft GPG key and add the VS Code repository:

```bash
sudo apt install -y software-properties-common apt-transport-https wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo install -D -o root -g root -m 644 - /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
```

Then, update the package cache and install VS Code:

```bash
sudo apt update
sudo apt install -y code
```

### Recommended VS Code Extensions

After installing VS Code, launch it and install the following extensions for a better development experience with Python and ROS. You can install them from the Extensions view (`Ctrl+Shift+X`) in VS Code.

-   **Python**: `ms-python.python` - Provides rich support for the Python language, including features such as IntelliSense, linting, debugging, and code navigation.
-   **ROS**: `ms-iot.vscode-ros` - Simplifies ROS development by providing support for ROS build systems, debugging, and visualization of ROS messages.

## 5. Project Setup

Finally, clone the project repository and build the ROS 2 workspace.

### Clone the Repository

```bash
git clone <your-repository-url>
cd drone-swarm-vlm
```

### Build the Workspace

Build the ROS 2 packages using `colcon`:

```bash
colcon build
```

### Run the Simulation

After a successful build, you can run the simulation using the provided script:

```bash
./scripts/run_simulation.sh
```
