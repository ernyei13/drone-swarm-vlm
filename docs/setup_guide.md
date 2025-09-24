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

## 4. Project Setup

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