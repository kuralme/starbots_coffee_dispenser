# Starbots Coffee Dispenser Project

## Overview

Starbots Coffee Dispenser is a robotics project that performs a complete Pick and Place task using the **UR3e robotic arm**, designed to operate in both Gazebo simulation and on a real-world setup. The system enables the robot to autonomously locate a coffee cup, pick it up, and place it on a coffee tray mounted on a Barista robot, simulating a real-world coffee-serving interaction.

The project integrates **MoveIt2** for motion planning and execution, and employs perception pipeline with object detection and point cloud processing to locate and interact with objects in the environment. It supports seamless operation in both Gazebo simulation and a real-world setup, utilizing the UR3e arm equipped with a point cloud cameras.

This system is built on ROS 2 and offers modular launch files to execute the full task pipeline — from perception and planning to actuation — in both simulated and physical environments.

---

<p align="center">
  <img src="./media/ur3e.png" width="60%">
</p>

## Prerequisites

- ROS 2 Humble
- Starbots Coffee/UR3e Gazebo package for simulation
- Physical UR3e robot and a Point Cloud camera for real-world testing
- `MoveIt2`, `colcon`, and `rosdep` installed


## Getting Started

1. Clone this repository into your ROS 2 workspace (`ros2_ws`),    creating it if it doesn't exist:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone <this-repo-url>
    cd ~/ros2_ws
    ```

2. Install dependencies:
    ```bash
    # Optional: Use if rosdep not initialized
    sudo rosdep init
    rosdep update
    ```
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    colcon build && source install/setup.bash
    ```

## Running the Pick & Place Task

### Test the Setup

Before running the main Pick and Place task, ensure your environment is correctly configured. Whether you're using **Gazebo simulation** or a **real UR3e robot**, follow these checks after launching the robot's control node (Gazebo or hardware interface).

- Check Active Controllers

    Make sure all required controllers are running:

    ```bash
    ros2 control list_controllers
    ```
    **Expected output:**
    ```
    joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    gripper_controller[position_controllers/GripperActionController] active
    ```

- Verify that joint state messages are being published:
    ```bash
    ros2 topic echo /joint_states
    ```
    You should see a stream of messages showing joint positions, velocities, etc.

- Ensure your camera is publishing expected topics:
    ```bash
    ros2 topic list | grep camera
    ```
    This should return a list of camera-related topics (e.g., /camera/color/image_raw, /camera/depth/points, etc.).

### Simulation


### Real Robot