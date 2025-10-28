# Starbots Coffee Dispenser Project

## Overview

Starbots Coffee Dispenser is a robotics project that performs a complete Pick and Place task using the **UR3e robotic arm**, designed to operate in both Gazebo simulation and on a real-world setup. The system enables the robot to autonomously locate a coffee cup, pick it up, and place it on a coffee tray mounted on a Barista robot, simulating a real-world coffee-serving interaction.

The project integrates the **MoveIt2** C++ API for motion planning and execution, while using a Python-based perception pipeline for **object detection** and **point cloud processing**. This perception system identifies and locates objects in the environment, and the point cloud data is further used to build an **OctoMap**. The OctoMap enables MoveIt2 to perform collision-aware planning by considering the surrounding environment. The current branch supports seamless operation within a Gazebo simulation setup.

To oversee the robot's high-level behavior, the system employs a custom **Behavior Tree (BT)**-based control logic. This modular and reactive framework manages task execution — including cup detection, picking, placing, and error handling — ensuring robust and flexible control throughout the coffee delivery process.

This system is built on ROS 2 and offers modular launch files to execute the full task pipeline — from perception and planning to actuation in a simulated environment. Additionally, Docker is used to build and containerize the system, and Docker Compose is leveraged to manage and start the multiple services, ensuring a consistent and efficient deployment across different setups.

---
[![Sim video](./media/final_sim.gif)](https://youtu.be/4XE2JNb8slM)

## Prerequisites

#### For host setup

- ROS 2 Humble
- Starbots Coffee/UR3e Gazebo package
- `MoveIt2`, `colcon`, and `rosdep` installed

#### For docker setup

- Docker Engine and Compose

## Getting Started (Host)

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

4. Start the Starbots Coffee Simulation

    ```bash
    ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
    ```

## Running the Pick & Place Task

<details>
<summary><b>Test the Setup</b></summary>

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

- Ensure both camera are publishing the expected topics:

    ```bash
    ros2 topic list | grep depth_sensor
    ```

    This should return a list of camera-related topics (e.g., /camera_depth_sensor/depth/image_raw, /wrist_rgbd_depth_sensor/depth/image_raw, etc.).

</details>

<details>
<summary><b>Run the nodes individually (optional)</b></summary>

MoveIt2 configuration node:

```bash
ros2 launch ur3e_sim_moveit_config move_group.launch.py
```

(Optional) RViz for visualization, also can give commands via Moveit2 plugin:

```bash
ros2 launch ur3e_sim_moveit_config moveit_rviz.launch.py
```

Cup detection node extracts cup and table surface points:

```bash
ros2 launch starbots_detection cup_detection.launch.py
```

Cup holder tray and hole detection node:

```bash
ros2 launch starbots_detection cup_holder_detection.launch.py
```

All detections including point cloud filter node:

```bash
ros2 launch starbots_detection detections.launch.py
```

</details>

#### Start starbots delivery service server

```bash
ros2 launch starbots_manipulation starbots_delivery.launch.py
# with rviz
ros2 launch starbots_manipulation starbots_delivery_rviz.launch.py
```

Send coffee delivery request to with given cup holder:

```bash
ros2 service call /deliver_coffee starbots_manipulation/srv/DeliverCup "{goal_cup_holder: 1}"
```

### Webapp

Visualize with foxglove webapp on browser:

- Start foxglove bridge

    ```bash
    ros2 launch starbots_webapp start_foxglove.launch.xml
    ```

- Get rosbridge address for connection (for The Construct env only)

    ```bash
    rosbridge_address
    ```

    Copy the address that is _wss://i-...robotigniteacademy.com/.../rosbridge/_ format

- Login [foxglove](https://app.foxglove.dev/dashboard) and connect via the rosbridge_address

- Enter "goal_cup_holder" number and call service to deliver the coffee cup to the desired hole on barista robot tray.

---

## Docker setup (recommended)

Using docker-compose, the required docker images will be downloaded and started up and ready.

Navigate to the docker directory of the repo and setup ROS2 containers.

```bash
cd docker
sudo chmod +x ros_entrypoint.sh
docker-compose up
```

The foxglove bridge also starts within the containers (with port 8765 as default).

To call coffee delivery service either follow webapp instructions step or use desktop app depending on your environment. [Desktop app](<https://foxglove.dev/download>) is recommended. Either set panels on your own or use _starbots_webapp/starbots_webapp_sim.json_

Spawn another cup:

```bash
docker exec starbots-ros2-gazebo bash /ros2_ws/src/spawn_cup.sh
```
