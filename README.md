# PROJECT SENTINEL

SENTINEL is an thesis robotics project developed using **ROS 2**. The project integrates robot maneuverability, localization & navigation in real time ( SLAM ), perception complete system to acquires and process images and a full architecture able to connect, maintain, escalates all these systems ( needs to be completed and detailed by the SENTINEL's developer team ).

## Technologies

- ROS 2 Jazzy
- Ubuntu 24.04 LTS
- GO2 Edu Pro
- Gazebo Harmonic

## Project Structure

To be completed by the development SENTINEL's developer team

## Installation and Dependencies

### 1. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros2-control
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-velodyne
sudo apt install ros-jazzy-velodyne-description
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### 2. Clone and Install SENTINEL Controller and Go2 Simulation Packages

```bash
cd ~/sentinel_ws/src
git clone https://github.com/ceres-agrobot/PROJECT-SENTINEL.git
```

### 3. Install Dependencies

```bash
cd ~/sentinel_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

SENTINEL uses **ROS 2 Jazzy**, **Gazebo Harmonic**, and **C++20** to simulate and control the Unitree Go2 quadruped robot.

### 1. Build the Workspace

```bash
cd ~/sentinel_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

### 2. Launch Gazebo Simulation

Start the simulation with the robot description, physics engine, ROS 2 bridge, and locomotion controllers.

```bash
ros2 launch sentinel_gz start_gz.launch.py enable_control:=true
```

The launch file initializes `main_world.sdf`, spawns the robot and environment models, and starts the control system.

### 3. Verify Controllers

Open a new terminal and check the available controllers.

```bash
source ~/sentinel_ws/install/setup.bash

ros2 control list_controllers \
    -c /sentinel/sim/controller_manager
```

Both `joint_state_broadcaster` and `joint_group_position_controller` should report `active`.

Check the robot's joint states:

```bash
ros2 topic echo /sentinel/sim/joint_states
```

### 4. Test Locomotion

Set the robot to its standing position:

```bash
ros2 topic pub --once \
    /sentinel/sim/mode \
    std_msgs/msg/String \
    '{data: "stand"}'
```

Enable walking:

```bash
ros2 topic pub --once \
    /sentinel/sim/mode \
    std_msgs/msg/String \
    '{data: "walk"}'
```

Send a forward velocity command:

```bash
ros2 topic pub -r 10 \
    /sentinel/sim/cmd_vel \
    geometry_msgs/msg/Twist \
    '{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Alternatively, use keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    --remap cmd_vel:=/sentinel/sim/cmd_vel
```

Only one velocity command source should be active at a time. Use `Ctrl+C` to stop publishing commands.

### 5. Custom SENTINEL Interfaces

The locomotion system also supports custom ROS 2 messages defined in `sentinel_interfaces`.

```bash
ros2 topic pub -r 10 \
    /sentinel/sim/velocity_command \
    sentinel_interfaces/msg/Velocities \
    '{linear_x: 0.08, linear_y: 0.0, angular_z: 0.0}'
```

Monitor the generated joint and foot references:

```bash
ros2 topic echo /sentinel/sim/joints_reference

ros2 topic echo /sentinel/sim/feet_reference
```

### 6. Stop the Robot

Stop the active velocity publisher using `Ctrl+C`, then return to the standing position:

```bash
ros2 topic pub --once \
    /sentinel/sim/mode \
    std_msgs/msg/String \
    '{data: "stand"}'
```

Other supported locomotion modes include `walk`, `crouch`, and `stop`.

### 7. Simulation Topics

All simulation interfaces use the `/sentinel/sim/` namespace to distinguish them from future physical robot interfaces under `/sentinel/real/`.

| Topic                                | Description                 |
| ------------------------------------ | --------------------------- |
| `/sentinel/sim/cmd_vel`              | Standard velocity commands  |
| `/sentinel/sim/velocity_command`     | Custom velocity commands    |
| `/sentinel/sim/mode`                 | Locomotion mode selection   |
| `/sentinel/sim/mode_state`           | Current locomotion mode     |
| `/sentinel/sim/joint_states`         | Measured joint states       |
| `/sentinel/sim/joints_reference`     | Custom joint references     |
| `/sentinel/sim/feet_reference`       | Foot trajectory references  |
| `/sentinel/sim/imu/data`             | Simulated IMU measurements  |
| `/sentinel/sim/rgb_image`            | Simulated RGB camera        |
| `/sentinel/sim/unitree_lidar/points` | Simulated LiDAR point cloud |

**Note:** The current simulation uses a simplified inverse-kinematics-based gait controller. It is not the proprietary Unitree locomotion controller and may require further tuning for stable dynamic walking.

## Contributors

**SENTINEL Development Team**

SENTINEL is developed and maintained by a multidisciplinary engineering team committed to advancing autonomous robotics, intelligent perception, and robotic solutions for structural inspection.

| Contributor              | GitHub                                 |
| ------------------------ | -------------------------------------- |
| Vales                    | —                                      |
| Sofis                    | —                                      |
| Elkins                   | —                                      |
| Iván Dario Orozco Ibáñez | [@Nidhood](https://github.com/Nidhood) |

## Acknowledgements

The SENTINEL Development Team gratefully acknowledges the collective efforts, technical expertise, and dedication of everyone who has contributed to the project's development.

We also recognize the open-source robotics community and the developers of **ROS 2, Gazebo, and Unitree's robotics tools**, whose technologies and contributions provide essential foundations for the design, simulation, and integration of SENTINEL.

Their continued commitment to open research, collaboration, and technological innovation plays a significant role in advancing the project's objectives.

---

![IEEE RAS Javeriana Footer](sentinel/public/img/footer.png)
