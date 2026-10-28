# ROS2 Autonomous Navigation Robot

A simulation-first autonomous mobile robot built with **ROS 2 and C++**.

The project implements a classical autonomous navigation pipeline using wheel odometry, IMU/LiDAR sensing, EKF state estimation, occupancy-grid mapping, A* global planning, replanning, and closed-loop path following in Gazebo.

## Features

- Custom differential-drive wheel odometry
- EKF sensor fusion using wheel odometry and IMU
- LiDAR-based occupancy-grid mapping
- Custom C++ A* global planner
- Unknown-space traversal with higher planning cost
- Obstacle inflation for robot clearance
- Periodic replanning with path-commitment hysteresis
- Lookahead-based closed-loop path following
- Dynamic navigation goals
- Navigation evaluation against Gazebo ground truth

## Environment

| Component | Environment |
| --- | --- |
| OS | Ubuntu 26.04 LTS |
| ROS 2 | Lyrical |
| Simulator | Gazebo Sim 10 |
| Language | C++17 |
| State estimation | `robot_localization` EKF |
| Simulation interface | `ros_gz_sim`, `ros_gz_bridge` |

The occupancy grid covers a **10 m × 10 m** area with a resolution of **0.1 m/cell**.

## Key ROS 2 Interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR range measurements |
| `/imu` | `sensor_msgs/msg/Imu` | IMU measurements |
| `/odom` | `nav_msgs/msg/Odometry` | Custom wheel odometry |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF-filtered state estimate |
| `/gazebo_odom` | `nav_msgs/msg/Odometry` | Gazebo ground-truth odometry |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy-grid map |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Navigation goal |
| `/planned_path` | `nav_msgs/msg/Path` | Current A* global path |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/goal_reached` | `std_msgs/msg/Bool` | Goal-completion event |

## Running the Project

Build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

Launch the complete simulation and navigation stack:

```bash
ros2 launch my_robot simulation.launch.py
```

Gazebo starts automatically with the robot, sensors, mapping, localization, planner, controller, and evaluation nodes.

### Send a Navigation Goal

For example, send the robot to `(3.0, 3.0)`:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}" \
--once
```

The robot builds the map from LiDAR observations, plans a route to the requested position, and updates the route as it moves.

## Demo

![Autonomous navigation demo](/src/my_robot/docs/navigation_demo.gif)

The demo shows the robot mapping the environment, planning around obstacles, replanning during motion, and autonomously reaching the requested goal.

## Evaluation

The navigation stack was tested from the same starting position in three scenarios of increasing difficulty. Each scenario was repeated three times.

| Scenario | Goal | Success | Mean Goal Error | Mean Distance | Localization RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| Straight path | `(4, 0)` | 3/3 | 0.098 m | 3.908 m | 0.006 m |
| Obstacle detour | `(3, 3)` | 3/3 | 0.098 m | 4.751 m | 0.005 m |
| Wall navigation | `(-4, 0)` | 3/3 | 0.098 m | 9.990 m | 0.004 m |

The evaluation node measures traveled distance and final goal error from the EKF estimate and compares localization against Gazebo ground truth.

## License

This project is licensed under the **Apache License 2.0**.

