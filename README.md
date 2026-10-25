# ROS2 Autonomous Navigation Robot

A simulation-first autonomous mobile robot built with **ROS 2 and C++**.

The project implements a classical navigation pipeline using LiDAR, EKF state estimation, occupancy-grid mapping, A* global planning, periodic replanning, and closed-loop path following in Gazebo.

## Current Status

The robot can receive a navigation goal, build and update an occupancy grid from LiDAR data, plan a route with A*, and follow the path using a lookahead controller. The planner periodically replans using the latest robot pose and map, allowing the path to update as previously unknown space becomes visible.

Unknown cells are traversable with a higher cost than known free cells, while occupied cells remain blocked.

## Key ROS 2 Interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR range measurements |
| `/imu` | `sensor_msgs/msg/Imu` | IMU measurements |
| `/odom` | `nav_msgs/msg/Odometry` | Custom wheel odometry |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF-filtered state estimate |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy-grid map |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Navigation goal |
| `/planned_path` | `nav_msgs/msg/Path` | Current A* global path |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity commands |

## Mapping and Planning

LiDAR measurements are transformed into the odometry frame using the latest EKF pose estimate. Ray tracing marks observed free space and obstacle endpoints in a persistent occupancy grid.

The custom C++ A* planner converts the robot and goal positions into grid coordinates and searches for a route. Known free cells are preferred, unknown cells receive a higher traversal cost, and occupied cells are blocked.

Planning runs periodically so the route can change as the robot moves and the map is updated.

## Path Following

The robot follows `/planned_path` using a lookahead-based controller instead of attempting to visit every individual A* grid cell.

The controller tracks progress along the path, selects a target farther ahead, and uses proportional heading control to generate angular velocity. Forward speed is reduced when the robot is poorly aligned with the path and when approaching the final goal.

This produces smoother motion than repeatedly stopping and rotating at each waypoint.

## Running the Project

Build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

Launch the simulation and navigation stack:

```bash
ros2 launch my_robot simulation.launch.py
```

Gazebo starts automatically with the simulation running.

### Send a Navigation Goal

For example, send the robot to:

```text
x = 3.0 m
y = 3.0 m
```

using:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}" \
--once
```

The planner will generate a path from the robot's current location to the requested goal and publish it on:

```text
/planned_path
```

The path follower then automatically generates velocity commands on:

```text
/cmd_vel
```

### RViz

When running under WSL, RViz can be started with:

```bash
QT_QPA_PLATFORM=xcb rviz2
```

Useful displays include:

```text
Fixed Frame: odom

Map:
    /map

Path:
    /planned_path
```

## Current Limitations

The planner currently treats the robot as a point, so planned paths do not yet account for the physical size of the chassis. The occupancy map also uses a simple free/occupied representation, and local dynamic-obstacle avoidance is not yet implemented.

## Planned additions

- Implementing obstacle inflation, followed by navigation testing.
- Full system integration, documentation, and demonstrations.
- Random obstacle generation for broader simulation testing (stretch goal).

