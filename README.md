# ROS2 Autonomous Navigation Robot

This project develops an autonomous mobile robot in ROS 2 and C++. The goal is to move from sensor data and state estimation to safe, closed-loop navigation: build a local map, plan a route around obstacles, and command the robot to follow that route.

## Current status

The current system delivers occupancy-grid mapping and A* global planning. It has been tested in both open space and a scene containing a mapped obstacle.

- LiDAR scans are converted into a local `nav_msgs/msg/OccupancyGrid`.
- Grid cells represent unknown, free, and occupied space.
- Filtered EKF odometry supplies the robot pose used to place scans in the map.
- Stale or missing filtered odometry causes scans to be skipped, preventing mapping with an old pose.
- An A* planner searches the occupancy grid for a collision-free route.
- The resulting `nav_msgs/msg/Path` is published for RViz visualization.

## Architecture

```text
LiDAR (/scan) + EKF odometry (/odometry/filtered)
                    |
                    v
          Occupancy-grid mapper (/map)
                    |
                    v
        A* global planner (/planned_path)
                    |
                    v
              RViz visualization
```

### Key ROS 2 interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR range measurements |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF-filtered pose estimate |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Local obstacle/free-space grid |
| `/planned_path` | `nav_msgs/msg/Path` | A* global path for visualization and future tracking |

## Mapping and planning

Each incoming LiDAR scan is transformed using the latest filtered robot pose. Ray-tracing marks traversed cells as free and scan endpoints as occupied, while cells that have not been observed remain unknown. Before a scan is processed, the mapper verifies that filtered odometry has been received and is no more than approximately 0.5 seconds old.

The planner converts start and goal positions into grid cells, runs A* over traversable cells, and publishes the recovered route as a ROS 2 path. This separates perception and planning cleanly and provides a useful foundation for motion control.

## Running the project

1. Build the ROS 2 workspace with `colcon build` and source the workspace.
2. Start the robot or simulator, LiDAR publisher, and EKF odometry source.
3. Launch the mapping and planning nodes.
4. Open RViz and display the occupancy grid and planned path.
5. Set or provide a planning goal and inspect the resulting route.

Launch-file names and parameter details will be documented as the project is consolidated.

## Current limitations

- The path is visualized only; the robot does not yet drive along it.
- Mapping and planning depend on timely, accurate EKF odometry.
- The planner is a basic global A* implementation; it does not yet include path smoothing, local obstacle avoidance, or replanning during motion.
- Obstacle scenarios are currently controlled rather than procedurally generated.

## Planned additions

- Closed-loop path following that converts `/planned_path` into `/cmd_vel` commands.
- Local obstacle handling, safe stopping behavior, and replanning while moving.
- Path smoothing, parameter tuning, recovery behavior, and testing across more scenarios.
- Full system integration, documentation, and demonstrations.
- Random obstacle generation for broader simulation testing (stretch goal).

