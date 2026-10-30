# Drone Obstacle Avoidance: ROS 2 Humble + Gazebo

A quadrotor model flies from point A to point B through a maze of walls and
cylinders. A* computes the global path, a simulated 2D lidar adds local
obstacle repulsion.

![demo](media/demo.gif)

## Features
- Procedural world generator (`worlds/complex.world` and `config/map.yaml` from one description)
- A* global planner on an inflated grid, with path shortcutting
- Lidar-based local repulsion (safety layer)
- Run metrics: time, path length, minimum clearance, close calls
- Unit tests (pytest) and CI (GitHub Actions)

## Architecture
    Gazebo (world + drone + lidar)
       |  /drone/scan (LaserScan)   /drone/odom (Odometry)
       v
    nav_node (A* + waypoint following + repulsion)
       |  /drone/cmd_vel (Twist)    /drone/plan (Path)
       v
    Gazebo (planar_move plugin)

## Requirements
- Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
- `sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rviz2 python3-yaml`

## Build and run
    mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
    git clone https://github.com/YOUR_USERNAME/drone-obstacle-avoidance.git drone_avoidance
    cd ~/drone_ws
    colcon build --symlink-install --packages-select drone_avoidance
    source install/setup.bash
    ros2 launch drone_avoidance complex.launch.py

New random map:

    python3 src/drone_avoidance/drone_avoidance/generate_world.py --out src/drone_avoidance --random 6 --seed 7

## Tests
    python3 -m pytest test/test_planner.py -q

## Limitations
- Kinematic model at a fixed altitude (no flight dynamics, no collision body)
- Static obstacles known in advance for the global plan

## Roadmap
- Dynamic obstacles and replanning
- Occupancy grid built from the lidar
- Batch benchmark over many seeds
- PX4 SITL with 3D planning

## License
MIT
