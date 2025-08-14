# Wall Follower Deployment Guide

This package provides multiple versions of wall following nodes for different ROS2 environments and Python versions.

## Node Variants

### For ROS2 Foxy (Python 3.8) - Real TurtleBot3
Use these nodes when deploying on real TurtleBot3 hardware with ROS2 Foxy:

- **foxy_compatible_wall_finder**: Python 3.8 compatible wall finder service
- **foxy_compatible_wall_following**: Python 3.8 compatible wall following client
- **Launch file**: `foxy_compatible_wall_following.launch.py`

### For ROS2 Humble/Later (Python 3.10+) - Development Environment
Use these nodes for development and simulation:

- **improved_service_wall_finder**: Enhanced wall finder service with advanced features
- **improved_service_wall_following**: Enhanced wall following client
- **Launch file**: `start_wall_following.launch.py`

## Quick Start

### Real TurtleBot3 (ROS2 Foxy)
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select wall_follower

# Source the workspace
source install/setup.bash

# Launch the system
ros2 launch wall_follower foxy_compatible_wall_following.launch.py
```

### Development/Simulation (ROS2 Humble+)
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select wall_follower

# Source the workspace
source install/setup.bash

# Launch the system
ros2 launch wall_follower start_wall_following.launch.py
```

## Manual Node Execution

### Real TurtleBot3 (Terminal 1 - Wall Finder Service)
```bash
ros2 run wall_follower foxy_compatible_wall_finder
```

### Real TurtleBot3 (Terminal 2 - Wall Following Client)
```bash
ros2 run wall_follower foxy_compatible_wall_following
```

## Key Differences

### Python 3.8 Compatible Versions
- Removed f-string formatting (uses .format() instead)
- Fallback implementations for numpy functions
- Compatible deque usage with fallback class
- No type hints that require Python 3.9+

### Enhanced Versions (Python 3.10+)
- Modern Python features (f-strings, type hints)
- Full numpy integration
- Advanced filtering algorithms
- More sophisticated state machine logic

## Troubleshooting

### "type object is not subscriptable" error
This typically occurs when using Python 3.9+ features on Python 3.8. Use the `foxy_compatible_*` nodes instead.

### Service connection issues
Ensure both nodes are running and the service topic `/find_wall` is available:
```bash
ros2 service list | grep find_wall
```

### Robot not moving
Check that the laser scan topic `/scan` is publishing data:
```bash
ros2 topic echo /scan --once
```

## Parameters

Both versions support the same parameters, configurable via launch files or command line:

- `target_distance`: Desired distance from wall (default: 0.25m)
- `forward_speed`: Normal forward speed (default: 0.15 m/s)
- `turn_speed`: Angular velocity for turns (default: 0.3 rad/s)
- `danger_distance`: Emergency stop distance (default: 0.12m)
- `front_obstacle_distance`: Front obstacle detection (default: 0.25m)

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Laser scanner data

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Services
- `/find_wall` (wall_follower_interfaces/FindWall): Wall finder service

## Dependencies

Ensure the following packages are installed:
- `wall_follower_interfaces` (for custom service definitions)
- `sensor_msgs`, `geometry_msgs` (standard ROS2 message packages)
- `rclpy` (ROS2 Python client library)

## Hardware Requirements

- Differential drive robot (TurtleBot3 or compatible)
- 360-degree laser scanner (LiDAR)
- ROS2 Foxy or later
- Python 3.8 or later
