# GitHub Copilot Instructions for ROS 2 Wall Following Robotics Project

## Project Overview

This is a comprehensive ROS 2 Humble robotics project focused on autonomous wall following capabilities using laser-based navigation. The project demonstrates professional robotics development practices with multiple simulation environments, custom interfaces, and modular architecture.

## Architecture & Technology Stack

### Core Technologies
- **ROS 2 Humble**: Primary robotics middleware
- **Python 3**: Main programming language for node implementations
- **Gazebo Classic & Gazebo Garden**: Dual simulation environment support
- **LIDAR/Laser Scanner**: Primary sensor for navigation and obstacle detection
- **Custom Interfaces**: Service and action definitions for robot coordination

### Package Structure
```
wall_follower/                    # Main robotics package
├── wall_follower/               # Python node implementations
│   ├── wall_following_*.py     # Multiple wall following strategies
│   ├── find_wall_service_*.py  # Wall detection service implementations
│   └── *_ros2_version.py       # Migrated implementations
├── launch/                      # ROS 2 launch configurations
├── worlds/                      # Gazebo simulation worlds (.sdf files)
└── setup.py                    # Package configuration with entry points

wall_follower_interfaces/         # Custom ROS 2 interfaces
├── srv/FindWall.srv             # Wall detection service definition
├── action/OdomRecord.action     # Odometry recording action
└── CMakeLists.txt               # Interface build configuration

actions_quiz/                     # Additional learning exercises
└── actions_quiz_msg/            # Action message definitions
```

## Development Guidelines for AI Assistants

### 1. ROS 2 Node Development Patterns

**Always follow these ROS 2 patterns when creating or modifying nodes:**

```python
# Standard ROS 2 node structure
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class YourRobotNode(Node):
    def __init__(self):
        super().__init__('node_name')
        
        # 1. Declare parameters first
        self.declare_parameter('param_name', default_value)
        self.param = self.get_parameter('param_name').value
        
        # 2. Create publishers/subscribers
        self.publisher = self.create_publisher(MessageType, 'topic', 10)
        self.subscription = self.create_subscription(
            MessageType, 'topic', self.callback, 10)
        
        # 3. Services and actions
        self.service = self.create_service(ServiceType, 'service', self.callback)
        self.action_client = ActionClient(self, ActionType, 'action')
        
    def callback(self, msg):
        # Process messages
        pass

def main():
    rclpy.init()
    node = YourRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
```

### 2. Laser Scan Processing

**This project heavily uses LaserScan data. Follow these patterns:**

```python
from sensor_msgs.msg import LaserScan
import math

def process_laser_data(self, scan_msg):
    ranges = list(scan_msg.ranges)
    
    # Always filter invalid readings
    valid_ranges = []
    for i, r in enumerate(ranges):
        if r > 0 and r != float('inf') and not math.isnan(r):
            valid_ranges.append((i, r))
    
    # Common ray indices for robot orientation
    front_idx = len(ranges) // 2  # Forward direction
    right_idx = 0                 # Right side (90 degrees)
    left_idx = len(ranges) - 1    # Left side (270 degrees)
    
    # Use specific rays for wall following (project convention)
    right_side_ray = 145 if len(ranges) > 145 else 0
```

### 3. Wall Following Control Logic

**The project implements PID-based wall following. Maintain these patterns:**

```python
def calculate_wall_following_velocity(self, right_distance, front_distance):
    # Constants used in project
    DESIRED_DISTANCE = 0.3  # meters from wall
    MAX_LINEAR_SPEED = 0.2  # m/s
    MAX_ANGULAR_SPEED = 0.3 # rad/s
    
    # PID control for wall following
    error = right_distance - DESIRED_DISTANCE
    angular_velocity = self.kp * error
    
    # Obstacle avoidance logic
    if front_distance < 0.5:  # Obstacle ahead
        linear_velocity = 0.0
        angular_velocity = MAX_ANGULAR_SPEED  # Turn away
    else:
        linear_velocity = MAX_LINEAR_SPEED
    
    return linear_velocity, angular_velocity
```

### 4. Service and Action Integration

**Follow the established service/action patterns:**

```python
# Service client usage (wall detection)
from wall_follower_interfaces.srv import FindWall

def call_wall_finder_service(self):
    client = self.create_client(FindWall, 'find_wall')
    if not client.wait_for_service(timeout_sec=10.0):
        self.get_logger().error('FindWall service not available')
        return False
    
    request = FindWall.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    return future.result().wallfound

# Action client usage (odometry recording)
from wall_follower_interfaces.action import OdomRecord
from action_msgs.msg import GoalStatus

def start_odometry_recording(self):
    self.action_client.wait_for_server()
    goal = OdomRecord.Goal()
    future = self.action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self, future)
```

### 5. Launch File Conventions

**When creating launch files, follow the project's multi-environment pattern:**

```python
# Standard launch file structure
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='wall_world.sdf')
    
    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        # Gazebo Garden or Classic based on argument
    )
    
    # Robot nodes
    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_following_enhanced',  # From setup.py entry_points
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time'),
        gazebo,
        wall_follower_node
    ])
```

### 6. Parameter Management

**Always use ROS 2 parameter system for configuration:**

```python
# In node constructor
self.declare_parameter('robot.max_speed', 0.2)
self.declare_parameter('robot.wall_distance', 0.3)
self.declare_parameter('control.kp_gain', 1.0)
self.declare_parameter('safety.obstacle_threshold', 0.5)

# Access parameters
self.max_speed = self.get_parameter('robot.max_speed').value
```

### 7. Error Handling and Logging

**Follow the project's logging and error handling patterns:**

```python
def robust_operation(self):
    try:
        # Operation logic
        self.get_logger().info("Starting operation...")
        
        # Timeout handling
        start_time = time.time()
        timeout = 30.0
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # Operation loop
            if self.check_success_condition():
                self.get_logger().info("Operation completed successfully")
                return True
                
        self.get_logger().warn("Operation timed out")
        return False
        
    except Exception as e:
        self.get_logger().error(f"Operation failed: {str(e)}")
        return False
```

## Simulation Environment Guidelines

### 1. Gazebo Integration
- **Classic Gazebo**: Use for compatibility with older systems
- **Gazebo Garden**: Preferred for new development with modern features
- **World Files**: Located in `worlds/` directory, use SDF format
- **Bridge Configuration**: Handle topic bridging between Gazebo versions

### 2. Testing Worlds
- `wall_world.sdf`: Basic wall following environment
- `garden_wall_world.sdf`: Gazebo Garden optimized version
- Custom worlds should include appropriate lighting and physics settings

## Code Quality Standards

### 1. Documentation
- **Docstrings**: Every class and method must have descriptive docstrings
- **Comments**: Explain complex robotics algorithms, especially PID control and state machines
- **Migration Notes**: When adapting ROS 1 code, document the conversion process

### 2. Variable Naming
- Use descriptive names: `front_laser_distance` not `fd`
- Follow ROS conventions: `cmd_vel_pub` for publishers
- Robot-specific prefixes: `wall_distance`, `obstacle_threshold`

### 3. State Management
- Use clear state enums or constants
- Document state transitions in robot behavior
- Handle edge cases in sensor data processing

## Common Patterns and Anti-patterns

### ✅ Do This
```python
# Clear robot state management
class RobotState:
    FINDING_WALL = "finding_wall"
    FOLLOWING_WALL = "following_wall"
    AVOIDING_OBSTACLE = "avoiding_obstacle"

# Robust sensor data validation
if (distance > 0 and distance != float('inf') and not math.isnan(distance)):
    # Process valid distance

# Proper rate control
rate = self.create_rate(10)  # 10 Hz standard for control loops
```

### ❌ Avoid This
```python
# Don't use hardcoded magic numbers
if laser_scan.ranges[90] < 0.3:  # What is 90? What is 0.3?

# Don't ignore invalid sensor data
distance = laser_scan.ranges[0]  # Could be inf, nan, or negative

# Don't use blocking operations without timeouts
while True:  # This can hang forever
```

## Integration with Existing Code

### When Modifying Existing Nodes:
1. **Preserve Interface Compatibility**: Don't change service/action definitions without updating all dependent nodes
2. **Parameter Backward Compatibility**: Maintain existing parameter names
3. **Topic Naming**: Follow the established topic naming conventions (`/cmd_vel`, `/scan`, `/odom`)
4. **Entry Points**: Update `setup.py` entry_points when adding new executables

### When Adding New Features:
1. **Follow Package Structure**: New nodes go in `wall_follower/` directory
2. **Add Launch Files**: Create corresponding launch configurations
3. **Update Documentation**: Extend this file with new patterns
4. **Test Integration**: Ensure compatibility with both Gazebo environments

## Debugging and Troubleshooting

### Common Issues:
1. **Laser Data Issues**: Always check for valid ranges before processing
2. **Service Timeouts**: Implement proper timeout handling for service calls
3. **Parameter Loading**: Verify parameter names match launch file configurations
4. **Gazebo Synchronization**: Ensure `use_sim_time` parameter is correctly set

### Debugging Tools:
```bash
# Monitor topics
ros2 topic echo /scan
ros2 topic echo /cmd_vel

# Check service availability
ros2 service list
ros2 service call /find_wall wall_follower_interfaces/srv/FindWall

# Parameter inspection
ros2 param list
ros2 param get /wall_follower_node parameter_name
```

This project represents a mature ROS 2 robotics system with professional development practices. When contributing, maintain the established patterns for consistency and reliability.
