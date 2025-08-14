# Wall Following FSM Nodes

This package contains two ROS2 Python nodes implementing finite state machines for wall finding and wall following using a differential drive robot with LiDAR.

## Nodes

### 1. wall_finder_fsm
Implements a finite state machine to find and approach a wall for wall following.

**States:**
- `MOVING_FORWARD`: Robot moves forward at full speed when path is clear
- `TURNING_LEFT`: Robot turns left while moving slowly when obstacle detected ahead
- `ALIGNING_TO_WALL`: Robot rotates to position wall on right side
- `WALL_FOUND`: Wall successfully positioned for following

**Parameters:**
- `max_speed`: Maximum forward speed (default: 0.3 m/s)
- `slow_speed`: Slow forward speed when turning (default: 0.1 m/s)
- `turn_speed`: Angular velocity for turning (default: 0.5 rad/s)
- `front_threshold_far`: Distance threshold for free path (default: 0.52 m)
- `front_threshold_near`: Distance threshold for obstacle detection (default: 0.48 m)
- `front_threshold_close`: Distance threshold for close obstacles (default: 0.3 m)
- `target_wall_distance`: Target distance to wall (default: 0.25 m)
- `wall_distance_tolerance`: Tolerance for wall distance (default: 0.05 m)
- `filter_window_size`: Moving average filter window size (default: 5)
- `hysteresis_margin`: Hysteresis margin to prevent oscillations (default: 0.02 m)

### 2. wall_following_fsm
Implements a finite state machine for following a wall on the robot's right side.

**States:**
- `WALL_FOLLOWING`: Normal wall following behavior
- `DANGER_AVOIDANCE`: Emergency avoidance when too close to wall

**Parameters:**
- `target_distance`: Target distance from wall (default: 0.25 m)
- `min_safe_distance`: Minimum safe distance (default: 0.2 m)
- `max_safe_distance`: Maximum safe distance (default: 0.3 m)
- `danger_distance`: Distance threshold for danger avoidance (default: 0.15 m)
- `forward_speed`: Normal forward speed (default: 0.2 m/s)
- `slow_speed`: Slow forward speed (default: 0.1 m/s)
- `turn_speed_slight`: Small angular velocity for corrections (default: 0.3 rad/s)
- `turn_speed_strong`: Large angular velocity for avoidance (default: 0.8 rad/s)
- `reverse_speed`: Reverse speed for danger avoidance (default: -0.1 m/s)
- `reverse_duration`: Time to reverse in danger avoidance (default: 0.5 s)
- `front_obstacle_distance`: Distance threshold for front obstacles (default: 0.3 m)

## Topics

**Subscribed:**
- `/scan` (sensor_msgs/LaserScan): LiDAR data input

**Published:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

## Launch Files

### Individual Nodes
```bash
# Run only wall finder
ros2 launch wall_follower wall_finder_only.launch.py

# Run only wall following
ros2 launch wall_follower wall_following_only.launch.py

# Test wall following with fake laser data
ros2 launch wall_follower test_wall_following.launch.py
```

### Combined Workflow
```bash
# Run wall finder followed by wall following (30-second delay)
ros2 launch wall_follower wall_following_fsm.launch.py
```

## Manual Execution

### Run individual nodes:
```bash
# Wall finder
ros2 run wall_follower wall_finder_fsm

# Wall following
ros2 run wall_follower wall_following_fsm

# Test laser publisher (for testing without real robot)
ros2 run wall_follower laser_test_publisher
```

### With custom parameters:
```bash
ros2 run wall_follower wall_following_fsm --ros-args -p target_distance:=0.3 -p forward_speed:=0.15
```

## Algorithm Details

### Sensor Processing
- Uses sector-based distance measurements:
  - **Front sector**: ±30° from forward direction
  - **Right sector**: 80°-100° from forward direction (right side)
- Applies median filtering with configurable window size to reduce noise
- Filters out invalid readings (inf, nan, too close/far)

### Control Logic

#### Wall Finder FSM
1. **Moving Forward**: Robot moves at full speed until obstacle detected
2. **Turning Left**: Slow forward motion with left turn when obstacle ahead
3. **Aligning**: Pure rotation to position wall on right side
4. **Wall Found**: Stop and signal successful wall positioning

#### Wall Following FSM
1. **Normal Following**:
   - In range [0.2m, 0.3m]: Move forward maintaining heading
   - Too far (>0.3m): Turn right slightly to approach wall
   - Too close (<0.2m): Turn left slightly to avoid collision
2. **Danger Avoidance**:
   - When distance ≤0.15m: Stop, reverse for 0.5s, rotate left until safe

### Hysteresis
Both FSMs implement hysteresis with configurable margins to prevent oscillations between states.

## Requirements
- ROS2 Humble
- Python 3.8+
- numpy
- sensor_msgs
- geometry_msgs

## Safety Features
- Emergency stop on critical proximity
- Timeout protection in alignment phase
- Smooth state transitions with hysteresis
- Configurable speed and distance parameters
- Non-blocking control loops at 10 Hz

## Testing
Use the test launch file to verify behavior without a real robot:
```bash
ros2 launch wall_follower test_wall_following.launch.py
```

This will start fake laser data publisher along with the wall following node.
