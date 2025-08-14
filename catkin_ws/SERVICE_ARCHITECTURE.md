# Service-Based Wall Following Architecture

## Overview

I've restructured the wall following system to use a service-based architecture as requested:

1. **Wall Finder Service Server** - Provides `/find_wall` service
2. **Wall Following Service Client** - Calls wall finder service before starting wall following
3. **Integrated Launch Files** - Runs both nodes together

## Architecture

```
┌─────────────────────┐    Service Call    ┌──────────────────────┐
│ Wall Following Node │ ───────────────────> │ Wall Finder Service  │
│  (Service Client)   │                     │    (Service Server)  │
└─────────────────────┘                     └──────────────────────┘
          │                                              │
          │ /cmd_vel                             /cmd_vel │
          ▼                                              ▼
      ┌─────────────────────────────────────────────────────┐
      │                Robot Platform                       │
      └─────────────────────────────────────────────────────┘
                              ▲
                    /scan     │
                ┌─────────────────────┐
                │   LiDAR Sensor      │
                └─────────────────────┘
```

## Service Interface

**Service**: `/find_wall` (wall_follower_interfaces/FindWall)

```
# Request (empty)
---
# Response
bool wallfound    # True if wall found and positioned successfully
```

## New Nodes

### 1. `improved_service_wall_finder.py`
- **Type**: Service Server
- **Service**: `/find_wall`
- **Behavior**: 
  - Idle state when not serving requests
  - Executes wall finding FSM when service called
  - Returns success/failure response
  - Enhanced with smooth control and robust filtering

### 2. `improved_service_wall_following.py`
- **Type**: Service Client + Wall Follower
- **Workflow**:
  1. Starts by calling `/find_wall` service
  2. Waits for wall finder to complete
  3. Begins wall following if service succeeds
  4. Handles error states if service fails

## State Machines

### Wall Finder Service States
1. **IDLE** - Waiting for service calls
2. **MOVING_FORWARD** - Moving forward until obstacle detected
3. **TURNING_LEFT** - Turning left while moving slowly
4. **ALIGNING_TO_WALL** - Rotating to position wall on right side
5. **WALL_FOUND** - Wall positioned successfully

### Wall Following States
1. **INITIALIZING** - Node startup
2. **FINDING_WALL** - Waiting for wall finder service
3. **WALL_FOLLOWING** - Normal wall following behavior
4. **DANGER_AVOIDANCE** - Emergency avoidance maneuvers
5. **ERROR_STATE** - Error handling and recovery

## Launch Files

### 1. `improved_service_wall_following.launch.py`
Launches both service server and client for real robot use:
```bash
ros2 launch wall_follower improved_service_wall_following.launch.py
```

### 2. `test_improved_wall_following.launch.py`
Complete testing setup with fake laser data:
```bash
ros2 launch wall_follower test_improved_wall_following.launch.py
```

## Key Features

### Service-Based Architecture Benefits
- **Modularity**: Wall finder and follower are separate, reusable components
- **Reliability**: Clear success/failure feedback through service response
- **Coordination**: Automatic sequencing from wall finding to wall following
- **Error Handling**: Proper error states and recovery mechanisms

### Enhanced Performance Features
- **Smooth Control**: Velocity smoothing and proportional control
- **Robust Filtering**: Multi-level noise reduction and outlier rejection
- **State Stability**: Hysteresis and timing constraints prevent oscillations
- **Simulation Optimized**: Parameters tuned for Gazebo performance

## Usage Examples

### Basic Usage
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && source install/setup.bash

# Run complete system with test data
ros2 launch wall_follower test_improved_wall_following.launch.py
```

### Manual Service Testing
```bash
# Terminal 1: Run wall finder service
ros2 run wall_follower improved_service_wall_finder

# Terminal 2: Run laser publisher (for testing)
ros2 run wall_follower enhanced_laser_test_publisher

# Terminal 3: Call service manually
ros2 service call /find_wall wall_follower_interfaces/srv/FindWall

# Terminal 4: Run wall following (will auto-call service)
ros2 run wall_follower improved_service_wall_following
```

### Parameter Customization
```bash
# Custom wall finder parameters
ros2 run wall_follower improved_service_wall_finder --ros-args \
    -p max_speed:=0.15 \
    -p target_wall_distance:=0.3 \
    -p service_timeout:=45.0

# Custom wall following parameters  
ros2 run wall_follower improved_service_wall_following --ros-args \
    -p target_distance:=0.3 \
    -p proportional_gain:=1.5 \
    -p forward_speed:=0.12
```

## Troubleshooting

### Service Connection Issues
- Ensure both nodes are running
- Check service availability: `ros2 service list`
- Verify interface: `ros2 interface show wall_follower_interfaces/srv/FindWall`

### Wall Finding Failures
- Check laser data: `ros2 topic echo /scan`
- Verify robot can move: `ros2 topic echo /cmd_vel`
- Adjust timeout parameters if needed

### Performance Issues
- Reduce speeds for unstable simulation
- Increase filter window size for noisy sensors
- Adjust hysteresis margins for oscillation problems

## Integration with Existing Code

The new service-based nodes are **fully compatible** with existing interfaces:
- Uses same `/scan` topic for laser input
- Publishes to same `/cmd_vel` topic for robot control
- Uses existing `wall_follower_interfaces` package
- Maintains same parameter structure for easy migration

## File Summary

**New Files Created:**
- `improved_service_wall_finder.py` - Service server node
- `improved_service_wall_following.py` - Service client + wall follower node
- `improved_service_wall_following.launch.py` - Launch both nodes
- Updated `test_improved_wall_following.launch.py` - Testing with service architecture

**Dependencies:**
- `wall_follower_interfaces` package (existing)
- `sensor_msgs`, `geometry_msgs` (standard ROS2)
- `numpy` for enhanced filtering

The system is now ready for smooth, service-coordinated wall following in simulation!
