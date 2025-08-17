
# Wall Follower for TurtleBot3



## System Components

### wall_follower Package


- **enhanced_wall_following.py**: Advanced wall following with improved control logic

### wall_follower_interfaces Package 

- **FindWall.srv**: Service definition for finding walls
- **OdomRecord.action**: Action definition for recording odometry data

## Mock Testing Mode

For testing without a real robot or Gazebo, we've created mock versions of some components:

- **mock_wall_finder.py**: A mock version of the wall finder service
- **mock_odom_recorder.py**: A mock version of the odometry recorder action server
- **run_mock_demo.sh**: A script to run the full system with the mock components

## Running the Project

### Running with Mock Components (No Robot Required)

#### Basic Wall Following:
```bash
# Run the basic demo script
~/ros2_ws/src/wall_follower/run_mock_demo.sh
```

#### Enhanced Wall Following:
```bash
# Run the enhanced demo script with improved control
~/ros2_ws/src/wall_follower/run_enhanced_mock_demo.sh
```

### Running with a Real TurtleBot3 in Gazebo

1. Launch Gazebo with TurtleBot3:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. In a new terminal, launch the wall follower system:

   #### Basic Wall Following:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch wall_follower main.launch.py
   ```
   
   #### Enhanced Wall Following (Recommended):
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch wall_follower enhanced_wall_follower.launch.py
   ```

3. With launch parameters (Enhanced version only):
   ```bash
   ros2 launch wall_follower enhanced_wall_follower.launch.py target_distance:=0.3 max_speed:=0.2 kp:=2.5 kd:=0.8
   ```

## Issues and Solutions

### Package Not Found Issue

If you encounter the "Package wall_follower not found" error, this is due to a ROS 2 indexing issue. Current workarounds:

1. Run each node individually using their full paths:
   ```bash
   ~/ros2_ws/install/wall_follower/lib/wall_follower/wall_finder
   ~/ros2_ws/install/wall_follower/lib/wall_follower/odom_recorder
   ~/ros2_ws/install/wall_follower/lib/wall_follower/wall_following
   ```

2. Use the included shell script for mock testing:
   ```bash
   ~/ros2_ws/src/wall_follower/run_mock_demo.sh
   ```

### Gazebo Issues

If you encounter issues with classic Gazebo crashing on startup (exit code 255), this is a known issue. There are two approaches to solve this:

1. Kill any lingering Gazebo processes and retry:
   ```bash
   pkill -f gazebo
   ```

2. Make sure you have the correct environment variable set:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

3. **Recommended Solution**: Migrate to Gazebo Garden (see the new Gazebo Garden integration section below).

## Gazebo Garden Integration

Due to stability issues with classic Gazebo, we've added support for Gazebo Garden, the next generation of Gazebo simulation.

### Installing Gazebo Garden

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Install Gazebo Garden
sudo apt install gz-garden

# Install ROS 2 - Gazebo Garden integration
sudo apt install ros-humble-ros-gz
```

### Running with Gazebo Garden

```bash
# Set TurtleBot3 model (if not already set)
export TURTLEBOT3_MODEL=burger

# Launch the integrated Garden Wall Follower
ros2 launch wall_follower garden_wall_follower.launch.py
```

### Testing Gazebo Garden Integration

To simply test if Gazebo Garden is working properly with ROS 2 bridge:

```bash
ros2 launch wall_follower test_garden.launch.py
```

For more details about migrating from classic Gazebo to Gazebo Garden, see the MIGRATION_PLAN.md file in the src directory.

## Enhanced Wall Following Features

The enhanced wall following implementation includes several improvements over the basic version:

1. **PD Control System**: Uses proportional-derivative control for smoother navigation
2. **Corner Detection**: Detects corners and adjusts navigation strategy accordingly
3. **Adaptive Speed Control**: Automatically reduces speed during turns for better control
4. **State Machine Architecture**: Uses states to manage different behaviors (following, turning)
5. **Safety Features**: Emergency stop when obstacles are too close
6. **Runtime Parameter Tuning**: Adjustable parameters via ROS 2 parameters
7. **More Advanced Laser Processing**: Better handling of laser scan data

## Visualization Tools

We've added a simple visualization tool to help debug and monitor the robot's behavior:

```bash
# Install required packages
sudo apt update && sudo apt install -y python3-pip python3-tk
pip3 install matplotlib numpy

# Run the visualization
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/wall_follower/wall_follower/visualize_robot.py
```

The visualization shows:
- Robot position and orientation
- Laser scan data around the robot
- Current linear and angular velocity
- Real-time path tracking

## Future Improvements

1. Fix the package indexing issue to allow launching via ROS 2 launch commands
2. Enhance visualization tools with more features
3. Implement dynamic obstacle avoidance while wall following
4. Add more advanced wall detection algorithms using machine learning
5. Create a web interface for remote control and monitoring
6. Complete the integration with Gazebo Garden by adding support for custom worlds
7. Create proper URDF/SDF models with wall setups for testing the wall follower algorithm

