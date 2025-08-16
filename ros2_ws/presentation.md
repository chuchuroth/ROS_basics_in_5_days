demo流程

+ 先main launch， 如果wall finder失败，单独wall following, 或者遇到障碍物，可在键盘调整位置后直接启动wall following

+ 展示odom action
+ rviz
+ rqt -explain structure
+ explain - robot why behave this way

+ __ros1用的是最简单的logic__
+ __ros2用复杂的obstacle avoidance__



逻辑大多没问题，参数设计不合理，因为需要考虑场地和机器人的尺寸，还有机器人的动力学参数也是考量因素。
并且我发现，根据这个课程里的gazebo环境和turtlebot机器人的尺寸，作业里设置的0.3米 0.2米并不合理，起码要0.8米，才不会一直撞墙。

actually doesn't make much difference to use a simple or complicated algorithm, the limit is on the sensor side, not helping to use a complicated algorithm, but really critical to make good use of the laser data, because the critical problem to solve is to correctly calculate the distance to the wall on the right hand side, to make a smooth route, but not possible to make a perfect route, reason is follows:


laser config /| 180 degree

to get correct laser configuration (based on hardware config) , better write some function( add a small debug print) in scripts so in log output can show where is the problem in details, it#s easier to debug problem one by one, first you need meaningful feedback to see where went wrong
在gazego里调试完，到了robot还得重新调试，因为物理参数不一定一样（前后左右是反着的）


hello everyone, today i am going to do a demo on robot following wall using ROS, this is a Course Project ROS Basics in 5 Days, The robot i am working with is a (TurtleBot3) which is running in Barcelona, Spain. You will connect remotely to it , behavior that makes the robot follow along the wall on its right hand side, This means that the robot must be moving forward at a 30cm distance from the wall, having the wall on its right hand side, the entire time.
so let's just start with the demo and after it's done i will walk through the codes and the file structure how they fit together and working as ros node.

0. prepare the terminal: catkin_ws  source all the workspace, this is more convenient, 
1. setup the simulator  it is provided from the instruction so i will copy it (source /home/user/simulation_ws/devel/setup.bash && roslaunch realrobotlab main.launch)
2. launch the launch file  (roslaunch ros_wall_following main.launch)
3. wait till find wall success, then start (rosrun ros_wall_following odom_result_client.py) to print the total distance traveled in real time via the feedback callback , there is multiple ways to print the data real time
4. in case got stucked, use keyboard to move  (rosrun teleop_twist_keyboard teleop_twist_keyboard.py)

next, explain the package structure and the codes

finally, I'd like to introduce some pratical tools, 
1. show action GUI - rosrun actionlib_tools axclient.py /record_odom  - explain (because server defined action is finish one lap then end action and give result... 为什么final odometry是一个list)
explain action server goal  feedback  reult

+ Odometry is a technique in robotics and engineering used to estimate a robot’s change in position over time based on data from motion sensors, such as wheel encoders or inertial sensors. It allows robots (whether wheeled, legged, or otherwise) to keep track of their position relative to a starting point by measuring and integrating movements detected by these sensors
Common applications of odometry include:
+ Mobile robots tracking their location as they navigate an environment.
+ Autonomous vehicles estimating their position between GPS updates.
+  industrial robots verifying task progress through precise movement calculation.
+ Odometry data is typically stored as a list (or array), tuple, or object containing the three values (x, y, θ) for the current pose. Each time sensors are processed, the x, y, and θ values are updated and kept in a data structure for use by the robot’s navigation system.

3. rviz - add - laserscan    explain

i implement much complicated algo in ros2 rosject, one with PID, please refer to that video if you are intreested
thank you very much for your atteintion and i hope you enjoy.

---


ROS2

split project into two parts: one for service interface which is compiled using cmake, the other is main package compiled using python, because conflict between Python and CMake builds. Let's split this into two packages:
wall_follower_interfaces for the ROS2 interfaces (using CMake)
wall_follower for the Python nodes

Modify the setup.py File to Execute Your Python File: entry point…
Modify setup.py to find the .launch.py files : data_files..

node is designed to handle a specific function or capability of the robot, eg. One node for controlling wheel motors, One node for managing a laser rangefinder, One node for face recognition… a 'entry point' can start one or more nodes.

using the MultiThreadedExecutor for better callback handling


Making velocity proportional to how far it needs to turn
Providing smoother transitions
Ensuring it doesn't **overshoot** as much
Stopping more precisely at the target angle


There are a few potential issues I can see:
The laser data filtering only checks for distances > 0.1, but doesn't handle 'inf' or 'nan' values that might come from the laser scanner.
The angular error calculation assumes the laser scan covers 360 degrees uniformly, but this might not be true - we should check the scan angle range.
The proportional gain (0.3) might be too low, causing slow response, or too high, causing oscillations.

so i make all three adjustments: Add better laser data validation and filtering, adjust the angle calculations based on actual laser scan parameters, Tune the control parameters


Better Wall Following Logic:

Uses PD (Proportional-Derivative) control for smoother wall following
Adds lookahead using diagonal measurements to anticipate turns
Adjusts speed dynamically based on wall proximity
Focuses on maintaining consistent right-side distance to the wall
Diagnostic Information:

Prints laser configuration and key readings every 2 seconds
Shows important control parameters (error, P-term, D-term)
Includes velocity commands in the output
Improved Error Handling:

Properly handles invalid laser readings
Uses try-except blocks to prevent crashes
Provides clear error messages
Smoother Motion:

Reduced maximum angular velocity for smoother movement
Applies gentler corrections when near the target distance
Uses derivative control to prevent oscillations








---

## tips
+ always think about hardware config and the effects when it comes to coding
+ add tolerances
+ simple algo works most of the time, so before change algo, try fine tune basic para, like speed or angle
+ avoid over-engineering

---

# Wall Follower for TurtleBot3

This project implements a wall follower behavior for the TurtleBot3 robot in ROS 2 Humble. The system includes three main components:

1. **Wall Finder** - A service that detects walls using the robot's LiDAR sensor
2. **Odometry Recorder** - An action server that records the robot's path
3. **Wall Following** - The main node that implements the wall following behavior

## System Components

### wall_follower Package

- **wall_finder.py**: Provides a service to find the nearest wall
- **odom_recorder.py**: Action server to record odometry data
- **wall_following.py**: Basic wall following implementation
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



---

Here’s a clear, conversational walk-through of those two core files so you can explain them in your demo — focusing on structure, why each part exists, and what the variables/attributes are doing.

---

## **1. `find_wall.py`** — The “get ready” brain

**Overall role:**
This node is a **ROS service server**. When another node calls `find_wall`, it makes the robot turn toward the nearest wall, drive to it, and rotate so the wall is on the robot’s right.

**Structure & flow:**

* **`class FindWallServer`**:
  Holds everything — ROS node setup, subscribers, publisher, and the service callback.

* **`__init__` method**:

  * `rospy.init_node('find_wall_server')` → registers the node.
  * `self.scan = None` → will store the latest LaserScan message.
  * Subscriber to `/scan` → calls `scan_callback` each time laser data arrives, storing it in `self.scan`.
  * Publisher to `/cmd_vel` → sends velocity commands to the robot.
  * Creates the service `/find_wall` → links to `handle_find_wall`.
  * `rospy.spin()` → keeps the node alive.

* **`scan_callback`**:
  Simple — just saves incoming laser scan messages in `self.scan` so the rest of the code can use them.

* **`handle_find_wall`**:

  1. **Wait for scan data** — so we don’t move with empty sensor readings.
  2. **Step 1: Turn toward the nearest wall**

     * Looks through `self.scan.ranges` to find the smallest distance (closest object).
     * Calculates if that closest beam is directly ahead; if not, rotates toward it by setting `twist.angular.z`.
  3. **Step 2: Move forward until \~0.3m away**

     * Drives straight (`twist.linear.x = 0.2`) until the front beam is ≤ 0.3 m.
  4. **Step 3: Align so wall is on right side**

     * Rotates until beam index 270 points at the wall.
  5. **Return `FindWallResponse(wallfound=True)`** — signals to the caller that we’re done.

**Key variables:**

* `self.scan` → latest LaserScan data (distance readings in meters).
* `front_idx` → index in `ranges` array for the straight-ahead laser beam.
* `target_idx = 270` → index representing the beam to the robot’s right side.
* `twist` → a `geometry_msgs/Twist` message for setting linear & angular velocity.

---

## **2. `wall_follower.py`** — The “drive along” brain

**Overall role:**
This node controls movement once the robot is positioned next to the wall. It also starts the odometry recording action before driving.

**Structure & flow:**

* **`class WallFollower`**:
  Contains the node’s setup, laser subscription, and main control logic.

* **`__init__` method**:

  1. Starts the ROS node with `rospy.init_node('wall_follower')`.
  2. **Waits for and calls `find_wall` service** — ensures we start in the right position.
  3. **Connects to odometry recording action server** (`record_odom`) — sends an empty goal to start recording before we move.
  4. Sets up:

     * Subscriber to `/scan` → triggers `scan_callback`.
     * Publisher to `/cmd_vel` → sends movement commands.
  5. Calls `rospy.spin()` to keep running.

* **`scan_callback`**:

  * Stores the latest LaserScan in `self.scan`.
  * Calls `follow_wall()` every time new scan data arrives.

* **`follow_wall`**:

  1. Picks out the **right-side laser beam** (`right_idx`) which is 90° to the right from the front.
  2. Reads `distance` to the wall on the right.
  3. Applies simple **distance control**:

     * If `distance > 0.3 m` → too far → turn right (`angular.z = -0.3`).
     * If `distance < 0.2 m` → too close → turn left (`angular.z = 0.3`).
     * Else → go straight.
  4. Adds **corner handling**:

     * If front distance < 0.5 m → turn left quickly while moving forward.
  5. Publishes the `twist` command.

**Key variables:**

* `self.scan` → latest LaserScan.
* `mid` → index of the front-facing beam in the scan array.
* `right_idx` → index of the beam 90° to the right.
* `distance` → distance from right wall.
* `twist` → velocity command to send to `/cmd_vel`.

---

## **Why it’s structured this way**

* **Classes** keep related data (like `self.scan`) and methods together, so callbacks and movement logic can share state easily.
* **Callbacks** (`scan_callback`) react to new sensor data instantly — essential for real-time control.
* **Service** (`handle_find_wall`) handles a one-time “setup” action before continuous driving.
* **Action client** in `wall_follower` ensures odometry logging starts **before** movement begins.
* **Variables** like `front_idx` and `right_idx` turn raw laser readings into meaningful control inputs (front obstacle detection, right wall distance).

---

If you’d like, I can also prepare a **demo narration script** where you can explain line-by-line what’s happening while showing the code during your presentation. That would make your audience follow along more easily.


---


Here’s a simple, conversational breakdown of the project so you can confidently demo it.

---

## Project Overview

We built a ROS package that makes a simulated robot:

1. **Find the nearest wall**.
2. **Follow that wall** while keeping a fixed distance.
3. **Record its odometry** (where it’s been and how far it’s traveled).

Everything runs in a Gazebo simulation with a TurtleBot3.

---

## Folder Structure (colloquial version)

```
ros_wall_following/
├── srv/FindWall.srv          # A custom service to trigger the 'find wall' behavior
├── action/OdomRecord.action  # A custom action to record odometry data over time
├── scripts/
│   ├── find_wall.py          # Service server that turns toward and approaches the nearest wall
│   ├── wall_follower.py      # Node that follows the wall and triggers odometry recording
│   ├── record_odom_server.py # Action server that logs robot positions and distance traveled
│   └── odom_result_client.py # Action client that displays odometry data in real time
└── launch/wall_following.launch  # Launch file to start everything at once
```

---

## What Each Script Does

- **find\_wall.py** → Listens to the robot’s laser scan, figures out where the closest wall is, rotates toward it, moves forward until it’s close, then turns to align so the wall is on its right.
- **wall\_follower.py** → Calls `find_wall` first, then continuously reads the laser scan. If it’s too far from the wall, it turns toward it; if too close, it turns away; if just right, it moves straight. It also checks for obstacles ahead to handle corners.
- **record\_odom\_server.py** → Runs in the background, taking odometry readings once per second, calculating total distance traveled, and sending live feedback. Stops when the robot has done a full lap and returns the full path.
- **odom\_result\_client.py** → Connects to the odometry action server, starts recording, and prints:
  - Live total distance traveled.
  - Full list of recorded `(x, y, theta)` positions when done.

---

## How It All Works Together

1. **Gazebo simulation** publishes `/scan` (laser) and `/odom` (position) topics.
2. The `` file starts:
   - The wall-finding service.
   - The wall-following controller.
   - The odometry recorder.
3. The **wall follower** calls `find_wall` to get in position.
4. Before starting to move along the wall, it sends a goal to the odometry action server so recording starts right away.
5. While moving, the odometry server sends live distance updates to the client.
6. Once the robot returns near its starting point, the odometry server stops and sends the full recorded path.

---

## Demo Flow Suggestion

1. Start Gazebo world: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
2. Start your package: `roslaunch ros_wall_following wall_following.launch`
3. In another terminal, run: `rosrun ros_wall_following odom_result_client.py`
4. Watch:
   - Robot finds and follows the wall.
   - Terminal shows live distance traveled.
   - At the end, it prints the whole route in coordinates.

That’s it — your robot now navigates AND logs its journey like a pro!

---














control logic:
sector based  + weight
PID


Next Steps for Real Gazebo Integration
To get the system working with a real TurtleBot3 in Gazebo, you would need to:

Fix Gazebo Issues:

Determine why Gazebo is crashing on startup (possibly due to hardware requirements or conflicting processes)
Make sure the correct environment variables are set for TurtleBot3
Test with Real Sensors:

The real wall_finder.py needs to process actual LiDAR data from the /scan topic
Ensure the robot can correctly identify and approach walls
Parameter Tuning:

Adjust control parameters in the wall following algorithm for smooth navigation
Tune wall detection thresholds based on the LiDAR characteristics
Fix Package Indexing:

Research why ROS 2 can't find the wall_follower package even though it's properly built and installed
This may involve checking the ROS 2 workspace setup or ament indexing system
The mock testing environment you now have provides a solid foundation for developing and debugging the control logic before moving to the real robot simulation.


---

Key Improvements in Enhanced Wall Following
State Machine Architecture:

Created distinct states for different behaviors (FINDING_WALL, FOLLOWING_WALL, TURNING_CORNER)
This allows the robot to adapt its control strategy based on its current situation
PD (Proportional-Derivative) Controller:

Improved control over the robot's movement by considering both the current error and the rate of change
Results in smoother corrections and less oscillation when following walls
Corner Detection:

Added logic to detect corners by analyzing patterns in laser readings
Allows the robot to handle right turns when there's an opening and left turns when obstacles block the path
Dynamic Speed Adjustment:

Reduces speed automatically when turning sharply
Maintains higher speeds when moving straight along walls
Improves overall stability and safety
Laser Data Processing:

More sophisticated processing of laser scan data
Filters out invalid readings and considers multiple regions around the robot
Provides better situational awareness for navigation
Safety Features:

Added emergency stop behavior when obstacles get too close
Prevents collisions even in unexpected situations
Configurable Parameters:

Made control parameters accessible via ROS 2 parameters
Allows tuning without code modifications
Supports command-line parameter passing through launch files
Enhanced Launch Files:

Created comprehensive launch files with parameter configuration
Supports both simulation and real-robot scenarios




check the exact topic name for laser data

# ros2
```
rm -rf build/ install/ log/
cd ros2_ws && colcon build && source install/setup.bash && ros2 launch wall_follower main.launch.py

ros2 topic echo /cmd_vel
ros2 topic echo /scan
ros2 service list | grep find_wall
ros2 service call /find_wall wall_follower_interfaces/srv/FindWall '{}'

ros2 action list
ros2 action info
ros2 action send_goal

```




# ros1
```
rosrun actionlib_tools axclient.py /record_odom wall_following/OdomRecordAction

catkin_make && source devel/setup.bash && roslaunch ros_wall_following main.launch
rosrun ros_wall_following odom_result_client.py
rostopic echo /scan
rostopic echo /cmd_vel
rosservice call /find_wall
```
