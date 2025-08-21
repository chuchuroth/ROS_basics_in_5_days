# ROS1 Demo Talking Script

Today, I am going to present a ROS program demo, which is an assignment from the course *ROS Basics in 5 Days* from Construct. I’ll explain some very basic ROS operations and concepts like ROS nodes, topics, services, and actions—the communication mechanisms defined in ROS. Essentially, I’ll cover what I’ve learned in this course. I’ll also discuss tools like `rqt`, `rosbags`, and the visualization tool `rviz`.

## Launching the Program
First, let me launch the program. This program controls a TurtleBot robot to perform a wall-following movement. Here’s the process:
- The robot publishes to `/scan` (laser) and `/odom` (position) topics.
- The wall follower calls the `find_wall` service to get into position.
- Before starting to move along the wall, it sends a goal to the odometry action server to start recording immediately.
- While moving, the odometry server sends live distance updates to the client.
- Once the robot returns near its starting point, the odometry server stops and sends the full recorded path.

## Program Structure
Let’s get a glimpse of the structure. First, let’s run `rosnode list` to see the three main nodes in this program. Now, let’s get some information about these nodes using `rosnode info`. Here’s what each node does:

- **wall_finder.py**: This node is a ROS service server. When another node calls `find_wall`, it listens to the robot’s laser scan, figures out where the closest wall is, rotates toward it, moves forward until it’s close, then turns to align so the wall is on its right.
- **wall_following.py**: The main node that implements the wall-following behavior. It calls `find_wall` first, then continuously reads the laser scan. If it’s too far from the wall, it turns toward it; if too close, it turns away; if just right, it moves straight. It also checks for obstacles ahead to handle corners and starts the odometry recording action before driving.
- **odom_recorder.py**: An action server that logs robot positions and distance traveled, taking odometry readings once per second, running in the background, calculating total distance traveled, and sending live feedback. It stops when the robot completes a full lap and returns the full path.

## Visualizing Node Connections
Let’s use `rqt_graph` to view the nodes and their topic connections graphically. This is super useful for investigating the structure of a ROS program. You can see the three nodes: `find_wall`, `wall_follower`, and `record_odom_server`. You can also see their relationships:
- The `wall_follower` node is the client that calls the `find_wall` service server and the `record_odom_server` action server.
- There are four topics highlighted in squares, showing which node is publishing and which is subscribing.
- The `record_odom` topic is published and subscribed to by both the `wall_follower` and `record_odom_server` nodes.
- The `cmd_vel` topic is published by both `find_wall` and `wall_follower` because they both send velocity commands to the robot.

## Exploring Topics
Now, let’s check the topics from the command line using `rostopic list` and `rostopic info`. These align with the `rqt_graph`. In ROS, there are multiple ways to do things, and you can use different tools to get basic information.

The `record_odom` topic comes from my custom action server, which logs robot positions and distance traveled. Let’s look at the action message structure:
- Run `rosmsg list | grep Odom` to get all action messages related to odometry.
- Run `rosmsg info ros_wall_following/OdomRecordAction` to see the detailed structure of this action.
- Run `rosmsg info ros_wall_following/OdomRecordResult` to see the result structure.

The action message `OdomRecord.action` is a custom action to record odometry data over time. The result is a structured list of odometry data, and the feedback is a float number representing the total distance. Odometry tracks the robot’s change in position relative to a starting point, represented by three values (x, y, θ), stored as an array.

To get real-time values from this action message, run:
- `rostopic echo /action_server/feedback`
- `rostopic echo /action_server/result`

This shows the total distance traveled so far. Another way to interact with the action server is using the GUI tool `axclient` from the `actionlib` package:
- Run `rosrun actionlib_tools axclient.py /record_odom ros_wall_following/OdomRecordAction`
This displays the same data as seen from the command line.

A third way to call the action server is to create a separate action client node. In this program, the `wall_follower` client sends a goal to the `record_odom_server`, which responds and sends its status back. The `odom_result_client.py` is an action client that displays odometry data in real time, separately calling the odometry action server to start recording and printing live total distance traveled and a full list of recorded (x, y, θ) positions when done.

The `record_odom_server` is defined such that when the robot completes a full lap, it provides feedback with the total distance moved and returns the list of recorded odometries.

## Exploring Services
Next, let’s check the services using `rosservice list`. We have the `find_wall` service. Run `rosservice info` to get details, and `rosservice call find_wall` to trigger it. In our program, this means the robot has found the wall, demonstrating a request-and-response mechanism.

To see how the service message is defined:
- Run `rossrv list | grep FindWall`
- Run `rossrv info ros_wall_following/FindWall`

The `FindWall.srv` is a custom service to trigger the "find wall" behavior. Another way to call the service is by creating a service client, which is included in the `wall_following` node.

## Visualization with RViz
Now, let’s use `rviz`, a tool to view from the robot’s perspective, representing what is published in the `/scan` topics. `rviz` is a more advanced tool, and other Construct courses cover topics like ROS navigation and creating maps (e.g., SLAM), which would be interesting to combine with this. For now, let’s just take a look.

## Interpreting Laser Scan Data
One critical task in this program is correctly interpreting the laser scan data. Run `rostopic echo /scan` to see the laser scan message structure. The TurtleBot’s geometry parameters range from -180° to +180°, unlike the Gazebo simulator robot, which ranges from 0° to 360°:

```
angle_min: -3.1241390705108643  # ≈ -179°
angle_max: 3.1415927410125732   # ≈ +180°
angle_increment: 0.008714509196579456  # ≈ 0.5°
```
 I also implementated sector-based laser analysis approach just like the task autonomous_exploration.py , which divides the 360° laser scan into 6 sectors.

To investigate specific scan data, run `rostopic echo /scan/ranges[100] --field ranges`. The `ranges` array contains distance detection data, where each value represents the sensor’s reading in a specific direction. For example, this could represent the distance to the wall on the right-hand side. You can also visualize this graphically using `rqt_plot /scan/ranges[100]`.

## Using rqt_console
Run `rqt_console` to get an overview of current events, callback functions, and messages being sent.

## Recording and Replaying Data with rosbag
Next, let’s record data using `rosbag record -a`. To investigate the bag, run `rosbag info` to see details like the date, file size, number of messages, topics, and message types. This is very useful in real-life applications, as you can store field testing data, recreate scenarios in a lab, and analyze what happened.

To replay the data, run `rosbag play`. You can also export scan data to a CSV file for further analysis by adding `/scan > scan.csv` to the command.

Some basic Linux commands for analyzing the CSV:
- `tail -n1 scan.csv` (last row)
- `cut -d',' -f1,2 odom.csv` (columns 1 and 2)
- `cut -d',' -f100 scan.csv` (specific column)

---


# ROS1 Wall Finder Service Node - Production-Ready Implementation

## Overview
This ROS1 node provides a robust wall-finding service that enables a robot to locate, approach, and align with the nearest wall for subsequent wall-following behavior. The node implements advanced sector-based laser analysis for precise wall detection and positioning.

## Purpose
- Locate the nearest wall in the robot's environment using 360° laser data
- Rotate the robot to face the nearest wall (Step 1: Alignment)
- Approach the wall to an optimal distance (Step 2: Positioning) 
- Align the robot so the wall is on the right side (Step 3: Preparation)
- Return success/failure status to calling nodes



## Key Technical Features
- Sector-based 360° laser analysis for robust obstacle detection
- Laser-guided control loops with continuous feedback
- Correct geometry calculations for any laser configuration
- Wrap-around handling for rear sector analysis

---
# ROS1 Wall Follower Client Node - Comprehensive Production Implementation



## Overview and Purpose
This file implements the ROS1 wall following client node that coordinates multiple services to achieve autonomous wall following behavior. It acts as the main controller that:
1. Calls the find_wall service to locate and approach the nearest wall
2. Starts odom recording action for path tracking  
3. Executes continuous wall following using laser-guided control
4. Maintains optimal distance from wall on robot's RIGHT side

## Architectural Design Decisions
This implementation follows a service client + action client + continuous control architecture that provides several key advantages:
The code is organized into logical sections that mirror the robot's operational workflow:
1. Service/Action Setup (preparation phase)
2. Sensor Callback (perception phase)
3. Control Logic (action phase)
4. Error Handling (safety phase)



4. PRODUCTION-READY ERROR HANDLING:
   - Graceful degradation when services unavailable
   - Comprehensive logging for operational monitoring
   - Robust exception handling for service timeouts


- Implements proportional control for distance maintenance
- Includes collision avoidance for front obstacles





