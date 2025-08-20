# ROS2 Demo Talking Script

Today, I am going to present a ROS2 program demo, which is an assignment from the course *ROS2 Basics in 5 Days* from Construct. I’ll explain some very basic ROS2 operations and concepts like ROS nodes, topics, services, and actions—the communication mechanisms defined in ROS2. Essentially, I’ll cover what I’ve learned in this course. I’ll also discuss tools like `rqt`, `ros2 bag`, and the visualization tool `rviz2`.

## Launching the Program
First, let me launch the program. I’ll move the robot to a corner to avoid interference from a concrete wall. Let’s open `rviz2` to get a view of the robot and understand why it tends to recognize the concrete wall.

This program controls a TurtleBot robot to perform a wall-following movement. Here’s the process:
- The robot publishes to `/scan` (laser) and `/odom` (position) topics.
- The wall follower calls the `find_wall` service to get into position.
- Before starting to move along the wall, it sends a goal to the odometry action server to start recording immediately.
- While moving, the odometry server sends live distance updates to the client.
- Once the robot returns near its starting point, the odometry server stops and sends the full recorded path.

## Program Structure
Let’s get a glimpse of the structure. First, run `ros2 node list` to see the three main nodes in this program. Now, let’s get information about these nodes using `ros2 node info`. Here’s what each node does:

- **wall_finder.py**: This node is a ROS2 service server. When another node calls `find_wall`, it listens to the robot’s laser scan, figures out where the closest wall is, rotates toward it, moves forward until it’s close, then turns to align so the wall is on its right.
- **wall_following.py**: The main node that implements the wall-following behavior. It calls `find_wall` first, then continuously reads the laser scan. If it’s too far from the wall, it turns toward it; if too close, it turns away; if just right, it moves straight. It also checks for obstacles ahead to handle corners and starts the odometry recording action before driving.
- **odom_recorder.py**: An action server that logs robot positions and distance traveled, calculating total distance and sending feedback. It stops when the robot completes a full lap and returns the full path.

## Visualizing Node Connections
Let’s use `rqt_graph` to view the nodes and their topic connections graphically. This is super useful for investigating the structure of a ROS2 program. You can see the three nodes: `find_wall`, `wall_follower`, and `record_odom_server`. You can also see their relationships:
- The `wall_follower` node is the client that calls the `find_wall` service server and the `record_odom_server` action server.
- There are four topics highlighted in squares, showing which node is publishing and which is subscribing.
- The `record_odom` topic is published and subscribed to by both the `wall_follower` and `record_odom_server` nodes.
- The `cmd_vel` topic is published by both `find_wall` and `wall_follower` because they both send velocity commands to the robot.

## Exploring Topics
Now, let’s check the topics from the command line using `ros2 topic list` and `ros2 topic info`. These align with the `rqt_graph`. In ROS2, there are multiple ways to do things, and you can use different tools to get basic information.

## Interpreting Laser Scan Data
One critical task in this program is correctly interpreting the laser scan data. Run `ros2 topic echo /scan` to see the laser scan message structure. The TurtleBot’s geometry parameters range from -180° to +180°, unlike the Gazebo simulator robot, which ranges from 0° to 360°.

## Recording and Replaying Data with ros2 bag
In ROS2, there’s no `rqt_console` or `rqt_plot`. Instead, it’s better to record a `ros2 bag` to work with data. Run `ros2 bag record -a` to record all topics, or `ros2 bag record /scan > scan.csv` to record a specific topic and export it to a CSV file for further analysis.

To investigate the bag, run `ros2 bag info` to see details like the date, file size, number of messages, topics, and message types. This is very useful in real-life applications, as you can store field testing data, recreate scenarios in a lab, and analyze what happened.

To replay the data, run `ros2 bag play`.

Some basic Linux commands for analyzing the CSV:
- `tail -n1 scan.csv` (last row)
- `cut -d',' -f1,2 odom.csv` (columns 1 and 2)
- `cut -d',' -f100 scan.csv` (specific column)

## Exploring Actions
In ROS2, use `ros2 action list` and `ros2 action info` to get action information. The `record_odom` topic comes from my custom action server, which logs robot positions and distance traveled.

To publish to a topic, first check the interface type using `ros2 interface list | grep wall` to see the custom interfaces. Then, run `ros2 interface show wall_follower_interfaces/action/OdomRecord` to get the structure of this action interface. The result is a structured list of odometry data, and the feedback is a float number representing the total distance. Odometry tracks the robot’s change in position relative to a starting point, represented by three values (x, y, θ), stored as an array.

To get the exact format for using the `OdomRecord` interface, run `ros2 interface proto wall_follower_interfaces/action/OdomRecord`. The `proto` command gives only the goal part, used when calling an action server. To get real-time values from this action message, run:
- `ros2 action send_goal /record_odom wall_follower_interfaces/action/OdomRecord "{}"`

This shows the total distance traveled so far.

A third way to call the action server is to create a separate action client node. In this program, the `wall_follower` client sends a goal to the `record_odom_server`, which responds and sends its status back. The `record_odom_server` is defined such that when the robot completes a full lap, it provides feedback with the total distance moved and returns the list of recorded odometries.

## Exploring Services
Next, let’s check the services using `ros2 service list`. We have the `find_wall` service. Run `ros2 service type /find_wall` to get the service type, which is required when calling the service. Then, run `ros2 service call /find_wall wall_follower_interfaces/srv/FindWall` to trigger it. In our program, this means the robot has found the wall, demonstrating a request-and-response mechanism.

To see how the service message is defined:
- Run `ros2 interface list | grep wall`
- Run `ros2 interface show wall_follower_interfaces/srv/FindWall`

The `FindWall.srv` is a custom service to trigger the "find wall" behavior. Another way to call the service is by creating a service client, which is included in the `wall_following` node.

## QoS Settings
In ROS2, the Quality of Service (QoS) settings of a topic must be compatible between the publisher and subscriber nodes, or communication won’t work. To get information about the QoS settings of a specific topic, run `ros2 topic info /scan -v`.