hello everyone, today i am going to share a ros project from the course ROS Basics in 5 days, let me open the terminal and wake up ros, the ros package is already compiled so i'll just launch the launch file to start the demo, which is for the robot to do a wall following movement, I also start a rosbag to record some data so we can later analyse.  So because I'll also do the demo for the course ROS2 Basics in 5 days after this demo, I'll focus on explaining the ros package structure and basic concept like ros node and topic and service and action, also debugging tools like rqt and rosbags, and visualization tools like rviz. For the ros2 demo I'll talk about how to get correct laser configuration, and different data-analysis methods and how to improve the accuracy and efficiency of your robot system, like to make it run more smoothly by implementing a PID controller or state machine. 

---

now let's first watch the robot moving while following wall. [robot running]

---

so let me introduce some graphical tools.
rqt_graph :  here you have a display of visual graph of the nodes running in ROS and their topic connections , you can see the three nodes find_wall, wall_follower, and the record_odom_server node, which matches the three nodes I have defined in my launch files. And the relationship between each node, the wall_follower node is the client who calls for find_wall the service server once in the beginning of the loop to find the wall, and calls for record_odom_server the action server the whole time while program running to record the odometry of the robot, now we have learned the communication mechanism of the action interface, we know the wall_follower client need to send goal to the action server so as response record_odom_server will send its status back to wall_follower.
and if you filter based on the topics, now you can see there is four topics highlighted in squares, also which node is publishing information into the topic and which node is subscribing to that topic, for example the record_odom topic is published from and subscribed to by both nodes the wall_follower and the reocrd_odom_server, and cmd_vel topic is published from both find_wall and wall_follower because they both send velocity command to robot.

---

rqt_plot : here you have a graphical view of some of the data which is maybe difficult to interpret in a numerical way, like the laser scan data which is a large trunk of array, so you can use this tool to view some of the critical data and get a Intuition of what's going on, like here the range[xx] from laser scan message indicates the right side laser beam of robot, which meaning the distance towards wall is changing this way.

---


now I'd like to introduce two ways to call Action Server in ros.
we learned from the action interface there are three parts, the goal the result and the feedback. calling an action server means sending a goal into it. and there is a special tool for that, the axclient, it's a GUI tool provided by the actionlib package, so we can interact with Action Server in a visual way. 

so here we have the data, what does the data mean, we need to first understand what is odometry, so odometry is the usage of motion sensors to know the robot’s change in position relative to a starting point, robot need to track their locations as they navigate, it is represented with three values (x, y, θ). Each time sensors are processed, the x, y, and θ values are updated and kept in a data structure like array.
and record_odom_server is defined in this way, When the robot has done a complete lap, as feedback, the action server will provide the total amount of meters that the robot has moved so far, and return the list of odometries recorded.
the second method is publishing the goal using Python code, I've created a separate action client node so we can use this node to call the action server and get the feedback information, the same information as we get using the axclient tool.

---

rviz : RVIZ is a tool to view from the perspective of robot, it is also representing of what is being published in the scan topics. rviz is a more advanced tool , there are other courses in construct covers topics like ros navigation and how to create a map, like the popular SLAM, would be more interesting to learn to combine with those topics. now we just have a look.

---

rosbag : now let's get the basic information of rosbag file , here you have the information the date, the size of the file, how many messages are been sent, and the topic and the message type, this is very useful in real life application, you can store some field testing data and recreate the scenario in a lab environment and analyse what has happened in a real life setting. 
now let's play this rosbag. 

---

finally let's have a look at the package structure, there are four files, in ros Terminologe they are called nodes.

- **wall_finder.py**: Provides a service to find the nearest wall, Overall role: This node is a ROS service server. When another node calls find_wall, it makes the robot turn toward the nearest wall, drive to it, and rotate so the wall is on the robot’s right.

- **wall_following.py**:  The main node that implements the wall following behavior, Overall role: This node controls movement once the robot is positioned next to the wall. It also starts the odometry recording action before driving.

- **odom_recorder.py**: Action server to record odometry data, running in the background, record where it's been and how far it has been traveled.

- **odom_result_client.py** which is used to call the action server to get the data separately, to view the data in a separate terminals.

let me quickly go through the codes, for more details i'll leave that to another demo, the algorithms implemented in ros2 project is more complicated than this one, which is also an interesting point, sometimes more complicated algorithms give better performance but not always the case, why is that, because software always needs to aglin with hardware configuration, fancy calculatin doesn't help a lot if the data produced by hardware is with bad quality or insufficiant in the first place.

and when solving real life problem, the right mindset would be to keep a clear goal, what we are trying to achieve and do not overkill, because very complicated algorithms can add to the complexity of the project easily but does not necessary produce better result. 










