hello today i am going to share a ros program demo and what i have learned in the course ROS Basics in 5 days, because I'll also do the demo for the course ROS2 Basics in 5 days after this demo, I'll focus on explaining the ros package structure and basic concept like ros node and topic and service and action, basically the communication mechanism of ros, and i'll also quickly cover some debugging tools like rqt and rosbags, and the visualization tool rviz. In the next ros2 demo I'll focus more on the coding side and talk about algorithms and how to interpret laser scan data, and some basic data analysis methods .


let me open the terminal and wake up ros, the ros package is already compiled so i'll just launch the launch file to start the demo, I also start a rosbag to record some data so we can later analyse. this Project is to design a ros package to control the turtlebot robot to do a wall following movement,

---

now let's first watch the robot moving while following wall. [robot running]

---

finally let's have a look at the package structure, there are four files, in ros Terminologe they are called nodes.

- **wall_finder.py**: Provides a service to find the nearest wall, Overall role: This node is a ROS service server. When another node calls find_wall, it Listens to the robot’s laser scan, figures out where the closest wall is, rotates toward it, moves forward until it’s close, then turns to align so the wall is on its right.

- **wall_following.py**:  The main node that implements the wall following behavior, would Calls find_wall first, then continuously reads the laser scan. If it’s too far from the wall, it turns toward it; if too close, it turns away; if just right, it moves straight. It also checks for obstacles ahead to handle corners. It also starts the odometry recording action before driving.

- **odom_recorder.py**: Action server to log robot positions and distance traveled, taking odometry readings once per second, running in the background, calculating total distance traveled, and sending live feedback. Stops when the robot has done a full lap and returns the full path.

- **odom_result_client.py**  Action client that displays odometry data in real time, separately call the odometry action server to let it starts recording, and prints out Live total distance traveled and a Full list of recorded (x, y, theta) positions when done.
-  **srv/FindWall.srv**       this is a custom service to trigger the 'find wall' behavior
-  **action/OdomRecord.action**  # A custom action to record odometry data over time

so roughly the process goes like this:
robot publishes to /scan (laser) and /odom (position) topics.
The wall follower calls find_wall to get in position.
Before starting to move along the wall, it sends a goal to the odometry action server so recording starts right away.
While moving, the odometry server sends live distance updates to the client.
Once the robot returns near its starting point, the odometry server stops and sends the full recorded path.

so a quick overview of the codes, for more details i'll leave that to another demo, the algorithms implemented in ros2 project is more complicated than this one, which is also an interesting point, sometimes more complicated algorithms give better performance but not always the case, why is that, because software always needs to aglin with hardware configuration, fancy calculatin doesn't help a lot if the data produced by hardware is with bad quality or insufficiant in the first place.

and when solving real life problem, the right mindset would be to keep a clear goal, what we are trying to achieve and do not overkill, because very complicated algorithms can add to the complexity of the project easily but does not necessary produce better result. 










