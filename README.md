# this repo is just for storing examples, all packages are not properly configured, names in launch files are not correct




**topic vs service vs action example**

+ laser: Topic to publish information, available all the time
+ Face-recognition: Service - call that service and WAIT: who is this person?
+ navigation system : Action
1. ROS Actions are ideal for long-running tasks that can be interrupted, like moving to a position or performing complex calculations.
2. They allow you to monitor the progress of a task, which isn’t possible with simple topics or services.
3. Implementing action servers and action clients will give your robot more flexibility and control over its tasks.


```

rosrun rviz rviz    # to know what the Robot was perceiving: In Global Options, select the Fixed Frame that suits you for the visualization of the data and The Add button



catkin\_create\_pkg my\_example\_pkg rospy std\_msgs    

rosservice list /the\_service\_name

rosservice info /the\_service\_name

rosservice call /the\_service\_name TAB-TAB



rosmsg show std\_msgs/Int32

roslaunch ardrone\_as action\_server.launch  # launch action server

rostopic list | grep ardrone\_action\_server  # check Action Server is up and running

rostopic info /<topic>

rostopic echo /<topic>

rostopic echo /ardrone\_action\_server/goal    # example: the three parts of action message

rostopic echo /ardrone\_action\_server/feedback

rostopic echo /ardrone\_action\_server/result

rostopic pub /\[name\_of\_action\_server]/goal /\[type\_of\_the\_message\_used\_by\_the\_topic] \[TAB]\[TAB]  # Publishing directly into the /goal topic of the Action Server

rosrun actionlib\_tools axclient.py /<name\_of\_action\_server>  # use GUI tool to interact with an Action Server



rostopic echo /rosout  # see all the ROS logs in the current nodes running in the system

rqt\_console



rosbag record -O laser.bag laser\_scan # record

rosbag info name\_bag\_file.bag     # extract

rosbag play name\_bag\_file.bag  \&\& rostopic echo /laser\_scan/ranges\[100] \&\& rqt\_plot /laser\_scan/ranges\[100]    # replay

```


---

ROS2

split project into two parts: one for service interface which is compiled using cmake, the other is main package compiled using python

Modify the setup.py File to Execute Your Python File: entry point…
Modify setup.py to find the .launch.py files : data_files..

node is designed to handle a specific function or capability of the robot, eg. One node for controlling wheel motors, One node for managing a laser rangefinder, One node for face recognition… a 'entry point' can start one or more nodes.
