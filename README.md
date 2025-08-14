
# Topic
eg. laser: Topic to publish information, available all the time

```
rosmsg show std\_msgs/Int32

rostopic info /<topic>
rostopic echo /<topic>
rostopic echo /rosout  # see all the ROS logs in the current nodes running in the system

```


---

# Service
eg. Face-recognition: Service - call that service and WAIT: who is this person?


```
rosservice list /the\_service\_name
rosservice info /the\_service\_name
rosservice call /the\_service\_name TAB-TAB

rossrv show trajectory_by_name_srv/TrajByName
rossrv list | grep MyCustomServiceMessage
```





---


# Action
eg. navigation system - Action
1. ROS Actions are ideal for long-running tasks that can be interrupted, like moving to a position or performing complex calculations.
2. They allow you to monitor the progress of a task, which isn’t possible with simple topics or services.
3. Implementing action servers and action clients will give your robot more flexibility and control over its tasks.

```

roslaunch ardrone\_as action\_server.launch  # launch action server

rostopic list | grep ardrone\_action\_server  # check Action Server is up and running

# example: the three parts of action message
rostopic echo /ardrone\_action\_server/goal   
rostopic echo /ardrone\_action\_server/feedback
rostopic echo /ardrone\_action\_server/result

# Publishing directly into the /goal topic of the Action Server
rostopic pub /\[name\_of\_action\_server]/goal /\[type\_of\_the\_message\_used\_by\_the\_topic] \[TAB]\[TAB] 

rosrun actionlib\_tools axclient.py /<name\_of\_action\_server>  # use GUI tool to interact with an Action Server



```


---

# Tools


```
rosrun rviz rviz    # to know what the Robot was perceiving: In Global Options, select the Fixed Frame that suits you for the visualization of the data and The Add button

rqt\_console

rosbag record -O laser.bag laser\_scan # record
rosbag info name\_bag\_file.bag     # extract
rosbag play name\_bag\_file.bag  \&\& rostopic echo /laser\_scan/ranges\[100] \&\& rqt\_plot /laser\_scan/ranges\[100]    # replay

```


---

# ROS2
```
# Test that interfaces are generated
ros2 interface list | grep <interface package>
```
