# ros 1

# Topic
eg. laser: Topic to publish information, available all the time

```
rosmsg show std\_msgs/Int32

rostopic list
rostopic info
rostopic echo
 

```


---

# Service
eg. Face-recognition, has two parts: request and responce



```
rossrv show trajectory_by_name_srv/TrajByName
rossrv list | grep MyCustomServiceMessage

rosservice list /the\_service\_name
rosservice info /the\_service\_name
rosservice call /the\_service\_name TAB-TAB


```





---


# Action
eg. navigation system - has three parts, goal, feedback ,result


```


# example: the three parts of action message
rostopic echo /ardrone\_action\_server/goal   
rostopic echo /ardrone\_action\_server/feedback
rostopic echo /ardrone\_action\_server/result

# Publishing directly into the /goal topic of the Action Server
rostopic pub /

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


---

# ros 2




