to have clean workspace: rm -rf build install log


---

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
rossrv list 

rosservice list 
rosservice info 
rosservice call 


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

# topic

```
ros2 node list 
ros2 node info /mars_rover_1
```

---

```

ros2 topic echo /cmd_vel
ros2 topic echo /laser_scan --once  (the last one)
ros2 topic echo /laser_scan --field ranges
# the LaserScan interface has an element named ranges, it contains the distance detection data, echo that field using the --field argument
# Each value in the ranges array represents the sensor's reading in a specific direction. Thus, each direction is represented by a position in this array.


ros2 topic info /laser_scan -v (To get information about the QoS setting of a specific topic)
```

---
# service
```


ros2 interface list  (To publish something to this topic, you need to first check what type of interface the Topic uses)

ros2 interface proto geometry_msgs/msg/Twist  # This command shows you a prototype (example) of the interface that you can use, example: next you can use this command to give one time speed to robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: - 0.1, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"  


ros2 service type /get_robot_status  # To Call the service, we need to know the Service Interface Type of service /get_robot_status
ros2 interface show std_srvs/srv/Trigger # structure of the message, it has two parts request and responce and separate by ---
ros2 interface proto std_srvs/srv/Trigger  # use the ros2 interface proto to get the exact format for using std_srvs/srv/Trigger messages, proto gives us only the REQUEST part, because it is the one we use when CALLING a service server
ros2 service call /get_robot_status std_srvs/srv/Trigger "{}

```

---

# action

```

ros2 interface show leo_description/action/Rotate 
ros2 action info /rotate -t (get the action type)
ros2 action send_goal <action_name> <action_type> <values>  (call action)
ros2 action send_goal /rotate leo_description/action/Rotate "{rotation_time: 5.0}" -f (f for feedback)
"
```



---





ros2 service list | grep find_wall
ros2 service call /find_wall wall_follower_interfaces/srv/FindWall '{}'

ros2 action list
ros2 action info /record_odom -t
ros2 action send_goal

ros2 action send_goal /go_to_pose leo_description/action/GoToPose "{x: -8.0, y: 6.0, yaw: 1.57}"


roscd; cd ../src
rosbag record -O laser.bag laser_scan
rosbag info laser.bag
rosbag play -l laser.bag
rostopic echo /scan/ranges[100]
rqt_plot /scan/ranges[100]



