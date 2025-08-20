to have clean workspace: 
```
rm -rf build install log
```

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
rostopic echo /action_server/goal   
rostopic echo /action_server/feedback
rostopic echo /action_server/result

# Publishing directly into the topic of the Action Server
rostopic pub /goal

rosrun actionlib_tools axclient.py   # use GUI tool to interact with an Action Server
rosrun actionlib_tools axclient.py  /record_odom  ros_wall_following/OdomRecordAction

```


---

# Tools


```
 rviz 
rqt_console  这个好像没用
rqt_plot /scan/ranges[100]
rqt_plot  /record_odom
rqt_graph




rosbag record /scan 
rosbag info    
rosbag play   



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
# Each value in the ranges array represents the sensor's reading in a specific direction. Thus, each direction is represented by a position in this array.


ros2 topic info /laser_scan -v (To get information about the QoS setting of a specific topic)
# The QoS settings of the topic Publisher and the Subscriber node need to be compatible.
# If they are not compatible, the communication between them won't work.
# Due to the fact that it is the Publisher that sets the QoS of the topic, the subscribers are depend on that configuration.
```

---
# service
```


ros2 interface list  (To publish something to this topic, you need to first check what type of interface the Topic uses)

ros2 interface proto geometry_msgs/msg/Twist  # This command shows you a prototype (example) of the interface that you can use, example: next you can use this command to give one time speed to robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: - 0.1, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"  

ros2 service list
ros2 service type /get_robot_status  # To Call the service, we need to know the Service Interface Type of service /get_robot_status
ros2 interface show std_srvs/srv/Trigger # structure of the message, it has two parts request and responce and separate by ---
ros2 interface proto std_srvs/srv/Trigger  # use the ros2 interface proto to get the exact format for using std_srvs/srv/Trigger messages, proto gives us only the REQUEST part, because it is the one we use when CALLING a service server
ros2 service call /get_robot_status std_srvs/srv/Trigger "{}
ros2 service call /find_wall wall_follower_interfaces/srv/FindWall '{}'

```

---

# action

```

ros2 interface show leo_description/action/Rotate

ros2 action list
ros2 action info /rotate -t (get the action type)
ros2 action send_goal <action_name> <action_type> <values>  (call action)
ros2 action send_goal /rotate leo_description/action/Rotate "{rotation_time: 5.0}" -f (f for feedback)
"
ros2 action send_goal /go_to_pose leo_description/action/GoToPose "{x: -8.0, y: 6.0, yaw: 1.57}"
```



---

# rosbag

```

rosbag record -O laser.bag laser_scan
rosbag info laser.bag
rosbag play -l laser.bag
```



---


/src

ros1 

rosbag record
rosbag info Name.bag
rosbag Play Name,bag
rostopic echo -b Name.bag -p /scan > data.csv





there are more ways to data analysis but this ways is ros specific

ros2

ros2 bag record /scan /odom /cmd_vel
ros2 bag info Name
ros2 bag Play Name<
ros2 Topic echo /scan --once  --csv > scan.csv

Linux CLI:
tail -n 1 odom.csv (last row)
cut -d',' -f1,2 odom.csv cut -d',' -f100 scan.csv (column 1 and 2)

---









ros2已经可以交了，但是在转圈的时候，总是转去墙的右边而不是正对墙。，可能右边laser beam更有优先权，在交接任务的时候有个很奇怪的突然转身的动作

ros1也有同样的问题，laser指错了，总是转啊转却找不到最近的墙





