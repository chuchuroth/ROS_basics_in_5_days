#!/usr/bin/env python
import rospy, actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall
from ros_wall_following.msg import OdomRecordAction, OdomRecordGoal

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')

        rospy.wait_for_service('find_wall')
        self.find_wall = rospy.ServiceProxy('find_wall', FindWall)

        rospy.loginfo("Connecting to record_odom action server...")
        client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
        client.wait_for_server()
        client.send_goal(OdomRecordGoal())

        rospy.loginfo("Calling find_wall service...")
        if self.find_wall().wallfound:
            rospy.loginfo("Starting wall following.")

        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.spin()

    def scan_callback(self, msg):
        self.scan = msg
        self.follow_wall()

    def follow_wall(self):
        if not self.scan:
            return
        mid = len(self.scan.ranges) // 2
        right_idx = mid - int((90 * 3.14159/180) / self.scan.angle_increment)
        distance = self.scan.ranges[right_idx]

        twist = Twist()
        if distance > 0.3:
            twist.angular.z = -0.3
        elif distance < 0.2:
            twist.angular.z = 0.3
        else:
            twist.angular.z = 0.0
        twist.linear.x = 0.2

        front = self.scan.ranges[mid]
        if front < 0.5:
            twist.angular.z = 1.0
            twist.linear.x = 0.2

        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
