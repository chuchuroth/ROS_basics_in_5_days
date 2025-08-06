#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_following.srv import FindWall

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')

        rospy.wait_for_service('find_wall')
        self.find_wall = rospy.ServiceProxy('find_wall', FindWall)
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
        # Compute index for right 90°, assuming ranges[0] is angle_min
        mid = len(self.scan.ranges) // 2
        right_idx = mid - int((90 * 3.14159/180) / self.scan.angle_increment)
        distance = self.scan.ranges[right_idx]

        twist = Twist()
        # Wall following PID-lite
        if distance > 0.3:
            twist.angular.z = -0.3  # steer right
        elif distance < 0.2:
            twist.angular.z = 0.3   # steer left
        else:
            twist.angular.z = 0.0
        twist.linear.x = 0.2


        # Check for wall crossing ahead
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