#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall, FindWallResponse

class FindWallServer:
    def __init__(self):
        rospy.init_node('find_wall_server')

        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.srv = rospy.Service('find_wall', FindWall, self.handle_find_wall)
        rospy.loginfo("Find_wall service ready.")
        rospy.spin()

    def scan_callback(self, msg):
        self.scan = msg

    def handle_find_wall(self, req):
        # wait for scan data
        while not self.scan:
            rospy.sleep(0.1)

        rate = rospy.Rate(10)
        twist = Twist()

        # 1. Point towards nearest wall
        while not rospy.is_shutdown():
            ranges = list(self.scan.ranges)
            # ignore inf, NaN
            filtered = [(i, r) for i, r in enumerate(ranges) if not rospy.is_shutdown() and r > 0]
            idx, _ = min(filtered, key=lambda x: x[1])
            # if front beam is shortest, break
            front_idx = len(ranges) // 2
            if idx == front_idx:
                break
            # rotate toward that index
            angle_to_turn = (idx - front_idx) * self.scan.angle_increment
            twist.angular.z = 0.5 * (1 if angle_to_turn > 0 else -1)
            self.pub.publish(twist)
            rate.sleep()


        # 2. Move forward until 0.3m
        twist = Twist()
        while self.scan.ranges[front_idx] > 0.3:
            twist.linear.x = 0.2
            self.pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0.0
        self.pub.publish(twist)

        # 3. Rotate until ray 270 points at wall
        target_idx = 270
        while self.scan.ranges[target_idx] == float('inf') or True:
            # find current shortest beam
            ranges = list(self.scan.ranges)
            idx, _ = min([(i, r) for i, r in enumerate(ranges) if r > 0], key=lambda x: x[1])
            if idx == target_idx:
                break
            twist.angular.z = 0.5
            self.pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0
        self.pub.publish(twist)

        rospy.loginfo("Wall found and aligned.")
        return FindWallResponse(wallfound=True)

if __name__ == '__main__':
    try:
        FindWallServer()
    except rospy.ROSInterruptException:
        pass


