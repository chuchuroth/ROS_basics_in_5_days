#!/usr/bin/env python
import rospy, math, actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from ros_wall_following.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback

class OdomRecorder:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('record_odom', OdomRecordAction, self.execute, False)
        self.server.start()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.positions = []
        self.total_distance = 0.0
        self.prev_x = self.prev_y = None

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def execute(self, goal):
        self.positions = []
        self.total_distance = 0.0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if hasattr(self, 'current_pose'):
                x = self.current_pose.position.x
                y = self.current_pose.position.y
                theta = 0.0  # simplified; extract yaw from quaternion if needed
                point = Point32(x=x, y=y, z=theta)
                self.positions.append(point)

                if self.prev_x is not None:
                    self.total_distance += math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
                self.prev_x, self.prev_y = x, y

                feedback = OdomRecordFeedback(current_total=self.total_distance)
                self.server.publish_feedback(feedback)

                if len(self.positions) > 10 and math.isclose(x, self.positions[0].x, abs_tol=0.2) and math.isclose(y, self.positions[0].y, abs_tol=0.2):
                    result = OdomRecordResult(list_of_odoms=self.positions)
                    self.server.set_succeeded(result)
                    return
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('record_odom_server')
    OdomRecorder()
    rospy.spin()
