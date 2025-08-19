#!/usr/bin/env python
import rospy, actionlib
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall
from ros_wall_following.msg import OdomRecordAction, OdomRecordGoal

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')

        # --- Try FindWall service ---
        wall_ok = False
        try:
            rospy.wait_for_service('find_wall', timeout=5.0)
            find_wall_client = rospy.ServiceProxy('find_wall', FindWall)
            rospy.loginfo("Calling find_wall service...")
            resp = find_wall_client()
            if resp.wallfound:
                rospy.loginfo("Wall found successfully ✅")
                wall_ok = True
            else:
                rospy.logwarn("find_wall responded but wall not found ❌")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"find_wall service call failed: {e}")

        # --- Try OdomRecord action ---
        odom_ok = False
        try:
            rospy.loginfo("Connecting to record_odom action server...")
            client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
            if client.wait_for_server(timeout=rospy.Duration(5.0)):
                client.send_goal(OdomRecordGoal())
                rospy.loginfo("Odom recording started ✅")
                odom_ok = True
            else:
                rospy.logwarn("record_odom action server not available ❌")
        except Exception as e:
            rospy.logwarn(f"Failed to start odom recording: {e}")

        if wall_ok and odom_ok:
            rospy.loginfo("Preparation complete. Starting wall-following.")
        else:
            rospy.logwarn("One or more preparations failed ❌. Starting wall-following anyway.")

        # --- Setup scan subscriber and cmd_vel publisher ---
        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.spin()



    def scan_callback(self, msg):
        self.scan = msg

        # Debug print once
        if not hasattr(self, "_printed_debug"):
            self._printed_debug = True
            rospy.loginfo(f"LaserScan config: angle_min={msg.angle_min:.2f} rad, "
                          f"angle_max={msg.angle_max:.2f} rad, "
                          f"angle_increment={msg.angle_increment:.4f} rad")
            rospy.loginfo("Here, ray[0] corresponds to angle_min "
                          f"({math.degrees(msg.angle_min):.1f}°). "
                          "Front is assumed at mid index.")
            rospy.loginfo("🚀 LASER CALLBACK WORKING! Starting wall following...")

        # Add periodic logging to see if callback is still working
        if not hasattr(self, "_callback_count"):
            self._callback_count = 0
        self._callback_count += 1
        
        if self._callback_count % 50 == 0:  # Every ~5 seconds at 10Hz
            rospy.loginfo(f"📡 Laser callback active (message #{self._callback_count})")

        self.follow_wall()


    def follow_wall(self):
        if not self.scan:
            rospy.logwarn("⚠️ follow_wall called but no scan data available!")
            return

        # Add debug logging to see if this method is running
        if not hasattr(self, "_follow_count"):
            self._follow_count = 0
        self._follow_count += 1
        
        if self._follow_count % 50 == 0:  # Every ~5 seconds
            rospy.loginfo(f"🤖 Wall following active (iteration #{self._follow_count})")

        mid = len(self.scan.ranges) // 2
        right_idx = mid - int((90 * 3.14159/180) / self.scan.angle_increment)
        distance = self.scan.ranges[right_idx]

        twist = Twist()
        if distance > 0.3:
            twist.angular.z = -0.4
        elif distance < 0.2:
            twist.angular.z = 0.4
        else:
            twist.angular.z = 0.0
        twist.linear.x = 0.1

        front = self.scan.ranges[mid]
        if front < 0.5:
            twist.angular.z = 3.0
            twist.linear.x = 0.1

        # Add debug logging for first few commands
        if self._follow_count <= 5:
            rospy.loginfo(f"🎮 Sending cmd: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f} "
                         f"(right_dist={distance:.2f}m, front_dist={front:.2f}m)")

        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
