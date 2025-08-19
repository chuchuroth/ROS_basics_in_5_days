#!/usr/bin/env python

"""
ROS1 Wall Follower - Minimal Implementation
Simple wall following using minimal laser processing
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall
import math

class WallFollowerMinimal:
    def __init__(self):
        rospy.init_node('wall_follower_minimal')
        
        # Call find_wall service first
        self.call_find_wall()
        
        # Publisher for robot movement
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscriber for laser data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Store laser data
        self.ranges = None
        
        rospy.loginfo("Minimal Wall Follower ready")
        rospy.spin()
    
    def call_find_wall(self):
        """Call find_wall service to locate wall"""
        try:
            rospy.wait_for_service('find_wall', timeout=5.0)
            find_wall = rospy.ServiceProxy('find_wall', FindWall)
            response = find_wall()
            if response.wallfound:
                rospy.loginfo("Wall found successfully!")
            else:
                rospy.logwarn("Wall not found, continuing anyway")
        except Exception as e:
            rospy.logwarn(f"Find wall service failed: {e}")
    
    def scan_callback(self, msg):
        """Process laser scan and control robot"""
        self.ranges = msg.ranges
        self.follow_wall()
    
    def get_right_distance(self):
        """Get distance to right side"""
        if not self.ranges:
            return float('inf')
        
        # Simple right side calculation (90° to the right)
        right_index = len(self.ranges) // 4
        return self.ranges[right_index]
    
    def get_front_distance(self):
        """Get distance to front"""
        if not self.ranges:
            return float('inf')
        
        front_index = len(self.ranges) // 2
        return self.ranges[front_index]
    
    def follow_wall(self):
        """Simple wall following control"""
        if not self.ranges:
            return
        
        # Get distances
        right_dist = self.get_right_distance()
        front_dist = self.get_front_distance()
        
        # Create movement command
        twist = Twist()
        twist.linear.x = 0.1  # Always move forward
        
        # Simple wall following logic
        if front_dist < 0.5:
            # Obstacle ahead - turn left sharply
            twist.angular.z = 0.5
        elif right_dist > 0.3:
            # Too far from wall - turn right
            twist.angular.z = -0.3
        elif right_dist < 0.2:
            # Too close to wall - turn left
            twist.angular.z = 0.3
        else:
            # Good distance - go straight
            twist.angular.z = 0.0
        
        # Publish command
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        WallFollowerMinimal()
    except rospy.ROSInterruptException:
        pass
