#!/usr/bin/env python

"""
ROS1 Wall Finder - Minimal Implementation
Simple wall finding service using minimal laser processing
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall, FindWallResponse
import math

class WallFinderMinimal:
    def __init__(self):
        rospy.init_node('wall_finder_minimal')
        
        # Publisher for robot movement
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscriber for laser data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Service server
        self.service = rospy.Service('find_wall', FindWall, self.find_wall_service)
        
        # Store laser data
        self.ranges = None
        
        rospy.loginfo("Minimal Wall Finder ready")
        rospy.spin()
    
    def scan_callback(self, msg):
        """Store laser scan data"""
        self.ranges = msg.ranges
    
    def get_closest_distance_and_direction(self):
        """Find closest obstacle and its direction"""
        if not self.ranges:
            return float('inf'), 0
        
        # Find minimum distance (closest wall)
        min_distance = min(self.ranges)
        min_index = self.ranges.index(min_distance)
        
        return min_distance, min_index
    
    def get_front_distance(self):
        """Get distance to front obstacle"""
        if not self.ranges:
            return float('inf')
        
        front_index = len(self.ranges) // 2
        return self.ranges[front_index]
    
    def get_right_distance(self):
        """Get distance to right side"""
        if not self.ranges:
            return float('inf')
        
        # Simple right side calculation
        right_index = len(self.ranges) // 4
        return self.ranges[right_index]
    
    def rotate_to_face_wall(self):
        """Rotate to face the closest wall"""
        rospy.loginfo("Step 1: Rotating to face closest wall...")
        
        while not rospy.is_shutdown():
            min_dist, min_idx = self.get_closest_distance_and_direction()
            front_dist = self.get_front_distance()
            
            # If closest wall is roughly in front, stop rotating
            front_range = len(self.ranges) // 8  # ±45° tolerance
            front_center = len(self.ranges) // 2
            
            if front_center - front_range <= min_idx <= front_center + front_range:
                break
            
            # Rotate towards closest wall
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop rotation
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Facing closest wall")
    
    def approach_wall(self):
        """Move forward until close to wall"""
        rospy.loginfo("Step 2: Approaching wall...")
        
        while not rospy.is_shutdown():
            front_dist = self.get_front_distance()
            
            if front_dist <= 0.3:
                break
            
            # Move forward slowly
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop movement
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Reached target distance from wall")
    
    def align_for_wall_following(self):
        """Rotate until wall is on right side"""
        rospy.loginfo("Step 3: Aligning for wall following...")
        
        # Rotate 90 degrees left to put wall on right side
        start_time = rospy.Time.now()
        rotation_duration = 3.0  # Approximate time for 90° rotation
        
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() >= rotation_duration:
                break
            
            twist = Twist()
            twist.angular.z = 0.5  # Turn left
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop rotation
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Aligned for wall following")
    
    def find_wall_service(self, request):
        """Service handler for wall finding"""
        rospy.loginfo("=== FIND WALL SERVICE CALLED ===")
        
        if not self.ranges:
            rospy.logwarn("No laser data available")
            return FindWallResponse(wallfound=False)
        
        try:
            # Execute 3-step process
            self.rotate_to_face_wall()
            self.approach_wall()
            self.align_for_wall_following()
            
            rospy.loginfo("Wall finding completed successfully")
            return FindWallResponse(wallfound=True)
            
        except Exception as e:
            rospy.logerr(f"Wall finding failed: {e}")
            return FindWallResponse(wallfound=False)

if __name__ == '__main__':
    try:
        WallFinderMinimal()
    except rospy.ROSInterruptException:
        pass
