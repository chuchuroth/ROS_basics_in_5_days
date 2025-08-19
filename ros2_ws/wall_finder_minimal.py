#!/usr/bin/env python3

"""
ROS2 Wall Finder - Minimal Implementation
Simple wall finding service using minimal laser processing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
import math

class WallFinderMinimal(Node):
    def __init__(self):
        super().__init__('wall_finder_minimal')
        
        # Publisher for robot movement
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Service server
        self.service = self.create_service(FindWall, '/find_wall', self.find_wall_service)
        
        # Store laser data
        self.ranges = None
        
        self.get_logger().info("Minimal Wall Finder ready")
    
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
        self.get_logger().info("Step 1: Rotating to face closest wall...")
        
        start_time = self.get_clock().now()
        while rclpy.ok():
            # Check timeout
            if (self.get_clock().now() - start_time).nanoseconds > 10e9:  # 10 seconds
                break
            
            min_dist, min_idx = self.get_closest_distance_and_direction()
            
            # If closest wall is roughly in front, stop rotating
            front_range = len(self.ranges) // 8  # ±45° tolerance
            front_center = len(self.ranges) // 2
            
            if front_center - front_range <= min_idx <= front_center + front_range:
                break
            
            # Rotate towards closest wall
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop rotation
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Facing closest wall")
    
    def approach_wall(self):
        """Move forward until close to wall"""
        self.get_logger().info("Step 2: Approaching wall...")
        
        start_time = self.get_clock().now()
        while rclpy.ok():
            # Check timeout
            if (self.get_clock().now() - start_time).nanoseconds > 10e9:  # 10 seconds
                break
            
            front_dist = self.get_front_distance()
            
            if front_dist <= 0.3:
                break
            
            # Move forward slowly
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop movement
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Reached target distance from wall")
    
    def align_for_wall_following(self):
        """Rotate until wall is on right side"""
        self.get_logger().info("Step 3: Aligning for wall following...")
        
        # Rotate 90 degrees left to put wall on right side
        start_time = self.get_clock().now()
        rotation_duration = 3.0  # Approximate time for 90° rotation
        
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= rotation_duration:
                break
            
            twist = Twist()
            twist.angular.z = 0.5  # Turn left
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop rotation
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Aligned for wall following")
    
    def find_wall_service(self, request, response):
        """Service handler for wall finding"""
        self.get_logger().info("=== FIND WALL SERVICE CALLED ===")
        
        if not self.ranges:
            self.get_logger().warn("No laser data available")
            response.wallfound = False
            return response
        
        try:
            # Execute 3-step process
            self.rotate_to_face_wall()
            self.approach_wall()
            self.align_for_wall_following()
            
            self.get_logger().info("Wall finding completed successfully")
            response.wallfound = True
            return response
            
        except Exception as e:
            self.get_logger().error(f"Wall finding failed: {e}")
            response.wallfound = False
            return response

def main(args=None):
    rclpy.init(args=args)
    node = WallFinderMinimal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
