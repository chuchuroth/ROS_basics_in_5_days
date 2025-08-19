#!/usr/bin/env python3

"""
ROS2 Wall Follower - Minimal Implementation
Simple wall following using minimal laser processing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
import math

class WallFollowerMinimal(Node):
    def __init__(self):
        super().__init__('wall_follower_minimal')
        
        # Call find_wall service first
        self.call_find_wall()
        
        # Publisher for robot movement
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Store laser data
        self.ranges = None
        
        self.get_logger().info("Minimal Wall Follower ready")
    
    def call_find_wall(self):
        """Call find_wall service to locate wall"""
        try:
            # Create service client
            client = self.create_client(FindWall, '/find_wall')
            
            # Wait for service
            if client.wait_for_service(timeout_sec=5.0):
                # Call service
                request = FindWall.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                
                if future.result() and future.result().wallfound:
                    self.get_logger().info("Wall found successfully!")
                else:
                    self.get_logger().warn("Wall not found, continuing anyway")
            else:
                self.get_logger().warn("Find wall service not available")
                
        except Exception as e:
            self.get_logger().warn(f"Find wall service failed: {e}")
    
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

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerMinimal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
