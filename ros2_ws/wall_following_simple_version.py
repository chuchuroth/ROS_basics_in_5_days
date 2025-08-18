#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class WallFollowingDemo(Node):
    def __init__(self):
        super().__init__('wall_following_demo')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/working_robot/cmd_vel', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Subscriber to our own scan data
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Control parameters - Conservative settings for stability
        self.target_distance = 0.3  # Target distance to wall (meters)
        self.kp = 1.0  # Proportional gain (reduced for smoother control)
        self.max_linear_speed = 0.1  # Reduced from 0.2 for stability
        self.max_angular_speed = 0.5  # Reduced from 1.0 for stability
        
        # Safety parameters - More conservative obstacle detection
        self.min_front_distance = 0.8  # Increased from 0.5 for earlier detection
        self.emergency_distance = 0.5   # Increased from 0.3 for safer stopping
        self.critical_distance = 0.3    # New: Critical emergency distance
        self.side_check_distance = 0.4  # New: Check sides for obstacles
        
        # Simulation parameters
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.simulation_time = 0.0
        
        # Create timer for publishing scan data and control
        self.scan_timer = self.create_timer(0.1, self.publish_simulated_scan)
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info("Wall Following Demo started!")
        self.get_logger().info("Watch the robot move in the Gazebo simulation")
        self.get_logger().info("Robot will follow the right wall using simulated LiDAR data")
        
        self.laser_msg = None
        
    def publish_simulated_scan(self):
        """Simulate LiDAR scan data based on known wall positions"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree increments
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Create 360 degree scan
        num_ranges = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        
        for i in range(num_ranges):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Simulate walls around the robot
            # Right wall at x = 2.0
            # Top wall at y = 4.0  
            # Bottom wall at y = -4.0
            
            # Calculate what the robot would see at this angle
            global_angle = self.robot_theta + angle
            
            # Check intersection with right wall (x = 2.0)
            if abs(math.cos(global_angle)) > 0.01:  # Avoid division by zero
                dist_to_right_wall = abs((2.0 - self.robot_x) / math.cos(global_angle))
                if math.cos(global_angle) > 0 and dist_to_right_wall > 0:
                    min_range = dist_to_right_wall
                else:
                    min_range = 10.0
            else:
                min_range = 10.0
                
            # Check intersection with top wall (y = 4.0)
            if abs(math.sin(global_angle)) > 0.01:
                dist_to_top_wall = abs((4.0 - self.robot_y) / math.sin(global_angle))
                if math.sin(global_angle) > 0 and dist_to_top_wall > 0:
                    min_range = min(min_range, dist_to_top_wall)
                    
            # Check intersection with bottom wall (y = -4.0)  
            if abs(math.sin(global_angle)) > 0.01:
                dist_to_bottom_wall = abs((-4.0 - self.robot_y) / math.sin(global_angle))
                if math.sin(global_angle) < 0 and dist_to_bottom_wall > 0:
                    min_range = min(min_range, dist_to_bottom_wall)
            
            # Add some noise and clamp to valid range
            if min_range > scan.range_max:
                min_range = scan.range_max
            if min_range < scan.range_min:
                min_range = scan.range_min
                
            ranges.append(min_range)
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        
    def laser_callback(self, msg):
        """Store laser data for control algorithm"""
        self.laser_msg = msg
        
    def control_callback(self):
        """Simple wandering robot with clear obstacle avoidance using laser scan data"""
        if self.laser_msg is None:
            self.get_logger().warn("No laser data received yet!")
            return
            
        # Get ranges for different directions
        ranges = self.laser_msg.ranges
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            self.get_logger().warn("Empty laser scan data!")
            return
            
        self.get_logger().info(f"Received {num_ranges} laser readings")
        
        # Calculate key direction indices
        front_idx = num_ranges // 2  # 0 degrees (front)
        left_idx = 3 * num_ranges // 4  # 90 degrees (left)  
        right_idx = num_ranges // 4  # -90 degrees (right)
        
        # Get distances with bounds checking
        def safe_range(idx):
            if 0 <= idx < len(ranges):
                return ranges[idx] if ranges[idx] != float('inf') else 10.0
            return 10.0
        
        # Sample multiple points for better obstacle detection
        front_distances = []
        for i in range(-10, 11):  # Sample ±10 indices around front
            idx = front_idx + i
            front_distances.append(safe_range(idx))
        
        front_distance = min(front_distances)  # Closest obstacle in front
        left_distance = safe_range(left_idx)
        right_distance = safe_range(right_idx)
        
        # Log the sensor readings clearly
        self.get_logger().info(f"SENSORS: Front={front_distance:.2f}m, Left={left_distance:.2f}m, Right={right_distance:.2f}m")
        
        # Create Twist message
        twist = Twist()
        
        # Simple but effective obstacle avoidance logic
        obstacle_threshold = 0.8  # Reduced for more sensitive detection
        
        if front_distance < obstacle_threshold:
            # OBSTACLE DETECTED IN FRONT!
            self.get_logger().info(f"OBSTACLE AHEAD! Distance: {front_distance:.2f}m")
            
            # Stop forward movement
            twist.linear.x = 0.0
            
            # Choose turn direction based on which side has more space
            if left_distance > right_distance:
                # Turn left (positive angular velocity)
                twist.angular.z = 0.5
                self.get_logger().info(f"TURNING LEFT - Left space: {left_distance:.2f}m > Right space: {right_distance:.2f}m")
            else:
                # Turn right (negative angular velocity)
                twist.angular.z = -0.5
                self.get_logger().info(f"TURNING RIGHT - Right space: {right_distance:.2f}m > Left space: {left_distance:.2f}m")
                
        else:
            # NO OBSTACLE IN FRONT - MOVE FORWARD
            twist.linear.x = 0.2  # Move forward at reasonable speed
            twist.angular.z = 0.0  # Go straight
            self.get_logger().info(f"MOVING FORWARD - Clear path ahead: {front_distance:.2f}m")
        
        # Always log what command we're sending
        self.get_logger().info(f"COMMAND: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Update robot position for simulation (rough estimate)
        dt = 0.1
        self.robot_x += twist.linear.x * math.cos(self.robot_theta) * dt
        self.robot_y += twist.linear.x * math.sin(self.robot_theta) * dt  
        self.robot_theta += twist.angular.z * dt
        self.simulation_time += dt

def main(args=None):
    rclpy.init(args=args)
    
    node = WallFollowingDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Wall following demo stopped by user")
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
