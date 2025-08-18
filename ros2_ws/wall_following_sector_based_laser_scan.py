#!/usr/bin/env python3

"""
Wall Following Control Node - Simplified with Sector-Based Analysis
==================================================================

This ROS 2 node implements autonomous wall following behavior for a robot.
The robot will:
1. Call the find_wall service to locate and align with nearest wall
2. Start odometry recording action for path tracking  
3. Execute continuous wall following control loop with sector-based obstacle detection

180-Degree Forward-Facing Laser Configuration:
- Total points: 720 (indices 0-719) 
- 0° (right): index 0
- 90° (front): index 360
- 180° (left): index 719
- Coverage: 180° forward hemisphere only

Wall Following Strategy:
- Maintain constant distance from RIGHT-side wall
- Avoid front obstacles using sector-based detection
- Simple control commands for robust operation

SECTOR-BASED ANALYSIS METHOD:
This implementation uses the sector-based laser analysis method for improved
obstacle detection and navigation decision making, dividing the 180° laser 
scan into meaningful sectors.

Author: ROS 2 Migration Team
Date: August 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
from wall_follower_interfaces.action import OdomRecord
import numpy as np
import math


class WallFollowing(Node):
    """
    Wall Following Control Node
    
    Implements autonomous wall following behavior with:
    - Service integration for wall finding
    - Action client for odometry recording
    - Bang-bang distance control for robust operation
    - Obstacle avoidance logic
    """
    
    def __init__(self):
        """
        Initialize the Wall Following Controller node with all necessary
        publishers, subscribers, service clients, and control parameters
        """
        super().__init__('wall_following_node')
        
        # ==================== PUBLISHERS & SUBSCRIBERS ====================
        
        # Publisher for robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to laser scan data for wall detection and avoidance
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        # ==================== SERVICE & ACTION CLIENTS ====================
        
        # Service client for finding and aligning with walls
        self.find_wall_client = self.create_client(FindWall, '/find_wall')
        
        # Action client for recording odometry data during wall following
        self.odom_action_client = ActionClient(self, OdomRecord, '/record_odom')
        
        # ==================== LASER DATA STORAGE ====================
        
        # Current laser scan data
        self.ranges = None
        self.num_ranges = None
        self.angle_min = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None
        
        # ==================== CONTROL PARAMETERS ====================
        
        # Wall following distance control
        self.min_wall_distance = 0.2       # meters - minimum distance from wall
        self.max_wall_distance = 0.3       # meters - maximum distance from wall
        
        # Obstacle detection
        self.front_obstacle_threshold = 0.5    # meters - front obstacle detection
        
        # Speed parameters
        self.forward_speed = 0.1            # m/s - normal forward speed
        self.turn_speed = 0.4               # rad/s - normal turning speed
        
        # ==================== STATE VARIABLES ====================
        
        # Operational state flags
        self.wall_found = False
        self.odom_recording = False
        self.preparation_complete = False
        
        # ==================== INITIALIZATION ====================
        
        self.get_logger().info("Wall Following Controller initialized")
        self.get_logger().info(f"Parameters: wall distance {self.min_wall_distance}-{self.max_wall_distance}m, "
                              f"obstacle threshold {self.front_obstacle_threshold}m")
        
        # Execute preparation sequence
        self.prepare_robot()
        
        # Start main control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def laser_callback(self, msg: LaserScan):
        """
        Process incoming laser scan data for 180-degree forward-facing laser
        
        Simple processing for wall following:
        - Store basic laser parameters
        - Filter out invalid readings
        - Store ranges for sector-based analysis
        
        Args:
            msg (LaserScan): Incoming laser scan message
        """
        # Store basic laser parameters
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        
        # Convert ranges to numpy array and filter invalid readings
        ranges = np.array(msg.ranges)
        self.ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        self.num_ranges = len(self.ranges)
        
        # Log laser info once for debugging
        if not hasattr(self, '_laser_logged'):
            self._laser_logged = True
            self.get_logger().info(f"180° Laser: {self.num_ranges} points, 0°-180° coverage")

    def get_sector_obstacle_detections(self, obstacle_threshold=0.8):
        """
        Detect obstacles in each sector using sector-based analysis
        
        Sectors for 180° laser (adapted from example file):
        - Right: indices 0-19 (0°-4.75°)
        - Front_Right: indices 20-60 (5°-15°) 
        - Front_Center: indices 340-380 (85°-95°)
        - Front_Left: indices 100-140 (25°-35°)
        - Left_Near: indices 141-179 (35.25°-44.75°)
        
        Args:
            obstacle_threshold (float): Distance threshold for obstacle detection
            
        Returns:
            dict: Boolean flags for obstacle detection in each sector
        """
        if self.ranges is None:
            return {}
        
        # Define key sectors for wall following
        sectors = {
            "Right": (0, 19),           # Right side wall
            "Front_Right": (20, 60),    # Front-right area
            "Front_Center": (340, 380), # Direct front
            "Front_Left": (100, 140),   # Front-left area  
            "Left_Near": (141, 179),    # Left side
        }
        
        detections = {}
        for sector_name, (start_idx, end_idx) in sectors.items():
            # Ensure indices are within valid range
            start_idx = max(0, start_idx)
            end_idx = min(len(self.ranges) - 1, end_idx)
            
            if start_idx <= end_idx:
                # Get minimum distance in sector
                sector_ranges = self.ranges[start_idx:end_idx + 1]
                valid_ranges = sector_ranges[np.isfinite(sector_ranges)]
                
                if len(valid_ranges) > 0:
                    min_distance = np.min(valid_ranges)
                    detections[sector_name] = min_distance < obstacle_threshold
                else:
                    detections[sector_name] = False
            else:
                detections[sector_name] = False
        
        return detections

    def get_right_wall_distance(self):
        """
        Get distance to right wall for wall following control
        
        Returns:
            float: Distance to right wall in meters
        """
        if self.ranges is None:
            return float('inf')
        
        # Right wall is at index 0 (0°)
        right_idx = 0
        # Average over small window for stability
        window = 2
        indices = [max(0, right_idx + i) for i in range(-window, window + 1)]
        distances = [self.ranges[i] for i in indices if i < len(self.ranges)]
        return np.mean(distances) if distances else float('inf')

    def get_front_distance(self):
        """
        Get distance to front obstacle
        
        Returns:
            float: Distance to front obstacle in meters
        """
        if self.ranges is None:
            return float('inf')
        
        # Front is at index 360 (90°)
        front_idx = 360
        # Average over small window for stability
        window = 5
        indices = [front_idx + i for i in range(-window, window + 1)]
        distances = [self.ranges[i] for i in indices if 0 <= i < len(self.ranges)]
        return np.mean(distances) if distances else float('inf')

    def calculate_wall_following_control(self):
        """
        Calculate velocity commands for wall following using sector-based obstacle detection
        
        Simple control strategy:
        1. Check sectors for front obstacles - turn left if detected
        2. Maintain distance from right wall - turn right/left as needed
        3. Go straight when conditions are good
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        # Get obstacle detections using sector-based analysis
        detections = self.get_sector_obstacle_detections(self.front_obstacle_threshold)
        
        # Get wall distance
        right_dist = self.get_right_wall_distance()
        
        # Priority 1: Front obstacle avoidance
        if detections.get("Front_Center", False) or detections.get("Front_Right", False):
            self.get_logger().info("Front obstacle detected - turning left")
            return self.forward_speed * 0.5, self.turn_speed  # Slow down and turn left
        
        # Priority 2: Wall distance control
        elif right_dist > self.max_wall_distance:
            # Too far from wall - turn right
            return self.forward_speed, -self.turn_speed
        elif right_dist < self.min_wall_distance:
            # Too close to wall - turn left
            return self.forward_speed, self.turn_speed
        else:
            # Good distance - go straight
            return self.forward_speed, 0.0

    def publish_velocity(self, linear_vel, angular_vel):
        """
        Publish velocity commands to the robot
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
        """
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """
        Send stop command to robot (zero velocities)
        """
        self.publish_velocity(0.0, 0.0)

    def prepare_robot(self):
        """
        Execute complete preparation sequence before starting wall following
        
        This orchestrates the setup process:
        1. Call find_wall service to locate and align with nearest wall
        2. Start odometry recording action for path tracking
        3. Mark preparation as complete to enable control loop
        """
        self.get_logger().info("=== ROBOT PREPARATION SEQUENCE ===")
        
        # Step 1: Find and align with wall
        if not self.call_find_wall_service():
            self.get_logger().error(" Could not find wall - aborting preparation")
            return False
        
        # Step 2: Start odometry recording
        if not self.start_odom_recording():
            self.get_logger().error(" Could not start odometry recording - aborting preparation")
            return False
        
        # Step 3: Mark preparation complete
        self.preparation_complete = True
        self.get_logger().info(" Preparation complete - starting wall following behavior")
        
        return True

    def call_find_wall_service(self):
        """
        Call the find_wall service to locate and align with the nearest wall
        
        This service will:
        1. Rotate robot to face nearest wall
        2. Approach wall to optimal distance
        3. Align robot so wall is on right side for following
        
        Returns:
            bool: True if service call successful and wall found, False otherwise
        """
        self.get_logger().info(" Calling find_wall service...")
        
        # Wait for service to become available
        self.get_logger().info("Waiting for find_wall service...")
        if not self.find_wall_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("find_wall service not available after 10 seconds!")
            return False
        
        # Create and send service request
        request = FindWall.Request()
        self.get_logger().info("Sending find_wall request...")
        
        try:
            # Call service with timeout
            future = self.find_wall_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
            
            # Check service response
            if future.result() is not None:
                response = future.result()
                if response.wallfound:
                    self.get_logger().info(" Wall found and robot aligned successfully!")
                    self.wall_found = True
                    return True
                else:
                    self.get_logger().warn(" find_wall service returned wallfound=False")
                    return False
            else:
                self.get_logger().error(" No response from /find_wall service (timeout)")
                return False
                
        except Exception as e:
            self.get_logger().error(f" find_wall service call failed: {str(e)}")
            return False

    def start_odom_recording(self):
        """
        Start the odometry recording action to track robot path during wall following
        
        This action will continuously record the robot's position and orientation
        throughout the wall following behavior for later analysis.
        
        Returns:
            bool: True if action started successfully, False otherwise
        """
        self.get_logger().info(" Starting odometry recording...")
        
        # Wait for action server to become available
        self.get_logger().info("Waiting for /record_odom action server...")
        if not self.odom_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/record_odom action server not available after 10 seconds!")
            return False
        
        # Create and send action goal
        goal_msg = OdomRecord.Goal()
        self.get_logger().info("Sending odometry recording goal...")
        
        try:
            # Send goal with timeout
            send_goal_future = self.odom_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            # Check goal acceptance
            goal_handle = send_goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                self.get_logger().info(" Odometry recording started successfully!")
                self.odom_recording = True
                return True
            else:
                self.get_logger().error(" Odometry recording goal rejected")
                return False
                
        except Exception as e:
            self.get_logger().error(f" Failed to start odometry recording: {str(e)}")
            return False

    def control_loop(self):
        """
        Main control loop for wall following behavior
        
        Simple control strategy:
        1. Wait for preparation to complete
        2. Calculate control commands using sector-based analysis
        3. Publish velocity commands
        """
        # Only run if preparation is complete
        if not self.preparation_complete:
            return
        
        # Ensure we have laser data
        if self.ranges is None:
            self.stop_robot()
            return
        
        try:
            # Calculate velocities using sector-based control
            linear_vel, angular_vel = self.calculate_wall_following_control()
            
            # Send commands to robot
            self.publish_velocity(linear_vel, angular_vel)
            
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")
            self.stop_robot()


def main(args=None):
    """
    Main entry point for the wall following controller node
    """
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create wall following controller
    controller = WallFollowing()
    
    try:
        controller.get_logger().info("Wall following system running - Press Ctrl+C to stop")
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("Wall following stopped by user")
        
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
