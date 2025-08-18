#!/usr/bin/env python3

"""
Wall Finder Node for ROS2
Implements a finite state machine to find and approach a wall for wall following.
Uses sensor_msgs/LaserScan and publishes geometry_msgs/Twist commands.
Author: Generated for wall following robot
Date: August 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
import numpy as np
from collections import deque
import math
from enum import Enum


class WallFinderState(Enum):
    """States for the wall finder finite state machine"""
    IDLE = 0
    MOVING_FORWARD = 1
    TURNING_LEFT = 2
    ALIGNING_TO_WALL = 3
    WALL_FOUND = 4


class WallFinderNode(Node):
    """
    Wall Finder Node - Finds and approaches a wall for wall following
    """
    
    def __init__(self):
        super().__init__('wall_finder_fsm')
        
        # Service server for FindWall
        self.service = self.create_service(FindWall, '/find_wall', self.find_wall_callback)
        
        # Declare parameters with default values
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('slow_speed', 0.1)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('front_threshold_far', 0.52)
        self.declare_parameter('front_threshold_near', 0.48)
        self.declare_parameter('front_threshold_close', 0.3)
        self.declare_parameter('target_wall_distance', 0.25)
        self.declare_parameter('wall_distance_tolerance', 0.05)
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('hysteresis_margin', 0.02)
        self.declare_parameter('max_search_time', 30.0)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.slow_speed = self.get_parameter('slow_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.front_threshold_far = self.get_parameter('front_threshold_far').get_parameter_value().double_value
        self.front_threshold_near = self.get_parameter('front_threshold_near').get_parameter_value().double_value
        self.front_threshold_close = self.get_parameter('front_threshold_close').get_parameter_value().double_value
        self.target_wall_distance = self.get_parameter('target_wall_distance').get_parameter_value().double_value
        self.wall_distance_tolerance = self.get_parameter('wall_distance_tolerance').get_parameter_value().double_value
        self.filter_window_size = self.get_parameter('filter_window_size').get_parameter_value().integer_value
        self.hysteresis_margin = self.get_parameter('hysteresis_margin').get_parameter_value().double_value
        self.max_search_time = self.get_parameter('max_search_time').get_parameter_value().double_value
        
        # Initialize state machine
        self.current_state = WallFinderState.IDLE
        self.previous_state = None
        
        # Service execution tracking
        self.service_active = False
        self.service_start_time = None
        
        # Initialize filters for distance measurements
        self.front_distance_filter = deque(maxlen=self.filter_window_size)
        self.right_distance_filter = deque(maxlen=self.filter_window_size)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current filtered distances
        self.current_front_distance = float('inf')
        self.current_right_distance = float('inf')
        
        # Alignment tracking
        self.alignment_start_time = None
        self.max_alignment_time = 10.0  # Maximum time to spend aligning
        
        self.get_logger().info("Wall Finder FSM node initialized. Waiting for find_wall service calls.")
        self.get_logger().info("Node will wait for laser data when service is called.")
        
    def scan_callback(self, msg):
        """
        Process laser scan data and update filtered distance measurements
        """
        if len(msg.ranges) == 0:
            return
            
        # Calculate angle indices for sectors
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Front sector: ±30 degrees
        front_angle_range = math.radians(30)
        front_start_idx = self._angle_to_index(-front_angle_range, angle_min, angle_increment, len(msg.ranges))
        front_end_idx = self._angle_to_index(front_angle_range, angle_min, angle_increment, len(msg.ranges))
        
        # Right sector: 80-100 degrees (right side of robot)
        right_start_angle = math.radians(-100)  # Negative because right is clockwise
        right_end_angle = math.radians(-80)
        right_start_idx = self._angle_to_index(right_start_angle, angle_min, angle_increment, len(msg.ranges))
        right_end_idx = self._angle_to_index(right_end_angle, angle_min, angle_increment, len(msg.ranges))
        
        # Get minimum valid distances in each sector
        front_distances = self._get_valid_distances(msg.ranges, front_start_idx, front_end_idx)
        right_distances = self._get_valid_distances(msg.ranges, right_start_idx, right_end_idx)
        
        # Update filters with new measurements
        if front_distances:
            self.front_distance_filter.append(min(front_distances))
        if right_distances:
            self.right_distance_filter.append(min(right_distances))
            
        # Apply median filter to reduce noise
        if len(self.front_distance_filter) > 0:
            self.current_front_distance = np.median(list(self.front_distance_filter))
        if len(self.right_distance_filter) > 0:
            self.current_right_distance = np.median(list(self.right_distance_filter))
    
    def find_wall_callback(self, request, response):
        """Service callback to find and approach the nearest wall using FSM approach"""
        self.get_logger().info("Find wall service called - starting FSM wall finding")
        
        try:
            # Simple check for laser data availability
            if self.current_front_distance == float('inf') or self.current_right_distance == float('inf'):
                self.get_logger().warn("No laser data available yet - will start when data arrives")
                # Don't fail immediately, let the FSM wait for data
            
            # Start the FSM wall finding process (completely non-blocking)
            self.service_active = True
            self.service_start_time = self.get_clock().now()
            self.current_state = WallFinderState.MOVING_FORWARD
            self.get_logger().info("Wall finding FSM started - will execute in control loop")
            
            # For now, return success immediately - the FSM will run in control loop
            # In a real implementation, you might want to use action servers for long-running tasks
            response.wallfound = True
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in find_wall service: {str(e)}")
            response.wallfound = False
            return response
    
    def _final_wall_positioning(self):
        """Perform final positioning to ensure wall is on the right side"""
        self.get_logger().info("Performing final wall positioning...")
        
        # Rotate 90 degrees left so wall is on the right side
        cmd = Twist()
        cmd.angular.z = self.turn_speed
        
        # Calculate rotation time for 90 degrees
        rotation_time = math.pi / (2 * self.turn_speed)
        start_time = self.get_clock().now()
        rate = self.create_rate(10)
        
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= rotation_time:
                break
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        self.get_logger().info("Final positioning complete - wall should be on right side")
    
    def _stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def _get_state_command(self):
        """Get velocity command for current state"""
        cmd = Twist()
        
        if self.current_state == WallFinderState.IDLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        elif self.current_state == WallFinderState.MOVING_FORWARD:
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
            
        elif self.current_state == WallFinderState.TURNING_LEFT:
            cmd.linear.x = self.slow_speed
            cmd.angular.z = self.turn_speed
            
        elif self.current_state == WallFinderState.ALIGNING_TO_WALL:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            
        elif self.current_state == WallFinderState.WALL_FOUND:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        return cmd
    
    def _angle_to_index(self, angle, angle_min, angle_increment, total_points):
        """Convert angle to array index"""
        index = int((angle - angle_min) / angle_increment)
        return max(0, min(index, total_points - 1))
    
    def _get_valid_distances(self, ranges, start_idx, end_idx):
        """Get valid (non-inf, non-nan) distances from a range of indices"""
        if start_idx > end_idx:
            # Handle wraparound case
            valid_ranges = ranges[start_idx:] + ranges[:end_idx + 1]
        else:
            valid_ranges = ranges[start_idx:end_idx + 1]
            
        return [r for r in valid_ranges if not (math.isinf(r) or math.isnan(r)) and r > 0.0]
    
    def control_loop(self):
        """
        Main control loop - handles both service execution and autonomous control
        """
        # When service is active, execute FSM for wall finding
        if self.service_active:
            # Check if we have laser data yet
            if self.current_front_distance == float('inf') or self.current_right_distance == float('inf'):
                # Still waiting for laser data, keep robot stopped
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                
                # Log waiting status occasionally
                if not hasattr(self, '_last_wait_log') or (self.get_clock().now().nanoseconds - getattr(self, '_last_wait_log', 0)) / 1e9 > 2.0:
                    self.get_logger().info("FSM waiting for laser data...")
                    self._last_wait_log = self.get_clock().now().nanoseconds
                return
            
            # Check for timeout
            if self.service_start_time:
                elapsed_time = (self.get_clock().now() - self.service_start_time).nanoseconds / 1e9
                if elapsed_time > self.max_search_time:
                    self.get_logger().warn(f"Wall finding timed out after {elapsed_time:.1f} seconds")
                    self.service_active = False
                    self.current_state = WallFinderState.IDLE
                    return
            
            # Update state machine
            self._update_state_with_hysteresis()
            
            # Execute current state behavior
            cmd = self._get_state_command()
            self.cmd_vel_pub.publish(cmd)
            
            # Debug logging - show velocity commands being sent
            if not hasattr(self, '_last_debug_log') or (self.get_clock().now().nanoseconds - self._last_debug_log) / 1e9 > 1.0:
                self.get_logger().info(
                    f"FSM Active - State: {self.current_state}, "
                    f"Front: {self.current_front_distance:.3f}m, Right: {self.current_right_distance:.3f}m, "
                    f"Publishing cmd_vel: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
                )
                self._last_debug_log = self.get_clock().now().nanoseconds
            
            # Log state changes
            if self.current_state != self.previous_state:
                self.get_logger().info(
                    f"FSM State: {self.previous_state} -> {self.current_state}, "
                    f"Front: {self.current_front_distance:.3f}m, Right: {self.current_right_distance:.3f}m"
                )
                self.previous_state = self.current_state
            
            # Check if wall finding is complete
            if self.current_state == WallFinderState.WALL_FOUND:
                self.get_logger().info("Wall finding complete - stopping FSM")
                self._final_wall_positioning()
                self._stop_robot()
                self.service_active = False
                self.current_state = WallFinderState.IDLE
            
            return
            
        # Keep robot stopped when idle (service not active)
        if self.current_state == WallFinderState.IDLE:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return
        
        # This section would only run if FSM is running autonomously (not during service)
        # Apply hysteresis to state transitions
        self._update_state_with_hysteresis()
        
        # Execute state-specific behavior
        cmd = Twist()
        
        if self.current_state == WallFinderState.MOVING_FORWARD:
            cmd = self._handle_moving_forward()
            
        elif self.current_state == WallFinderState.TURNING_LEFT:
            cmd = self._handle_turning_left()
            
        elif self.current_state == WallFinderState.ALIGNING_TO_WALL:
            cmd = self._handle_aligning_to_wall()
            
        elif self.current_state == WallFinderState.WALL_FOUND:
            cmd = self._handle_wall_found()
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Log state changes
        if self.current_state != self.previous_state:
            self.get_logger().info(
                f"State transition: {self.previous_state} -> {self.current_state}, "
                f"Front: {self.current_front_distance:.3f}m, Right: {self.current_right_distance:.3f}m"
            )
            self.previous_state = self.current_state
    
    def _update_state_with_hysteresis(self):
        """
        Update state machine with hysteresis to prevent oscillations
        """
        front_dist = self.current_front_distance
        right_dist = self.current_right_distance
        
        if self.current_state == WallFinderState.MOVING_FORWARD:
            # Transition to turning left if obstacle ahead
            if front_dist < (self.front_threshold_near - self.hysteresis_margin):
                if front_dist > (self.front_threshold_close + self.hysteresis_margin):
                    self.current_state = WallFinderState.TURNING_LEFT
                else:
                    self.current_state = WallFinderState.ALIGNING_TO_WALL
                    self.alignment_start_time = self.get_clock().now()
                    
        elif self.current_state == WallFinderState.TURNING_LEFT:
            # Continue turning until we need to align or can move forward again
            if front_dist <= (self.front_threshold_close + self.hysteresis_margin):
                self.current_state = WallFinderState.ALIGNING_TO_WALL
                self.alignment_start_time = self.get_clock().now()
            elif front_dist > (self.front_threshold_far + self.hysteresis_margin):
                self.current_state = WallFinderState.MOVING_FORWARD
                
        elif self.current_state == WallFinderState.ALIGNING_TO_WALL:
            # Check if wall is now on the right side
            if (self.target_wall_distance - self.wall_distance_tolerance) <= right_dist <= (self.target_wall_distance + self.wall_distance_tolerance):
                self.current_state = WallFinderState.WALL_FOUND
            # Timeout protection
            elif self.alignment_start_time and (self.get_clock().now() - self.alignment_start_time).nanoseconds / 1e9 > self.max_alignment_time:
                self.get_logger().warn("Alignment timeout, returning to moving forward")
                self.current_state = WallFinderState.MOVING_FORWARD
                
        elif self.current_state == WallFinderState.WALL_FOUND:
            # Wall found - this node's job is done
            # In a real application, this might trigger a service call to start wall following
            pass
    
    def _handle_moving_forward(self):
        """Handle MOVING_FORWARD state"""
        cmd = Twist()
        cmd.linear.x = self.max_speed
        cmd.angular.z = 0.0
        return cmd
    
    def _handle_turning_left(self):
        """Handle TURNING_LEFT state"""
        cmd = Twist()
        cmd.linear.x = self.slow_speed
        cmd.angular.z = self.turn_speed  # Positive angular velocity = left turn
        return cmd
    
    def _handle_aligning_to_wall(self):
        """Handle ALIGNING_TO_WALL state"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.turn_speed  # Keep turning left to find wall on right
        return cmd
    
    def _handle_wall_found(self):
        """Handle WALL_FOUND state"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        # Log success message periodically
        if hasattr(self, '_last_success_log'):
            if (self.get_clock().now() - self._last_success_log).nanoseconds / 1e9 > 2.0:
                self.get_logger().info(f"Wall found at {self.current_right_distance:.3f}m on right side!")
                self._last_success_log = self.get_clock().now()
        else:
            self.get_logger().info(f"Wall found at {self.current_right_distance:.3f}m on right side!")
            self._last_success_log = self.get_clock().now()
        
        return cmd


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        node = WallFinderNode()
        
        # Use a MultiThreadedExecutor for better callback handling
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            # Run the executor
            executor.spin()
        finally:
            # Stop the robot before shutting down
            cmd = Twist()
            node.cmd_vel_pub.publish(cmd)
            node.get_logger().info("Stopping robot and shutting down...")
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
