#!/usr/bin/env python3

"""
Simplified Wall Following Node for ROS2
Implements simplified wall following logic with attention zone control.
Uses sensor_msgs/LaserScan, publishes geometry_msgs/Twist commands, and integrates OdomRecord action.
Author: Generated for wall following robot
Date: August 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wall_follower_interfaces.srv import FindWall
from wall_follower_interfaces.action import OdomRecord
import numpy as np
from collections import deque
import math
import time
from enum import Enum


class WallFollowingState(Enum):
    """States for the simplified wall following finite state machine"""
    INITIALIZING = 0
    FINDING_WALL = 1
    WALL_FOLLOWING = 2
    CRASH_STATE = 3  # Robot stuck - needs human intervention
    ERROR_STATE = 4


class SimplifiedWallFollowingNode(Node):
    """
    Simplified Wall Following Node - Uses attention zone logic for wall following
    """
    
    def __init__(self):
        super().__init__('simplified_wall_following')
        
        # Service client for wall finder (kept for boss requirement)
        self.wall_finder_client = self.create_client(FindWall, '/find_wall')
        
        # Action client for odometry recording
        self.odom_action_client = ActionClient(self, OdomRecord, '/record_odom')
        
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.15)  # Slower for safety
        self.declare_parameter('rotation_speed', 0.3)   # For rotations
        self.declare_parameter('slight_rotation_angle', math.radians(5))  # 5 degrees
        self.declare_parameter('attention_zone_start', -90)  # degrees
        self.declare_parameter('attention_zone_end', -70)    # degrees
        self.declare_parameter('crash_timeout', 10.0)  # seconds without movement
        self.declare_parameter('movement_threshold', 0.05)  # minimum movement to detect
        self.declare_parameter('wall_finder_timeout', 30.0)
        
        # Get parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.slight_rotation_angle = self.get_parameter('slight_rotation_angle').value
        self.attention_zone_start = math.radians(self.get_parameter('attention_zone_start').value)
        self.attention_zone_end = math.radians(self.get_parameter('attention_zone_end').value)
        self.crash_timeout = self.get_parameter('crash_timeout').value
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.wall_finder_timeout = self.get_parameter('wall_finder_timeout').value
        
        # Distance thresholds
        self.too_close_threshold = 0.2    # meters
        self.good_min_threshold = 0.2     # meters
        self.good_max_threshold = 0.3     # meters
        self.far_min_threshold = 0.3      # meters
        self.far_max_threshold = 0.5      # meters
        self.ignore_threshold = 0.5       # meters
        
        # Initialize state machine
        self.current_state = WallFollowingState.INITIALIZING
        self.previous_state = None
        
        # Service and action tracking
        self.wall_finder_future = None
        self.odom_action_future = None
        self.state_change_time = time.time()
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current sensor data
        self.current_scan = None
        self.current_odom = None
        
        # Crash detection
        self.last_position = None
        self.last_movement_time = time.time()
        self.position_history = deque(maxlen=50)  # 5 seconds at 10Hz
        
        # Wall following state
        self.rotation_target = None  # For tracking rotation goals
        self.back_forth_phase = 'forward'  # For oscillation movement
        
        self.get_logger().info("Simplified Wall Following node initialized")
        self.get_logger().info(f"Attention Zone: {math.degrees(self.attention_zone_start):.1f}° to {math.degrees(self.attention_zone_end):.1f}°")
        self.get_logger().info("Starting wall finder service call...")
        
    def scan_callback(self, msg):
        """Store the latest laser scan data"""
        self.current_scan = msg
    
    def odom_callback(self, msg):
        """Process odometry data for crash detection"""
        self.current_odom = msg
        
        # Extract position
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        current_time = time.time()
        
        # Check for movement
        if self.last_position is not None:
            distance_moved = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2
            )
            
            if distance_moved > self.movement_threshold:
                self.last_movement_time = current_time
        
        self.last_position = current_pos
        self.position_history.append((current_time, current_pos))
    
    def get_wall_info(self):
        """
        Analyze laser scan to get wall information
        Returns: (min_distance, is_in_attention_zone, multiple_walls_detected)
        """
        if self.current_scan is None:
            return float('inf'), False, False
        
        ranges = self.current_scan.ranges
        if len(ranges) == 0:
            return float('inf'), False, False
        
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        # Convert attention zone angles to indices
        attention_start_idx = self._angle_to_index(self.attention_zone_start, angle_min, angle_increment, len(ranges))
        attention_end_idx = self._angle_to_index(self.attention_zone_end, angle_min, angle_increment, len(ranges))
        
        # Get valid distances in attention zone
        attention_distances = self._get_valid_distances(ranges, attention_start_idx, attention_end_idx)
        
        # Get minimum distance in attention zone
        attention_min_dist = min(attention_distances) if attention_distances else float('inf')
        
        # Check for walls in other areas (for multiple wall detection)
        all_valid_distances = [r for r in ranges if not (math.isinf(r) or math.isnan(r)) and r > 0.0]
        
        # Count walls in different sectors
        wall_sectors = 0
        sector_size = len(ranges) // 8  # Divide into 8 sectors
        
        for i in range(0, len(ranges), sector_size):
            sector_distances = [r for r in ranges[i:i+sector_size] if not (math.isinf(r) or math.isnan(r)) and 0.0 < r < 0.5]
            if sector_distances and min(sector_distances) < 0.5:
                wall_sectors += 1
        
        multiple_walls = wall_sectors > 2  # More than 2 sectors have walls
        
        # Find overall minimum distance
        overall_min_dist = min(all_valid_distances) if all_valid_distances else float('inf')
        
        # Check if closest wall is in attention zone
        is_in_attention_zone = (attention_min_dist <= overall_min_dist + 0.05)  # Small tolerance
        
        return overall_min_dist, is_in_attention_zone, multiple_walls
    
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
    
    def _get_attention_zone_min_distance(self):
        """Get minimum distance specifically in the attention zone"""
        if self.current_scan is None:
            return float('inf')
        
        ranges = self.current_scan.ranges
        if len(ranges) == 0:
            return float('inf')
        
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        # Convert attention zone angles to indices
        attention_start_idx = self._angle_to_index(self.attention_zone_start, angle_min, angle_increment, len(ranges))
        attention_end_idx = self._angle_to_index(self.attention_zone_end, angle_min, angle_increment, len(ranges))
        
        # Get valid distances in attention zone
        attention_distances = self._get_valid_distances(ranges, attention_start_idx, attention_end_idx)
        
        return min(attention_distances) if attention_distances else float('inf')
    
    def _call_wall_finder_service(self):
        """Call the wall finder service"""
        self.current_state = WallFollowingState.FINDING_WALL
        self.state_change_time = time.time()
        
        # Wait for service to be available
        if not self.wall_finder_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Wall finder service not available after 5 seconds")
            self.current_state = WallFollowingState.ERROR_STATE
            return
        
        # Create and send request
        request = FindWall.Request()
        self.wall_finder_future = self.wall_finder_client.call_async(request)
        self.wall_finder_future.add_done_callback(self._wall_finder_response_callback)
        
        self.get_logger().info("Wall finder service call sent...")
    
    def _wall_finder_response_callback(self, future):
        """Handle wall finder service response"""
        try:
            response = future.result()
            if response.wallfound:
                self.get_logger().info("Wall finder service succeeded - starting wall following")
                self.current_state = WallFollowingState.WALL_FOLLOWING
                self.state_change_time = time.time()
            else:
                self.get_logger().error("Wall finder service failed - entering error state")
                self.current_state = WallFollowingState.ERROR_STATE
        except Exception as e:
            self.get_logger().error(f"Wall finder service call failed: {str(e)}")
            self.current_state = WallFollowingState.ERROR_STATE
    
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
        Main control loop implementing the simplified wall following logic
        """
        # Handle initial state transition
        if self.current_state == WallFollowingState.INITIALIZING:
            self._call_wall_finder_service()
            return
        
        # Check for timeout in finding wall state
        if self.current_state == WallFollowingState.FINDING_WALL:
            if time.time() - self.state_change_time > self.wall_finder_timeout:
                self.get_logger().error("Wall finder service timeout")
                self.current_state = WallFollowingState.ERROR_STATE
        
        # Check for crash state (robot not moving)
        if (self.current_state == WallFollowingState.WALL_FOLLOWING and 
            time.time() - self.last_movement_time > self.crash_timeout):
            self.get_logger().error("Robot appears to be stuck - entering crash state")
            self.current_state = WallFollowingState.CRASH_STATE
        
        # Execute state-specific behavior
        cmd = Twist()
        
        if self.current_state == WallFollowingState.INITIALIZING:
            cmd = self._handle_initializing()
            
        elif self.current_state == WallFollowingState.FINDING_WALL:
            cmd = self._handle_finding_wall()
            
        elif self.current_state == WallFollowingState.WALL_FOLLOWING:
            cmd = self._handle_simplified_wall_following()
            
        elif self.current_state == WallFollowingState.CRASH_STATE:
            cmd = self._handle_crash_state()
            
        elif self.current_state == WallFollowingState.ERROR_STATE:
            cmd = self._handle_error_state()
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Log state changes
        if self.current_state != self.previous_state:
            self.get_logger().info(f"State transition: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
    
    def _handle_simplified_wall_following(self):
        """
        Implement simplified wall following logic with better multiple wall handling
        """
        cmd = Twist()
        
        # Get wall information
        wall_distance, in_attention_zone, multiple_walls = self.get_wall_info()
        
        # Get the closest wall in attention zone specifically
        attention_wall_distance = self._get_attention_zone_min_distance()
        
        # Log current status
        self.get_logger().info(
            f"Wall: {wall_distance:.3f}m, Zone Wall: {attention_wall_distance:.3f}m, Multiple: {multiple_walls}",
            throttle_duration_sec=2.0
        )
        
        # SIMPLIFIED PRIORITY SYSTEM:
        # 1. If multiple walls detected: just follow the closest one and ignore complexity
        # 2. Always prefer moving forward when possible
        # 3. Only rotate when absolutely necessary
        
        if multiple_walls:
            # MULTIPLE WALLS: Use simple "follow the closest" approach
            closest_distance = wall_distance
            
            if closest_distance < 0.15:  # Very close - emergency avoidance
                cmd.linear.x = 0.0
                cmd.angular.z = self.rotation_speed * 0.8  # Turn away from closest wall
                self.get_logger().info("Multiple walls + too close: emergency turn")
                
            elif closest_distance < 0.25:  # Close - gentle avoidance
                cmd.linear.x = self.forward_speed * 0.4  # Slow forward
                cmd.angular.z = self.rotation_speed * 0.3  # Gentle turn
                self.get_logger().info("Multiple walls + close: slow forward + gentle turn")
                
            else:  # 0.25m+ - normal following
                cmd.linear.x = self.forward_speed * 0.7  # Moderate speed
                cmd.angular.z = 0.0  # Go straight
                self.get_logger().info("Multiple walls + good distance: forward")
                
        else:
            # SINGLE WALL: Use attention zone logic but simplified
            if attention_wall_distance < float('inf'):
                # Wall is in attention zone - follow it
                if attention_wall_distance < 0.15:
                    # Too close
                    cmd.linear.x = self.forward_speed * 0.3
                    cmd.angular.z = self.rotation_speed * 0.4  # Turn away gently
                    self.get_logger().info("Single wall too close: slow forward + turn away")
                    
                elif attention_wall_distance < 0.35:
                    # Good distance - follow normally
                    cmd.linear.x = self.forward_speed
                    cmd.angular.z = 0.0
                    self.get_logger().info("Single wall good distance: forward")
                    
                else:
                    # Too far - approach gradually
                    cmd.linear.x = self.forward_speed * 0.8
                    cmd.angular.z = -self.rotation_speed * 0.2  # Turn slightly toward wall
                    self.get_logger().info("Single wall far: approach gradually")
                    
            else:
                # No wall in attention zone - search for one
                if wall_distance < 0.6:  # There's a wall somewhere
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.rotation_speed * 0.5  # Turn to find wall
                    self.get_logger().info("Wall exists but not in zone: searching")
                else:
                    # No walls nearby - explore
                    cmd.linear.x = self.forward_speed
                    cmd.angular.z = 0.0
                    self.get_logger().info("No walls nearby: exploring")
        
        return cmd
    
    def _handle_back_forth_movement(self):
        """
        Handle back and forth movement to approach wall
        """
        cmd = Twist()
        
        # Simple oscillation logic
        if self.back_forth_phase == 'forward':
            cmd.linear.x = self.forward_speed * 0.7  # Slower approach
            cmd.angular.z = 0.0
            # Switch phases periodically or based on distance
            if hasattr(self, '_back_forth_start_time'):
                if time.time() - self._back_forth_start_time > 1.0:  # 1 second forward
                    self.back_forth_phase = 'backward'
                    self._back_forth_start_time = time.time()
            else:
                self._back_forth_start_time = time.time()
        else:  # backward
            cmd.linear.x = -self.forward_speed * 0.3  # Gentle backward
            cmd.angular.z = 0.0
            if time.time() - self._back_forth_start_time > 0.5:  # 0.5 second backward
                self.back_forth_phase = 'forward'
                self._back_forth_start_time = time.time()
        
        return cmd
    
    
    def _call_wall_finder_service(self):
        """Call the wall finder service (kept for boss requirement)"""
        self.current_state = WallFollowingState.FINDING_WALL
        self.state_change_time = time.time()
        
        # Wait for service to be available
        if not self.wall_finder_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("Wall finder service not available - starting directly")
            self._start_wall_following()
            return
        
        # Create and send request
        request = FindWall.Request()
        self.wall_finder_future = self.wall_finder_client.call_async(request)
        self.wall_finder_future.add_done_callback(self._wall_finder_response_callback)
        
        self.get_logger().info("Wall finder service call sent...")
    
    def _wall_finder_response_callback(self, future):
        """Handle wall finder service response"""
        try:
            response = future.result()
            if response.wallfound:
                self.get_logger().info("Wall finder service succeeded - starting wall following")
                self._start_wall_following()
            else:
                self.get_logger().warning("Wall finder service failed - starting anyway")
                self._start_wall_following()
        except Exception as e:
            self.get_logger().warning(f"Wall finder service call failed: {str(e)} - starting anyway")
            self._start_wall_following()
    
    def _start_wall_following(self):
        """Start wall following and odometry recording"""
        self.current_state = WallFollowingState.WALL_FOLLOWING
        self.state_change_time = time.time()
        self.last_movement_time = time.time()
        
        # Start odometry recording action
        self._start_odom_recording()
    
    def _start_odom_recording(self):
        """Start the odometry recording action"""
        if not self.odom_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("Odometry action server not available")
            return
        
        goal = OdomRecord.Goal()
        # Goal can be empty or contain specific requirements
        
        self.odom_action_future = self.odom_action_client.send_goal_async(
            goal,
            feedback_callback=self._odom_feedback_callback
        )
        self.odom_action_future.add_done_callback(self._odom_response_callback)
        self.get_logger().info("Started odometry recording action")
    
    def _odom_feedback_callback(self, feedback_msg):
        """Handle odometry recording feedback"""
        feedback = feedback_msg.feedback
        # Log total distance traveled periodically
        self.get_logger().info(
            f"Total distance traveled: {feedback.total_distance:.2f}m",
            throttle_duration_sec=5.0
        )
    
    def _odom_response_callback(self, future):
        """Handle odometry recording action completion"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("Odometry recording goal accepted")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._odom_result_callback)
            else:
                self.get_logger().warning("Odometry recording goal rejected")
        except Exception as e:
            self.get_logger().error(f"Odometry action failed: {str(e)}")
    
    def _odom_result_callback(self, future):
        """Handle odometry recording final result"""
        try:
            result = future.result().result
            self.get_logger().info(f"Lap completed! Recorded {len(result.list_of_odoms)} odometry points")
            # Could restart the action for continuous recording
            self._start_odom_recording()
        except Exception as e:
            self.get_logger().error(f"Odometry result error: {str(e)}")
    
    def _handle_initializing(self):
        """Handle initialization state"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd
    
    def _handle_finding_wall(self):
        """Handle wall finding state"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd
    
    def _handle_crash_state(self):
        """Handle crash state - robot needs human intervention"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        # Log error periodically
        if not hasattr(self, '_last_crash_log'):
            self.get_logger().error("CRASH STATE: Robot stuck - human intervention required!")
            self._last_crash_log = time.time()
        elif time.time() - self._last_crash_log > 5.0:
            self.get_logger().error("Still in crash state - please help the robot!")
            self._last_crash_log = time.time()
        
        return cmd
    
    def _handle_error_state(self):
        """Handle error state - stop robot and log error"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        # Log error periodically
        if not hasattr(self, '_last_error_log'):
            self.get_logger().error("In error state - robot stopped.")
            self._last_error_log = time.time()
        elif time.time() - self._last_error_log > 5.0:
            self.get_logger().error("Still in error state - check system")
            self._last_error_log = time.time()
        
        return cmd


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        node = SimplifiedWallFollowingNode()
        
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
