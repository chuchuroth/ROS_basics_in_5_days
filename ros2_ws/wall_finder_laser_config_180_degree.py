#!/usr/bin/env python3

"""
Wall Finder Service Node - 180° Forward Laser Corrected Version
================================================================

This ROS 2 node provides a service to find and align the robot with the nearest wall.
The robot will:
1. Rotate to face the nearest wall (closest obstacle)
2. Move forward until 0.3m from the wall
3. Rotate until the wall is positioned on the right side for wall following

180-Degree Forward-Facing Laser Configuration:
- Total points: 720 (indices 0-719) for real TurtleBot
- Coverage: 180° forward-facing only (right to left)
- 0° (right): index 0
- 90° (front): index 360
- 180° (left): index 719
- Note: No rear coverage - robot only sees forward hemisphere

Author: ROS 2 Migration Team
Date: August 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
import numpy as np
import math
import time


class WallFinder(Node):
    """
    Wall Finder Service Node for 180° Forward-Facing Laser
    
    Provides the 'find_wall' service that locates the nearest wall and positions
    the robot appropriately for wall following behavior using 180° laser coverage.
    """
    
    def __init__(self):
        """
        Initialize the Wall Finder node with publishers, subscribers, and service
        """
        super().__init__('wall_finder_node')  # Match the node name from logs
        
        # ==================== PUBLISHERS & SUBSCRIBERS ====================
        
        # Publisher for robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Wait for /scan topic to be available before subscribing
        self.get_logger().info("🔍 Checking for /scan topic availability...")
        scan_topic_found = False
        check_attempts = 0
        max_check_attempts = 10
        
        while not scan_topic_found and check_attempts < max_check_attempts:
            topic_names = self.get_topic_names_and_types()
            scan_topics = [name for name, types in topic_names if name == '/scan']
            
            if scan_topics:
                self.get_logger().info("✅ /scan topic found, creating subscription...")
                scan_topic_found = True
            else:
                check_attempts += 1
                self.get_logger().info(f"⏳ /scan topic not found (attempt {check_attempts}/{max_check_attempts}), waiting...")
                time.sleep(1.0)
        
        if not scan_topic_found:
            self.get_logger().warn("⚠️ /scan topic not found, but proceeding with subscription anyway")
        
        # Create laser scan subscription with compatible QoS settings for real robot
        self.get_logger().info("📡 Setting up laser scan subscription with robust QoS...")
        
        # Use QoS settings that work well with real robot sensors
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        
        # Best effort QoS is often better for real sensor data
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Better for real sensors
            durability=DurabilityPolicy.VOLATILE        # Latest data only
        )
        
        # Subscriber to laser scan data
        try:
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, qos_profile)
            self.get_logger().info("✅ Subscription created with BEST_EFFORT QoS")
        except Exception as e:
            self.get_logger().warn(f"⚠️ BEST_EFFORT QoS failed: {e}, trying default...")
            # Fallback to default QoS
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
            self.get_logger().info("✅ Subscription created with default QoS")
        
        # ==================== SERVICE SERVER ====================
        
        # Service server for find_wall requests
        self.srv = self.create_service(FindWall, 'find_wall', self.handle_find_wall)
        
        # ==================== LASER DATA STORAGE ====================
        
        # Current laser scan data (filtered and processed)
        self.ranges = None
        self.num_ranges = None
        self.angle_min = None
        self.angle_increment = None
        
        # Callback tracking for debugging
        self._scan_callback_count = 0
        
        # ==================== ROBOT PARAMETERS ====================
        
        # Movement speeds
        self.rotation_speed = 0.3      # rad/s for normal rotation
        self.approach_speed = 0.02     # m/s for approaching wall
        self.target_distance = 0.3     # meters from wall
        
        # Timeout parameters
        self.rotation_timeout = 16.0   # seconds
        self.approach_timeout = 10.0   # seconds
        
        # Alignment tolerance for final positioning
        self.alignment_tolerance_deg = 15  # degrees
        
        self.get_logger().info("WallFinder service server ready (180° forward laser).")
        self.get_logger().info(f"Parameters: rotation_speed={self.rotation_speed}, "
                              f"target_distance={self.target_distance}m, "
                              f"tolerance={self.alignment_tolerance_deg}°")
        
        # Allow a brief moment for subscription to fully establish
        self.get_logger().info("🔄 Allowing subscription to establish...")
        time.sleep(2.0)  # Give subscription time to connect
        
        # Force some early spins to trigger callbacks
        self.get_logger().info("🔄 Testing subscription with early callback attempts...")
        for i in range(5):
            rclpy.spin_once(self, timeout_sec=0.5)
            if hasattr(self, '_scan_callback_count'):
                self.get_logger().info(f"✅ Early callback success! Received {self._scan_callback_count} callbacks")
                break
            time.sleep(0.2)
        
        if not hasattr(self, '_scan_callback_count'):
            self.get_logger().warn("⚠️ No early callbacks received, but continuing initialization")
        
        self.get_logger().info("✅ Node initialization complete, ready for service calls")

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming laser scan data for 180° forward-facing laser
        
        180-Degree Forward-Facing Laser Layout:
        - 0° (right): index 0
        - 90° (front): index 360  
        - 180° (left): index 719
        - Coverage: 180° forward hemisphere only
        
        Enhanced to extract all LaserScan message fields for advanced sensor awareness
        
        Args:
            msg (LaserScan): Incoming laser scan message with full field utilization
        """
        # Debug: Increment callback counter and log first few callbacks
        self._scan_callback_count += 1
        if self._scan_callback_count <= 3:
            self.get_logger().info(f"✅ Scan callback #{self._scan_callback_count} received successfully!")
        if not hasattr(self, '_scan_callback_count'):
            self._scan_callback_count = 0
            self.get_logger().info("🎯 FIRST LASER SCAN RECEIVED! 180° laser subscription working properly.")
        
        self._scan_callback_count += 1
        
        # Log periodic callback activity for debugging
        if self._scan_callback_count <= 5:
            self.get_logger().info(f"📡 Scan callback #{self._scan_callback_count} - processing {len(msg.ranges)} points (180° coverage)")
        elif self._scan_callback_count == 10:
            self.get_logger().info(f"📡 180° laser subscription stable - {self._scan_callback_count} callbacks processed")
        
        # ==================== CORE RANGE DATA PROCESSING ====================
        
        # Convert to numpy array for efficient processing
        ranges = np.array(msg.ranges)
        
        # Replace infinite and NaN values with large safe distance (10.0m)
        # This prevents errors in min/max calculations
        self.ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        # Store basic laser scan parameters for angle calculations
        self.num_ranges = len(self.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        
        # ==================== COMPREHENSIVE LASERSCAN MESSAGE EXTRACTION ====================
        
        # Header information (coordinate frame and timing)
        self.scan_frame_id = msg.header.frame_id
        self.scan_timestamp = msg.header.stamp
        
        # Angular configuration parameters
        self.angle_max = msg.angle_max
        self.angular_range = self.angle_max - self.angle_min
        self.scan_resolution_deg = math.degrees(self.angle_increment)
        
        # Range limitations and specifications
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        
        # Timing information for scan dynamics
        self.scan_time = msg.scan_time
        self.time_increment = msg.time_increment
        
        # Intensity data availability and processing
        self.has_intensity_data = len(msg.intensities) > 0
        if self.has_intensity_data:
            self.intensities = np.array(msg.intensities)
        else:
            self.intensities = None
        
        # ==================== ENHANCED LASER DATA METRICS ====================
        
        # Calculate comprehensive scan statistics
        self.total_scan_points = len(self.ranges)
        self.valid_scan_points = np.sum(np.isfinite(ranges))
        self.scan_coverage_percent = (self.valid_scan_points / self.total_scan_points) * 100
        
        # Range statistics for quality assessment
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) > 0:
            self.range_mean = np.mean(valid_ranges)
            self.range_std = np.std(valid_ranges)
            self.closest_valid_range = np.min(valid_ranges)
            self.farthest_valid_range = np.max(valid_ranges)
        else:
            self.range_mean = self.range_std = self.closest_valid_range = self.farthest_valid_range = 0.0
        
        # Calculate scan frequency and timing metrics
        if self.scan_time > 0:
            self.scan_frequency_hz = 1.0 / self.scan_time
        else:
            self.scan_frequency_hz = 0.0
        
        # ==================== COMPREHENSIVE LOGGING (FIRST SCAN ONLY) ====================
        
        # Log comprehensive laser configuration once for debugging
        if not hasattr(self, '_detailed_laser_info_logged'):
            self._detailed_laser_info_logged = True
            self.log_detailed_laser_info_finder()
        
        # Basic laser data logging (legacy compatibility)
        if hasattr(self, '_logged_laser_info'):
            return
        self._logged_laser_info = True
        self.get_logger().info(f"180° Laser data: {self.num_ranges} points, "
                              f"coverage: 0° to 180°, "
                              f"resolution: 0.25°/point")

    def get_standard_laser_indices_corrected(self):
        """
        Calculate standard laser indices for 180-degree forward-facing laser
        
        Based on your robot's actual laser configuration:
        - Coverage: 180° forward-facing only (right to left)
        - 0° (right): index 0
        - 90° (front): index 360
        - 180° (left): index 719
        - Back: Not available (no rear coverage)
        
        Returns:
            dict: Dictionary with corrected standard direction indices
        """
        return {
            'right': 0,        # 0° - right side (start of scan)
            'front': 360,      # 90° - forward direction
            'left': 719,       # 180° - left side (end of scan)
            'back': None       # No rear coverage on this laser
        }

    def average_range(self, center_idx, window=2):
        """
        Calculate average distance over a window of laser points for stability
        
        This reduces noise by averaging multiple adjacent laser readings
        around the target direction.
        
        Args:
            center_idx (int): Center index for averaging
            window (int): Number of points on each side to include (default: 2)
            
        Returns:
            float: Average distance, or inf if no laser data available
        """
        if self.ranges is None:
            return float('inf')
            
        # Create list of indices within the window (handle boundaries for 180° laser)
        idxs = []
        for i in range(-window, window + 1):
            idx = center_idx + i
            if 0 <= idx < self.num_ranges:  # Only include valid indices for 180° laser
                idxs.append(idx)
        
        # Calculate and return average distance
        if idxs:
            return float(np.mean([self.ranges[i] for i in idxs]))
        else:
            return float('inf')

    def rotate_until_condition(self, condition_fn, direction, timeout_sec=16.0):
        """
        Rotate the robot until a specific condition is met or timeout occurs
        
        Args:
            condition_fn (callable): Function that returns True when condition is met
            direction (float): Angular velocity (positive = counterclockwise)
            timeout_sec (float): Maximum time to rotate before giving up
        """
        twist = Twist()
        start_time = time.time()
        
        self.get_logger().info(f"Starting rotation (direction={direction:.2f} rad/s)")
        
        # Continue rotating until condition is met or timeout
        while rclpy.ok() and not condition_fn():
            # Check for timeout
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"Rotation timeout reached ({timeout_sec}s), moving to next step.")
                break
                
            # Send rotation command
            twist.angular.z = direction
            self.cmd_pub.publish(twist)
            
            # Process incoming messages briefly
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("Rotation completed")

    def move_until_close(self, target_dist=0.3, timeout_sec=10.0):
        """
        Move forward or backward until the robot is at target distance from front obstacle
        
        This method handles both cases:
        - If robot is too far (> target_dist): move forward
        - If robot is too close (< target_dist): move backward
        
        Args:
            target_dist (float): Desired distance from wall in meters
            timeout_sec (float): Maximum time to move before giving up
        """
        twist = Twist()
        start_time = time.time()
        # Get front index for 180° laser (index 360)
        front_idx = 360
        tolerance = 0.05  # 5cm tolerance for target distance
        
        # Check initial distance to determine movement direction
        initial_dist = self.average_range(front_idx)
        
        if abs(initial_dist - target_dist) <= tolerance:
            self.get_logger().info(f"Already at target distance ({initial_dist:.2f}m), no movement needed")
            return
        
        # Determine movement direction and log intention
        if initial_dist > target_dist:
            self.get_logger().info(f"Too far from wall ({initial_dist:.2f}m) - moving forward to reach {target_dist}m")
            move_forward = True
        else:
            self.get_logger().info(f"Too close to wall ({initial_dist:.2f}m) - moving backward to reach {target_dist}m")
            move_forward = False
        
        # Continue moving until target distance reached or timeout
        while rclpy.ok():
            current_dist = self.average_range(front_idx)
            
            # Check if we've reached the target distance (within tolerance)
            if abs(current_dist - target_dist) <= tolerance:
                self.get_logger().info(f"Target distance reached: {current_dist:.2f}m")
                break
            
            # Check for timeout
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(f"Move timeout reached ({timeout_sec}s).")
                break
            
            # Send movement command based on direction needed
            if move_forward and current_dist > target_dist:
                # Still too far - continue moving forward
                twist.linear.x = self.approach_speed
            elif not move_forward and current_dist < target_dist:
                # Still too close - continue moving backward
                twist.linear.x = -self.approach_speed
            else:
                # We've crossed the target - fine adjustment needed
                distance_error = current_dist - target_dist
                if abs(distance_error) <= tolerance:
                    break
                # Fine adjustment with slower speed
                twist.linear.x = -0.5 * self.approach_speed * (distance_error / abs(distance_error))
            
            self.cmd_pub.publish(twist)
            
            # Process incoming messages briefly
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Log progress every 2 seconds
            if int(time.time() - start_time) % 2 == 0:
                direction_str = "forward" if twist.linear.x > 0 else "backward"
                self.get_logger().info(f"Moving {direction_str}, current distance: {current_dist:.2f}m")
        
        # Stop movement
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        
        final_dist = self.average_range(front_idx)
        self.get_logger().info(f"Position adjustment completed, final distance: {final_dist:.2f}m")

    def handle_find_wall(self, request, response):
        """
        Handle incoming find_wall service requests for 180° forward-facing laser
        
        Executes the 3-step wall finding sequence:
        1. Rotate until facing the nearest wall (front direction has minimum distance)
        2. Move forward until 0.3m from the wall
        3. Rotate until the wall is positioned on the right side
        
        Args:
            request: Service request (empty for FindWall)
            response: Service response with wallfound boolean
            
        Returns:
            FindWall.Response: Response indicating success/failure
        """
        self.get_logger().info("=== STARTING WALL-FINDING SEQUENCE (180° Forward Laser) ===")
        
        # Check if laser data is available, wait if necessary
        if self.ranges is None:
            self.get_logger().info("⏳ Waiting for 180° laser data to become available...")
            self.get_logger().info("📡 Topic /scan is publishing, but subscription callback hasn't been triggered yet")
            self.get_logger().info("🔄 Forcing subscription callback processing...")
            
            # First, let's check if the topic is still available
            topic_names = self.get_topic_names_and_types()
            scan_topics = [name for name, types in topic_names if name == '/scan']
            
            if scan_topics:
                self.get_logger().info("✅ /scan topic is available in ROS graph")
            else:
                self.get_logger().error("❌ /scan topic not found in ROS graph!")
                self.get_logger().error("   Available topics:")
                for name, types in topic_names[:10]:  # Show first 10 topics
                    self.get_logger().error(f"     {name}")
                response.wallfound = False
                return response
            
            max_wait_time = 30.0  # Wait time for callback to trigger
            start_wait = time.time()
            callback_attempts = 0
            
            while self.ranges is None and (time.time() - start_wait) < max_wait_time:
                callback_attempts += 1
                
                # Very aggressive spinning with shorter timeouts to catch the callback
                for spin_cycle in range(10):  # Increased cycles
                    rclpy.spin_once(self, timeout_sec=0.1)  # Shorter timeout
                    if self.ranges is not None:
                        break
                    time.sleep(0.05)  # Very short sleep
                
                # Log progress every 3 seconds with more detail
                elapsed = time.time() - start_wait
                if int(elapsed) % 3 == 0 and elapsed > 2:
                    self.get_logger().info(f"   Attempt {callback_attempts}: Still waiting for callback... ({elapsed:.1f}s elapsed)")
                    
                    # Check if we've received any callback at all
                    if hasattr(self, '_scan_callback_count'):
                        self.get_logger().info(f"   Callbacks received so far: {self._scan_callback_count}")
                    else:
                        self.get_logger().info("   No scan callbacks received yet - checking subscription health")
                        # Check subscription status
                        if self.scan_sub:
                            self.get_logger().info(f"   Subscription object exists: {type(self.scan_sub)}")
                        else:
                            self.get_logger().error("   Subscription object is None!")
                
                time.sleep(0.2)  # Shorter sleep between attempts
            
            if self.ranges is None:
                self.get_logger().error("❌ 180° laser subscription callback never triggered after 30 seconds")
                if hasattr(self, '_scan_callback_count'):
                    self.get_logger().error(f"   Total callbacks received: {self._scan_callback_count}")
                else:
                    self.get_logger().error("   No scan callbacks were ever received")
                self.get_logger().error("   This indicates a subscription timing issue on real robot")
                self.get_logger().error("   Try restarting the nodes or checking topic remapping")
                response.wallfound = False
                return response
            else:
                self.get_logger().info("✅ 180° laser subscription callback triggered successfully!")
                if hasattr(self, '_scan_callback_count'):
                    self.get_logger().info(f"   Total callbacks processed: {self._scan_callback_count}")
        
        # Ensure we have fresh laser data
        self.get_logger().info("📡 Ensuring fresh 180° laser data...")
        for _ in range(5):  # Get more fresh samples
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.1)
        
        # Get standard laser indices for 180° laser
        indices = self.get_standard_laser_indices_corrected()
        front_idx = indices['front']  # 360
        right_idx = indices['right']  # 0
        
        self.get_logger().info(f"Using 180° laser indices - Front: {front_idx}, Right: {right_idx}")
        
        # ==================== STEP 1: FACE NEAREST WALL ====================
        self.get_logger().info("Step 1: Rotating to face nearest wall (180° scan analysis)...")
        
        # First: Stop robot completely for stable analysis
        self.get_logger().info("🛑 Stopping robot for comprehensive 180° laser analysis...")
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        
        # Give time for stable laser readings and collect multiple samples
        time.sleep(1.0)  # Longer wait for very stable readings
        for _ in range(3):  # Get multiple fresh samples
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Perform comprehensive 180-degree laser analysis
        analysis_result = self.analyze_180_degree_laser_data()
        
        if analysis_result is None:
            self.get_logger().error("❌ Failed to analyze 180° laser data - aborting wall finding")
            response.wallfound = False
            return response
        
        # Check if robot is already aligned with the nearest wall
        if analysis_result['target_achieved']:
            self.get_logger().info("✅ Robot is already facing the nearest wall - proceeding to Step 2")
        else:
            # Determine rotation parameters from analysis
            target_index = analysis_result['shortest_index']
            rotation_direction_sign = analysis_result['rotation_direction']
            angular_difference = abs(analysis_result['angular_difference'])
            
            # Set rotation speed and direction
            rotation_speed = rotation_direction_sign * self.rotation_speed
            
            self.get_logger().info(f"🎯 EXECUTING ROTATION (180° laser):")
            self.get_logger().info(f"   Target: Index {target_index} ({analysis_result['shortest_angle']:.1f}°)")
            self.get_logger().info(f"   Distance to target: {analysis_result['shortest_averaged']:.3f}m")
            self.get_logger().info(f"   Rotation needed: {angular_difference} indices")
            direction_name = "counterclockwise" if rotation_direction_sign > 0 else "clockwise"
            self.get_logger().info(f"   Direction: {direction_name} at {abs(rotation_speed):.2f} rad/s")
            
            # Define improved condition function that uses our analysis
            def front_is_minimum():
                """Check if front direction has the minimum distance (using 180° analysis)"""
                # Get fresh analysis
                current_analysis = self.analyze_180_degree_laser_data()
                if current_analysis is None:
                    return False
                
                is_aligned = current_analysis['target_achieved']
                
                if not is_aligned:
                    self.get_logger().info(f"🔄 Still rotating... Current shortest at index {current_analysis['shortest_index']}, "
                                          f"target front index {front_idx}")
                else:
                    self.get_logger().info("🎯 TARGET ACHIEVED! Front is now aligned with nearest wall")
                
                return is_aligned
            
            # Execute rotation with intelligent monitoring
            self.get_logger().info("🔄 Starting intelligent rotation...")
            self.rotate_until_condition(front_is_minimum, rotation_speed, self.rotation_timeout)
            
            # Final verification after rotation
            final_analysis = self.analyze_180_degree_laser_data()
            if final_analysis and final_analysis['target_achieved']:
                self.get_logger().info("✅ Rotation completed successfully - robot is now facing nearest wall")
            else:
                self.get_logger().warn("⚠️ Rotation completed but alignment may not be perfect")
        
        # ==================== STEP 2: APPROACH WALL ====================
        self.get_logger().info("Step 2: Moving forward to approach wall...")
        
        # Move forward until target distance from wall
        self.move_until_close(self.target_distance, self.approach_timeout)
        
        # ==================== STEP 3: ALIGN WALL ON RIGHT SIDE ====================
        self.get_logger().info("Step 3: Rotating to position wall on right side...")
        
        # Calculate tolerance in terms of laser indices
        tolerance_idx = int(self.num_ranges * self.alignment_tolerance_deg / 180.0)  # 180° not 360°
        
        def right_side_aligned():
            """Check if wall is positioned on right side within tolerance"""
            min_idx = np.argmin(self.ranges)
            distance_from_right = abs(min_idx - right_idx)
            
            is_aligned = distance_from_right <= tolerance_idx
            if not is_aligned:
                # Use averaged reading for consistent scale
                min_dist = self.average_range(min_idx)
                self.get_logger().info(f"Min distance at index {min_idx} ({min_dist:.2f}m), "
                                      f"right index: {right_idx}, distance: {distance_from_right}")
            return is_aligned
        
        # Rotate clockwise until wall is on right side (since right is at index 0)
        self.rotate_until_condition(right_side_aligned, -self.rotation_speed, self.rotation_timeout)
        
        # ==================== COMPLETION ====================
        
        # Final status check
        final_front_dist = self.average_range(front_idx)
        final_right_dist = self.average_range(right_idx)
        
        self.get_logger().info("=== WALL-FINDING SEQUENCE COMPLETED (180° Laser) ===")
        self.get_logger().info(f"Final distances - Front: {final_front_dist:.2f}m, Right: {final_right_dist:.2f}m")
        self.get_logger().info("Robot is positioned and ready for wall-following behavior.")
        
        # Set success response
        response.wallfound = True
        return response
        
    def analyze_180_degree_laser_data(self):
        """
        Comprehensive 180-degree forward-facing laser analysis
        
        This method examines the forward hemisphere laser rays, finds the shortest distances,
        and provides detailed analysis for decision making with 180° coverage limitation.
        
        Returns:
            dict: Comprehensive analysis results with target information and scan quality
        """
        if self.ranges is None:
            return None
        
        self.get_logger().info("🔍 CONDUCTING 180-DEGREE FORWARD LASER ANALYSIS...")
        
        # ==================== ENHANCED RANGE ANALYSIS ====================
        
        # Convert to numpy array for efficient analysis
        ranges_array = np.array(self.ranges)
        
        # Find all valid (non-infinite) readings
        valid_mask = np.isfinite(ranges_array) & (ranges_array > 0.0)
        valid_ranges = ranges_array[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        
        if len(valid_ranges) == 0:
            self.get_logger().error("No valid laser readings found!")
            return None
        
        # Find the absolute minimum distance
        min_distance = np.min(valid_ranges)
        min_global_idx = valid_indices[np.argmin(valid_ranges)]
        
        # Find top 5 shortest distances for analysis
        sorted_indices = np.argsort(valid_ranges)
        top_5_shortest = []
        
        for i in range(min(5, len(sorted_indices))):
            idx = valid_indices[sorted_indices[i]]
            distance = valid_ranges[sorted_indices[i]]
            # Calculate corresponding angle: index * 0.25° for 180° laser
            angle_deg = idx * 0.25
            averaged_dist = self.average_range(idx)
            
            top_5_shortest.append({
                'index': idx,
                'distance': distance,
                'angle': angle_deg,
                'averaged_distance': averaged_dist
            })
        
        # ==================== DIRECTIONAL ANALYSIS ====================
        
        # Analyze key directional distances with 180° laser limitation
        key_directions = self.get_standard_laser_indices_corrected()
        directional_analysis = {}
        
        for direction, idx in key_directions.items():
            if idx is None:  # Handle 'back' direction which doesn't exist
                directional_analysis[direction] = {
                    'index': None,
                    'distance': float('inf'),
                    'averaged_distance': float('inf'),
                    'angle': None,
                    'available': False,
                    'note': 'No rear coverage on 180° laser'
                }
                continue
                
            # Basic distance measurements
            raw_distance = self.ranges[idx]
            averaged_distance = self.average_range(idx)
            angle = idx * 0.25  # 0.25° per index
            
            direction_data = {
                'index': idx,
                'distance': raw_distance,
                'averaged_distance': averaged_distance,
                'angle': angle,
                'available': True,
                'is_valid': np.isfinite(raw_distance) and raw_distance > 0.0
            }
            
            directional_analysis[direction] = direction_data
        
        # ==================== ROTATION CALCULATION FOR 180° LASER ====================
        
        # Calculate target rotation needed (front direction is index 360)
        front_idx = 360
        angular_diff = min_global_idx - front_idx
        
        # Calculate precise angular difference in degrees (for 180° laser)
        angular_diff_degrees = angular_diff * 0.25  # 0.25° per index
        
        # Determine rotation direction and magnitude
        if angular_diff == 0:
            rotation_needed = "NONE - Already aligned"
            rotation_direction = 0
        elif angular_diff > 0:
            rotation_needed = f"COUNTERCLOCKWISE by {angular_diff} indices ({angular_diff_degrees:.1f}°)"
            rotation_direction = 1
        else:
            rotation_needed = f"CLOCKWISE by {abs(angular_diff)} indices ({abs(angular_diff_degrees):.1f}°)"
            rotation_direction = -1
        
        # ==================== ANALYSIS COMPILATION ====================
        
        analysis_result = {
            'laser_type': '180_degree_forward',
            'total_laser_points': self.num_ranges,
            'valid_readings': len(valid_ranges),
            'shortest_distance': min_distance,
            'shortest_index': min_global_idx,
            'shortest_angle': min_global_idx * 0.25,
            'shortest_averaged': self.average_range(min_global_idx),
            'top_5_shortest': top_5_shortest,
            'directional_analysis': directional_analysis,
            'angular_difference': angular_diff,
            'angular_difference_degrees': angular_diff_degrees,
            'rotation_needed': rotation_needed,
            'rotation_direction': rotation_direction,
            'target_achieved': (angular_diff == 0),
        }
        
        # Print analysis results
        self.print_180_laser_analysis(analysis_result)
        
        return analysis_result

    def print_180_laser_analysis(self, analysis):
        """
        Print detailed 180° laser analysis results
        
        Args:
            analysis (dict): Analysis results from analyze_180_degree_laser_data
        """
        self.get_logger().info("=" * 65)
        self.get_logger().info("📊 180-DEGREE FORWARD LASER ANALYSIS RESULTS")
        self.get_logger().info("=" * 65)
        
        # Basic statistics
        self.get_logger().info(f"📏 Laser Coverage: 180° forward hemisphere")
        self.get_logger().info(f"   Total points: {analysis['total_laser_points']}")
        self.get_logger().info(f"   Valid readings: {analysis['valid_readings']}")
        self.get_logger().info("")
        
        # Shortest distance detected
        self.get_logger().info(f"🎯 SHORTEST DISTANCE DETECTED:")
        self.get_logger().info(f"   Index: {analysis['shortest_index']}")
        self.get_logger().info(f"   Raw distance: {analysis['shortest_distance']:.3f}m")
        self.get_logger().info(f"   Averaged distance: {analysis['shortest_averaged']:.3f}m")
        self.get_logger().info(f"   Angle: {analysis['shortest_angle']:.1f}°")
        self.get_logger().info("")
        
        # Top 5 shortest distances
        self.get_logger().info(f"🏆 TOP 5 SHORTEST DISTANCES:")
        for i, data in enumerate(analysis['top_5_shortest'], 1):
            self.get_logger().info(f"   #{i}: Index {data['index']} → "
                                  f"{data['distance']:.3f}m (avg: {data['averaged_distance']:.3f}m) "
                                  f"at {data['angle']:.1f}°")
        self.get_logger().info("")
        
        # Directional analysis
        self.get_logger().info(f"🧭 DIRECTIONAL ANALYSIS (180° Forward):")
        for direction, data in analysis['directional_analysis'].items():
            if not data.get('available', True):
                self.get_logger().info(f"   {direction.upper():>5} ❌: {data.get('note', 'Not available')}")
                continue
                
            validity_str = "✅" if data['is_valid'] else "❌"
            self.get_logger().info(f"   {direction.upper():>5} {validity_str}: Index {data['index']} → "
                                  f"{data['distance']:.3f}m (avg: {data['averaged_distance']:.3f}m) "
                                  f"at {data['angle']:.1f}°")
        self.get_logger().info("")
        
        # Rotation decision
        self.get_logger().info(f"🔄 ROTATION DECISION:")
        self.get_logger().info(f"   Current front index: 360")
        self.get_logger().info(f"   Target index: {analysis['shortest_index']}")
        self.get_logger().info(f"   Angular difference: {analysis['angular_difference']} indices ({analysis['angular_difference_degrees']:.1f}°)")
        self.get_logger().info(f"   Action needed: {analysis['rotation_needed']}")
        
        if analysis['target_achieved']:
            self.get_logger().info("   ✅ ROBOT IS ALREADY ALIGNED WITH NEAREST WALL!")
        else:
            direction_arrow = "↶" if analysis['rotation_direction'] > 0 else "↷"
            self.get_logger().info(f"   🎯 WILL ROTATE {direction_arrow} TO ALIGN WITH TARGET")
        
        self.get_logger().info("=" * 65)

    def log_detailed_laser_info_finder(self):
        """
        Log comprehensive 180° laser scanner information
        """
        self.get_logger().info("🔍 WALL FINDER - 180° FORWARD LASER SCANNER INFORMATION")
        self.get_logger().info("=" * 60)
        
        # Header information
        self.get_logger().info(f"📡 Scan Header:")
        self.get_logger().info(f"   Frame ID: {self.scan_frame_id}")
        if self.scan_timestamp:
            timestamp_sec = self.scan_timestamp.sec + self.scan_timestamp.nanosec / 1e9
            self.get_logger().info(f"   Timestamp: {timestamp_sec:.3f}s")
        
        # Angular parameters for 180° laser
        self.get_logger().info(f"📐 Angular Configuration (180° Forward):")
        self.get_logger().info(f"   Coverage: 180° forward hemisphere only")
        self.get_logger().info(f"   Start Angle: 0° (right side)")
        self.get_logger().info(f"   End Angle: 180° (left side)")
        self.get_logger().info(f"   Angular Resolution: 0.25°/point (180°/720)")
        
        # Range parameters
        self.get_logger().info(f"📏 Range Configuration:")
        self.get_logger().info(f"   Range Min: {self.range_min:.2f}m")
        self.get_logger().info(f"   Range Max: {self.range_max:.2f}m")
        
        # Data quality
        self.get_logger().info(f"📊 Data Quality:")
        self.get_logger().info(f"   Total Scan Points: {self.total_scan_points}")
        self.get_logger().info(f"   Valid Scan Points: {self.valid_scan_points}")
        self.get_logger().info(f"   Scan Coverage: {self.scan_coverage_percent:.1f}%")
        
        # Standard direction mapping for 180° laser
        self.get_logger().info(f"🎯 180° Laser Direction Mapping:")
        indices = self.get_standard_laser_indices_corrected()
        for direction, idx in indices.items():
            if idx is None:
                self.get_logger().info(f"   {direction.capitalize():>5}: Not available (no rear coverage)")
            else:
                angle = idx * 0.25  # 0.25° per index
                current_distance = self.average_range(idx) if hasattr(self, 'ranges') else 0.0
                self.get_logger().info(f"   {direction.capitalize():>5}: Index {idx:>4} → {angle:>6.1f}° ({current_distance:.2f}m)")
        
        self.get_logger().info("=" * 60)


def main(args=None):
    """
    Main entry point for the wall finder node
    """
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create and run the wall finder node
    node = WallFinder()
    
    try:
        # Spin the node to handle service requests
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
