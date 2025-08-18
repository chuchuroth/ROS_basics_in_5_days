#!/usr/bin/env python3

"""
Wall Finder Service Node - 180° Forward Laser Simplified Version
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

SECTOR-BASED ANALYSIS METHOD:
This implementation uses the sector-based laser analysis method for improved
wall detection, dividing the 180° laser scan into meaningful sectors.

IMPORTANT: This node ONLY handles wall finding. Obstacle detection and avoidance
during wall following is handled by the separate wall_following node.

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
        self.get_logger().info(" Checking for /scan topic availability...")
        scan_topic_found = False
        check_attempts = 0
        max_check_attempts = 10
        
        while not scan_topic_found and check_attempts < max_check_attempts:
            topic_names = self.get_topic_names_and_types()
            scan_topics = [name for name, types in topic_names if name == '/scan']
            
            if scan_topics:
                self.get_logger().info(" /scan topic found, creating subscription...")
                scan_topic_found = True
            else:
                check_attempts += 1
                self.get_logger().info(f" /scan topic not found (attempt {check_attempts}/{max_check_attempts}), waiting...")
                time.sleep(1.0)
        
        if not scan_topic_found:
            self.get_logger().warn(" /scan topic not found, but proceeding with subscription anyway")
        
        # Create laser scan subscription with compatible QoS settings for real robot
        self.get_logger().info(" Setting up laser scan subscription with robust QoS...")
        
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
            self.get_logger().info(" Subscription created with BEST_EFFORT QoS")
        except Exception as e:
            self.get_logger().warn(f" BEST_EFFORT QoS failed: {e}, trying default...")
            # Fallback to default QoS
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
            self.get_logger().info(" Subscription created with default QoS")
        
        # ==================== SERVICE SERVER ====================
        
        # Service server for find_wall requests
        self.srv = self.create_service(FindWall, '/find_wall', self.handle_find_wall)
        
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
        self.rotation_speed = 0.3      # rad/s for rotation
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
        self.get_logger().info(" Allowing subscription to establish...")
        time.sleep(2.0)  # Give subscription time to connect
        
        # Force some early spins to trigger callbacks
        self.get_logger().info(" Testing subscription with early callback attempts...")
        for i in range(5):
            rclpy.spin_once(self, timeout_sec=0.5)
            if hasattr(self, '_scan_callback_count'):
                self.get_logger().info(f" Early callback success! Received {self._scan_callback_count} callbacks")
                break
            time.sleep(0.2)
        
        if not hasattr(self, '_scan_callback_count'):
            self.get_logger().warn(" No early callbacks received, but continuing initialization")
        
        self.get_logger().info(" Node initialization complete, ready for service calls")

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming laser scan data for 180° forward-facing laser
        
        Simple processing for wall finding:
        - Store basic laser parameters
        - Filter out invalid readings
        - Store ranges for sector-based analysis
        
        Args:
            msg (LaserScan): Incoming laser scan message
        """
        # Debug: Track callback count
        self._scan_callback_count += 1
        if self._scan_callback_count <= 3:
            self.get_logger().info(f" Scan callback #{self._scan_callback_count} received")
        
        # Convert to numpy array and filter invalid readings
        ranges = np.array(msg.ranges)
        self.ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        # Store basic laser scan parameters
        self.num_ranges = len(self.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        
        # Store comprehensive laser parameters for sector analysis
        self.angle_max = msg.angle_max
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        
        # Log laser info once
        if not hasattr(self, '_laser_logged'):
            self._laser_logged = True
            self.get_logger().info(f"180° Laser: {self.num_ranges} points, 0°-180° coverage")

    def get_standard_laser_indices_corrected(self):
        """
        Get standard laser indices for 180-degree forward-facing laser
        
        Returns:
            dict: Dictionary with standard direction indices
        """
        return {
            'right': 0,        # 0° - right side
            'front': 360,      # 90° - forward direction
            'left': 719,       # 180° - left side
            'back': None       # No rear coverage
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
            self.get_logger().info(" Waiting for 180° laser data to become available...")
            self.get_logger().info(" Topic /scan is publishing, but subscription callback hasn't been triggered yet")
            self.get_logger().info(" Forcing subscription callback processing...")
            
            # First, let's check if the topic is still available
            topic_names = self.get_topic_names_and_types()
            scan_topics = [name for name, types in topic_names if name == '/scan']
            
            if scan_topics:
                self.get_logger().info(" /scan topic is available in ROS graph")
            else:
                self.get_logger().error(" /scan topic not found in ROS graph!")
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
                self.get_logger().error(" 180° laser subscription callback never triggered after 30 seconds")
                if hasattr(self, '_scan_callback_count'):
                    self.get_logger().error(f"   Total callbacks received: {self._scan_callback_count}")
                else:
                    self.get_logger().error("   No scan callbacks were ever received")
                self.get_logger().error("   This indicates a subscription timing issue on real robot")
                self.get_logger().error("   Try restarting the nodes or checking topic remapping")
                response.wallfound = False
                return response
            else:
                self.get_logger().info(" 180° laser subscription callback triggered successfully!")
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
        self.get_logger().info(" Stopping robot for comprehensive 180° laser analysis...")
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        
        # Give time for stable laser readings and collect multiple samples
        time.sleep(1.0)  # Longer wait for very stable readings
        for _ in range(3):  # Get multiple fresh samples
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Perform comprehensive 180-degree sector-based laser analysis
        analysis_result = self.analyze_180_degree_laser_data_sector_based()
        
        if analysis_result is None:
            self.get_logger().error(" Failed to analyze 180° laser data - aborting wall finding")
            response.wallfound = False
            return response
        
        # Check if robot is already aligned with the nearest wall
        if analysis_result['target_achieved']:
            self.get_logger().info(" Robot is already facing the nearest wall - proceeding to Step 2")
        else:
            # Determine rotation parameters from sector-based analysis
            if analysis_result['closest_sector_stats'] and analysis_result['closest_sector_stats']['min_index'] is not None:
                target_index = analysis_result['closest_sector_stats']['min_index']
                target_angle = analysis_result['closest_sector_stats']['min_angle']
                closest_distance = analysis_result['closest_distance']
                closest_sector = analysis_result['closest_sector']
            else:
                self.get_logger().error(" Invalid sector analysis results - cannot determine target")
                response.wallfound = False
                return response
                
            rotation_direction_sign = analysis_result['rotation_direction']
            angular_difference = abs(analysis_result['angular_difference'])
            
            # Set rotation speed and direction
            rotation_speed = rotation_direction_sign * self.rotation_speed
            
            self.get_logger().info(f" EXECUTING ROTATION (Sector-Based 180° Analysis):")
            self.get_logger().info(f"   Target: Index {target_index} ({target_angle:.1f}°)")
            self.get_logger().info(f"   Closest Sector: {closest_sector}")
            self.get_logger().info(f"   Distance to target: {closest_distance:.3f}m")
            self.get_logger().info(f"   Rotation needed: {angular_difference} indices")
            direction_name = "counterclockwise" if rotation_direction_sign > 0 else "clockwise"
            self.get_logger().info(f"   Direction: {direction_name} at {abs(rotation_speed):.2f} rad/s")
            
            # Define improved condition function that uses sector-based analysis
            def front_is_minimum():
                """Check if front direction has the minimum distance (using sector-based analysis)"""
                # Get fresh sector-based analysis
                current_analysis = self.analyze_180_degree_laser_data_sector_based()
                if current_analysis is None:
                    return False
                
                is_aligned = current_analysis['target_achieved']
                
                if not is_aligned:
                    closest_sector = current_analysis['closest_sector']
                    closest_distance = current_analysis['closest_distance']
                    self.get_logger().info(f" Still rotating... Closest obstacle in {closest_sector} sector "
                                          f"at {closest_distance:.3f}m, target: front alignment")
                else:
                    self.get_logger().info(" TARGET ACHIEVED! Front is now aligned with nearest wall")
                
                return is_aligned
            
            # Execute rotation with intelligent monitoring
            self.get_logger().info(" Starting intelligent rotation...")
            self.rotate_until_condition(front_is_minimum, rotation_speed, self.rotation_timeout)
            
            # Final verification after rotation with sector-based analysis
            final_analysis = self.analyze_180_degree_laser_data_sector_based()
            if final_analysis and final_analysis['target_achieved']:
                self.get_logger().info(" Rotation completed successfully - robot is now facing nearest wall")
                self.get_logger().info(f" Final closest sector: {final_analysis['closest_sector']} "
                                      f"({final_analysis['closest_distance']:.3f}m)")
            else:
                self.get_logger().warn(" Rotation completed but alignment may not be perfect")
                if final_analysis:
                    self.get_logger().warn(f" Current closest sector: {final_analysis['closest_sector']} "
                                          f"({final_analysis['closest_distance']:.3f}m)")
        
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
        
    def analyze_180_degree_laser_data_sector_based(self):
        """
        Sector-based 180-degree forward-facing laser analysis
        
        This method uses the sector-based approach from the example file, dividing the 
        180° laser scan into meaningful sectors for obstacle detection and wall finding.
        
        Sectors for 180° laser (adapted from example):
        - Right: indices 0-19 (0°-4.75°)
        - Front_Right: indices 20-60 (5°-15°) 
        - Front_Center: indices 340-380 (85°-95°)
        - Front_Left: indices 100-140 (25°-35°)
        - Left: indices 141-179 (35.25°-44.75°)
        - Far_Left: indices 580-719 (145°-179.75°)
        
        Returns:
            dict: Sector-based analysis results with minimum distances per sector
        """
        if self.ranges is None:
            return None
        
        self.get_logger().info("🔍 CONDUCTING SECTOR-BASED 180° LASER ANALYSIS...")
        
        # ==================== SECTOR DEFINITIONS ====================
        # Define sectors based on the example method, adapted for 180° laser
        sectors = {
            "Right": (0, 19),           # 0°-4.75° - far right side
            "Front_Right": (20, 60),    # 5°-15° - front-right quadrant
            "Right_Mid": (61, 99),      # 15.25°-24.75° - right-middle
            "Front_Left": (100, 140),   # 25°-35° - front-left quadrant  
            "Left_Near": (141, 179),    # 35.25°-44.75° - near left side
            "Left_Mid": (180, 259),     # 45°-64.75° - left-middle
            "Left_Far": (260, 339),     # 65°-84.75° - far left side
            "Front_Center": (340, 380), # 85°-95° - center front (most important)
            "Front_Wide": (320, 400),   # 80°-100° - wide front area
            "Left_Forward": (401, 480), # 100.25°-120° - left-forward
            "Left_Side": (481, 560),    # 120.25°-140° - left side
            "Far_Left": (561, 719)      # 140.25°-179.75° - far left
        }
        
        # ==================== SECTOR ANALYSIS ====================
        ranges_array = np.array(self.ranges)
        
        # Calculate minimum distance for each sector
        min_distances = {}
        sector_stats = {}
        
        for sector_name, (start_idx, end_idx) in sectors.items():
            # Ensure indices are within valid range
            start_idx = max(0, start_idx)
            end_idx = min(len(self.ranges) - 1, end_idx)
            
            if start_idx <= end_idx and start_idx < len(self.ranges):
                # Extract sector data
                sector_ranges = ranges_array[start_idx:end_idx + 1]
                
                # Filter out invalid readings
                valid_sector_ranges = sector_ranges[np.isfinite(sector_ranges) & (sector_ranges > 0.0)]
                
                if len(valid_sector_ranges) > 0:
                    min_distance = np.min(valid_sector_ranges)
                    min_idx_in_sector = np.argmin(sector_ranges) + start_idx
                    avg_distance = np.mean(valid_sector_ranges)
                    
                    min_distances[sector_name] = min_distance
                    sector_stats[sector_name] = {
                        'min_distance': min_distance,
                        'avg_distance': avg_distance,
                        'min_index': min_idx_in_sector,
                        'min_angle': min_idx_in_sector * 0.25,  # 0.25° per index
                        'valid_readings': len(valid_sector_ranges),
                        'total_readings': len(sector_ranges),
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'coverage_angle_start': start_idx * 0.25,
                        'coverage_angle_end': end_idx * 0.25
                    }
                else:
                    min_distances[sector_name] = float('inf')
                    sector_stats[sector_name] = {
                        'min_distance': float('inf'),
                        'avg_distance': float('inf'),
                        'min_index': None,
                        'min_angle': None,
                        'valid_readings': 0,
                        'total_readings': len(sector_ranges),
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'coverage_angle_start': start_idx * 0.25,
                        'coverage_angle_end': end_idx * 0.25
                    }
            else:
                min_distances[sector_name] = float('inf')
                sector_stats[sector_name] = None
        
        # ==================== FIND GLOBAL MINIMUM ====================
        # Find the sector with absolute minimum distance
        valid_sectors = {k: v for k, v in min_distances.items() if v != float('inf')}
        
        if valid_sectors:
            closest_sector = min(valid_sectors, key=valid_sectors.get)
            closest_distance = valid_sectors[closest_sector]
            closest_sector_stats = sector_stats[closest_sector]
        else:
            self.get_logger().error("No valid sectors found!")
            return None
        
        # ==================== ROTATION DECISION ====================
        # Calculate rotation needed to align front with closest obstacle
        front_center_idx = 360  # Front center index for 180° laser
        
        if closest_sector_stats and closest_sector_stats['min_index'] is not None:
            target_idx = closest_sector_stats['min_index']
            angular_diff = target_idx - front_center_idx
            angular_diff_degrees = angular_diff * 0.25
            
            # Determine rotation direction
            if angular_diff == 0:
                rotation_needed = "NONE - Already aligned"
                rotation_direction = 0
            elif angular_diff > 0:
                rotation_needed = f"COUNTERCLOCKWISE by {angular_diff} indices ({angular_diff_degrees:.1f}°)"
                rotation_direction = 1
            else:
                rotation_needed = f"CLOCKWISE by {abs(angular_diff)} indices ({abs(angular_diff_degrees):.1f}°)"
                rotation_direction = -1
        else:
            rotation_needed = "UNKNOWN - No valid target"
            rotation_direction = 0
            angular_diff = 0
            angular_diff_degrees = 0.0
        
        # ==================== ANALYSIS COMPILATION ====================
        analysis_result = {
            'analysis_method': 'sector_based_180_degree',
            'sectors': sectors,
            'sector_stats': sector_stats,
            'min_distances': min_distances,
            'closest_sector': closest_sector,
            'closest_distance': closest_distance,
            'closest_sector_stats': closest_sector_stats,
            'angular_difference': angular_diff,
            'angular_difference_degrees': angular_diff_degrees,
            'rotation_needed': rotation_needed,
            'rotation_direction': rotation_direction,
            'target_achieved': (angular_diff == 0),
            'total_laser_points': len(self.ranges),
            'valid_sectors': len(valid_sectors)
        }
        
        # Print analysis results
        self.print_sector_based_analysis(analysis_result)
        
        return analysis_result

    def analyze_180_degree_laser_data(self):
        """
        Wrapper method that calls the new sector-based analysis
        
        Maintained for compatibility with existing code while using the improved
        sector-based analysis method from the example file.
        
        Returns:
            dict: Analysis results using sector-based method
        """
        return self.analyze_180_degree_laser_data_sector_based()

    def print_sector_based_analysis(self, analysis):
        """
        Print detailed sector-based 180° laser analysis results
        
        Args:
            analysis (dict): Analysis results from analyze_180_degree_laser_data_sector_based
        """
        self.get_logger().info("=" * 70)
        self.get_logger().info(" SECTOR-BASED 180° LASER ANALYSIS RESULTS")
        self.get_logger().info("=" * 70)
        
        # Basic statistics
        self.get_logger().info(f" Analysis Method: {analysis['analysis_method']}")
        self.get_logger().info(f" Total Laser Points: {analysis['total_laser_points']}")
        self.get_logger().info(f" Valid Sectors: {analysis['valid_sectors']}")
        self.get_logger().info("")
        
        # Closest obstacle detection
        self.get_logger().info(f" CLOSEST OBSTACLE DETECTED:")
        self.get_logger().info(f"   Sector: {analysis['closest_sector']}")
        self.get_logger().info(f"   Distance: {analysis['closest_distance']:.3f}m")
        
        if analysis['closest_sector_stats']:
            stats = analysis['closest_sector_stats']
            self.get_logger().info(f"   Index: {stats['min_index']}")
            self.get_logger().info(f"   Angle: {stats['min_angle']:.1f}°")
            self.get_logger().info(f"   Sector Coverage: {stats['coverage_angle_start']:.1f}° - {stats['coverage_angle_end']:.1f}°")
            self.get_logger().info(f"   Valid Readings: {stats['valid_readings']}/{stats['total_readings']}")
        self.get_logger().info("")
        
        # Sector-by-sector breakdown
        self.get_logger().info(f" SECTOR-BY-SECTOR ANALYSIS:")
        
        # Sort sectors by minimum distance for better readability
        sorted_sectors = sorted(analysis['min_distances'].items(), key=lambda x: x[1])
        
        for sector_name, min_dist in sorted_sectors:
            if min_dist == float('inf'):
                status_symbol = "❌"
                dist_str = "No valid data"
            elif min_dist < 0.5:
                status_symbol = "🔴"  # Very close
                dist_str = f"{min_dist:.3f}m"
            elif min_dist < 1.0:
                status_symbol = "🟡"  # Moderate distance
                dist_str = f"{min_dist:.3f}m"
            else:
                status_symbol = "🟢"  # Far
                dist_str = f"{min_dist:.3f}m"
            
            stats = analysis['sector_stats'].get(sector_name)
            if stats and stats is not None:
                angle_range = f"({stats['coverage_angle_start']:.1f}°-{stats['coverage_angle_end']:.1f}°)"
                self.get_logger().info(f"   {sector_name:>12} {status_symbol}: {dist_str:>10} {angle_range}")
            else:
                self.get_logger().info(f"   {sector_name:>12} {status_symbol}: {dist_str:>10} (Invalid sector)")
        
        self.get_logger().info("")
        
        # Rotation decision
        self.get_logger().info(f" ROTATION DECISION:")
        self.get_logger().info(f"   Current front index: 360 (90.0°)")
        if analysis['closest_sector_stats'] and analysis['closest_sector_stats']['min_index']:
            target_idx = analysis['closest_sector_stats']['min_index']
            target_angle = analysis['closest_sector_stats']['min_angle']
            self.get_logger().info(f"   Target index: {target_idx} ({target_angle:.1f}°)")
        self.get_logger().info(f"   Angular difference: {analysis['angular_difference']} indices ({analysis['angular_difference_degrees']:.1f}°)")
        self.get_logger().info(f"   Action needed: {analysis['rotation_needed']}")
        
        if analysis['target_achieved']:
            self.get_logger().info("    ✅ ROBOT IS ALREADY ALIGNED WITH NEAREST WALL!")
        else:
            direction_arrow = "↶" if analysis['rotation_direction'] > 0 else "↷"
            self.get_logger().info(f"    🔄 WILL ROTATE {direction_arrow} TO ALIGN WITH TARGET")
        
        self.get_logger().info("=" * 70)

    def print_180_laser_analysis(self, analysis):
        """
        Print detailed 180° laser analysis results
        
        Args:
            analysis (dict): Analysis results from analyze_180_degree_laser_data
        """
        self.get_logger().info("=" * 65)
        self.get_logger().info(" 180-DEGREE FORWARD LASER ANALYSIS RESULTS")
        self.get_logger().info("=" * 65)
        
        # Basic statistics
        self.get_logger().info(f" Laser Coverage: 180° forward hemisphere")
        self.get_logger().info(f"   Total points: {analysis['total_laser_points']}")
        self.get_logger().info(f"   Valid readings: {analysis['valid_readings']}")
        self.get_logger().info("")
        
        # Shortest distance detected
        self.get_logger().info(f" SHORTEST DISTANCE DETECTED:")
        self.get_logger().info(f"   Index: {analysis['shortest_index']}")
        self.get_logger().info(f"   Raw distance: {analysis['shortest_distance']:.3f}m")
        self.get_logger().info(f"   Averaged distance: {analysis['shortest_averaged']:.3f}m")
        self.get_logger().info(f"   Angle: {analysis['shortest_angle']:.1f}°")
        self.get_logger().info("")
        
        # Top 5 shortest distances
        self.get_logger().info(f" TOP 5 SHORTEST DISTANCES:")
        for i, data in enumerate(analysis['top_5_shortest'], 1):
            self.get_logger().info(f"   #{i}: Index {data['index']} → "
                                  f"{data['distance']:.3f}m (avg: {data['averaged_distance']:.3f}m) "
                                  f"at {data['angle']:.1f}°")
        self.get_logger().info("")
        
        # Directional analysis
        self.get_logger().info(f" DIRECTIONAL ANALYSIS (180° Forward):")
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
        self.get_logger().info(f" ROTATION DECISION:")
        self.get_logger().info(f"   Current front index: 360")
        self.get_logger().info(f"   Target index: {analysis['shortest_index']}")
        self.get_logger().info(f"   Angular difference: {analysis['angular_difference']} indices ({analysis['angular_difference_degrees']:.1f}°)")
        self.get_logger().info(f"   Action needed: {analysis['rotation_needed']}")
        
        if analysis['target_achieved']:
            self.get_logger().info("    ROBOT IS ALREADY ALIGNED WITH NEAREST WALL!")
        else:
            direction_arrow = "↶" if analysis['rotation_direction'] > 0 else "↷"
            self.get_logger().info(f"    WILL ROTATE {direction_arrow} TO ALIGN WITH TARGET")
        
        self.get_logger().info("=" * 65)


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
