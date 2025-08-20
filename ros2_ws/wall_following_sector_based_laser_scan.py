#!/usr/bin/env python3

"""
Wall Following Control Node - Simplified with Sector-Based Analysis
==================================================================

This ROS 2 node implements autonomous wall following behavior for a robot.
The robot will:
1. Call the find_wall service to locate and align with nearest wall
2. Start odometry recording action for path tracking  
3. Execute continuous wall following control loop with sector-based obstacle detection

360-Degree Full-Circle Laser Configuration:
- Coverage: Full 360° around robot
- angle_min ≈ -179° (rear-left)
- angle_max ≈ +180° (rear-right)  
- angle_increment ≈ 0.5° per reading
- Total points ≈ 720
- Key directions calculated dynamically:
  * Front (0°): calculated from laser params
  * Right (+90°): calculated from laser params
  * Left (-90°): calculated from laser params

Wall Following Strategy:
- Maintain constant distance from RIGHT-side wall
- Avoid front obstacles using sector-based detection
- Simple control commands for robust operation

SECTOR-BASED ANALYSIS METHOD:
This implementation uses the sector-based laser analysis method for improved
obstacle detection and navigation decision making, dividing the 360° laser 
scan into meaningful sectors with correct geometry calculations.

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
        self.turn_speed = 0.3               # rad/s - normal turning speed
        
        # ==================== STATE VARIABLES ====================
        
        # Operational state flags
        self.wall_found = False
        self.odom_recording = False
        self.preparation_complete = False
        self.laser_initialized = False  # Track if laser callback has been triggered
        
        # ==================== INITIALIZATION ====================
        
        self.get_logger().info("Wall Following Controller initialized")
        self.get_logger().info(f"Parameters: wall distance {self.min_wall_distance}-{self.max_wall_distance}m, "
                              f"obstacle threshold {self.front_obstacle_threshold}m")
        
        # This node will call find_wall service then start wall following
        self.get_logger().info("Will call find_wall service first, then start wall following...")
        
        # CALL FIND_WALL IMMEDIATELY (like ROS1 version) - don't wait for laser data!
        self.get_logger().info("=== CALLING FIND_WALL SERVICE IMMEDIATELY ===")
        if self.call_find_wall_service():
            self.get_logger().info(" Find wall service completed successfully")
            self.wall_found = True
        else:
            self.get_logger().warn(" Find wall service failed - continuing anyway")
        
        # Mark preparation complete so control loop can start
        self.preparation_complete = True
        self.get_logger().info("=== PREPARATION COMPLETE - READY FOR WALL FOLLOWING ===")
        
        # Start main control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def laser_callback(self, msg: LaserScan):
        """
        Process incoming laser scan data for 360-degree full-circle laser
        
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
        
        # Log laser info once for debugging (like working file)
        if not hasattr(self, '_laser_logged'):
            self._laser_logged = True
            self.get_logger().info("LASER CALLBACK TRIGGERED! Processing first laser message...")
            self.get_logger().info(f"LaserScan config: angle_min={self.angle_min:.3f} rad, "
                                  f"angle_max={self.angle_max:.3f} rad, "
                                  f"angle_increment={self.angle_increment:.6f} rad")
            self.get_logger().info(f"Total ranges: {self.num_ranges}")
            
            # Calculate indices using the CORRECT method (actual laser geometry)
            # For angle_min=-3.12, angle_max=3.14, increment=0.0087:
            front_idx = int((0.0 - self.angle_min) / self.angle_increment)  # 0° relative to angle_min
            right_idx = int((math.pi/2 - self.angle_min) / self.angle_increment)  # +90° relative to angle_min
            left_idx = int((-math.pi/2 - self.angle_min) / self.angle_increment)  # -90° relative to angle_min
            back_idx = int((math.pi - self.angle_min) / self.angle_increment)  # 180° relative to angle_min
            
            self.get_logger().info(f"CORRECT indices for actual laser geometry:")
            self.get_logger().info(f"  Front (0°): {front_idx}")
            self.get_logger().info(f"  Right (+90°): {right_idx}")
            self.get_logger().info(f"  Left (-90°): {left_idx}")
            self.get_logger().info(f"  Back (180°): {back_idx}")
            
            # Mark laser as initialized - preparation already complete
            if not self.laser_initialized:
                self.laser_initialized = True
                self.get_logger().info(" Laser data received! Wall following can now start...")

    def get_sector_obstacle_detections(self, obstacle_threshold=0.8):
        """
        Detect obstacles in each sector using sector-based analysis
        
        Sectors for 360° laser with correct geometry:
        - Uses actual laser parameters to calculate sector indices
        - Adapts to robot's specific laser configuration
        
        Args:
            obstacle_threshold (float): Distance threshold for obstacle detection
            
        Returns:
            dict: Boolean flags for obstacle detection in each sector
        """
        if self.ranges is None or self.angle_min is None or self.angle_increment is None:
            return {}
        
        # Calculate sector indices based on actual laser geometry
        def angle_to_index(angle_deg):
            angle_rad = angle_deg * math.pi / 180.0
            idx = int((angle_rad - self.angle_min) / self.angle_increment)
            return max(0, min(idx, len(self.ranges) - 1))
        
        # Define key sectors for wall following (using actual angles)
        # Narrowed Front_Center for less sensitive obstacle detection
        sectors = {
            "Right": (angle_to_index(0), angle_to_index(40)),      # Right side wall (90° ± 20°)
            "Front_Right": (angle_to_index(-50), angle_to_index(0)), # Front-right area
            "Front_Center": (angle_to_index(-90), angle_to_index(-50)), # Direct front (0° ± 10°) - NARROWED
            "Front_Left": (angle_to_index(-180), angle_to_index(-140)), # Front-left area  
            "Left": (angle_to_index(90), angle_to_index(-230)),     # Left side (-90° ± 20°)
        }
        
        detections = {}
        for sector_name, (start_idx, end_idx) in sectors.items():
            # Ensure indices are within valid range
            start_idx = max(0, start_idx)
            end_idx = min(len(self.ranges) - 1, end_idx)
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
        
        Uses CORRECT geometry for 360° laser with actual parameters:
        - angle_min ≈ -3.12 rad (-179°)
        - angle_max ≈ +3.14 rad (+180°) 
        - Right (+90°) is at index calculated from actual geometry
        
        Returns:
            float: Distance to right wall in meters
        """
        if self.ranges is None or self.angle_increment is None or self.angle_min is None:
            return float('inf')
        
        # CORRECT method for actual laser geometry
        right_idx = int((math.pi/2 - self.angle_min) / self.angle_increment)  # +90° relative to angle_min
        
        # Ensure index is within valid range
        right_idx = max(0, min(right_idx, len(self.ranges) - 1))
        
        # Get distance (add small averaging for stability)
        window = 2
        indices = []
        for i in range(-window, window + 1):
            idx = right_idx + i
            if 0 <= idx < len(self.ranges):
                indices.append(idx)
        
        distances = [self.ranges[i] for i in indices if self.ranges[i] > 0 and math.isfinite(self.ranges[i])]
        return np.mean(distances) if distances else float('inf')

    def get_front_distance(self):
        """
        Get distance to front obstacle for obstacle avoidance
        
        Uses CORRECT geometry for 360° laser with actual parameters:
        - angle_min ≈ -3.12 rad (-179°)
        - angle_max ≈ +3.14 rad (+180°)
        - Front (0°) is at index calculated from actual geometry
        
        Returns:
            float: Distance to front obstacle in meters
        """
        if self.ranges is None or self.angle_min is None or self.angle_increment is None:
            return float('inf')
        
        # CORRECT method for actual laser geometry
        front_idx = int((0.0 - self.angle_min) / self.angle_increment)  # 0° relative to angle_min
        
        # Ensure index is within valid range
        front_idx = max(0, min(front_idx, len(self.ranges) - 1))
        
        # Get front distance with small averaging window
        window = 2
        indices = [front_idx + i for i in range(-window, window + 1)]
        distances = [self.ranges[i] for i in indices if 0 <= i < len(self.ranges) and self.ranges[i] > 0 and math.isfinite(self.ranges[i])]
        return np.mean(distances) if distances else float('inf')

    def calculate_wall_following_control(self):
        """
        Calculate velocity commands for wall following using SIMPLE control logic
        
        Based on the working ros1_wall_follower_final_version.py:
        1. Simple bang-bang control for wall distance
        2. Simple front obstacle avoidance
        3. No complex sector analysis - just direct measurements
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        # Get wall and front distances using the working method
        right_dist = self.get_right_wall_distance()
        front_dist = self.get_front_distance()
        
        # Simple bang-bang control like the working file
        twist_linear = 0.1  # Always move forward
        twist_angular = 0.0  # Default: go straight
        
        # Wall distance control (copy from working file)
        if right_dist > 0.3:
            twist_angular = -0.2  # Turn right (toward wall)
        elif right_dist < 0.2:
            twist_angular = 0.4   # Turn left (away from wall)
        else:
            twist_angular = 0.0   # Go straight
        
        # Front obstacle avoidance (copy from working file)
        if front_dist < 0.5:
            twist_angular = 3.0   # Strong left turn
            twist_linear = 0.1    # Keep moving
        
        return twist_linear, twist_angular

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

    def delayed_prepare_robot(self):
        """
        Delayed preparation method called after laser data is available
        This ensures the laser subscription is fully active before starting
        """
        self.get_logger().info("Starting delayed robot preparation...")
        self.prepare_robot()

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
        self.get_logger().info("Step 1: Finding and aligning with wall...")
        if not self.call_find_wall_service():
            self.get_logger().error(" Could not find wall - aborting preparation")
            return False
        self.get_logger().info("Step 1: Wall found and aligned!")
        
        # Step 2: Start odometry recording (optional - don't fail if not available)
        self.get_logger().info("Step 2: Attempting to start odometry recording...")
        if not self.start_odom_recording():
            self.get_logger().warn(" Could not start odometry recording - continuing without it")
            # Don't abort - continue with wall following anyway
        else:
            self.get_logger().info("Step 2: Odometry recording started!")
        
        # Step 3: Mark preparation complete
        self.get_logger().info("Step 3: Marking preparation as complete...")
        self.preparation_complete = True
        self.get_logger().info("Step 3: Preparation complete - starting wall following behavior")
        
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
        Start the odometry recording action to track robot path during wall following.
        Prints feedback (distance) as received, and prints the result (path and total distance) when done.
        """
        self.get_logger().info(" Starting odometry recording...")
        self.get_logger().info("Waiting for /record_odom action server...")
        if not self.odom_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/record_odom action server not available after 10 seconds!")
            return False
        goal_msg = OdomRecord.Goal()
        self.get_logger().info("Sending odometry recording goal...")
        # Send goal and register feedback/result callbacks
        future = self.odom_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.odom_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is not None and goal_handle.accepted:
            self.get_logger().info(" Odometry recording started successfully!")
            self.odom_recording = True
            # Register result callback
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.odom_result_callback)
            return True
        else:
            self.get_logger().error(" Odometry recording goal rejected")
            return False

    def odom_feedback_callback(self, feedback_msg):
        # Throttle feedback printing to every 10 seconds, but print immediately if near start
        current_total = feedback_msg.feedback.current_total
        now = self.get_clock().now().nanoseconds / 1e9
        # Use instance variables to track last print and near-start flag
        if not hasattr(self, '_last_odom_feedback_time'):
            self._last_odom_feedback_time = 0.0
        if not hasattr(self, '_near_start_printed'):
            self._near_start_printed = False

        # Try to get last odom point and first odom point from result if available
        near_start = False
        try:
            # If odometry points are available in feedback, check distance to start
            if hasattr(feedback_msg.feedback, 'list_of_odoms') and feedback_msg.feedback.list_of_odoms:
                odoms = feedback_msg.feedback.list_of_odoms
                first = odoms[0]
                last = odoms[-1]
                dx = last.x - first.x
                dy = last.y - first.y
                distance_to_start = math.sqrt(dx*dx + dy*dy)
                if distance_to_start < 0.2:
                    near_start = True
        except Exception:
            pass

        # Print feedback if 10 seconds have passed or if near start and not yet printed
        if (now - self._last_odom_feedback_time > 10.0) or (near_start and not self._near_start_printed):
            msg = f"[Odom Feedback] Current total distance: {current_total:.3f} m"
            if near_start and not self._near_start_printed:
                msg = "\n====================\n!!! ROBOT NEAR STARTING POSITION !!!\n" + msg + "\n===================="
                self._near_start_printed = True
            self.get_logger().info(msg)
            print(msg, flush=True)
            self._last_odom_feedback_time = now

    def odom_result_callback(self, future):
        # Print result (list of odometry points and total distance)
        result = future.result().result
        odom_points = result.list_of_odoms
        msg = f"[Odom Result] Path points recorded: {len(odom_points)}"
        self.get_logger().info(msg)
        print(msg, flush=True)
        if odom_points:
            for i, pt in enumerate(odom_points):
                pt_msg = f"  Point {i+1}: x={pt.x:.3f}, y={pt.y:.3f}"
                self.get_logger().info(pt_msg)
                print(pt_msg, flush=True)
        # Try to print total distance if available (if server adds it to result)
        if hasattr(result, 'total_distance'):
            total_msg = f"[Odom Result] Total distance: {result.total_distance:.3f} m"
            self.get_logger().info(total_msg)
            print(total_msg, flush=True)


    def control_loop(self):
        """
        Main control loop for wall following behavior
        
        Simple control strategy:
        1. Wait for laser data to be available
        2. Calculate control commands using simple wall following
        3. Publish velocity commands
        """
        # Wait for laser data to be available before doing anything
        if not self.laser_initialized or self.ranges is None:
            # Don't spam logs - only log occasionally
            if not hasattr(self, '_waiting_logged') or self._waiting_logged % 50 == 0:
                self.get_logger().info("Waiting for laser data to start wall following...")
            if not hasattr(self, '_waiting_logged'):
                self._waiting_logged = 0
            self._waiting_logged += 1
            self.stop_robot()
            return
        
        try:
            # Calculate velocities using wall following control
            linear_vel, angular_vel = self.calculate_wall_following_control()
            
            # Log control decisions every 2 seconds for monitoring
            if not hasattr(self, '_control_log_counter'):
                self._control_log_counter = 0
            self._control_log_counter += 1
            
            if self._control_log_counter % 20 == 0:  # Every 2 seconds at 10Hz
                right_dist = self.get_right_wall_distance()
                front_dist = self.get_front_distance()
                self.get_logger().info(f" Wall following: right={right_dist:.2f}m, front={front_dist:.2f}m, "
                                      f"cmd: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
            
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
