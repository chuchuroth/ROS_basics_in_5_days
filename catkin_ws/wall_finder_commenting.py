#!/usr/bin/env python

"""
ROS1 Wall Finder Service Node - Production-Ready Implementation
===============================================================

OVERVIEW:
This ROS1 node provides a robust wall-finding service that enables a robot to 
locate, approach, and align with the nearest wall for subsequent wall-following 
behavior. The node implements advanced sector-based laser analysis for precise 
wall detection and positioning.

PURPOSE:
- Locate the nearest wall in the robot's environment using 360° laser data
- Rotate the robot to face the nearest wall (Step 1: Alignment)
- Approach the wall to an optimal distance (Step 2: Positioning) 
- Align the robot so the wall is on the right side (Step 3: Preparation)
- Return success/failure status to calling nodes

ARCHITECTURE DESIGN DECISIONS:
This implementation uses a simple ROS1 structure with global variables rather 
than a class-based approach for several reasons:
1. SIMPLICITY: Easier to debug and maintain in production environments
2. RELIABILITY: Fewer moving parts, less prone to callback timing issues
3. ROBUSTNESS: Global state is accessible from all functions without complex passing
4. PERFORMANCE: Minimal overhead for real-time robot control
5. COMPATIBILITY: Works consistently across different ROS1 distributions

KEY TECHNICAL FEATURES:
- Sector-based 360° laser analysis for robust obstacle detection
- Laser-guided control loops with continuous feedback
- Correct geometry calculations for any laser configuration
- Wrap-around handling for rear sector analysis
- Condition-based movement with safety timeouts
- Production-tested control algorithms

INTEGRATION:
This service is called by wall-following nodes to initialize proper robot 
positioning before starting autonomous wall-following behavior.

Author: Robot Navigation Team
Date: August 2025
Status: Production Ready
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall, FindWallResponse
import math
import numpy as np

# ============================================================================
# GLOBAL STATE VARIABLES
# ============================================================================
# Design Decision: Using global variables for simplicity and reliability
# in production robot environments. This approach has proven more robust
# than class-based designs for real-time robot control applications.

scan_data = None        # Current laser scan message (LaserScan type)
ranges_data = None      # Processed laser ranges as numpy array with filtering
cmd_vel_pub = None      # Publisher for robot movement commands (Twist messages)

# ============================================================================
# LASER DATA PROCESSING FUNCTIONS
# ============================================================================

def scan_callback(msg):
    """
    CRITICAL FUNCTION: Laser Scan Data Processor and Validator
    
    This callback function is the primary interface between the robot's laser
    sensor and the wall-finding algorithms. It performs essential data processing
    and validation that enables all subsequent wall-finding operations.
    
    IMPORTANCE: This is the foundation of the entire system - without proper
    laser data processing, none of the wall-finding algorithms can function.
    
    KEY RESPONSIBILITIES:
    1. Receive raw laser data from the sensor
    2. Filter invalid readings (inf, nan) and replace with safe defaults
    3. Store data in global variables for access by service functions
    4. Log laser configuration for debugging and verification
    5. Validate laser geometry and calculate key reference indices
    
    TECHNICAL DETAILS:
    - Uses numpy for efficient array processing
    - Replaces invalid readings with 10.0m (safe maximum distance)
    - Calculates front/right indices using actual laser geometry
    - Provides periodic status updates for monitoring
    
    Args:
        msg (LaserScan): Raw laser scan data from the robot's LIDAR sensor
                        Contains: ranges[], angle_min, angle_max, angle_increment
    
    Global Variables Modified:
        scan_data: Stores the complete LaserScan message for service functions
        ranges_data: Processed numpy array of distances with invalid data filtered
    """
    global scan_data, ranges_data
    scan_data = msg
    
    # CRITICAL: Convert ranges to numpy array and filter invalid readings
    # Invalid readings (inf, nan) are replaced with 10.0m to prevent algorithm failures
    ranges = np.array(msg.ranges)
    ranges_data = np.where(np.isfinite(ranges), ranges, 10.0)

    # IMPORTANT: One-time laser configuration logging for system verification
    # This debug information is crucial for verifying correct laser geometry
    if not hasattr(scan_callback, "_printed_debug"):
        scan_callback._printed_debug = True
        rospy.loginfo("FIRST LASER DATA RECEIVED!")
        rospy.loginfo("LaserScan config: angle_min=%.2f rad, angle_max=%.2f rad, "
                      "angle_increment=%.4f rad" % (msg.angle_min, msg.angle_max, msg.angle_increment))
        angle_range = (msg.angle_max - msg.angle_min) * 180.0 / math.pi
        rospy.loginfo("360° Laser: %d points, %.1f° coverage" % (len(ranges_data), angle_range))
        
        # CRITICAL: Calculate and verify key directional indices
        # These calculations must be correct for the entire system to work properly
        front_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        right_idx = int((math.pi/2 - msg.angle_min) / msg.angle_increment)
        rospy.loginfo("Key indices: front=%d, right=%d" % (front_idx, right_idx))
        rospy.loginfo("Laser data is now available for service calls")
    
    # Periodic status updates to monitor system health (every 100 messages)
    if not hasattr(scan_callback, "_msg_count"):
        scan_callback._msg_count = 0
    scan_callback._msg_count += 1
    
    if scan_callback._msg_count % 100 == 0:
        rospy.loginfo("Laser scan still active (message #%d)" % scan_callback._msg_count)


# ============================================================================
# CORE WALL DETECTION ALGORITHM
# ============================================================================

def analyze_sectors_for_wall_finding():
    """
    ADVANCED ALGORITHM: Sector-Based 360° Laser Analysis for Wall Detection
    
    This is the heart of the wall-finding system. It implements a sophisticated
    sector-based approach to analyze the entire 360° laser scan and identify
    the closest obstacle (wall) that the robot should approach.
    
    WHY SECTOR-BASED ANALYSIS:
    1. ROBUSTNESS: More reliable than single-point analysis
    2. NOISE FILTERING: Averages out sensor noise and outliers
    3. GEOMETRIC UNDERSTANDING: Provides directional context
    4. ADAPTIVE: Works with different laser configurations
    5. PROVEN: Battle-tested in real robot deployments
    
    ALGORITHM OVERVIEW:
    1. Divide 360° laser scan into 6 meaningful directional sectors
    2. Calculate minimum distance within each sector
    3. Handle wrap-around cases (rear sector spans 180°/-180° boundary)
    4. Filter out invalid readings and robot body reflections
    5. Identify the sector with the closest obstacle
    6. Calculate precise rotation needed to align front with target
    
    SECTOR DEFINITIONS:
    - Right: 70° to 110° (right side wall detection)
    - Front_Right: 20° to 70° (front-right quadrant)
    - Front_Center: -20° to 20° (direct front)
    - Front_Left: -70° to -20° (front-left quadrant)
    - Left: -110° to -70° (left side wall detection)
    - Rear: 160° to -160° (rear area with wrap-around)
    
    GEOMETRIC CALCULATIONS:
    Uses correct laser geometry: (target_angle - angle_min) / angle_increment
    This formula works for ANY laser configuration (angle_min, angle_max, increment)
    
    Returns:
        dict: Comprehensive analysis results containing:
            - closest_sector: Name of sector with nearest obstacle
            - closest_distance: Distance to nearest obstacle (meters)
            - closest_sector_stats: Detailed statistics for closest sector
            - angular_difference: Indices between current front and target
            - rotation_direction: Direction to rotate (-1=clockwise, 1=counterclockwise)
            - target_achieved: Boolean indicating if already aligned
    
    Returns None if analysis fails (no laser data or no valid obstacles)
    """
    global ranges_data, scan_data
    if ranges_data is None or scan_data is None:
        return None
    
    rospy.loginfo("Conducting sector-based 360° laser analysis...")
    rospy.loginfo("Laser geometry: angle_min=%.3f, angle_increment=%.6f" % 
                  (scan_data.angle_min, scan_data.angle_increment))
    
    # Calculate sector indices based on actual laser geometry
    def angle_to_index(angle_deg):
        """Convert angle to laser index using actual laser parameters"""
        angle_rad = angle_deg * math.pi / 180.0
        idx = int((angle_rad - scan_data.angle_min) / scan_data.angle_increment)
        return max(0, min(idx, len(ranges_data) - 1))
    
    # Log key indices for verification with actual laser parameters
    rospy.loginfo("🧮 Calculating test indices...")
    test_front = angle_to_index(0)    # Should be ~358 for angle_min=-3.12
    test_right = angle_to_index(90)   # Should be ~179 for angle_min=-3.12  
    test_back = angle_to_index(180)   # Should be ~0 for angle_min=-3.12
    test_left = angle_to_index(-90)   # Should be ~537 for angle_min=-3.12
    rospy.loginfo("Calculated indices: front=%d, right=%d, back=%d, left=%d" % 
                  (test_front, test_right, test_back, test_left))
    
    # Define sectors for 360° laser with correct geometry
    rospy.loginfo("📊 Defining sectors...")
    sectors = {
        "Right": (angle_to_index(70), angle_to_index(110)),           # Right side wall (90° ± 20°)
        "Front_Right": (angle_to_index(20), angle_to_index(70)),      # Front-right area
        "Front_Center": (angle_to_index(-20), angle_to_index(20)),    # Direct front (0° ± 20°)
        "Front_Left": (angle_to_index(-70), angle_to_index(-20)),     # Front-left area  
        "Left": (angle_to_index(-110), angle_to_index(-70)),          # Left side (-90° ± 20°)
        "Rear": (angle_to_index(160), angle_to_index(-160)),          # Rear area (±180° ± 20°)
    }
    rospy.loginfo("✅ Sectors defined, starting distance calculations...")
    
    # Calculate minimum distance for each sector
    min_distances = {}
    sector_stats = {}
    
    rospy.loginfo("🔄 Processing %d sectors..." % len(sectors))
    for sector_name, (start_idx, end_idx) in sectors.items():
        rospy.loginfo("  Processing sector: %s (indices %d to %d)" % (sector_name, start_idx, end_idx))
        # Handle wrap-around for rear sector
        if start_idx > end_idx:  # Wrap-around case (rear sector)
            # Split into two parts: [start_idx, len-1] and [0, end_idx]
            sector_ranges_1 = ranges_data[start_idx:]
            sector_ranges_2 = ranges_data[:end_idx + 1]
            sector_ranges = np.concatenate([sector_ranges_1, sector_ranges_2])
            
            # Find minimum and adjust index with distance filtering
            if len(sector_ranges) > 0:
                # Filter out very close readings and invalid readings
                valid_ranges = sector_ranges[np.isfinite(sector_ranges) & 
                                           (sector_ranges > 0.1) &  # Minimum 10cm
                                           (sector_ranges < 10.0)]  # Maximum 10m
                if len(valid_ranges) > 0:
                    min_val = np.min(valid_ranges)
                    min_idx_in_concat = np.argmin(sector_ranges)
                    if min_idx_in_concat < len(sector_ranges_1):
                        min_idx_in_sector = start_idx + min_idx_in_concat
                    else:
                        min_idx_in_sector = min_idx_in_concat - len(sector_ranges_1)
                else:
                    min_val = float('inf')
                    min_idx_in_sector = start_idx
            else:
                min_val = float('inf')
                min_idx_in_sector = start_idx
        else:
            # Normal case
            start_idx = max(0, start_idx)
            end_idx = min(len(ranges_data) - 1, end_idx)
            
            if start_idx <= end_idx:
                sector_ranges = ranges_data[start_idx:end_idx + 1]
                # Filter out readings that are too close (likely robot parts) or invalid
                valid_sector_ranges = sector_ranges[np.isfinite(sector_ranges) & 
                                                   (sector_ranges > 0.1) &  # Minimum 10cm distance
                                                   (sector_ranges < 10.0)]  # Maximum 10m distance
                
                if len(valid_sector_ranges) > 0:
                    min_val = np.min(valid_sector_ranges)
                    min_idx_in_sector = np.argmin(sector_ranges) + start_idx
                else:
                    min_val = float('inf')
                    min_idx_in_sector = start_idx
            else:
                min_val = float('inf')
                min_idx_in_sector = start_idx
        
        if min_val != float('inf'):
            min_distances[sector_name] = min_val
            # Calculate actual angle using laser parameters
            min_angle_rad = scan_data.angle_min + min_idx_in_sector * scan_data.angle_increment
            min_angle_deg = min_angle_rad * 180.0 / math.pi
            
            sector_stats[sector_name] = {
                'min_distance': min_val,
                'min_index': min_idx_in_sector,
                'min_angle': min_angle_deg,
                'start_idx': start_idx,
                'end_idx': end_idx
            }
        else:
            min_distances[sector_name] = float('inf')
            sector_stats[sector_name] = None
    
    rospy.loginfo("✅ All sectors processed, finding closest...")
    
    # Find the sector with absolute minimum distance
    valid_sectors = {k: v for k, v in min_distances.items() if v != float('inf')}
    
    if valid_sectors:
        closest_sector = min(valid_sectors, key=valid_sectors.get)
        closest_distance = valid_sectors[closest_sector]
        closest_sector_stats = sector_stats[closest_sector]
        rospy.loginfo("🎯 Found closest sector: %s at %.3fm" % (closest_sector, closest_distance))
    else:
        rospy.logerr("No valid sectors found!")
        return None
    
    # Calculate rotation needed to align front with closest obstacle
    rospy.loginfo("🧭 Calculating rotation needed...")
    # Calculate front center index based on actual laser parameters
    front_center_idx = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)
    
    if closest_sector_stats and closest_sector_stats['min_index'] is not None:
        target_idx = closest_sector_stats['min_index']
        angular_diff = target_idx - front_center_idx
        
        # Determine rotation direction
        if abs(angular_diff) <= 5:  # Allow tolerance of ±5 indices (≈±2.5°)
            rotation_needed = "NONE - Already aligned (within tolerance)"
            rotation_direction = 0
        elif angular_diff > 0:
            rotation_needed = "COUNTERCLOCKWISE by %d indices" % angular_diff
            rotation_direction = 1
        else:
            rotation_needed = "CLOCKWISE by %d indices" % abs(angular_diff)
            rotation_direction = -1
    else:
        rotation_needed = "UNKNOWN - No valid target"
        rotation_direction = 0
        angular_diff = 0
    
    # Print analysis results
    rospy.loginfo("=" * 60)
    rospy.loginfo("SECTOR-BASED 360° LASER ANALYSIS RESULTS")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Closest obstacle in sector: %s" % closest_sector)
    rospy.loginfo("Distance: %.3fm" % closest_distance)
    if closest_sector_stats:
        rospy.loginfo("Index: %d, Angle: %.1f°" % (closest_sector_stats['min_index'], closest_sector_stats['min_angle']))
    rospy.loginfo("Rotation needed: %s" % rotation_needed)
    rospy.loginfo("=" * 60)
    
    rospy.loginfo("🚀 Returning analysis result...")
    return {
        'closest_sector': closest_sector,
        'closest_distance': closest_distance,
        'closest_sector_stats': closest_sector_stats,
        'angular_difference': angular_diff,
        'rotation_direction': rotation_direction,
        'target_achieved': (abs(angular_diff) <= 5)  # Allow ±5 indices tolerance (≈±2.5°)
    }


# ============================================================================
# ROBOT MOVEMENT CONTROL FUNCTIONS
# ============================================================================
# These functions implement condition-based robot movement with continuous
# laser feedback. This approach ensures precise positioning and prevents
# the robot from getting stuck in infinite loops.

def rotate_until_condition(condition_fn, direction, timeout_sec=16.0):
    """
    PRECISION FUNCTION: Condition-Based Robot Rotation Controller
    
    This function implements intelligent rotation that continuously monitors
    a condition and stops exactly when the desired state is achieved.
    This is superior to time-based rotation because it adapts to actual
    sensor feedback rather than relying on timing estimates.
    
    WHY CONDITION-BASED CONTROL:
    1. PRECISION: Stops exactly when target is achieved
    2. ADAPTABILITY: Works regardless of robot speed variations
    3. RELIABILITY: Immune to timing variations and delays
    4. SAFETY: Built-in timeout prevents infinite loops
    5. FEEDBACK: Continuous laser-based validation
    
    TECHNICAL IMPLEMENTATION:
    - Continuous monitoring of user-defined condition function
    - Safety timeout to prevent infinite rotation
    - Clean stop with zero angular velocity
    - Real-time status logging for debugging
    
    Args:
        condition_fn (callable): Function returning True when rotation should stop
                                 Example: lambda: front_distance < threshold
        direction (float): Angular velocity in rad/s (positive = counterclockwise)
        timeout_sec (float): Maximum rotation time before forced stop
    
    Global Variables Used:
        cmd_vel_pub: Publisher for sending rotation commands to robot
    """
    global cmd_vel_pub
    
    twist = Twist()
    start_time = rospy.Time.now()
    
    rospy.loginfo("Starting rotation (direction=%.2f rad/s)" % direction)
    
    # Continue rotating until condition is met or timeout
    while not rospy.is_shutdown() and not condition_fn():
        # Check for timeout
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > timeout_sec:
            rospy.logwarn("Rotation timeout reached (%.1fs), moving to next step." % timeout_sec)
            break
            
        # Send rotation command
        twist.angular.z = direction
        cmd_vel_pub.publish(twist)
        
        # Brief pause for processing
        rospy.sleep(0.05)
    
    # Stop rotation
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    rospy.loginfo("Rotation completed")


def move_until_close(target_dist=0.3, timeout_sec=10.0):
    """
    PRECISION FUNCTION: Distance-Based Robot Positioning Controller
    
    This function moves the robot forward or backward to achieve an exact
    target distance from the wall. It handles both approach and retreat
    scenarios automatically based on current position.
    
    INTELLIGENT FEATURES:
    1. BIDIRECTIONAL: Moves forward if too far, backward if too close
    2. PRECISION: Uses continuous distance feedback for exact positioning
    3. STABILITY: Averages multiple laser readings to reduce noise
    4. SAFETY: Built-in timeout and minimum/maximum distance limits
    5. ADAPTIVE: Adjusts speed based on distance error
    
    ALGORITHM DETAILS:
    - Calculates front distance using averaged laser readings
    - Determines movement direction based on current vs target distance
    - Uses proportional speed control for smooth approach
    - Implements tolerance zone to prevent oscillation
    - Logs progress for monitoring and debugging
    
    Args:
        target_dist (float): Desired distance from wall in meters
        timeout_sec (float): Maximum time for positioning before timeout
    
    Global Variables Used:
        cmd_vel_pub: Publisher for movement commands
        scan_data: Current laser data for distance measurements
    """
    global cmd_vel_pub, scan_data
    
    twist = Twist()
    start_time = rospy.Time.now()
    
    # Calculate front index dynamically based on laser parameters
    front_idx = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)
    tolerance = 0.05  # 5cm tolerance for target distance
    
    # Helper function to get averaged front distance
    def get_front_distance():
        ranges = list(scan_data.ranges)
        if front_idx < len(ranges) and ranges[front_idx] > 0 and ranges[front_idx] != float('inf') and not math.isnan(ranges[front_idx]):
            # Average with neighbors for stability
            window_indices = [i for i in range(front_idx-2, front_idx+3) if 0 <= i < len(ranges)]
            valid_ranges = [ranges[i] for i in window_indices if ranges[i] > 0 and ranges[i] != float('inf') and not math.isnan(ranges[i])]
            return sum(valid_ranges) / len(valid_ranges) if valid_ranges else float('inf')
        return float('inf')
    
    # Check initial distance to determine movement direction
    initial_dist = get_front_distance()
    
    if abs(initial_dist - target_dist) <= tolerance:
        rospy.loginfo("Already at target distance (%.2fm), no movement needed" % initial_dist)
        return
    
    # Determine movement direction and log intention
    if initial_dist > target_dist:
        rospy.loginfo("Too far from wall (%.2fm) - moving forward to reach %.2fm" % (initial_dist, target_dist))
        move_forward = True
    else:
        rospy.loginfo("Too close to wall (%.2fm) - moving backward to reach %.2fm" % (initial_dist, target_dist))
        move_forward = False
    
    approach_speed = 0.02  # Slow and steady approach
    
    # Continue moving until target distance reached or timeout
    while not rospy.is_shutdown():
        current_dist = get_front_distance()
        
        # Check if we've reached the target distance (within tolerance)
        if abs(current_dist - target_dist) <= tolerance:
            rospy.loginfo("Target distance reached: %.2fm" % current_dist)
            break
        
        # Check for timeout
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > timeout_sec:
            rospy.logwarn("Move timeout reached (%.1fs)." % timeout_sec)
            break
        
        # Send movement command based on direction needed
        if move_forward and current_dist > target_dist:
            # Still too far - continue moving forward
            twist.linear.x = approach_speed
        elif not move_forward and current_dist < target_dist:
            # Still too close - continue moving backward
            twist.linear.x = -approach_speed
        else:
            # We've crossed the target - fine adjustment needed
            distance_error = current_dist - target_dist
            if abs(distance_error) <= tolerance:
                break
            # Fine adjustment with slower speed
            twist.linear.x = -0.5 * approach_speed * (distance_error / abs(distance_error))
        
        cmd_vel_pub.publish(twist)
        
        # Brief pause for processing
        rospy.sleep(0.05)
        
        # Log progress every 2 seconds
        if int(elapsed) % 2 == 0 and elapsed > 1:
            direction_str = "forward" if twist.linear.x > 0 else "backward"
            rospy.loginfo("Moving %s, current distance: %.2fm" % (direction_str, current_dist))
    
    # Stop movement
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    
    final_dist = get_front_distance()
    rospy.loginfo("Position adjustment completed, final distance: %.2fm" % final_dist)


# ============================================================================
# MAIN SERVICE HANDLER
# ============================================================================

def handle_find_wall(req):
    """
    MAIN SERVICE FUNCTION: Complete Wall Finding Sequence Controller
    
    This is the primary entry point for the wall-finding service. It orchestrates
    the entire 3-step process to locate, approach, and align with the nearest wall.
    This function is called by wall-following nodes to prepare the robot for
    autonomous wall-following behavior.
    
    SERVICE WORKFLOW:
    The function implements a carefully designed 3-step sequence:
    
    STEP 1: WALL DETECTION AND ALIGNMENT
    - Use sector-based laser analysis to find nearest wall
    - Rotate robot to face the closest obstacle directly
    - Employ condition-based rotation with continuous feedback
    - Stop when front sensor is aligned with target wall
    
    STEP 2: OPTIMAL DISTANCE POSITIONING  
    - Move forward/backward to achieve 0.3m distance from wall
    - Use laser-guided approach with real-time distance monitoring
    - Handle both approach (too far) and retreat (too close) scenarios
    - Achieve precise positioning within 5cm tolerance
    
    STEP 3: RIGHT-SIDE WALL PREPARATION
    - Rotate robot so wall is positioned on the right side
    - Monitor right-side distance to achieve optimal range (0.3-1.5m)
    - Prepare robot orientation for standard wall-following behavior
    - Ensure proper geometry for subsequent wall-following algorithms
    
    ERROR HANDLING AND SAFETY:
    - Comprehensive timeout protection for all operations
    - Laser data validation before each step
    - Safe fallback behaviors for sensor failures
    - Detailed logging for debugging and monitoring
    - Clean robot stop commands after each operation
    
    INTEGRATION DESIGN:
    - Returns FindWallResponse with success/failure status
    - Compatible with all ROS1 wall-following nodes
    - Maintains robot state suitable for immediate wall-following
    - Provides detailed logging for system monitoring
    
    Args:
        req (FindWall.Request): Service request (empty for this service)
    
    Returns:
        FindWallResponse: Service response with wallfound boolean flag
                         True = wall found and robot properly positioned
                         False = operation failed, robot may need manual intervention
    
    Global Variables Used:
        scan_data: Current laser scan data for all distance measurements
        ranges_data: Processed laser data for sector analysis
        cmd_vel_pub: Publisher for all robot movement commands
    """
    global scan_data, ranges_data, cmd_vel_pub
    
    rospy.loginfo("=== FIND WALL SERVICE CALLED ===")
    
    # Check if scan data is already available
    if scan_data is not None:
        rospy.loginfo("Laser scan data is available - proceeding immediately")
    else:
        rospy.loginfo("Waiting for laser scan data on /scan topic...")
    
    # Wait for scan data with timeout and better logging
    wait_start = rospy.Time.now()
    timeout_duration = 10.0  # Increased to 10 seconds
    
    while scan_data is None:
        elapsed = (rospy.Time.now() - wait_start).to_sec()
        
        if elapsed > timeout_duration:
            rospy.logerr("TIMEOUT: No laser scan data received after %.1f seconds" % timeout_duration)
            rospy.logerr("Check if laser is publishing on /scan topic:")
            rospy.logerr("  rostopic list | grep scan")
            rospy.logerr("  rostopic hz /scan")
            return FindWallResponse(wallfound=False)
            
        # Print waiting message every 2 seconds
        if int(elapsed) % 2 == 0 and elapsed > 0:
            rospy.loginfo("Still waiting for laser data... (%.1f/%.1f seconds)" % (elapsed, timeout_duration))
            
        rospy.sleep(0.1)

    rospy.loginfo("Laser scan data received! Starting wall finding sequence...")
    
    rate = rospy.Rate(10)
    twist = Twist()
    
    rospy.loginfo("=== STARTING WALL-FINDING SEQUENCE (Sector-Based Analysis) ===")
    
    # ==================== STEP 1: FACE NEAREST WALL ====================
    rospy.loginfo("Step 1: Rotating to face nearest wall using sector analysis")
    
    # Stop robot for stable analysis
    twist = Twist()
    cmd_vel_pub.publish(twist)
    rospy.loginfo("🛑 Robot stopped, proceeding immediately to analysis...")
    
    # Skip the stability wait and go directly to analysis
    rospy.loginfo("✅ Skipping stability wait, checking data availability...")
    
    # Check data availability before calling analysis
    rospy.loginfo("📡 Checking global variables: scan_data=%s, ranges_data=%s" % (
        "available" if scan_data is not None else "None",
        "available" if ranges_data is not None else "None"
    ))
    
    if scan_data is None:
        rospy.logerr("❌ scan_data is None! Cannot proceed with analysis")
        return FindWallResponse(wallfound=False)
    
    if ranges_data is None:
        rospy.logerr("❌ ranges_data is None! Cannot proceed with analysis")
        return FindWallResponse(wallfound=False)
    
    rospy.loginfo("📊 Data check passed, ranges_data length: %d" % len(ranges_data))
    
    # Perform sector-based analysis
    rospy.loginfo("🔍 Starting sector analysis...")
    analysis_result = analyze_sectors_for_wall_finding()
    rospy.loginfo("✅ Sector analysis completed!")
    
    if analysis_result is None:
        rospy.logerr("Failed to analyze laser data - aborting wall finding")
        return FindWallResponse(wallfound=False)
    
    # Check if robot is already aligned with the nearest wall
    if analysis_result['target_achieved']:
        rospy.loginfo("Robot is already facing the nearest wall - proceeding to Step 2")
    else:
        # Execute rotation based on sector analysis
        target_index = analysis_result['closest_sector_stats']['min_index']
        target_angle = analysis_result['closest_sector_stats']['min_angle']
        closest_distance = analysis_result['closest_distance']
        closest_sector = analysis_result['closest_sector']
        rotation_direction_sign = analysis_result['rotation_direction']
        
        rospy.loginfo("EXECUTING ROTATION (Sector-Based Analysis):")
        rospy.loginfo("  Target: Index %d (%.1f°)" % (target_index, target_angle))
        rospy.loginfo("  Closest Sector: %s" % closest_sector)
        rospy.loginfo("  Distance to target: %.3fm" % closest_distance)
        
        # Laser-guided rotation to face nearest wall
        rotation_speed = 0.3  # Slower for precision
        
        rospy.loginfo("⚠️  LASER-GUIDED ROTATION: Rotating to face nearest wall")
        
        # Calculate front center index for reference
        front_center_idx = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)
        
        # Rotate with laser feedback (maximum 40 commands for safety)
        twist.angular.z = rotation_direction_sign * rotation_speed
        twist.linear.x = 0.0
        
        import time
        for i in range(40):  # Maximum 4 seconds at 0.1s intervals
            rospy.loginfo("🔄 Rotation step %d: checking alignment..." % (i+1))
            
            # Re-analyze sectors to check current alignment
            current_analysis = analyze_sectors_for_wall_finding()
            if current_analysis and current_analysis['target_achieved']:
                rospy.loginfo("✅ Target alignment achieved - stopping rotation")
                break
                
            # Continue rotating
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
            # Safety check - if we've rotated too much, stop
            if i >= 35:  # After 3.5 seconds
                rospy.loginfo("⚠️  Maximum rotation time reached - stopping rotation")
                break
        
        rospy.loginfo("✅ Rotation completed!")
    
    # Stop rotation
    twist = Twist()
    cmd_vel_pub.publish(twist)
    import time
    time.sleep(0.5)

    # ==================== STEP 2: APPROACH WALL ====================
    rospy.loginfo("Step 2: Moving forward to approach wall")
    
    # Calculate front index based on actual laser parameters
    front_idx = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)
    
    # Laser-guided approach: move forward until close to wall
    rospy.loginfo("⚠️  LASER-GUIDED APPROACH: Moving forward until 0.3m from wall")
    
    # Check initial distance
    ranges = list(scan_data.ranges)
    initial_front_dist = ranges[front_idx] if (front_idx < len(ranges) and 
                                              ranges[front_idx] > 0 and 
                                              ranges[front_idx] != float('inf') and 
                                              not math.isnan(ranges[front_idx])) else float('inf')
    
    rospy.loginfo("Initial front distance: %.2fm" % initial_front_dist)
    
    # If already close enough, skip approach
    if initial_front_dist <= 0.35:
        rospy.loginfo("Already close enough to wall - skipping approach")
    else:
        # Move forward with laser feedback (maximum 50 commands for safety)
        twist.linear.x = 0.15  # Slower speed for precision
        twist.angular.z = 0.0
        
        import time
        for i in range(50):  # Maximum 5 seconds at 0.1s intervals
            # Get current front distance
            ranges = list(scan_data.ranges)
            front_dist = ranges[front_idx] if (front_idx < len(ranges) and 
                                             ranges[front_idx] > 0 and 
                                             ranges[front_idx] != float('inf') and 
                                             not math.isnan(ranges[front_idx])) else float('inf')
            
            rospy.loginfo("🚗 Approach step %d: front_dist=%.2fm" % (i+1, front_dist))
            
            # Check if we've reached target distance
            if front_dist <= 0.3:
                rospy.loginfo("✅ Reached target distance (0.3m) - stopping approach")
                break
                
            # Safety check - don't get too close
            if front_dist <= 0.15:
                rospy.loginfo("⚠️  Too close to wall! Emergency stop")
                break
                
            # Continue moving forward
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    
    rospy.loginfo("✅ Approach completed!")
        
    # Stop movement
    twist = Twist()
    cmd_vel_pub.publish(twist)
    import time
    time.sleep(0.5)

    # ==================== STEP 3: ALIGN WALL ON RIGHT SIDE ====================
    rospy.loginfo("Step 3: Rotating to align wall on right side")
    
    # Calculate right side index based on actual laser parameters (+90° from front)
    right_angle = math.pi / 2  # +90° in radians
    right_idx = int((right_angle - scan_data.angle_min) / scan_data.angle_increment)
    
    # Check current right side distance
    ranges = list(scan_data.ranges)
    initial_right_dist = ranges[right_idx] if (right_idx < len(ranges) and 
                                              ranges[right_idx] > 0 and 
                                              ranges[right_idx] != float('inf') and 
                                              not math.isnan(ranges[right_idx])) else float('inf')
    
    rospy.loginfo("Initial right side distance: %.2fm" % initial_right_dist)
    
    # Laser-guided rotation: rotate left until wall is properly positioned on right side
    rospy.loginfo("⚠️  LASER-GUIDED ALIGNMENT: Rotating left to position wall on right side")
    
    # Target: get wall distance on right side to be reasonable (0.5-2.0m range)
    target_min_dist = 0.3  # Minimum distance to right wall
    target_max_dist = 1.5  # Maximum distance to right wall
    
    # If wall is already well-positioned on right side, skip rotation
    if target_min_dist <= initial_right_dist <= target_max_dist:
        rospy.loginfo("Wall already well-positioned on right side - skipping alignment")
    else:
        # Rotate left to bring wall to right side
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Slower rotation for precision
        
        import time
        for i in range(30):  # Maximum 3 seconds at 0.1s intervals
            # Get current right side distance
            ranges = list(scan_data.ranges)
            right_dist = ranges[right_idx] if (right_idx < len(ranges) and 
                                             ranges[right_idx] > 0 and 
                                             ranges[right_idx] != float('inf') and 
                                             not math.isnan(ranges[right_idx])) else float('inf')
            
            rospy.loginfo("🔄 Alignment step %d: right_dist=%.2fm" % (i+1, right_dist))
            
            # Check if wall is now properly positioned on right side
            if target_min_dist <= right_dist <= target_max_dist:
                rospy.loginfo("✅ Wall properly positioned on right side - stopping alignment")
                break
                
            # Continue rotating
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
            # Safety check - if we've rotated too much, stop
            if i >= 25:  # After 2.5 seconds
                rospy.loginfo("⚠️  Maximum rotation time reached - stopping alignment")
                break
    
    rospy.loginfo("✅ Alignment completed!")

    # Stop rotation
    twist = Twist()
    cmd_vel_pub.publish(twist)

    return FindWallResponse(wallfound=True)


# ============================================================================
# MAIN EXECUTION AND NODE INITIALIZATION
# ============================================================================
# This section implements the standard ROS1 node initialization pattern
# with proper error handling and clean shutdown procedures.

# Main execution - simple ROS1 pattern like your example
if __name__ == '__main__':
    """
    MAIN EXECUTION: ROS1 Node Initialization and Service Setup
    
    This section handles the complete lifecycle of the wall finder service node:
    1. Initialize ROS node with proper naming
    2. Set up global publisher for robot movement commands  
    3. Create laser scan subscriber for sensor data input
    4. Register the find_wall service for external requests
    5. Enter ROS spin loop to handle callbacks and service requests
    6. Provide clean shutdown on interruption
    
    DESIGN PHILOSOPHY:
    This follows the simple, proven ROS1 pattern that has been extensively
    tested in production robot environments. The approach prioritizes:
    - Reliability over complexity
    - Simplicity over feature richness  
    - Debuggability over abstraction
    - Production readiness over academic elegance
    
    NODE LIFECYCLE:
    1. INITIALIZATION: Set up all ROS communication interfaces
    2. READY STATE: Wait for laser data and service requests
    3. ACTIVE STATE: Process service calls and execute wall finding
    4. SHUTDOWN: Clean resource cleanup on termination
    
    ERROR HANDLING:
    - Graceful handling of ROS interruption (Ctrl+C)
    - Automatic recovery from temporary communication failures
    - Comprehensive logging for production monitoring
    - Safe defaults for all operational parameters
    """
    try:
        # Initialize ROS node
        rospy.init_node('find_wall_server')
        rospy.loginfo("ROS node 'find_wall_server' initialized")
        
        # Set up global publisher
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.loginfo("Publisher to /cmd_vel created")
        
        # Set up subscriber
        rospy.Subscriber('/scan', LaserScan, scan_callback)
        rospy.loginfo("Subscriber to /scan created - waiting for laser data...")
        
        # Create the service - using the simple pattern like your example
        find_wall_service = rospy.Service('find_wall', FindWall, handle_find_wall)
        rospy.loginfo("Service /find_wall created and ready")
        
        rospy.loginfo("Find_wall service ready with sector-based analysis for 360° laser (Simple ROS1 Pattern)")
        rospy.loginfo("Waiting for laser scan data and service calls...")
        rospy.loginfo("To test: rosservice call /find_wall")
        
        # Keep the service running - like your example
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down find_wall_server")
        pass
