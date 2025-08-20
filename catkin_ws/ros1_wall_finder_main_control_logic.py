#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall, FindWallResponse
import math
import numpy as np

# Global variables for ROS1 simple structure
scan_data = None
ranges_data = None
cmd_vel_pub = None

def scan_callback(msg):
    """Callback for laser scan data"""
    global scan_data, ranges_data
    scan_data = msg
    
    # Convert ranges to numpy array and filter invalid readings
    ranges = np.array(msg.ranges)
    ranges_data = np.where(np.isfinite(ranges), ranges, 10.0)

    # Debug print laser configuration once
    if not hasattr(scan_callback, "_printed_debug"):
        scan_callback._printed_debug = True
        rospy.loginfo("FIRST LASER DATA RECEIVED!")
        rospy.loginfo("LaserScan config: angle_min=%.2f rad, angle_max=%.2f rad, "
                      "angle_increment=%.4f rad" % (msg.angle_min, msg.angle_max, msg.angle_increment))
        angle_range = (msg.angle_max - msg.angle_min) * 180.0 / math.pi
        rospy.loginfo("360° Laser: %d points, %.1f° coverage" % (len(ranges_data), angle_range))
        
        # Calculate and log key indices for verification
        front_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        right_idx = int((math.pi/2 - msg.angle_min) / msg.angle_increment)
        rospy.loginfo("Key indices: front=%d, right=%d" % (front_idx, right_idx))
        rospy.loginfo("Laser data is now available for service calls")
    
    # Print periodic status (every 100 messages to avoid spam)
    if not hasattr(scan_callback, "_msg_count"):
        scan_callback._msg_count = 0
    scan_callback._msg_count += 1
    
    if scan_callback._msg_count % 100 == 0:
        rospy.loginfo("Laser scan still active (message #%d)" % scan_callback._msg_count)


def analyze_sectors_for_wall_finding():
    """
    Sector-based laser analysis for wall finding (ROS1 version)
    
    Divides 360° laser scan into meaningful sectors to find nearest wall.
    Uses actual laser parameters for correct geometry calculations.
    
    Returns:
        dict: Analysis results with closest sector information
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
    rospy.loginfo(" Calculating test indices...")
    test_front = angle_to_index(180)    # Should be ~358 for angle_min=-3.12
    test_right = angle_to_index(0)   # Should be ~179 for angle_min=-3.12  
    test_back = angle_to_index(540)   # Should be ~0 for angle_min=-3.12
    test_left = angle_to_index(360)   # Should be ~537 for angle_min=-3.12
    rospy.loginfo("Calculated indices: front=%d, right=%d, back=%d, left=%d" % 
                  (test_front, test_right, test_back, test_left))
    
    # Define sectors for 360° laser with correct geometry
    rospy.loginfo(" Defining sectors...")
    sectors = {
        "Right": (angle_to_index(0), angle_to_index(40)),           # Right side wall (90° ± 20°)
        "Front_Right": (angle_to_index(-50), angle_to_index(0)),      # Front-right area
        "Front_Center": (angle_to_index(-90), angle_to_index(-50)),    # Direct front (0° ± 20°)
        "Front_Left": (angle_to_index(-140), angle_to_index(-90)),     # Front-left area  
        "Left": (angle_to_index(-180), angle_to_index(-140)),          # Left side (-90° ± 20°)
        "Rear": (angle_to_index(90), angle_to_index(-230)),          # Rear area (±180° ± 20°)
    }
    rospy.loginfo(" Sectors defined, starting distance calculations...")
    
    # Calculate minimum distance for each sector
    min_distances = {}
    sector_stats = {}
    
    rospy.loginfo(" Processing %d sectors..." % len(sectors))
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
    
    rospy.loginfo(" All sectors processed, finding closest...")
    
    # Find the sector with absolute minimum distance
    valid_sectors = {k: v for k, v in min_distances.items() if v != float('inf')}
    
    if valid_sectors:
        closest_sector = min(valid_sectors, key=valid_sectors.get)
        closest_distance = valid_sectors[closest_sector]
        closest_sector_stats = sector_stats[closest_sector]
        rospy.loginfo(" Found closest sector: %s at %.3fm" % (closest_sector, closest_distance))
    else:
        rospy.logerr("No valid sectors found!")
        return None
    
    # Calculate rotation needed to align front with closest obstacle
    rospy.loginfo(" Calculating rotation needed...")
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
    
    rospy.loginfo(" Returning analysis result...")
    return {
        'closest_sector': closest_sector,
        'closest_distance': closest_distance,
        'closest_sector_stats': closest_sector_stats,
        'angular_difference': angular_diff,
        'rotation_direction': rotation_direction,
        'target_achieved': (abs(angular_diff) <= 5)  # Allow ±5 indices tolerance (≈±2.5°)
    }


def handle_find_wall(req):
    """Service callback function - follows the simple ROS1 pattern"""
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
    rospy.loginfo(" Robot stopped, proceeding immediately to analysis...")
    
    # Skip the stability wait and go directly to analysis
    rospy.loginfo(" Skipping stability wait, checking data availability...")
    
    # Check data availability before calling analysis
    rospy.loginfo(" Checking global variables: scan_data=%s, ranges_data=%s" % (
        "available" if scan_data is not None else "None",
        "available" if ranges_data is not None else "None"
    ))
    
    if scan_data is None:
        rospy.logerr(" scan_data is None! Cannot proceed with analysis")
        return FindWallResponse(wallfound=False)
    
    if ranges_data is None:
        rospy.logerr(" ranges_data is None! Cannot proceed with analysis")
        return FindWallResponse(wallfound=False)
    
    rospy.loginfo(" Data check passed, ranges_data length: %d" % len(ranges_data))
    
    # Perform sector-based analysis
    rospy.loginfo(" Starting sector analysis...")
    analysis_result = analyze_sectors_for_wall_finding()
    rospy.loginfo(" Sector analysis completed!")
    
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
        
        rospy.loginfo("  LASER-GUIDED ROTATION: Rotating to face nearest wall")
        
        # Calculate front center index for reference
        front_center_idx = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)
        
        # Rotate with laser feedback (maximum 40 commands for safety)
        twist.angular.z = rotation_direction_sign * rotation_speed
        twist.linear.x = 0.0
        
        import time
        for i in range(40):  # Maximum 4 seconds at 0.1s intervals
            rospy.loginfo(" Rotation step %d: checking alignment..." % (i+1))
            
            # Re-analyze sectors to check current alignment
            current_analysis = analyze_sectors_for_wall_finding()
            if current_analysis and current_analysis['target_achieved']:
                rospy.loginfo(" Target alignment achieved - stopping rotation")
                break
                
            # Continue rotating
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
            # Safety check - if we've rotated too much, stop
            if i >= 35:  # After 3.5 seconds
                rospy.loginfo("  Maximum rotation time reached - stopping rotation")
                break
        
        rospy.loginfo(" Rotation completed!")
    
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
    rospy.loginfo("  LASER-GUIDED APPROACH: Moving forward until 0.3m from wall")
    
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
            
            rospy.loginfo(" Approach step %d: front_dist=%.2fm" % (i+1, front_dist))
            
            # Check if we've reached target distance
            if front_dist <= 0.3:
                rospy.loginfo(" Reached target distance (0.3m) - stopping approach")
                break
                
            # Safety check - don't get too close
            if front_dist <= 0.15:
                rospy.loginfo("  Too close to wall! Emergency stop")
                break
                
            # Continue moving forward
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    
    rospy.loginfo(" Approach completed!")
        
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
    rospy.loginfo("  LASER-GUIDED ALIGNMENT: Rotating left to position wall on right side")
    
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
            
            rospy.loginfo(" Alignment step %d: right_dist=%.2fm" % (i+1, right_dist))
            
            # Check if wall is now properly positioned on right side
            if target_min_dist <= right_dist <= target_max_dist:
                rospy.loginfo(" Wall properly positioned on right side - stopping alignment")
                break
                
            # Continue rotating
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
            # Safety check - if we've rotated too much, stop
            if i >= 25:  # After 2.5 seconds
                rospy.loginfo("  Maximum rotation time reached - stopping alignment")
                break
    
    rospy.loginfo(" Alignment completed!")

    # Stop rotation
    twist = Twist()
    cmd_vel_pub.publish(twist)

    rospy.loginfo("Wall found and aligned using sector-based analysis.")
    return FindWallResponse(wallfound=True)

# Main execution 
if __name__ == '__main__':
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
