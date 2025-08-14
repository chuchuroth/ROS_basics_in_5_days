#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_wall_following.srv import FindWall, FindWallResponse
import math 


class FindWallServer:
    def __init__(self):
        rospy.init_node('find_wall_server')

        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.srv = rospy.Service('find_wall', FindWall, self.handle_find_wall)
        rospy.loginfo("Find_wall service ready.")
        rospy.spin()

    def scan_callback(self, msg):
        self.scan = msg

    def handle_find_wall(self, req):
        # wait for scan data
        while not self.scan:
            rospy.sleep(0.1)

        rate = rospy.Rate(10)
        twist = Twist()
        
        # Calculate indices based on laser configuration
        num_rays = len(self.scan.ranges)
        front_idx = num_rays // 2  # Front of the robot
        right_side_idx = 0  # Ray 0 = right side of robot
        
        rospy.loginfo(f"Laser has {num_rays} rays. Front index: {front_idx}, Right side index (ray 0): {right_side_idx}")

        # 1. Point towards nearest wall
        rospy.loginfo("Step 1: Rotating to face nearest wall")
        while not rospy.is_shutdown():
            ranges = list(self.scan.ranges)
            # Filter out invalid readings (inf, nan, 0)
            filtered = [(i, r) for i, r in enumerate(ranges) 
                       if r > 0 and r != float('inf') and not math.isnan(r)]
            
            if not filtered:
                rospy.logwarn("No valid laser readings!")
                continue
                
            min_idx, min_dist = min(filtered, key=lambda x: x[1])
            rospy.loginfo(f"Closest obstacle at index {min_idx}, distance {min_dist:.2f}m")
            
            # Check if we're already facing the wall (front ray is shortest)
            front_dist = ranges[front_idx] if ranges[front_idx] > 0 and ranges[front_idx] != float('inf') else float('inf')
            
            # Allow some tolerance (within 5 indices of front)
            if abs(min_idx - front_idx) <= 5 or front_dist < min_dist + 0.1:
                rospy.loginfo("Now facing the nearest wall")
                break
            
            # Determine rotation direction
            # If min_idx < front_idx, we need to rotate right (negative angular.z)
            # If min_idx > front_idx, we need to rotate left (positive angular.z)
            if min_idx < front_idx:
                twist.angular.z = -0.3  # Rotate right
            else:
                twist.angular.z = 0.3   # Rotate left
                
            twist.linear.x = 0.0
            self.pub.publish(twist)
            rate.sleep()

        # Stop rotation
        twist = Twist()
        self.pub.publish(twist)
        rospy.sleep(0.5)

        # 2. Move forward until 0.3m from wall
        rospy.loginfo("Step 2: Moving forward to wall")
        while not rospy.is_shutdown():
            ranges = list(self.scan.ranges)
            front_dist = ranges[front_idx]
            
            if front_dist <= 0 or front_dist == float('inf') or math.isnan(front_dist):
                rospy.logwarn("Invalid front distance reading!")
                break
                
            rospy.loginfo(f"Front distance: {front_dist:.2f}m")
            
            if front_dist <= 0.3:
                rospy.loginfo("Reached 0.3m from wall")
                break
                
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rate.sleep()
            
        # Stop movement
        twist = Twist()
        self.pub.publish(twist)
        rospy.sleep(0.5)

        # 3. Rotate until right side (ray 270) is pointing at wall
        rospy.loginfo("Step 3: Rotating to align ray 145 (right side) with wall")
        target_idx = 145 if num_rays > 145 else 0  # Use ray 270 if available
        
        while not rospy.is_shutdown():
            ranges = list(self.scan.ranges)
            
            # Get distances for different directions
            if target_idx < len(ranges):
                right_dist = ranges[target_idx] if ranges[target_idx] > 0 and ranges[target_idx] != float('inf') and not math.isnan(ranges[target_idx]) else float('inf')
            else:
                rospy.logwarn(f"Ray {target_idx} not available, using ray 0")
                target_idx = 0
                right_dist = ranges[target_idx] if ranges[target_idx] > 0 and ranges[target_idx] != float('inf') and not math.isnan(ranges[target_idx]) else float('inf')
                
            front_dist = ranges[front_idx] if ranges[front_idx] > 0 and ranges[front_idx] != float('inf') and not math.isnan(ranges[front_idx]) else float('inf')
            
            rospy.loginfo(f"Ray {target_idx} (right side) distance: {right_dist:.2f}m, Front distance: {front_dist:.2f}m")
            
            # Find the current closest ray to determine if ray 270 is pointing at the wall
            filtered = [(i, r) for i, r in enumerate(ranges) 
                       if r > 0 and r != float('inf') and not math.isnan(r)]
            
            if not filtered:
                rospy.logwarn("No valid laser readings!")
                continue
                
            min_idx, min_dist = min(filtered, key=lambda x: x[1])
            
            # Check if ray 270 is now pointing at the wall (is the closest or very close to closest)
            if target_idx == min_idx or abs(right_dist - min_dist) < 0.1:
                rospy.loginfo(f"Ray {target_idx} now aligned with wall (distance: {right_dist:.2f}m)")
                break
            
            # Rotate left to bring ray 0 toward wall
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            self.pub.publish(twist)
            rate.sleep()

        # Stop rotation
        twist = Twist()
        self.pub.publish(twist)

        rospy.loginfo("Wall found and aligned.")
        return FindWallResponse(wallfound=True)

if __name__ == '__main__':
    try:
        FindWallServer()
    except rospy.ROSInterruptException:
        pass
