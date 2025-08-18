#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from wall_follower_interfaces.action import OdomRecord
import math
import time

class OdomRecorderNode(Node):
    def __init__(self):
        super().__init__('odom_recorder_node')
        
        # Action server
        self.action_server = ActionServer(
            self, OdomRecord, '/record_odom', self.record_odom_callback)
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Odometry tracking variables
        self.current_odom = None
        self.first_odom = None
        self.last_odom = Point()
        self.odom_record = []
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.recording = False
        self.last_record_time = 0.0
        
        self.get_logger().info("Odometry Recorder Node started")
    
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.current_odom = msg
        
        if self.recording and self.first_odom is None:
            # Store the first odometry reading
            self.first_odom = Point()
            self.first_odom.x = msg.pose.pose.position.x
            self.first_odom.y = msg.pose.pose.position.y
            self.first_odom.z = msg.pose.pose.position.z
            
            # Initialize tracking variables
            self.last_x = self.first_odom.x
            self.last_y = self.first_odom.y
            self.last_record_time = time.time()
            
            self.get_logger().info(f"First odom recorded: ({self.first_odom.x:.2f}, {self.first_odom.y:.2f})")
    
    async def record_odom_callback(self, goal_handle):
        """Action server callback for recording odometry"""
        self.get_logger().info("Started recording odometry")
        
        # Reset recording variables
        self.first_odom = None
        self.last_odom = Point()
        self.odom_record = []
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.recording = True
        self.last_record_time = time.time()
        
        # Feedback message
        feedback_msg = OdomRecord.Feedback()
        
        rate = self.create_rate(10)  # 10 Hz checking rate
        
        while rclpy.ok() and self.recording:
            current_time = time.time()
            
            # Check if we have odometry data
            if self.current_odom is None:
                await rate.sleep()
                continue
            
            # Update last_odom with current position
            self.last_odom.x = self.current_odom.pose.pose.position.x
            self.last_odom.y = self.current_odom.pose.pose.position.y
            self.last_odom.z = self.current_odom.pose.pose.position.z
            
            # Record point every second
            if current_time - self.last_record_time >= 1.0:
                # Create a copy of the current position
                point = Point()
                point.x = self.last_odom.x
                point.y = self.last_odom.y
                point.z = self.last_odom.z
                self.odom_record.append(point)
                
                # Calculate distance traveled since last recorded point
                if len(self.odom_record) > 1:
                    dx = self.last_odom.x - self.last_x
                    dy = self.last_odom.y - self.last_y
                    distance = math.sqrt(dx*dx + dy*dy)
                    self.total_distance += distance
                
                # Update last position
                self.last_x = self.last_odom.x
                self.last_y = self.last_odom.y
                self.last_record_time = current_time
                
                self.get_logger().info(f"Recorded point: ({self.last_odom.x:.2f}, {self.last_odom.y:.2f}), Total distance: {self.total_distance:.2f}m")
            
            # Check if robot returned to starting position (within 5cm)
            if self.first_odom is not None:
                dx = self.last_odom.x - self.first_odom.x
                dy = self.last_odom.y - self.first_odom.y
                distance_to_start = math.sqrt(dx*dx + dy*dy)
                
                if distance_to_start < 0.05:  # Less than 5cm
                    self.get_logger().info(f"Robot returned to starting position! Distance to start: {distance_to_start:.3f}m")
                    break
            
            # Publish feedback
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            
            await rate.sleep()
        
        # Prepare result
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        
        self.recording = False
        goal_handle.succeed()
        
        self.get_logger().info(f"Odometry recording completed. Total points: {len(self.odom_record)}, Total distance: {self.total_distance:.2f}m")
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
