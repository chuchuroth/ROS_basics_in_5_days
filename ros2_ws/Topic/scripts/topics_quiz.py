#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('topics_quiz_node')

        # Subscriber to LaserScan
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber to Odometry
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber to NASA mission commands
        self.subscriber_mission = self.create_subscription(
            String,
            '/nasa_mission',
            self.mission_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for rover status
        self.status_publisher_ = self.create_publisher(String, 'mars_rover_status', 10)

        # Initialize state variables
        self.current_goal = None
        self.goal_tolerance = 0.1
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0  # Yaw angle of the robot
        self.mission_active = False
        self.home_position = {'x': 0.0, 'y': 0.0}
        self.pickup_position = {'x': -2.342, 'y': 2.432}

        self.get_logger().info("Autonomous Exploration Node Ready...")

    def laserscan_callback(self, msg):
        # Define sectors and minimum distances for obstacle detection
        sectors = {
            "Front_Left": (100, 140),
            "Front_Right": (20, 60),
            "Left": (141, 179),
            "Right": (0, 19)
        }

        min_distances = {key: float('inf') for key in sectors}
        for sector, (start_idx, end_idx) in sectors.items():
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        obstacle_threshold = 0.8  # meters
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        # If a mission is active, navigate while avoiding obstacles
        if self.mission_active and self.current_goal:
            self.navigate_to_goal(detections)

    def odom_callback(self, msg):
        # Update the current position and yaw from odometry
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # Check if the goal has been reached
        if self.current_goal:
            distance_to_goal = math.sqrt(
                (self.current_position['x'] - self.current_goal['x'])**2 +
                (self.current_position['y'] - self.current_goal['y'])**2
            )
            if distance_to_goal < self.goal_tolerance:
                self.stop_robot()
                self.mission_active = False
                self.current_goal = None
                self.get_logger().info("Goal reached. Robot stopped.")
                self.status_publisher_.publish(String(data='goal-reached'))

    def mission_callback(self, msg):
        # Handle mission commands
        if msg.data == "Go-Home":
            self.get_logger().info("Received 'Go-Home' command.")
            if self.is_at_position(self.home_position):
                self.get_logger().info("Already at home. No need to move.")
                self.stop_robot()
                self.status_publisher_.publish(String(data='already-at-home'))
            else:
                self.get_logger().info("Moving to home position (0, 0).")
                self.current_goal = self.home_position
                self.mission_active = True

        elif msg.data == "Go-Pickup":
            self.get_logger().info("Received 'Go-Pickup' command.")
            if self.is_at_position(self.pickup_position):
                self.get_logger().info("Already at pickup. No need to move.")
                self.stop_robot()
                self.status_publisher_.publish(String(data='already-at-pickup'))
            else:
                self.get_logger().info("Moving to pickup position (5.0, -3.0).")
                self.current_goal = self.pickup_position
                self.mission_active = True

    def is_at_position(self, target_position):
        """Check if the robot is within the tolerance of a target position."""
        distance_to_target = math.sqrt(
            (self.current_position['x'] - target_position['x'])**2 +
            (self.current_position['y'] - target_position['y'])**2
        )
        return distance_to_target < self.goal_tolerance

    def navigate_to_goal(self, detections):
        # Calculate the desired yaw to the goal
        goal_x = self.current_goal['x']
        goal_y = self.current_goal['y']
        desired_yaw = math.atan2(goal_y - self.current_position['y'], goal_x - self.current_position['x'])
        yaw_error = desired_yaw - self.yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        action = Twist()

        # Adjust this threshold for smoother turning
        yaw_threshold = 0.2
        turning_speed = 0.6
        forward_speed = 0.2

        # Obstacle avoidance logic
        if detections["Front_Left"] and detections["Front_Right"]:
            # If both front sectors detect obstacles, stop moving forward and turn
            action.angular.z = -turning_speed if yaw_error > 0 else turning_speed
            action.linear.x = 0.0
            self.get_logger().info('Obstacle ahead in both front sectors, turning.')
        elif detections["Front_Left"]:
            # If obstacle on the front left, turn slightly to the right
            action.angular.z = -turning_speed
            action.linear.x = forward_speed * 0.5  # Slow down while turning
            self.get_logger().info('Obstacle detected on front left, turning right.')
        elif detections["Front_Right"]:
            # If obstacle on the front right, turn slightly to the left
            action.angular.z = turning_speed
            action.linear.x = forward_speed * 0.5  # Slow down while turning
            self.get_logger().info('Obstacle detected on front right, turning left.')
        elif abs(yaw_error) > yaw_threshold:
            # If the yaw error is large, turn towards the goal
            action.angular.z = turning_speed if yaw_error > 0 else -turning_speed
            action.linear.x = forward_speed * 0.5  # Move slowly while adjusting direction
            self.get_logger().info(f'Adjusting yaw to align with goal. Yaw error: {yaw_error:.2f}')
        else:
            # No obstacles, proceed towards the goal
            action.linear.x = forward_speed
            action.angular.z = 0.0
            self.get_logger().info('No obstacles, moving towards goal.')

        # Publish the movement command
        self.publisher_.publish(action)

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()

    # Detect the robot's initial position and decide where to move
    rclpy.spin_once(node)  # Process initial odometry
    if not node.is_at_position(node.home_position) and not node.is_at_position(node.pickup_position):
        node.get_logger().info("Initial position not at home or pickup. Moving to home position.")
        node.current_goal = node.home_position
        node.mission_active = True

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
