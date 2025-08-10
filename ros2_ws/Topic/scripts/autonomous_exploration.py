#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('autonomous_exploration_node')

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

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state variables
        self.turning = False
        self.turn_direction = -0.5  # Default to turning right
        self.distance_from_origin = 0.0
        self.returning_to_origin = False
        self.current_position = {'x': 0.0, 'y': 0.0}  # Store the current x and y position
        self.yaw = 0.0  # Yaw angle of the robot

        self.get_logger().info("Autonomous Exploration Node Ready...")

    def laserscan_callback(self, msg):
        # If returning to the origin, skip normal exploration behavior
        if self.returning_to_origin:
            self.return_to_origin()
            return

        # Define the sectors
        sectors = {
            "Right_Rear": (0, 33),
            "Right": (34, 66),
            "Front_Right": (67, 100),
            "Front_Left": (101, 133),
            "Left": (134, 166),
            "Left_Rear": (167, 199)
        }

        # Initialize the minimum distances for each sector
        min_distances = {key: float('inf') for key in sectors.keys()}

        # Find the minimum distance in each sector
        for sector, (start_idx, end_idx) in sectors.items():
            # Ensure the index range is within bounds and not empty
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        # Define the threshold for obstacle detection
        obstacle_threshold = 0.8  # meters

        # Determine detected obstacles
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        # Determine suggested action based on detection
        action = Twist()

        # If obstacles are detected in both front sectors, continue turning
        if detections["Front_Left"] or detections["Front_Right"]:
            if not self.turning:
                # Start turning if not already turning
                self.turning = True
                self.turn_direction = -0.5  # Turning right
            action.angular.z = self.turn_direction  # Continue turning
            self.get_logger().info('Obstacle ahead, turning to clear path.')
        else:
            self.turning = False  # Stop turning when the front is clear
            # Priority 2: Side detections
            if detections["Left"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = -0.3  # Slight right turn
                self.get_logger().info('Obstacle on the left, turning slightly right.')
            elif detections["Right"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = 0.3  # Slight left turn
                self.get_logger().info('Obstacle on the right, turning slightly left.')
            # Priority 3: Rear detections
            elif detections["Right_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the right rear, moving forward.')
            elif detections["Left_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the left rear, moving forward.')
            else:
                action.linear.x = 0.5  # Move forward
                self.get_logger().info('No obstacles, moving forward.')

        # Publish the action command
        self.publisher_.publish(action)

    def odom_callback(self, msg):
        # Extract the x, y coordinates from the odometry message
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        # Calculate the distance from the origin (0,0)
        self.distance_from_origin = math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
        self.get_logger().info(f'Distance from origin: {self.distance_from_origin:.2f} meters')

        # Calculate the yaw (orientation around the z-axis)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # If the distance exceeds PERIMETER_DIST meters, initiate return to origin behavior
        PERIMETER_DIST = 5.0
        if self.distance_from_origin > PERIMETER_DIST:
            self.returning_to_origin = True
        elif self.distance_from_origin <= PERIMETER_DIST and self.returning_to_origin:
            # If the rover is back within 7 meters of the origin, resume normal operation
            self.returning_to_origin = False
            self.get_logger().info('Within 7 meters of origin, resuming normal operation.')

    def return_to_origin(self):
        action = Twist()

        # Calculate the desired angle to the origin
        desired_yaw = math.atan2(-self.current_position['y'], -self.current_position['x'])

        # Calculate the difference between current yaw and desired yaw
        yaw_error = desired_yaw - self.yaw

        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # If the yaw error is significant, rotate towards the origin
        if abs(yaw_error) > 0.1:  # 0.1 radians threshold for orientation
            action.angular.z = 0.5 if yaw_error > 0 else -0.5
            self.get_logger().info(f'Turning towards origin. Yaw error: {yaw_error:.2f}')
        else:
            # If oriented towards the origin, move forward
            action.linear.x = 0.5
            self.get_logger().info('Heading towards origin.')

        # Publish the action command
        self.publisher_.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()