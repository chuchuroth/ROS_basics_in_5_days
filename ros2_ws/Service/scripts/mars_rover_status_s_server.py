#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn
import time

class TurnServiceServer(Node):
    def __init__(self):
        super().__init__('turn_service_server')
        
        # Create service server
        self.srv = self.create_service(Turn, '/turn', self.turn_callback)
        
        # Create publisher for robot movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Turn Service Server Ready.')

    def turn_callback(self, request, response):
        """
        Service callback to handle turn requests
        """
        self.get_logger().info(f'Incoming request: direction={request.direction}, '
                              f'angular_velocity={request.angular_velocity}, '
                              f'time={request.time}')

        try:
            # Validate the direction parameter
            if request.direction.lower() not in ['left', 'right']:
                self.get_logger().error(f'Invalid direction: {request.direction}. Must be "left" or "right"')
                response.success = False
                return response

            # Validate angular velocity (should be positive)
            if request.angular_velocity <= 0:
                self.get_logger().error(f'Invalid angular velocity: {request.angular_velocity}. Must be positive')
                response.success = False
                return response

            # Validate time (should be positive)
            if request.time <= 0:
                self.get_logger().error(f'Invalid time: {request.time}. Must be positive')
                response.success = False
                return response

            # Create twist message for robot movement
            twist = Twist()
            
            # Set angular velocity based on direction
            if request.direction.lower() == 'left':
                twist.angular.z = abs(request.angular_velocity)  # Positive for left (counter-clockwise)
            else:  # right
                twist.angular.z = -abs(request.angular_velocity)  # Negative for right (clockwise)
            
            # Set linear velocity to 0 (pure rotation)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0

            self.get_logger().info(f'Starting to turn {request.direction} at {request.angular_velocity} rad/s for {request.time} seconds')

            # Use more precise timing with threading or timer
            import threading
            
            # Flag to control publishing
            self.turning = True
            
            # Create a timer to stop turning after specified time
            def stop_turning():
                self.turning = False
                stop_twist = Twist()
                self.publisher_.publish(stop_twist)
                self.get_logger().info('Turn completed - robot stopped')
            
            # Set timer to stop after specified duration
            timer = threading.Timer(request.time, stop_turning)
            timer.start()
            
            # Publish twist commands at high frequency until timer stops
            rate = self.create_rate(20)  # 20 Hz publishing rate for better control
            start_time = self.get_clock().now()
            
            while self.turning and rclpy.ok():
                self.publisher_.publish(twist)
                try:
                    rate.sleep()
                except:
                    break
                    
                # Double-check timing as backup
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed >= request.time:
                    break

            # Ensure robot stops
            self.turning = False
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            self.publisher_.publish(stop_twist)  # Send twice to ensure it stops
            
            # Clean up timer
            if timer.is_alive():
                timer.cancel()

            self.get_logger().info(f'Turn operation completed successfully')
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Error during turn operation: {str(e)}')
            # Ensure robot stops in case of error
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    
    turn_service_server = TurnServiceServer()
    
    try:
        rclpy.spin(turn_service_server)
    except KeyboardInterrupt:
        turn_service_server.get_logger().info('Service server interrupted')
    finally:
        # Send stop command before shutdown
        stop_twist = Twist()
        turn_service_server.publisher_.publish(stop_twist)
        turn_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
