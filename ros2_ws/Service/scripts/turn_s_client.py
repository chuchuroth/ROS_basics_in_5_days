#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn
import sys

class TurnServiceClient(Node):
    def __init__(self):
        super().__init__('turn_service_client')
        
        # Create service client
        self.cli = self.create_client(Turn, '/turn')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Turn Service Client Ready.')

    def send_request(self, direction='right', angular_velocity=0.2, time_duration=10.0):
        """
        Send a turn request to the service server
        """
        request = Turn.Request()
        request.direction = direction
        request.angular_velocity = angular_velocity
        request.time = time_duration

        self.get_logger().info(f'Sending request: direction={direction}, '
                              f'angular_velocity={angular_velocity}, '
                              f'time={time_duration}')

        # Call the service
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    turn_service_client = TurnServiceClient()
    
    try:
        # Make the robot spin according to specifications:
        # - Direction: right
        # - Angular velocity: 0.2 rad/s
        # - Duration: 10 seconds
        response = turn_service_client.send_request(
            direction='right',
            angular_velocity=0.2,
            time_duration=10.0
        )
        
        if response.success:
            turn_service_client.get_logger().info('Turn request completed successfully! Client exiting...')
        else:
            turn_service_client.get_logger().error('Turn request failed! Client exiting...')
            
    except KeyboardInterrupt:
        turn_service_client.get_logger().info('Client interrupted')
    except Exception as e:
        turn_service_client.get_logger().error(f'Error: {str(e)}')
    finally:
        turn_service_client.destroy_node()
        rclpy.shutdown()
        
    # Exit the program after completing the service call
    exit(0)

if __name__ == '__main__':
    main()
