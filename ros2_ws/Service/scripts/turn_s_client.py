import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn

def main(args=None):
    rclpy.init(args=args)

    node = Node('turn_s_client')
    client = node.create_client(Turn, 'turn')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /turn not available, waiting...')

    request = Turn.Request()
    request.direction = "right"
    request.angular_velocity = 0.2  # rad/s
    request.time = 10.0  # Turn for 10 seconds

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Result: %s' % future.result().success)
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
