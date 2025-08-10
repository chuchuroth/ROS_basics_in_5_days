# Import the custom TextRecognition service interface
from custom_interfaces.srv import TextRecognition
# Import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class TextRecognitionClientCustom(Node):

    def __init__(self):
        # Initialize the node with the name 'text_recognition_client_custom'
        super().__init__('text_recognition_client_custom')
        
        # Create the Service Client object
        # This defines the name ('/text_recognition_service_custom') and type (TextRecognition) of the Service Server to connect to.
        name_service = '/text_recognition_service_custom'
        self.client = self.create_client(TextRecognition, name_service)
        
        # Wait for the service to be available (checks every second)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service '+name_service+' not available, waiting again...')

        # Create an empty TextRecognition request
        self.req = TextRecognition.Request()

    def send_request(self, label):
        # Set the request label
        self.req.label = label

        # Send the request asynchronously
        self.future = self.client.call_async(self.req)


def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)
    
    # Declare the node constructor
    client = TextRecognitionClientCustom()

    # Label to search for (can be changed based on requirements)
    label_to_find = "FOOD"
    
    # Run the send_request() method
    client.send_request(label_to_find)

    while rclpy.ok():
        # Spin once to check for a service response
        rclpy.spin_once(client)
        
        if client.future.done():
            try:
                # Check if a response from the service was received
                response = client.future.result()
            except Exception as e:
                # Log any exceptions
                client.get_logger().info(f'Service call failed: {e}')
            else:
                # Log the service response
                client.get_logger().info(f'Success: {response.success}, Bounding Box: {response.start_x}, {response.start_y}, {response.end_x}, {response.end_y}')
            break

    # Destroy the client node
    client.destroy_node()
    
    # Shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()