#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
from mars_rover_tasks.text_recog import TextRecognition

class TextRecognitionService(Node):
    def __init__(self):
        super().__init__('text_recognition_service')

        # Path to the EAST text detection model and parameters
        east_model_path = '/home/user/ros2_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb'
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0
        
        self.text_recognizer = TextRecognition(east_model_path, min_confidence, width, height, padding)
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/leo/camera/image_raw',  # Change this to the correct image topic
            self.image_callback,
            1
        )

        # Create a service to handle text recognition requests
        name_service = '/text_recognition_service'
        self.srv = self.create_service(Trigger, name_service, self.handle_text_recognition_request)

        # Variable to store the last detected text
        self.last_detected_text = ''

        self.get_logger().info(name_service+" Service Server Ready...")

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Perform text recognition
        results = self.text_recognizer.recognize_text(cv_image)

        # Extract recognized text and store the last one found
        if results:
            for (start_x, start_y, end_x, end_y), text in results:
                cleaned_text = text.rstrip(".\n\x0c")
                self.last_detected_text = cleaned_text
                break  # Only keep the first recognized text
        else:
            self.last_detected_text = ''
        
        self.get_logger().info(f'Result: {self.last_detected_text}')

    def handle_text_recognition_request(self, request, response):
        # Respond with success = True if the detected text is "FOOD" or "WASTE"
        detected_text = self.last_detected_text.upper()

        if detected_text in ['FOOD', 'WASTE']:
            response.success = True
        else:
            response.success = False

        # Always return the detected text in the message element
        response.message = detected_text if detected_text else 'No text detected'

        self.get_logger().info(f'Service called. Detected text: {response.message}, Success: {response.success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TextRecognitionService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()