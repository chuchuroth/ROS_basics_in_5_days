#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.srv import TextRecognition
import cv2
from mars_rover_tasks.text_recog import TextRecognition as TextRecognitionModel

class TextRecognitionServiceCustom(Node):
    def __init__(self):
        super().__init__('text_recognition_service_custom')

        # Path to the EAST text detection model and parameters
        east_model_path = '/home/user/ros2_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb'
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0

        self.text_recognizer = TextRecognitionModel(east_model_path, min_confidence, width, height, padding)
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/leo/camera/image_raw',  # Change this to the correct image topic
            self.image_callback,
            1
        )

        # Create a service to handle text recognition requests
        name_service = '/text_recognition_service_custom'
        self.srv = self.create_service(TextRecognition, name_service, self.handle_text_recognition_request)

        # Variable to store the last detected text and bounding box
        self.last_detected_text = ''
        self.last_bounding_box = (0, 0, 0, 0)  # start_x, start_y, end_x, end_y

        self.get_logger().info(name_service + " CUSTOM Interface Service Server Ready...")

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Perform text recognition
        results = self.text_recognizer.recognize_text(cv_image)

        # Extract recognized text and bounding box coordinates
        if results:
            for (start_x, start_y, end_x, end_y), text in results:
                cleaned_text = text.rstrip(".\n\x0c")
                self.last_detected_text = cleaned_text
                self.last_bounding_box = (start_x, start_y, end_x, end_y)
                break  # Only keep the first recognized text
        else:
            self.last_detected_text = ''
            self.last_bounding_box = (0, 0, 0, 0)

        self.get_logger().info(f'Result: {self.last_detected_text}')

    def handle_text_recognition_request(self, request, response):
        detected_text = self.last_detected_text.upper()
        requested_label = request.label.upper()

        if detected_text == requested_label:
            response.success = True
            response.start_x, response.start_y, response.end_x, response.end_y = self.last_bounding_box
        else:
            response.success = False
            response.start_x = response.start_y = response.end_x = response.end_y = 0

        self.get_logger().info(f'Service called. Requested label: {request.label}, Detected text: {detected_text}, Success: {response.success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TextRecognitionServiceCustom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()