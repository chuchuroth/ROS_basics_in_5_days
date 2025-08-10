import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from mars_rover_tasks.text_recog import TextRecognition


class TextRecognitionNode(Node):
    def __init__(self):
        super().__init__('text_recognition_node')
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
    
    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Perform text recognition
        results = self.text_recognizer.recognize_text(cv_image)
        # Print or log the results
        for (start_x, start_y, end_x, end_y), text in results:
            cleaned_text = text.rstrip(".\n\x0c")
            position = str(start_x)+"-"+str(start_y)+"-"+str(end_x)+"-"+str(end_y)
            self.get_logger().info(f'Result: {cleaned_text}, ({position})')
            # You can also publish or process the recognized text further


def main(args=None):
    rclpy.init(args=args)
    node = TextRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()