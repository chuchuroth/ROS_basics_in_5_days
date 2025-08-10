import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry  # Import Odometry message type
from custom_interfaces.msg import RoverEvents  # Import RoverEvents custom message type
from cv_bridge import CvBridge
import cv2
from mars_rover_tasks.plant_detector import PlantDetector  # Import the AI class

class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Initialize the PlantDetector
        path_to_model = "/home/user/ros2_ws/src/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth"
        self.plant_detector = PlantDetector(model_path=path_to_model)
        
        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/leo/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscribe to the odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning

        # Initialize the Publisher for rover events
        self.publisher_ = self.create_publisher(RoverEvents, '/mars_rover_events', 10)
        
        # Variable to store the latest odometry message
        self.current_odom = None

    def odom_callback(self, msg):
        # Store the current odometry data
        self.current_odom = msg

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Use the PlantDetector to make a prediction
        prediction = self.plant_detector.predict(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Create a RoverEvents message
        rover_event = RoverEvents()
        
        # Determine the result message based on the prediction
        if prediction > 0.5:
            rover_event.info.data = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(rover_event.info.data)
            self.get_logger().warning("Publishing mars rover event...")
            rover_event.info.data = f"Plant detected with confidence: {prediction:.2f}"
            # If the odometry data is available, include the rover's location
            if self.current_odom:
                rover_event.rover_location = self.current_odom.pose.pose  # Copy the pose data from the odometry

            # Publish the RoverEvents message
            self.publisher_.publish(rover_event)
        else:
            rover_event.info.data = f"No plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(rover_event.info.data)
        


def main(args=None):
    rclpy.init(args=args)

    plant_detector_node = PlantDetectorNode()

    rclpy.spin(plant_detector_node)

    # Destroy the node explicitly (optional)
    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()