#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection')
        # Subscribe to the camera topic (ensure the topic name matches your sensor)
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        self.error_publisher = self.create_publisher(Float64, '/object_error', 10)
        self.bridge = CvBridge()

        # Declare HSV threshold parameters for green detection
        self.declare_parameter("lower_green_h", 40)
        self.declare_parameter("lower_green_s", 40)
        self.declare_parameter("lower_green_v", 40)
        self.declare_parameter("upper_green_h", 80)
        self.declare_parameter("upper_green_s", 255)
        self.declare_parameter("upper_green_v", 255)

        self.lower_green = np.array([
            self.get_parameter("lower_green_h").value,
            self.get_parameter("lower_green_s").value,
            self.get_parameter("lower_green_v").value
        ])
        self.upper_green = np.array([
            self.get_parameter("upper_green_h").value,
            self.get_parameter("upper_green_s").value,
            self.get_parameter("upper_green_v").value
        ])

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (using "rgb8" encoding)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Convert image from RGB to HSV and create a mask for green regions
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        error_msg = Float64()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            object_center = (x + w // 2, y + h // 2)
            image_center_x = cv_image.shape[1] / 2
            error = image_center_x - object_center[0]
            error_msg.data = error
            self.get_logger().info(f"Green object detected. Error: {error:.2f}")
        else:
            error_msg.data = 0.0
            self.get_logger().info("Green object not detected.")

        self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Object detection node terminated")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
