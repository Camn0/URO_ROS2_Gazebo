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
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher BARU: Satu untuk error horizontal (X/Angular), satu untuk error jarak (Y/Linear)
        self.error_x_publisher = self.create_publisher(Float64, '/object_error_x', 10)
        self.error_y_publisher = self.create_publisher(Float64, '/object_error_y', 10)
        
        self.bridge = CvBridge()

        # Deklarasi parameter (termasuk setpoint Area target baru)
        self.declare_parameter("lower_green_h", 40)
        self.declare_parameter("lower_green_s", 40)
        self.declare_parameter("lower_green_v", 40)
        self.declare_parameter("upper_green_h", 80)
        self.declare_parameter("upper_green_s", 255)
        self.declare_parameter("upper_green_v", 255)
        
        # Parameter ini menentukan 'seberapa dekat' robot harus berhenti.
        # Anda perlu MENYESUAIKAN (tuning) nilai ini. 
        # Jika robot berhenti terlalu jauh, buat angkanya LEBIH BESAR.
        # Jika robot menabrak, buat angkanya LEBIH KECIL.
        self.declare_parameter("target_area_setpoint", 10000.0)
        self.target_area = self.get_parameter("target_area_setpoint").value

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
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            current_area = cv2.contourArea(largest_contour) # Hitung area kontur
            
            x, y, w, h = cv2.boundingRect(largest_contour)
            object_center_x = x + w // 2
            image_center_x = cv_image.shape[1] // 2

            # 1. Hitung Error Angular (Horizontal)
            error_x = float(image_center_x - object_center_x)
            
            # 2. Hitung Error Jarak (Area)
            # Ini akan positif jika jauh (area < target) -> menyuruh PID maju
            # Ini akan negatif jika terlalu dekat (area > target) -> menyuruh PID mundur
            error_y = float(self.target_area - current_area)

            # Publikasikan kedua error
            msg_x = Float64()
            msg_x.data = error_x
            self.error_x_publisher.publish(msg_x)
            
            msg_y = Float64()
            msg_y.data = error_y
            self.error_y_publisher.publish(msg_y)
            
            self.get_logger().info(f"Object Found. Area Error: {error_y:.2f}, Angular Error: {error_x:.2f}")

        else:
            # Jika tidak ada yang terlihat, jangan publikasikan apa pun.
            # Ini akan memicu failsafe "berputar mencari" di movement_control.
            self.get_logger().info("Object not detected.")
            pass 

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
