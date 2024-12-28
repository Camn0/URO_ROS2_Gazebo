#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.get_logger().info('Object Detection Node Initialized')

    def detect_white_object(self, frame):
        mask = cv2.inRange(frame, (200, 200, 200), (255, 255, 255))
        return cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

