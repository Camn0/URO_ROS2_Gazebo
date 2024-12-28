#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Movement Control Node Initialized')

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

