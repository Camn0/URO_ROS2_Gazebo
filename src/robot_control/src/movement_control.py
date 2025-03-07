#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MovementControlNode(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Float64,
            '/angular_correction',
            self.correction_callback,
            10
        )
        # Declare parameters for linear velocity and angular scaling
        self.declare_parameter("linear_velocity", 0.2)
        self.declare_parameter("angular_scale", 1.0)
        self.linear_velocity = self.get_parameter("linear_velocity").value
        self.angular_scale = self.get_parameter("angular_scale").value

        self.last_correction_time = self.get_clock().now()
        # Create a timer to check if correction messages have stopped (to perform a search maneuver)
        self.create_timer(0.5, self.timer_callback)

    def correction_callback(self, msg):
        # Update the last received time for corrections
        self.last_correction_time = self.get_clock().now()
        # Scale the angular correction if needed
        angular_correction = msg.data * self.angular_scale
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_correction
        self.publisher_.publish(twist)
        self.get_logger().info(f'Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_correction_time).nanoseconds / 1e9
        # If no corrections are received for over one second, rotate to search for the object
        if time_diff > 1.0:
            twist = Twist()
            twist.linear.x = 0.0  # Stop forward motion
            twist.angular.z = 0.2 # Rotate slowly
            self.publisher_.publish(twist)
            self.get_logger().info('No recent correction. Rotating to search for the object...')

def main(args=None):
    rclpy.init(args=args)
    node = MovementControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Movement control node terminated')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
