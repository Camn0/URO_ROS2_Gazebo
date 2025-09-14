#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
   		 Float64,
   		 '/object_error',  # <-- SESUAIKAN DENGAN PUBLISHER 
   		 self.error_callback,
    		10
	)
        self.publisher_ = self.create_publisher(Float64, '/angular_correction', 10)
        
        # Declare PID parameters so they can be tuned via ROS parameters
        self.declare_parameter("kp", 0.005)
        self.declare_parameter("ki", 0.0001)
        self.declare_parameter("kd", 0.001)
        
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value
        self.pid = PIDController(kp, ki, kd)

    def error_callback(self, msg):
        error = msg.data
        correction = self.pid.compute(error)
        correction_msg = Float64()
        correction_msg.data = correction
        self.publisher_.publish(correction_msg)
        self.get_logger().info(f'Error: {error:.2f} | Angular correction: {correction:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PID Controller node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
