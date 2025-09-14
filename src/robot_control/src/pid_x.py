#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDX(Node):
    def __init__(self):
        super().__init__('pid_x')
        self.sub = self.create_subscription(Float64, '/object_error_x', self.callback, 10)
        self.pub = self.create_publisher(Float64, '/angular_correction', 10)
        
        # PID parameters for angular correction
        # Error (piksel) kita maks sekitar 320. Output (rad/s) maks sekitar 1.0.
        # Kp harus sekitar 1.0/320 = ~0.003. Kita setel Ki dan Kd ke 0 dulu.
        # Kita buat kemudi 3x lebih kuat, dan tambahkan rem (Kd) yang kuat untuk stabilitas
        # Menaikkan Kp untuk kemudi yang lebih cepat, dan sedikit Kd untuk meredamnya
        # Sedikit meningkatkan kecepatan dan pengereman kemudi agar tetap seimbang
        self.kp = 0.02
        self.ki = 0.0
        self.kd = 0.04
        self.integral = 0.0
        self.prev_error = 0.0

    def callback(self, msg):
        error = msg.data
        self.integral += error
        derivative = error - self.prev_error
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        cmd = Float64()
        cmd.data = correction
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDX()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
