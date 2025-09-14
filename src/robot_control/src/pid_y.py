#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDY(Node):
    def __init__(self):
        super().__init__('pid_y')
        self.sub = self.create_subscription(Float64, '/object_error_y', self.callback, 10)
        self.pub = self.create_publisher(Float64, '/linear_correction', 10)
        
        # PID parameters for distance control
        # Error (Area) kita sangat besar (misal 50.000), tapi output (speed) harus kecil (misal < 0.5)
        # Jadi Kp harus sangat kecil. Ki dan Kd disetel ke 0 untuk menstabilkan dulu.
        # Kita buat kecepatan maju 2x lebih kuat (peningkatan lebih kecil dari kemudi)
        # Kita tambahkan rem (Kd) yang kuat untuk mencegah overshoot#
	# Tuning untuk Setpoint 10.000 yang baru:
        # Kita perlu Kp yang jauh lebih tinggi untuk mendapatkan kecepatan dari error maks 8000 yang kecil.
        self.kp = 0.000125
        self.ki = 0.0
        self.kd = 0.003
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
    node = PIDY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
