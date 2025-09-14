#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MovementControlNode(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Dua subscriber, satu untuk linear (dari pid_y) dan satu untuk angular (dari pid_x)
        self.linear_sub = self.create_subscription(
            Float64,
            '/linear_correction',
            self.linear_callback,
            10
        )
        self.angular_sub = self.create_subscription(
            Float64,
            '/angular_correction',
            self.angular_callback,
            10
        )

        # Variabel untuk menyimpan nilai koreksi terbaru
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        self.last_correction_time = self.get_clock().now()

        # Parameter kecepatan pencarian (saat objek hilang)
        self.declare_parameter("search_angular_velocity", 0.8) # Menggunakan 0.8 yang lebih cepat
        self.search_speed = self.get_parameter("search_angular_velocity").value

        # Timer ini adalah loop kontrol utama. Ia berjalan 20x per detik (50ms)
        self.create_timer(0.05, self.control_loop)

    def linear_callback(self, msg):
        # Simpan koreksi linear dan perbarui timestamp
        self.latest_linear_x = msg.data
        self.last_correction_time = self.get_clock().now()

    def angular_callback(self, msg):
        # Simpan koreksi angular dan perbarui timestamp
        self.latest_angular_z = msg.data
        self.last_correction_time = self.get_clock().now()

    def control_loop(self):
        current_time = self.get_clock().now()
        time_diff_sec = (current_time - self.last_correction_time).nanoseconds / 1e9
        
        twist = Twist()

        if time_diff_sec > 1.0:
            # Failsafe: Jika tidak ada pesan koreksi (deteksi) selama > 1 detik,
            # berarti objek hilang. Berhenti maju dan berputar untuk mencari.
            twist.linear.x = 0.0
            twist.angular.z = self.search_speed
            self.get_logger().info('No detection. Searching...')
        else:
            # Objek terdeteksi: Terapkan koreksi PID yang diterima secara LANGSUNG.
            twist.linear.x = self.latest_linear_x
            twist.angular.z = self.latest_angular_z
            self.get_logger().info(f'Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')

        # Publikasikan perintah gerakan
        self.publisher_.publish(twist)


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
