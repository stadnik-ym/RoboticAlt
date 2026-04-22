import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import LaserScan
import time

class LD06Node(Node):
    def __init__(self):
        super().__init__('ld06_node')

        # Параметри
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Serial
        self.ser = serial.Serial(port, baud, timeout=1)

        # Publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(f"LD06 started on {port} @ {baud}")

        # Буфер
        self.buffer = []

        # Таймер
        self.timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        try:
            data = self.ser.read(100)
            if data:
                self.buffer.extend(data)
                self.parse_packets()
        except Exception as e:
            self.get_logger().error(str(e))

    def parse_packets(self):
        while len(self.buffer) >= 47:
            # шукаємо заголовок
            if self.buffer[0] != 0x54:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:47]
            self.buffer = self.buffer[47:]

            self.process_packet(packet)

    def process_packet(self, packet):
        # кількість точок
        count = packet[1]

        # початковий кут
        start_angle = (packet[3] << 8 | packet[2]) / 100.0

        distances = []
        angles = []

        for i in range(count):
            offset = 4 + i * 3
            dist = packet[offset] | (packet[offset + 1] << 8)
            angle = start_angle + i * 360.0 / count

            distances.append(dist / 1000.0)  # мм → м
            angles.append(math.radians(angle))

        self.publish_scan(angles, distances)

    def publish_scan(self, angles, distances):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"

        msg.angle_min = min(angles)
        msg.angle_max = max(angles)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(angles)

        msg.time_increment = 0.0
        msg.scan_time = 0.1

        msg.range_min = 0.02
        msg.range_max = 8.0

        msg.ranges = distances

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LD06Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
