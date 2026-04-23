import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import LaserScan


class LD06Node(Node):
    def __init__(self):
        super().__init__('ld06_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=1)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(f"LD06 started on {port} @ {baud}")

        self.buffer = bytearray()
        self.timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        try:
            data = self.ser.read(256)
            if data:
                self.buffer.extend(data)
                self.parse_packets()
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def parse_packets(self):
        PACKET_SIZE = 47

        while len(self.buffer) >= PACKET_SIZE:
            # шукаємо заголовок
            if self.buffer[0] != 0x54:
                self.buffer.pop(0)
                continue

            # другий байт має бути 0x2C
            if self.buffer[1] != 0x2C:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:PACKET_SIZE]
            del self.buffer[:PACKET_SIZE]

            self.process_packet(packet)

    def process_packet(self, packet):
        try:
            # Формат LD06:
            # [0]  header = 0x54
            # [1]  ver_len = 0x2C
            # [2:4] speed
            # [4:6] start_angle
            # [6:42] 12 points * 3 bytes (dist_l, dist_h, confidence)
            # [42:44] end_angle
            # [44:46] timestamp
            # [46] crc

            POINTS_PER_PACKET = 12

            start_angle = ((packet[5] << 8) | packet[4]) / 100.0
            end_angle = ((packet[43] << 8) | packet[42]) / 100.0

            # якщо кут перейшов через 360
            if end_angle < start_angle:
                end_angle += 360.0

            if POINTS_PER_PACKET > 1:
                step = (end_angle - start_angle) / (POINTS_PER_PACKET - 1)
            else:
                step = 0.0

            distances = []
            angles = []

            for i in range(POINTS_PER_PACKET):
                offset = 6 + i * 3

                dist_mm = packet[offset] | (packet[offset + 1] << 8)
                # packet[offset + 2] = confidence, поки не використовуємо

                angle_deg = start_angle + step * i
                angle_deg = angle_deg % 360.0

                distances.append(dist_mm / 1000.0)   # мм -> м
                angles.append(math.radians(angle_deg))

            self.publish_scan(angles, distances)

        except Exception as e:
            self.get_logger().error(f"Packet parse error: {e}")

    def publish_scan(self, angles, distances):
        if not angles or not distances:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"

        msg.angle_min = min(angles)
        msg.angle_max = max(angles)

        if len(angles) > 1:
            msg.angle_increment = (msg.angle_max - msg.angle_min) / (len(angles) - 1)
        else:
            msg.angle_increment = 0.0

        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.02
        msg.range_max = 8.0
        msg.ranges = distances

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LD06Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
