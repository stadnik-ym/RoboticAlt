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

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baud, timeout=1)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.buffer = bytearray()

        self.get_logger().info(f'LD06 started on {port} @ {baud}')

        self.timer = self.create_timer(0.02, self.read_data)

    def read_data(self):
        data = self.ser.read(256)
        if data:
            self.buffer.extend(data)
            self.parse_packets()

    def parse_packets(self):
        PACKET_SIZE = 47

        while len(self.buffer) >= PACKET_SIZE:
            if self.buffer[0] != 0x54:
                self.buffer.pop(0)
                continue

            if self.buffer[1] != 0x2C:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:PACKET_SIZE]
            del self.buffer[:PACKET_SIZE]

            self.process_packet(packet)

    def process_packet(self, packet):
        POINTS = 12

        start_angle = ((packet[5] << 8) | packet[4]) / 100.0
        end_angle = ((packet[43] << 8) | packet[42]) / 100.0

        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / (POINTS - 1)

        angles = []
        distances = []

        for i in range(POINTS):
            offset = 6 + i * 3

            dist = packet[offset] | (packet[offset + 1] << 8)

            angle = start_angle + i * step
            angle = angle % 360.0

            angles.append(math.radians(angle))
            distances.append(dist / 1000.0)

        self.publish_scan(angles, distances)

    def publish_scan(self, angles, distances):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"

        msg.angle_min = min(angles)
        msg.angle_max = max(angles)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / max(len(angles) - 1, 1)

        msg.range_min = 0.02
        msg.range_max = 8.0

        msg.ranges = distances

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LD06Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
