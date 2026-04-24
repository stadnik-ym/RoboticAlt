import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stop_dist', 0.05)
        self.declare_parameter('front_angle_deg', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        self.front_dist = float('inf')
        self.have_scan = False

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)

        self.get_logger().info(
            f'Lidar safety started: {self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'stop_dist={self.stop_dist}'
        )

    def scan_callback(self, msg):
        values = []
        angle = msg.angle_min

        for r in msg.ranges:
            angle_deg = math.degrees(angle)

            if -self.front_angle_deg <= angle_deg <= self.front_angle_deg:
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    values.append(r)

            angle += msg.angle_increment

        if values:
            values.sort()
            idx = max(0, min(len(values) - 1, int(len(values) * 0.1)))
            self.front_dist = values[idx]
            self.have_scan = True
        else:
            self.front_dist = float('inf')
            self.have_scan = False

    def cmd_callback(self, cmd):
        safe_cmd = Twist()

        obstacle_ahead = self.have_scan and self.front_dist < self.stop_dist
        wants_forward = cmd.linear.x > 0.0

        if obstacle_ahead and wants_forward:
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0

            self.get_logger().warn(
                f'SAFETY STOP: obstacle at {self.front_dist:.2f} m'
            )
        else:
            safe_cmd = cmd

        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
