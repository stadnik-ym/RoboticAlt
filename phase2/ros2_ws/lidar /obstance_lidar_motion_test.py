import math
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class State(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    FOLLOW_WALL = 4
    RETURN_HEADING = 5


class ObstacleAvoidanceSafetyNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_safety_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('mode', 'auto')  # auto | manual

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('manual_cmd_vel_topic', '/cmd_vel_manual')

        self.declare_parameter('forward_speed', 0.11)
        self.declare_parameter('turn_speed', 0.80)
        self.declare_parameter('wall_follow_speed', 0.10)

        self.declare_parameter('forward_angular_compensation', -0.07)

        self.declare_parameter('obstacle_dist', 0.50)
        self.declare_parameter('clear_dist', 0.80)
        self.declare_parameter('side_desired_dist', 0.35)
        self.declare_parameter('side_open_dist', 0.90)

        self.declare_parameter('manual_stop_dist', 0.45)

        self.declare_parameter('wall_kp', 1.80)
        self.declare_parameter('return_gain', 1.00)

        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('exit_confirm_cycles', 8)

        self.mode = str(self.get_parameter('mode').value).strip().lower()

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.manual_cmd_vel_topic = self.get_parameter('manual_cmd_vel_topic').value

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.wall_follow_speed = float(self.get_parameter('wall_follow_speed').value)

        self.forward_angular_comp = float(
            self.get_parameter('forward_angular_compensation').value
        )

        self.obstacle_dist = float(self.get_parameter('obstacle_dist').value)
        self.clear_dist = float(self.get_parameter('clear_dist').value)
        self.side_desired_dist = float(self.get_parameter('side_desired_dist').value)
        self.side_open_dist = float(self.get_parameter('side_open_dist').value)

        self.manual_stop_dist = float(self.get_parameter('manual_stop_dist').value)

        self.wall_kp = float(self.get_parameter('wall_kp').value)
        self.return_gain = float(self.get_parameter('return_gain').value)

        self.control_period = float(self.get_parameter('control_period').value)
        self.exit_confirm_cycles = int(self.get_parameter('exit_confirm_cycles').value)

        if self.mode not in ['auto', 'manual']:
            self.get_logger().warn(f'Unknown mode "{self.mode}", fallback to auto')
            self.mode = 'auto'

        # ---------------- ROS interfaces ----------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.manual_cmd_sub = self.create_subscription(
            Twist,
            self.manual_cmd_vel_topic,
            self.manual_cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(self.control_period, self.control_loop)

        # ---------------- State ----------------
        self.state = State.FORWARD
        self.latest_scan = None
        self.latest_manual_cmd = Twist()

        self.avoid_side = None
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.return_start_time = None

        self.exit_counter = 0
        self.last_log_state = None
        self.manual_blocked = False

        self.get_logger().info(f'Node started in mode: {self.mode}')
        self.get_logger().info(
            f'Forward compensation: x={self.forward_speed:.2f}, '
            f'z={self.forward_angular_comp:.2f}'
        )

    # =========================================================
    # Callbacks
    # =========================================================
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def manual_cmd_callback(self, msg: Twist):
        self.latest_manual_cmd = msg

    # =========================================================
    # Scan helpers
    # =========================================================
    def get_sector_min_distance(self, scan: LaserScan, angle_min_deg: float, angle_max_deg: float) -> float:
        if scan is None or not scan.ranges:
            return float('inf')

        result = []
        angle = scan.angle_min

        for r in scan.ranges:
            angle_deg = math.degrees(angle)

            if angle_min_deg <= angle_deg <= angle_max_deg:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    result.append(r)

            angle += scan.angle_increment

        if not result:
            return float('inf')

        result.sort()
        idx = max(0, min(len(result) - 1, int(len(result) * 0.1)))
        return result[idx]

    def get_regions(self):
        if self.latest_scan is None:
            return None

        scan = self.latest_scan

        return {
            'front': self.get_sector_min_distance(scan, -15.0, 15.0),
            'front_left': self.get_sector_min_distance(scan, 15.0, 60.0),
            'front_right': self.get_sector_min_distance(scan, -60.0, -15.0),
            'left': self.get_sector_min_distance(scan, 60.0, 100.0),
            'right': self.get_sector_min_distance(scan, -100.0, -60.0),
        }

    # =========================================================
    # Motion helpers
    # =========================================================
    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_twist(self, msg: Twist):
        self.cmd_pub.publish(msg)

    def publish_forward_compensated(self, linear_x: float, extra_angular_z: float = 0.0):
        angular = self.forward_angular_comp + extra_angular_z
        angular = max(min(angular, 1.5), -1.5)
        self.publish_cmd(linear_x, angular)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def log_state_once(self, text: str):
        if self.last_log_state != text:
            self.get_logger().info(text)
            self.last_log_state = text

    # =========================================================
    # Manual safety mode
    # =========================================================
    def handle_manual_mode(self, regions):
        front = regions['front']
        manual_cmd = self.latest_manual_cmd

        # Запобіжник: блокуємо тільки коли є спроба їхати вперед
        wants_forward = manual_cmd.linear.x > 0.0

        if wants_forward and front < self.manual_stop_dist:
            if not self.manual_blocked:
                self.get_logger().warn(
                    f'MANUAL SAFETY STOP: obstacle at {front:.2f} m'
                )
                self.manual_blocked = True

            self.stop_robot()
            return

        if self.manual_blocked:
            self.get_logger().info('Manual path clear again, commands are allowed.')
            self.manual_blocked = False

        # Перешкоди нема -> пропускаємо ручну команду
        self.publish_twist(manual_cmd)

    # =========================================================
    # Auto mode
    # =========================================================
    def handle_auto_mode(self, regions):
        front = regions['front']
        left = regions['left']
        right = regions['right']
        front_left = regions['front_left']
        front_right = regions['front_right']

        # ---------------- FORWARD ----------------
        if self.state == State.FORWARD:
            self.log_state_once('State: FORWARD')

            if front < self.obstacle_dist:
                if left >= right:
                    self.avoid_side = 'left'
                    self.state = State.TURN_LEFT
                    self.turn_start_time = self.now_sec()
                    self.get_logger().info(
                        f'Obstacle ahead. Turning LEFT. '
                        f'front={front:.2f}, left={left:.2f}, right={right:.2f}'
                    )
                else:
                    self.avoid_side = 'right'
                    self.state = State.TURN_RIGHT
                    self.turn_start_time = self.now_sec()
                    self.get_logger().info(
                        f'Obstacle ahead. Turning RIGHT. '
                        f'front={front:.2f}, left={left:.2f}, right={right:.2f}'
                    )

                self.exit_counter = 0
                self.stop_robot()
                return

            self.publish_forward_compensated(self.forward_speed, 0.0)
            return

        # ---------------- TURN_LEFT ----------------
        if self.state == State.TURN_LEFT:
            self.log_state_once('State: TURN_LEFT')

            self.publish_cmd(0.03, self.turn_speed)

            if front > self.clear_dist and front_left > self.clear_dist * 0.8:
                self.turn_duration = self.now_sec() - self.turn_start_time
                self.state = State.FOLLOW_WALL
                self.exit_counter = 0
                self.get_logger().info(
                    f'Finished LEFT turn. Duration={self.turn_duration:.2f}s. Enter FOLLOW_WALL.'
                )
            return

        # ---------------- TURN_RIGHT ----------------
        if self.state == State.TURN_RIGHT:
            self.log_state_once('State: TURN_RIGHT')

            self.publish_cmd(0.03, -self.turn_speed)

            if front > self.clear_dist and front_right > self.clear_dist * 0.8:
                self.turn_duration = self.now_sec() - self.turn_start_time
                self.state = State.FOLLOW_WALL
                self.exit_counter = 0
                self.get_logger().info(
                    f'Finished RIGHT turn. Duration={self.turn_duration:.2f}s. Enter FOLLOW_WALL.'
                )
            return

        # ---------------- FOLLOW_WALL ----------------
        if self.state == State.FOLLOW_WALL:
            self.log_state_once(f'State: FOLLOW_WALL ({self.avoid_side})')

            if front < self.obstacle_dist * 0.8:
                if self.avoid_side == 'left':
                    self.publish_cmd(0.0, self.turn_speed)
                else:
                    self.publish_cmd(0.0, -self.turn_speed)
                self.exit_counter = 0
                return

            if self.avoid_side == 'left':
                side_dist = right
                error = self.side_desired_dist - side_dist
                wall_correction = -self.wall_kp * error
            else:
                side_dist = left
                error = self.side_desired_dist - side_dist
                wall_correction = self.wall_kp * error

            wall_correction = max(min(wall_correction, 1.0), -1.0)
            self.publish_forward_compensated(self.wall_follow_speed, wall_correction)

            if front > self.clear_dist and side_dist > self.side_open_dist:
                self.exit_counter += 1
            else:
                self.exit_counter = 0

            if self.exit_counter >= self.exit_confirm_cycles:
                self.state = State.RETURN_HEADING
                self.return_start_time = self.now_sec()
                self.get_logger().info(
                    f'Exit detected. Enter RETURN_HEADING. '
                    f'turn_duration={self.turn_duration:.2f}s'
                )
            return

        # ---------------- RETURN_HEADING ----------------
        if self.state == State.RETURN_HEADING:
            self.log_state_once('State: RETURN_HEADING')

            return_duration = self.turn_duration * self.return_gain
            elapsed = self.now_sec() - self.return_start_time

            if elapsed < return_duration:
                if self.avoid_side == 'left':
                    self.publish_cmd(0.03, -self.turn_speed)
                else:
                    self.publish_cmd(0.03, self.turn_speed)
            else:
                self.state = State.FORWARD
                self.exit_counter = 0
                self.get_logger().info('Return completed. Back to FORWARD.')
                self.publish_forward_compensated(self.forward_speed, 0.0)
            return

    # =========================================================
    # Main loop
    # =========================================================
    def control_loop(self):
        regions = self.get_regions()
        if regions is None:
            self.log_state_once('Waiting for /scan...')
            self.stop_robot()
            return

        if self.mode == 'manual':
            self.log_state_once('Mode: MANUAL_SAFETY')
            self.handle_manual_mode(regions)
        else:
            self.handle_auto_mode(regions)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
