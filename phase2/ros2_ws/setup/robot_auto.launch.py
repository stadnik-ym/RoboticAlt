from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------- MOTORS ----------------
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen',
            emulate_tty=True
        ),

        # ---------------- LIDAR OBSTACLE AVOIDANCE ----------------
        Node(
            package='robot_bringup',
            executable='lidar_obstacle_node',
            name='lidar_obstacle_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',

                'forward_speed': 0.11,
                'forward_angular_compensation': -0.07,

                'turn_speed': 0.75,
                'wall_follow_speed': 0.09,

                'obstacle_dist': 0.55,
                'clear_dist': 0.85,
                'side_desired_dist': 0.30,
                'side_open_dist': 0.85,

                'wall_kp': 1.4,
                'return_gain': 1.0,
            }]
        ),
    ])
