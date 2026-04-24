from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------- STREAM (залишаємо) ----------------
        Node(
            package='robot_bringup',
            executable='ros_stream',
            name='image_stream_node',
            output='screen',
            emulate_tty=True
        ),

        # ---------------- MOTORS ----------------
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen',
            emulate_tty=True
        ),

        # ---------------- LIDAR SAFETY ----------------
        Node(
            package='your_package',  # <-- заміни на пакет де lidar_safety_node
            executable='lidar_safety_node',
            name='lidar_safety_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',
                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
                'stop_dist': 0.05,      # 5 см
                'front_angle_deg': 20.0
            }]
        ),

    ])
