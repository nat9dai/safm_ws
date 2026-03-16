from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='conversion_pkg',
            executable='sensor_combined_to_imu_node',
            name='sensor_combined_to_imu_node',
            output='screen',
        ),
        Node(
            package='conversion_pkg',
            executable='odom_to_visual_odom',
            name='odom_to_visual_odom',
            output='screen',
        ),
    ])
