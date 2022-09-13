from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk',
            namespace='base_gps',
            executable='base_node',
            name='sim'
        ),
        Node(
            package='rtk',
            namespace='base_rtk',
            executable='base_rtk',
            name='sim'
        )
    ])