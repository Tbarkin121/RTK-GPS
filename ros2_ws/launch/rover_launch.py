from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk',
            namespace='rover_gps',
            executable='rover_node',
            name='sim'
        ),
        Node(
            package='rtk',
            namespace='rover_rtk',
            executable='rover_rtk',
            name='sim'
        )
    ])