from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk',
            # namespace='rover_gps',
            executable='rover_node',
            # name='sim'
        ),
        Node(
            package='rtk',
            # namespace='rtmc3_sub',
            executable='rtcm3_sub',
            # name='sim'
        )
    ])