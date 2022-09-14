from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk',
            # namespace='base_gps',
            executable='base_node',
            # name='sim'
        ),
        Node(
            package='rtk',
            # namespace='rtcm3_pub',
            executable='rtcm3_pub',
            # name='sim'
        ),
        Node(
            package='bag_recorder_nodes_py',
            # namespace='data_recorder',
            executable='simple_bag_recorder',
            # name='sim'
        )
    ])