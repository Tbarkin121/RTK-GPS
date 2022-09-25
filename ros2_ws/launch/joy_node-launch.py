import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 1.0,
		        'coalesce_interval': 100
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        # launch_ros.actions.Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy_node',
        #     parameters=[
        #         {'enable_button': -1},
        #         {'require_enable_button': False},
        #         {'axis_linear.x': 0},
        #         {'axis_linear.y': 2},
        #         {'axis_linear.z': 6},
        #         {'axis_angular.pitch': 5},
        #         {'axis_angular.roll': 3},
        #         {'axis_angular.yaw': 7},
        #         {'scale_linear.x': 1.0},
        #         {'scale_linear.y': 1.0},
        #         {'scale_linear.z': 1.0},
        #         {'scale_angular.pitch': 1.0},
        #         {'scale_angular.roll': 1.0},
        #         {'scale_angular.yaw': 1.0}
        #     ],
        #     arguments=['--ros-args', '--log-level', 'info']
        # )
    ])