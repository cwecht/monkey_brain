
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('monkey_brain_demos'),
                               'config/atomic')
    return LaunchDescription([
        Node(
            package='monkey_brain_node',
            executable='monkey_brain_node',
            name='mb_atomic',
            namespace='mb_atomic',
            parameters=[
              {'path_to_scxml': os.path.join(config_path, 'state_machine.scxml')},
              {'path_to_ios': os.path.join(config_path, 'plugins.yaml')},
              #  {'state_machine_observer': 'yasmin_state_observer' },
            ],
            ros_arguments=['--log-level', 'info'],
            output='screen'
        )
    ])
