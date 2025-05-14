import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('monkey_brain_demos'),
                               'config/service_client_demo')
    return LaunchDescription([
        TimerAction(
           actions=[
               Node(
                   package='yasmin_demos',
                   executable='add_two_ints_server',
                   output='screen'
               )],
           period=0.0
        ),
        Node(
            package='monkey_brain_node',
            executable='monkey_brain_node',
            name='monkey_brain',
            namespace='monkey_brain_demo',
            parameters=[
              {'path_to_scxml': os.path.join(config_path, 'state_machine.scxml')},
              {'path_to_ios': os.path.join(config_path, 'plugins.yaml')},
            ],
            ros_arguments=['--log-level', 'info'],
            output='screen',
            on_exit=Shutdown()
        )
    ])
