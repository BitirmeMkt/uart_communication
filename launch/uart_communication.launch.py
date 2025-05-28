from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('uart_communication')

    param_file = os.path.join(pkg_path, 'config', 'uart_communication.yaml')

    return LaunchDescription([
        ComposableNodeContainer(
            name='uart_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='uart_communication',
                    plugin='uart_communication::UartCommunication',
                    name='uart_communication_node',
                    parameters=[param_file]
                ),
            ],
            output='screen'
        )
    ])
