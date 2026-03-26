import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nortek_nucleus_ros_interface'),
        'config',
        'nortek_nucleus_ros_interface_params.yaml',
    )

    namespace = LaunchConfiguration('namespace')

    nortek_nucleus_ros_interface_node = Node(
        package='nortek_nucleus_ros_interface',
        executable='nortek_nucleus_ros_interface_node',
        name='nortek_nucleus_ros_interface_node',
        namespace=namespace,
        parameters=[config],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the nortek_nucleus_ros_interface node',
            ),
            nortek_nucleus_ros_interface_node,
        ]
    )
