import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('delta_motors'), 'config', 'motors.yaml')
    mock = LaunchConfiguration('mock')
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false',
                              description='Run without hardware'),
        Node(
            package='delta_motors',
            executable='motor_node',
            name='delta_motors',
            output='screen',
            parameters=[config, {'mock': ParameterValue(mock, value_type=bool)}],
        ),
    ])
