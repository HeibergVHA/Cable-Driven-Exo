import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    i2c_scanner_node = Node(
        package='python_utils',
        executable='i2c_scanner_node',
        name='i2c_scanner_node',
        output='screen'
    )

    return LaunchDescription([
        i2c_scanner_node#,
        #mission_planner_node,
        #DroneController
    ])
