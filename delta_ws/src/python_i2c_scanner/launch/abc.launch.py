import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    python_node = Node(
        package='python_i2c_scanner',
        executable='python_node',
        name='python_node',
        output='screen'
    )

    return LaunchDescription([
        python_node#,
        #mission_planner_node,
        #DroneController
    ])
