from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='playground_service_py',
            executable='services_node',
            name='services_node',
            output='screen',
            parameters=[]
        )
    ])
