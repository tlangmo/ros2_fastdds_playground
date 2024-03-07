from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fps',
            default_value='10',
            description='Frame rate of the image publisher'
        ),
        Node(
            package='playground_publish_py',
            executable='heavy_pub_node',
            name='heavy_pub_node',
            output='screen',
            parameters=[{'fps': LaunchConfiguration('fps')}]
        )
    ])
