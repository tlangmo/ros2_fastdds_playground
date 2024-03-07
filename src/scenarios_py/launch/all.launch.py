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
        DeclareLaunchArgument(
            'period_sec',
            default_value='10',
            description='Interval period of the qos publisher in seconds'
        ),

        Node(
            package='playground_publish_py',
            executable='heavy_pub_node',
            name='heavy_pub_node',
            output='screen',
            parameters=[{'fps': LaunchConfiguration('fps')}]
        ),
        Node(
            package='playground_publish_py',
            executable='qos_pub_node',
            name='qos_pub_node',
            output='screen',
            parameters=[{'period_sec': LaunchConfiguration('period_sec')}]
        ),
        Node(
            package='playground_services_py',
            executable='service_node',
            name='service_node',
            output='screen',
            parameters=[]
        )
    ])
