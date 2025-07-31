from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='pop',
        description='Namespace for all nodes'
    )
    period_arg = DeclareLaunchArgument(
        'period',
        default_value='20.0',
        description='Period (seconds)'
    )
    namespace = LaunchConfiguration('namespace')
    period = LaunchConfiguration('period')
    
    return LaunchDescription([
        namespace_arg,
        period_arg,
    
        # Launch figure-8 waypoint publisher
        Node(
            package='bsk_controller',
            namespace=namespace,
            executable='figure8_waypoint_publisher',
            name='figure8_waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'period': period}
            ],
        ),
    ])
