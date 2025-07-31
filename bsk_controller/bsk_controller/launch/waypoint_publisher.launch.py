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
        default_value='15.0',
        description='Period (seconds) to stay at each waypoint'
    )
    namespace = LaunchConfiguration('namespace')
    period = LaunchConfiguration('period')

    return LaunchDescription([
        namespace_arg,
        period_arg,
        Node(
            package='bsk_controller',
            namespace=namespace,
            executable='waypoint_publisher',
            name='waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'period': period}
            ],
        ),
    ])
