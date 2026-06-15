from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='HTTP bind host'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8000',
        description='HTTP dashboard port'
    )
    drive_timeout_arg = DeclareLaunchArgument(
        'drive_timeout',
        default_value='0.5',
        description='Seconds before auto stop if no drive command is received'
    )

    node = Node(
        package='car_web',
        executable='dashboard_server',
        name='car_web_dashboard',
        output='screen',
        parameters=[{
            'http_host': LaunchConfiguration('host'),
            'http_port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'drive_timeout': ParameterValue(LaunchConfiguration('drive_timeout'), value_type=float),
        }]
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        drive_timeout_arg,
        node,
    ])
