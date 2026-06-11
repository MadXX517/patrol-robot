from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='text')
    output_dir_arg = DeclareLaunchArgument('output_dir', default_value='')
    api_key_arg = DeclareLaunchArgument('api_key', default_value='')
    timeout_arg = DeclareLaunchArgument('timeout', default_value='60.0')

    node = Node(
        package='car_report',
        executable='report_service',
        name='report_service',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'output_dir': LaunchConfiguration('output_dir'),
            'api_key': LaunchConfiguration('api_key'),
            'timeout': ParameterValue(LaunchConfiguration('timeout'), value_type=float),
        }],
    )

    return LaunchDescription([
        mode_arg,
        output_dir_arg,
        api_key_arg,
        timeout_arg,
        node,
    ])
