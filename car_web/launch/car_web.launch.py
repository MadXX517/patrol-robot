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
    # 电量:/PowerVoltage 原始字节 x scale = 伏特;full/empty 用于估算百分比(按电池组校准)
    voltage_scale_arg = DeclareLaunchArgument('voltage_scale', default_value='0.178')
    voltage_full_arg = DeclareLaunchArgument('voltage_full', default_value='12.6')
    voltage_empty_arg = DeclareLaunchArgument('voltage_empty', default_value='9.9')
    llm_api_key_arg = DeclareLaunchArgument(
        'llm_api_key',
        default_value='',
        description='Optional DashScope API key override for car_llm voice control'
    )
    llm_wake_word_arg = DeclareLaunchArgument(
        'llm_wake_word',
        default_value='小星',
        description='Wake word for car_llm voice control'
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
            'voltage_scale': ParameterValue(LaunchConfiguration('voltage_scale'), value_type=float),
            'voltage_full': ParameterValue(LaunchConfiguration('voltage_full'), value_type=float),
            'voltage_empty': ParameterValue(LaunchConfiguration('voltage_empty'), value_type=float),
            'llm_api_key': LaunchConfiguration('llm_api_key'),
            'llm_wake_word': LaunchConfiguration('llm_wake_word'),
        }]
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        drive_timeout_arg,
        voltage_scale_arg,
        voltage_full_arg,
        voltage_empty_arg,
        llm_api_key_arg,
        llm_wake_word_arg,
        node,
    ])
