from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('enable_frontend', default_value='true'),
        DeclareLaunchArgument('enable_executor', default_value='true'),
        DeclareLaunchArgument('enable_report_service', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/aibox'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('enable_serial', default_value='true'),
        DeclareLaunchArgument('enable_asr', default_value='true'),
        DeclareLaunchArgument('enable_llm', default_value='true'),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('api_key', default_value=''),
        DeclareLaunchArgument(
            'base_url',
            default_value='https://dashscope.aliyuncs.com/compatible-mode/v1',
        ),
        DeclareLaunchArgument('asr_model', default_value='paraformer-realtime-v2'),
        DeclareLaunchArgument('llm_model', default_value='qwen-plus'),
        DeclareLaunchArgument('tts_model', default_value='qwen-tts'),
        DeclareLaunchArgument('tts_voice', default_value='Serena'),
        DeclareLaunchArgument('linear_speed', default_value='0.12'),
        DeclareLaunchArgument('lateral_speed', default_value='0.10'),
        DeclareLaunchArgument('angular_speed', default_value='0.60'),
        DeclareLaunchArgument('enable_report', default_value='true'),
        DeclareLaunchArgument('report_mode', default_value='text'),
    ]

    frontend = Node(
        package='car_voice',
        executable='voice_frontend',
        name='voice_frontend',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_frontend')),
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'enable_serial': ParameterValue(LaunchConfiguration('enable_serial'), value_type=bool),
            'enable_asr': ParameterValue(LaunchConfiguration('enable_asr'), value_type=bool),
            'enable_llm': ParameterValue(LaunchConfiguration('enable_llm'), value_type=bool),
            'enable_tts': ParameterValue(LaunchConfiguration('enable_tts'), value_type=bool),
            'api_key': LaunchConfiguration('api_key'),
            'base_url': LaunchConfiguration('base_url'),
            'asr_model': LaunchConfiguration('asr_model'),
            'llm_model': LaunchConfiguration('llm_model'),
            'tts_model': LaunchConfiguration('tts_model'),
            'tts_voice': LaunchConfiguration('tts_voice'),
        }],
    )

    executor = Node(
        package='car_voice',
        executable='voice_executor',
        name='voice_executor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_executor')),
        parameters=[{
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'lateral_speed': ParameterValue(LaunchConfiguration('lateral_speed'), value_type=float),
            'angular_speed': ParameterValue(LaunchConfiguration('angular_speed'), value_type=float),
            'enable_report': ParameterValue(LaunchConfiguration('enable_report'), value_type=bool),
            'enable_tts': ParameterValue(LaunchConfiguration('enable_tts'), value_type=bool),
        }],
    )

    report_service = Node(
        package='car_report',
        executable='report_service',
        name='report_service',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_report_service')),
        parameters=[{
            'mode': LaunchConfiguration('report_mode'),
        }],
    )

    return LaunchDescription(args + [frontend, executor, report_service])
