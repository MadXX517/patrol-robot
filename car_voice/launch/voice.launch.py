# car_voice 语音助手 launch
# 用法: ros2 launch car_voice voice.launch.py enable_tts:=true enable_asr:=true
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # API Key 默认读取环境变量 DASHSCOPE_API_KEY,也可 api_key:=... 覆盖
        DeclareLaunchArgument('api_key',
                              default_value=EnvironmentVariable('DASHSCOPE_API_KEY', default_value='')),
        DeclareLaunchArgument('llm_model', default_value='qwen-flash'),
        DeclareLaunchArgument('tts_voice', default_value='Serena'),
        DeclareLaunchArgument('tts_model', default_value='qwen3-tts-flash'),
        DeclareLaunchArgument('sound_volume', default_value='90'),
        DeclareLaunchArgument('asr_model', default_value='paraformer-realtime-v2'),
        DeclareLaunchArgument('wake_word', default_value='小星'),
        DeclareLaunchArgument('dashboard_url', default_value='http://127.0.0.1:8000'),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('enable_asr', default_value='true'),
    ]

    voice_node = Node(
        package='car_voice',
        executable='voice_main',
        name='voice_main',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--api_key', LaunchConfiguration('api_key'),
            '--llm_model', LaunchConfiguration('llm_model'),
            '--tts_voice', LaunchConfiguration('tts_voice'),
            '--tts_model', LaunchConfiguration('tts_model'),
            '--sound_volume', LaunchConfiguration('sound_volume'),
            '--asr_model', LaunchConfiguration('asr_model'),
            '--wake_word', LaunchConfiguration('wake_word'),
            '--dashboard_url', LaunchConfiguration('dashboard_url'),
            '--enable_tts', LaunchConfiguration('enable_tts'),
            '--enable_asr', LaunchConfiguration('enable_asr'),
        ],
    )

    return LaunchDescription(args + [voice_node])
