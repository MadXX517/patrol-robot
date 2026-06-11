from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
import time

def generate_launch_description():
    # ============================ 通用参数 ============================ #
    api_key_arg = DeclareLaunchArgument(
        'api_key', # 要求语音转文字模型、视觉理解模型、语言大模型、文字转语音模型都是同一家提供，一个api_key支持所有多模态大模型
        default_value='sk-a07b9af862154c89a34741c80db78f7a', # 记得替换api key
        description='API Key for LLM service'
    )

    base_url_arg = DeclareLaunchArgument(
        'base_url', # 要求与api_key是同一家服务商提供
        default_value='https://dashscope.aliyuncs.com/compatible-mode/v1', # 通义千问的base url
        description='Base URL for LLM service'
    )

    yaml_file_arg = DeclareLaunchArgument(
        'yaml_file',
        default_value='chat_prompt_car.yaml', # prompt配置文件路径，要求yaml文件放置在config文件夹下
        description='YAML file for prompt configuration'
    )
    
    # ============================ tty串口参数 ============================ #
    serial_port_arg = DeclareLaunchArgument(
        'aibox_serial_port',
        default_value='/dev/aibox', # /dev/ttyUSB的名称，一般无需更改
        description='Serial port for ttyUSB communicatio n'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200', # 波特率，与下位机的波特率相同
        description='Baudrate for serial communication'
    )

    speaker_sound_volume_arg = DeclareLaunchArgument(
        'sound_volume',
        default_value='85', # 音量，0-100, 默认85
        description='set sound volume of speaker'
    )

    # ============================ 语言大模型参数 ============================ #
    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='qwen-plus', # 语言大模型，可用模型名称qwen-max、qwen-plus、qwen-flash、qwen-turbo（已停止更新，不建议使用）
        description='LLM model for command parsing'
    )
    
    # ============================ 文字转语音（即语音生成）大模型参数 ============================ #
    tts_voice_arg = DeclareLaunchArgument(
        'tts_voice',
        default_value='Serena', # 支持音色'Cherry', 'Ethan', 'Serena','Chelsie',通义千问提供以上音色。
        description='Voice for TTS playback'
    )

    tts_model_arg = DeclareLaunchArgument(
        'tts_model',
        default_value='qwen-tts', # 文字转语音tts大模型：qwen3-tts-flash、qwen-tts
        description='TTS model'
    )

    # ============================ 语音转文字（即语音识别）大模型参数 ============================ #
    asr_model_arg = DeclareLaunchArgument(
        'asr_model',
        default_value='paraformer-realtime-v2', # 自动语音转文字asr大模型：paraformer-realtime-v2
        description='ASR model'
    )

    max_sentence_silence_arg = DeclareLaunchArgument(
        'max_sentence_silence',
        default_value='800',
        description='max sentence silence'
    )

    # 是否接入机器人控制(机械臂/底盘)。第一阶段默认 false,仅语音对话。
    enable_ros_control_arg = DeclareLaunchArgument(
        'enable_ros_control',
        default_value='false',
        description='enable robot control (arm/chassis); first stage keeps it false'
    )

    # 麦克风触发方式:continuous=持续聆听(无按钮 aibox,默认);button=按键按住说话
    mic_trigger_arg = DeclareLaunchArgument(
        'mic_trigger',
        default_value='continuous',
        description='mic trigger: continuous (always-on, no button) or button'
    )

    # ============================ 实现部分 ============================ #
    # 仅在启用机器人控制时才拉起底盘 car_base(纯语音对话无需底盘)
    base_serial_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('car_base'),
            'launch/car_base.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('enable_ros_control')),
    )

    # llm_main.py 安装于 share/car_llm/src/(见 setup.py),不再硬编码 /home/pi 路径
    llm_main_path = PathJoinSubstitution([
        FindPackageShare('car_llm'), 'src', 'llm_main.py'
    ])

    # 启动 LLM 主程序
    llm_main_process = ExecuteProcess(
        cmd=[
            'python3', '-u',
            llm_main_path,
            '--api_key', LaunchConfiguration('api_key'),
            '--base_url', LaunchConfiguration('base_url'),
            '--yaml_file', LaunchConfiguration('yaml_file'),
            '--aibox_serial_port', LaunchConfiguration('aibox_serial_port'),
            '--baudrate', LaunchConfiguration('baudrate'),
            '--sound_volume', LaunchConfiguration('sound_volume'),
            '--llm_model', LaunchConfiguration('llm_model'),
            '--tts_voice', LaunchConfiguration('tts_voice'),
            '--tts_model', LaunchConfiguration('tts_model'),
            '--asr_model', LaunchConfiguration('asr_model'),
            '--max_sentence_silence', LaunchConfiguration('max_sentence_silence'),
            '--enable_ros_control', LaunchConfiguration('enable_ros_control'),
            '--mic_trigger', LaunchConfiguration('mic_trigger')
        ],
        output='screen',
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(api_key_arg)
    ld.add_action(base_url_arg)
    ld.add_action(yaml_file_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(baudrate_arg)
    ld.add_action(speaker_sound_volume_arg)
    ld.add_action(llm_model_arg)
    ld.add_action(tts_voice_arg)
    ld.add_action(tts_model_arg)
    ld.add_action(asr_model_arg)
    ld.add_action(max_sentence_silence_arg)
    ld.add_action(enable_ros_control_arg)
    ld.add_action(mic_trigger_arg)
    ld.add_action(base_serial_launch)
    ld.add_action(llm_main_process)
    return ld
