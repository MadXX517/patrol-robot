# 手势控制:gesture_command 节点(MediaPipe 手势 -> /patrol/command)
# 相机由 car_web 的 core 或单独启动提供(/camera/color/image_raw)。
# 与 person_follow 共用相机;本节点只发高层命令,不碰 /cmd_vel。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_topic_arg = DeclareLaunchArgument(
        'color_topic', default_value='/camera/color/image_raw')
    confirm_frames_arg = DeclareLaunchArgument('confirm_frames', default_value='6')
    armed_timeout_arg = DeclareLaunchArgument('armed_timeout', default_value='15.0')
    confirm_hold_arg = DeclareLaunchArgument('confirm_hold', default_value='3.0')
    cmd_cooldown_arg = DeclareLaunchArgument('cmd_cooldown', default_value='3.0')
    process_every_arg = DeclareLaunchArgument('process_every', default_value='1')
    detect_width_arg = DeclareLaunchArgument('detect_width', default_value='480')
    flip_arg = DeclareLaunchArgument('flip', default_value='false')
    enable_dynamic_arg = DeclareLaunchArgument('enable_dynamic', default_value='true')
    model_path_arg = DeclareLaunchArgument('model_path', default_value='')

    gesture_node = Node(
        package='car_patrol',
        executable='gesture_command',
        name='gesture_command',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'confirm_frames': LaunchConfiguration('confirm_frames'),
            'armed_timeout': LaunchConfiguration('armed_timeout'),
            'confirm_hold': LaunchConfiguration('confirm_hold'),
            'cmd_cooldown': LaunchConfiguration('cmd_cooldown'),
            'process_every': LaunchConfiguration('process_every'),
            'detect_width': LaunchConfiguration('detect_width'),
            'flip': LaunchConfiguration('flip'),
            'enable_dynamic': LaunchConfiguration('enable_dynamic'),
            'model_path': LaunchConfiguration('model_path'),
        }],
    )

    return LaunchDescription([
        color_topic_arg, confirm_frames_arg, armed_timeout_arg, confirm_hold_arg,
        cmd_cooldown_arg, process_every_arg, detect_width_arg, flip_arg,
        enable_dynamic_arg, model_path_arg,
        gesture_node,
    ])
