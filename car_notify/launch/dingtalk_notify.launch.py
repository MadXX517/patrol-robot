from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    event_topic_arg = DeclareLaunchArgument(
        'event_topic',
        default_value='/car_report/event',
        description='car_report event topic'
    )
    webhook_url_arg = DeclareLaunchArgument(
        'webhook_url',
        default_value='',
        description='DingTalk custom robot webhook URL; empty means use node default'
    )
    keyword_arg = DeclareLaunchArgument(
        'keyword',
        default_value='巡逻告警',
        description='DingTalk keyword security text'
    )
    cooldown_sec_arg = DeclareLaunchArgument(
        'cooldown_sec',
        default_value='5.0',
        description='Cooldown seconds for the same event class'
    )
    min_score_arg = DeclareLaunchArgument(
        'min_score',
        default_value='0.5',
        description='Minimum event score to send'
    )
    target_classes_arg = DeclareLaunchArgument(
        'target_classes',
        default_value='',
        description='Comma-separated class names; empty means all classes'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='10.0',
        description='HTTP timeout seconds'
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Print payload without sending DingTalk request'
    )

    node = Node(
        package='car_notify',
        executable='dingtalk_notifier',
        name='dingtalk_notifier',
        output='screen',
        parameters=[{
            'event_topic': LaunchConfiguration('event_topic'),
            'webhook_url': LaunchConfiguration('webhook_url'),
            'keyword': LaunchConfiguration('keyword'),
            'cooldown_sec': ParameterValue(LaunchConfiguration('cooldown_sec'), value_type=float),
            'min_score': ParameterValue(LaunchConfiguration('min_score'), value_type=float),
            'target_classes': LaunchConfiguration('target_classes'),
            'timeout': ParameterValue(LaunchConfiguration('timeout'), value_type=float),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
        }]
    )

    return LaunchDescription([
        event_topic_arg,
        webhook_url_arg,
        keyword_arg,
        cooldown_sec_arg,
        min_score_arg,
        target_classes_arg,
        timeout_arg,
        dry_run_arg,
        node,
    ])
