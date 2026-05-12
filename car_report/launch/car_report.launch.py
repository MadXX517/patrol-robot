from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_output_dir = get_package_share_directory('car_report') + '/data'

    objects_topic_arg = DeclareLaunchArgument(
        'objects_topic',
        default_value='/car_yolo/object_detect',
        description='YOLO structured object detection topic'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic used for event screenshots'
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory for event logs, screenshots, and reports'
    )
    min_score_arg = DeclareLaunchArgument(
        'min_score',
        default_value='0.5',
        description='Minimum confidence score to record an event'
    )
    cooldown_sec_arg = DeclareLaunchArgument(
        'cooldown_sec',
        default_value='5.0',
        description='Cooldown seconds for the same class'
    )
    target_classes_arg = DeclareLaunchArgument(
        'target_classes',
        default_value='',
        description='Comma-separated class names; empty means all classes'
    )
    save_image_arg = DeclareLaunchArgument(
        'save_image',
        default_value='true',
        description='Whether to save an image snapshot for each event'
    )
    max_image_bytes_arg = DeclareLaunchArgument(
        'max_image_bytes',
        default_value='5242880',
        description='Maximum saved snapshot size in bytes'
    )
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='85',
        description='Initial JPEG quality for event snapshots'
    )

    recorder_node = Node(
        package='car_report',
        executable='event_recorder',
        name='event_recorder',
        output='screen',
        parameters=[{
            'objects_topic': LaunchConfiguration('objects_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'min_score': ParameterValue(LaunchConfiguration('min_score'), value_type=float),
            'cooldown_sec': ParameterValue(LaunchConfiguration('cooldown_sec'), value_type=float),
            'target_classes': LaunchConfiguration('target_classes'),
            'save_image': ParameterValue(LaunchConfiguration('save_image'), value_type=bool),
            'max_image_bytes': ParameterValue(LaunchConfiguration('max_image_bytes'), value_type=int),
            'jpeg_quality': ParameterValue(LaunchConfiguration('jpeg_quality'), value_type=int),
        }]
    )

    return LaunchDescription([
        objects_topic_arg,
        image_topic_arg,
        output_dir_arg,
        min_score_arg,
        cooldown_sec_arg,
        target_classes_arg,
        save_image_arg,
        max_image_bytes_arg,
        jpeg_quality_arg,
        recorder_node,
    ])
