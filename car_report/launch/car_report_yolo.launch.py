from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_output_dir = get_package_share_directory('car_report') + '/data'

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic used by YOLO and event screenshots'
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
    confirm_frames_arg = DeclareLaunchArgument(
        'confirm_frames',
        default_value='1',
        description='Consecutive detected frames required before recording an event'
    )
    area_name_arg = DeclareLaunchArgument(
        'area_name',
        default_value='通信节点外围警戒线',
        description='Security area name written into event JSON'
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
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='cpu',
        description='Torch compute device when yolo_backend=torch'
    )
    yolo_backend_arg = DeclareLaunchArgument(
        'yolo_backend',
        default_value='rknn',
        description='YOLO inference backend: rknn or torch'
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov5s',
        description='YOLO model name in car_yolo config (yolov5s = COCO 80 classes; traffic_640n_7 = traffic scene only)'
    )
    yolo_rknn_model_arg = DeclareLaunchArgument(
        'yolo_rknn_model',
        default_value='',
        description='RKNN model file name in car_yolo config, or an absolute path; empty means <yolo_model>.rknn'
    )
    yolo_conf_thres_arg = DeclareLaunchArgument(
        'yolo_conf_thres',
        default_value='0.25',
        description='YOLO confidence threshold for object detection and annotated result images'
    )
    yolo_show_result_arg = DeclareLaunchArgument(
        'yolo_show_result',
        default_value='false',
        description='Whether YOLO opens a display window'
    )
    yolo_pub_result_img_arg = DeclareLaunchArgument(
        'yolo_pub_result_img',
        default_value='false',
        description='Whether YOLO publishes annotated result images'
    )

    yolo_node = Node(
        package='car_yolo',
        executable='yolo_detect',
        output='screen',
        parameters=[{
            'backend': LaunchConfiguration('yolo_backend'),
            'device': LaunchConfiguration('yolo_device'),
            'model': LaunchConfiguration('yolo_model'),
            'rknn_model': LaunchConfiguration('yolo_rknn_model'),
            'image_topic': LaunchConfiguration('image_topic'),
            'conf_thres': ParameterValue(LaunchConfiguration('yolo_conf_thres'), value_type=float),
            'target_classes': LaunchConfiguration('target_classes'),
            'show_result': ParameterValue(LaunchConfiguration('yolo_show_result'), value_type=bool),
            'pub_result_img': ParameterValue(LaunchConfiguration('yolo_pub_result_img'), value_type=bool),
        }]
    )

    recorder_node = Node(
        package='car_report',
        executable='event_recorder',
        name='event_recorder',
        output='screen',
        parameters=[{
            'objects_topic': '/car_yolo/object_detect',
            'image_topic': LaunchConfiguration('image_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'min_score': ParameterValue(LaunchConfiguration('min_score'), value_type=float),
            'cooldown_sec': ParameterValue(LaunchConfiguration('cooldown_sec'), value_type=float),
            'target_classes': LaunchConfiguration('target_classes'),
            'confirm_frames': ParameterValue(LaunchConfiguration('confirm_frames'), value_type=int),
            'area_name': LaunchConfiguration('area_name'),
            'save_image': ParameterValue(LaunchConfiguration('save_image'), value_type=bool),
            'max_image_bytes': ParameterValue(LaunchConfiguration('max_image_bytes'), value_type=int),
            'jpeg_quality': ParameterValue(LaunchConfiguration('jpeg_quality'), value_type=int),
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        output_dir_arg,
        min_score_arg,
        cooldown_sec_arg,
        target_classes_arg,
        confirm_frames_arg,
        area_name_arg,
        save_image_arg,
        max_image_bytes_arg,
        jpeg_quality_arg,
        yolo_backend_arg,
        yolo_device_arg,
        yolo_model_arg,
        yolo_rknn_model_arg,
        yolo_conf_thres_arg,
        yolo_show_result_arg,
        yolo_pub_result_img_arg,
        yolo_node,
        recorder_node,
    ])
