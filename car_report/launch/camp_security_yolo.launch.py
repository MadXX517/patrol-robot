from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('car_report'),
                    'launch',
                    'car_report_yolo.launch.py',
                ])
            ),
            launch_arguments={
                'yolo_conf_thres': '0.6',
                'min_score': '0.65',
                'cooldown_sec': '20',
                'confirm_frames': '3',
                'target_classes': 'person,backpack,suitcase,handbag',
                'area_name': '通信节点外围警戒线',
                'save_image': 'true',
                'yolo_pub_result_img': 'true',
            }.items(),
        )
    ])
