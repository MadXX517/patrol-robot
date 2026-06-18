# 人体跟随:car_yolo(person 检测) + person_follow(mode=follow)
# 相机/底盘由 car_web 的 core 或单独启动提供(/camera/color/image_raw、/camera/depth/image_rect_raw、/cmd_vel)。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='follow',
                                     description='follow | track_only')
    follow_distance_arg = DeclareLaunchArgument('follow_distance', default_value='1.2')
    safe_distance_arg = DeclareLaunchArgument('safe_distance', default_value='0.8')
    lidar_safe_distance_arg = DeclareLaunchArgument('lidar_safe_distance', default_value='0.45')
    max_lin_arg = DeclareLaunchArgument('max_lin', default_value='0.25')
    max_ang_arg = DeclareLaunchArgument('max_ang', default_value='0.8')
    machine_type_arg = DeclareLaunchArgument('machine_type', default_value='Mec')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')
    conf_thres_arg = DeclareLaunchArgument('conf_thres', default_value='0.4')

    yolo_node = Node(
        package='car_yolo',
        executable='yolo_detect',
        name='car_yolo',
        output='screen',
        parameters=[{
            'backend': 'rknn',
            'device': 'cpu',
            'model': 'yolov5s',
            'rknn_model': 'yolov5s.rknn',
            'img_size': 640,
            'conf_thres': LaunchConfiguration('conf_thres'),
            'iou_thres': 0.45,
            'npu_core': 'auto',
            'image_topic': '/camera/color/image_raw',
            'show_result': False,
            'pub_result_img': False,
        }],
    )

    person_follow_node = Node(
        package='car_patrol',
        executable='person_follow',
        name='person_follow',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'machine_type': LaunchConfiguration('machine_type'),
            'follow_distance': LaunchConfiguration('follow_distance'),
            'safe_distance': LaunchConfiguration('safe_distance'),
            'lidar_safe_distance': LaunchConfiguration('lidar_safe_distance'),
            'max_lin': LaunchConfiguration('max_lin'),
            'max_ang': LaunchConfiguration('max_ang'),
            'auto_start': LaunchConfiguration('auto_start'),
        }],
    )

    return LaunchDescription([
        mode_arg, follow_distance_arg, safe_distance_arg, lidar_safe_distance_arg, max_lin_arg, max_ang_arg,
        machine_type_arg, auto_start_arg, conf_thres_arg,
        yolo_node, person_follow_node,
    ])
