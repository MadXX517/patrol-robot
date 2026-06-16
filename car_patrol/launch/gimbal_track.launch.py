# 云台追踪:car_yolo(person 检测) + person_follow(mode=track_only,底盘不动仅云台追踪)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')
    pkg = get_package_share_directory('car_patrol')
    human_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'human_follow.launch.py')),
        launch_arguments={
            'mode': 'track_only',
            'auto_start': LaunchConfiguration('auto_start'),
        }.items(),
    )
    return LaunchDescription([auto_start_arg, human_follow])
