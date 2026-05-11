import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取当前包的路径
    lidar_dir = get_package_share_directory('car_base')

    # 定义LidarController节点
    lidar_controller_node = Node(
        package='car_vision',  # 包名
        executable='lidar_controller',  # 可执行文件名
        name='lidar_controller',  # 节点名
        output='screen',  # 输出到屏幕
        parameters=[{               # 参数配置
            'running_mode': 1,      # 默认启动为避障模式
            'machine_type':'Mec'}   # 底盘选择
        ]
    )

    return LaunchDescription([
        lidar_controller_node
    ])