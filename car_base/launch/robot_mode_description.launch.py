import os
import xacro  # 新增：导入xacro处理模块
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction,
    LogInfo,
    IncludeLaunchDescription, 
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    car_urdf_dir = os.path.join(get_package_share_directory('car_urdf'), 'urdf')
    car_xacro_path = os.path.join(car_urdf_dir, 'yeahbot_c2.xacro')
    if not os.path.exists(car_xacro_path):
        car_xacro_path = os.path.join(car_urdf_dir, 'yeahbot_c2.urdf')
    
    # 2. 处理XACRO文件，生成URDF XML内容
    # 若需要传递xacro参数，可在process_file中添加 mappings={"参数名": "参数值"}
    doc = xacro.process_file(car_xacro_path)
    urdf_xml = doc.toxml()

    car_description = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            # 3. 关键修改：通过parameters传递解析后的URDF内容（替代原arguments方式）
            parameters=[{
                'robot_description': urdf_xml,
                'use_sim_time': False  # 根据实际场景调整（仿真时设为True）
            }],
            # 可选：输出日志便于调试
            output='screen'
        )
    ])
 
    ld = LaunchDescription()
    ld.add_action(car_description)
    return ld
