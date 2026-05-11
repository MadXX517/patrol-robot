import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定义一个 launch 参数 open_rviz，默认值为 true
    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Whether to open RViz2'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('car_nav2'),
        'rviz',
        'car.rviz'
    )
    # 获取参数 open_rviz 的值
    open_rviz = LaunchConfiguration('open_rviz')

    # 定义 rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],  # 设置 rviz 配置文件路径
        condition=IfCondition(open_rviz)  # 根据 open_rviz 参数决定是否启动
    )

    # 将所有组件加载到 LaunchDescription 中
    return LaunchDescription([
        open_rviz_arg,  # 声明参数
        rviz_node       # 启动 rviz2 节点
    ])