from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():

    # 自动驾驶节点配置
    driver_node = Node(
        package='car_vision',
        executable='driver',
        output='screen',
        parameters=[{               # 参数配置
            'machine_type':'Mec'}   # 底盘选择
        ]
    )

    return LaunchDescription([
        driver_node      # 启动自动驾驶
    ])