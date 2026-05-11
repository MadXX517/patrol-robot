from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():

    # 自动驾驶节点配置
    body_control_node = Node(
        package='car_vision',
        executable='body_control',
        output='screen',
        parameters=[{               # 参数配置
            'machine_type':'Mec'}   # 底盘选择
        ]
    )

    return LaunchDescription([
        body_control_node      
    ])