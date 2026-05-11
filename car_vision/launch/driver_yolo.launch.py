import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():

    # YOLO检测节点配置
    yolo_node = Node(
        package='car_yolo',
        executable='yolo_detect',
        output='screen',
        parameters=[
            {"device": "cpu",
            #"model": "garbage_classification",
            "model": "traffic_640n_7",
            #"model": "yolov5s",
            "image_topic": "/camera/color/image_raw",
            #"camera_info_topic": "/camera/camera_info",
            #"camera_info_file": f"{package_share_directory}/config/camera_info.yaml",
            #"show_result": True,
            #"pub_result_img": True
            }
        ]
    )

    # 自动驾驶节点配置
    driver_node = Node(
        package='car_vision',
        executable='driver',
        output='screen',
        parameters=[{               # 参数配置
            'machine_type':'Mec'}   # 底盘选择
        ]
    )

    # 延时启动配置（YOLO启动5秒后启动自动驾驶）
    delayed_driver_launch = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=yolo_node,
            on_start=[
                TimerAction(
                    period=8.0,  # 5秒延时
                    actions=[driver_node],
                )
            ]
        )
    )

    # 自动驾驶节点退出时关闭整个系统
    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=driver_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='driver node exited'))
            ]
        )
    )

    return LaunchDescription([
        yolo_node,                      # 首先启动YOLO节点
        delayed_driver_launch,      # 注册延时启动自动驾驶
        shutdown_on_exit                # 注册退出
    ])