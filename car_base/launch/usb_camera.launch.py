import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    cam_parameters = [
        {'video_device': '/dev/video0',
         'image_width': 640,
         'image_height': 480,
         'framerate': 30.0,
         'av_device_format': 'YUV422P',
         'camera_name': 'usb_cam',
         'camera_info_url': 'package://car_base/config/usb_cam.yaml'
         }
    ]

    return LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=cam_parameters,
            remappings=[
            	('image_raw', '/usb_cam/image_raw'),
            	('image_raw/compressed', '/usb_cam/image_compressed'),
                ('image_raw/compressedDepth', '/usb_cam/compressedDepth'),
                ('image_raw/theora', '/usb_cam/image_raw/theora'),
                ('camera_info', '/usb_cam/camera_info'),
            ]
        ),
    ])


