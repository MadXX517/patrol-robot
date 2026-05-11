import os
from glob import glob
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def resolve_lidar_port():
        by_id_ports = sorted(glob('/dev/serial/by-id/*CP210*'))
        if by_id_ports:
                return by_id_ports[0]

        if os.path.exists('/dev/ttyUSB0'):
                return '/dev/ttyUSB0'

        return '/dev/lidar'


def generate_launch_description():
        lidar_dir = get_package_share_directory('rplidar_ros')
        lidar_launch_dir = os.path.join(lidar_dir, 'launch')
        lidar_port = resolve_lidar_port()

        a1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_a1_launch.py')),
                launch_arguments={'serial_port': lidar_port}.items(),
        )
        c1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_c1_launch.py')),
                launch_arguments={'serial_port': lidar_port}.items(),
        )

        ld = LaunchDescription()
        '''
        Please select your lidar here, options include:
        a1,c1

        '''
        ld.add_action(c1)

        return ld
