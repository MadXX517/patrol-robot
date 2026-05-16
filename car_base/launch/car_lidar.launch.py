import math
import os
from glob import glob
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


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

        # Wrap the rplidar include in a GroupAction with SetRemap so the
        # driver publishes to /scan_raw; the scan_angle_filter node
        # republishes the cleaned scan on /scan for AMCL/costmap/SLAM.
        a1 = GroupAction([
                SetRemap(src='scan', dst='scan_raw'),
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_a1_launch.py')),
                        launch_arguments={'serial_port': lidar_port}.items(),
                ),
        ])
        c1 = GroupAction([
                SetRemap(src='scan', dst='scan_raw'),
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_c1_launch.py')),
                        launch_arguments={'serial_port': lidar_port}.items(),
                ),
        ])

        # Filter out the lidar-frame [-pi/2, +pi/2] sector (car rear / self
        # occlusion). Configure via parameters if mounting changes.
        scan_filter = Node(
                package='car_base',
                executable='scan_angle_filter.py',
                name='scan_angle_filter',
                output='screen',
                parameters=[{
                        'input_topic': '/scan_raw',
                        'output_topic': '/scan',
                        'block_lower': -math.pi / 2.0,
                        'block_upper':  math.pi / 2.0,
                }],
        )

        ld = LaunchDescription()
        '''
        Please select your lidar here, options include:
        a1,c1

        '''
        ld.add_action(c1)
        ld.add_action(scan_filter)

        return ld
