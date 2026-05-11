import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition

def generate_launch_description():
    base_dir = get_package_share_directory('car_base')
    base_launch_dir = os.path.join(base_dir, 'launch')

    gmapping_dir = get_package_share_directory('car_gmapping')
    gmapping_launch_dir = os.path.join(gmapping_dir, 'launch')

    # serial = IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(base_launch_dir, 'car_base.launch.py')),)
    # lidar = IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(base_launch_dir, 'car_lidar.launch.py')),)            
    
    gmapping = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gmapping_launch_dir, 'slam_gmapping.launch.py')),)     

    ld = LaunchDescription()
    '''
    Please select your lidar here, options include:
    a1,c1

    '''
    # ld.add_action(serial)
    # ld.add_action(lidar)
    ld.add_action(gmapping)
    return ld
