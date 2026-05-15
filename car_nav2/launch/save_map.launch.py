#ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    # 保存到当前用户家目录下的 maps/car（与具体用户名解耦，避免硬编码 /home/pi）
    map_dir = os.path.expanduser('~/maps')
    os.makedirs(map_dir, exist_ok=True)
    map_basename = os.path.join(map_dir, 'car')

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_basename],

        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)

    return ld
 
