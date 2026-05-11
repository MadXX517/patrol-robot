from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():

    robot_parameters = [
        {'usart_port_name': '/dev/ttyS9',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_link',
         'odom_frame_id': 'odom_combined',
         'cmd_vel': 'cmd_vel',
         'product_number': 0}
    ]

    return LaunchDescription([
        launch_ros.actions.Node(
            package='car_base',
            executable='car_base_node',
            parameters=robot_parameters,
        )
    ])
