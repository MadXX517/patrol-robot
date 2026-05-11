from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    #base_serial
    base_serial_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('car_base'),
            'launch/base_serial.launch.py'
        ])
    )

    #usb_cam
    usb_cam_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('car_base'),
            'launch/usb_camera.launch.py'
        ])
    )


    #rosbridge_websocket
    rosbridge_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rosbridge_server'),
            'launch/rosbridge_websocket_launch.xml'
        ])
    )

    #web_video_server
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    return LaunchDescription([
        base_serial_launch,
        usb_cam_launch,
        rosbridge_launch,
        web_video_server_node
    ])

