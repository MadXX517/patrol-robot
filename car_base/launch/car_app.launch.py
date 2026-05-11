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

    #camera (astra depth camera)
    camera_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('car_base'),
            'launch/car_camera.launch.py'
        ]),
        launch_arguments={'camera_type': 'depth'}.items()
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
        camera_launch,
        rosbridge_launch,
        web_video_server_node
    ])

