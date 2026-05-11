from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    use_depth_camera = LaunchConfiguration('use_depth_camera', default='false')
    
    use_depth_camera_arg = DeclareLaunchArgument(
        'use_depth_camera',
        default_value='false',
        description='启动深度相机模式 (true/false) 或使用参数 usb/depth'
    )
    
    # 转换参数逻辑
    use_depth_condition = IfCondition(LaunchConfiguration('use_depth_camera'))
    use_usb_condition = UnlessCondition(LaunchConfiguration('use_depth_camera'))
    
    # 基础功能启动
    base_serial_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('car_base'),
            'launch/car_base.launch.py'
        ])
    )

    rosbridge_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rosbridge_server'),
            'launch/rosbridge_websocket_launch.xml'
        ])
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )
    
    # 手势识别节点
    app_hand_gesture_arm = Node(
        package='car_app',
        executable='app_hand_gesture_arm',
        name='app_hand_gesture_arm',
        output='screen',
    )

    app_color_follow = Node(
        package='car_app',
        executable='app_object_tracking',
        name='app_object_tracking',
        output='screen',
    )

    # 颜色识别节点
    app_color_pick = Node(
        package='car_app',
        executable='app_color_pick',
        name='app_color_pick',
        output='screen',
        condition=use_usb_condition
    )

    # 深度版本颜色识别节点
    app_color_pick_d = Node(
        package='car_app',
        executable='app_color_pick_d',
        name='app_color_pick_d',
        output='screen',
        condition=use_depth_condition
    )

    # 标签堆叠节点
    app_tag_stackup = Node(
        package='car_app',
        executable='app_tag_stackup',
        name='app_tag_stackup',
        output='screen',
        condition=use_usb_condition
    )

    # 深度版本标签堆叠节点
    app_tag_stackup_d = Node(
        package='car_app',
        executable='app_tag_stackup_d',
        name='app_tag_stackup_d',
        output='screen',
        condition=use_depth_condition
    )
    
    # 日志信息
    usb_mode_log = LogInfo(
        msg="启动USB摄像头模式",
        condition=use_usb_condition
    )
    
    depth_mode_log = LogInfo(
        msg="启动深度摄像头模式",
        condition=use_depth_condition
    )

    return LaunchDescription([
        use_depth_camera_arg,
        base_serial_launch,
        rosbridge_launch,
        web_video_server_node,
        app_hand_gesture_arm,
        app_color_follow,
        app_color_pick,
        app_color_pick_d,
        app_tag_stackup,
        app_tag_stackup_d,
        usb_mode_log,
        depth_mode_log
    ])