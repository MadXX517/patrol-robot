from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def resolve_depth_camera_launch():
    candidates = [
        ('astra_camera', 'astra_pro.launch.xml', FrontendLaunchDescriptionSource),
        ('astra_camera', 'dabai_dcw.launch.xml', FrontendLaunchDescriptionSource),
        ('orbbec_camera', 'astra.launch.py', PythonLaunchDescriptionSource),
        ('orbbec_camera', 'astra2.launch.py', PythonLaunchDescriptionSource),
        ('orbbec_camera', 'dabai_dcw2.launch.py', PythonLaunchDescriptionSource),
    ]

    for package_name, launch_name, source_type in candidates:
        try:
            package_share = get_package_share_directory(package_name)
            return os.path.join(package_share, 'launch', launch_name), source_type
        except Exception:
            continue

    raise RuntimeError('Neither orbbec_camera nor astra_camera is available for depth camera launch')

def generate_launch_description():
    # 声明一个参数，用于判断启动哪种相机
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='depth',  # 默认值为 'depth'
        description='Type of camera to launch (depth or usb)'
    )

    # 获取深度相机和USB相机的launch文件路径
    depth_camera_launch_file, depth_camera_source = resolve_depth_camera_launch()

    usb_camera_launch_file = os.path.join(
        get_package_share_directory('car_base'),  # 替换为USB相机包名
        'launch',
        'usb_camera.launch.py'  # 替换为USB相机的launch文件名
    )

    # 根据参数决定启动哪个相机节点
    depth_camera_launch = IncludeLaunchDescription(
        depth_camera_source(depth_camera_launch_file),
        launch_arguments={
            'color_width': '1280',
            'color_height': '720',
            'color_fps': '30',
            'use_uvc_camera': 'true',
            'uvc_vendor_id': '0x2bc5',
            'uvc_product_id': '0x0501',
            'uvc_camera_format': 'mjpeg',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'ir_width': '640',
            'ir_height': '480',
            'ir_fps': '30',
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'depth'"])
        )
    )

    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usb_camera_launch_file),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'usb'"])
        )
    )

    # 创建LaunchDescription并添加动作
    return LaunchDescription([
        camera_type_arg,
        depth_camera_launch,
        usb_camera_launch,
    ])
