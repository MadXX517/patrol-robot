import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 关键参数配置（核心修复：启用激光+里程计+视觉融合）
    parameters = [
        {
            # 基础配置：复用 gmapping 中正常的传感器话题
            'queue_size': 20,
            'frame_id': 'base_link',  # 修正：机器人基准坐标系（必须是 base_link，而非相机）
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,  # 启用深度图（深度相机必填）
            
            # 1. 启用激光雷达（复用 gmapping 正常的激光）
            'use_laser_scan': True,
            'laser_scan_topic': '/scan',  # 与 gmapping 激光话题一致
            'laser_min_range': 0.1,  # 按你的激光雷达实际参数调整
            'laser_max_range': 10.0,  # 按实际参数调整
            
            # 2. 启用轮式里程计（复用 gmapping 正常的里程计）
            'use_odometry': True,
            'odom_topic': '/odom_combined',  # 若 gmapping 用 /odom，这里改为 /odom（关键：保持一致）
            
            # 3. 融合策略（核心！视觉+激光ICP融合，修正视觉漂移）
            'OdomStrategy': 3,  # 2=视觉+激光ICP融合（推荐）
            'VO/Strategy': 0,  # 视觉里程计用 ORB-SLAM2（默认稳定）
            'VO/MinimumInliers': 20,  # 提高视觉匹配阈值，减少误匹配
            
            # 4. 闭环检测优化（修正累积误差）
            'LoopClosureThreshold': 0.75,  # 降低阈值，更容易触发闭环
            'LoopClosureMinimumInliers': 25,  # 提高闭环内点要求，减少误闭环
            'LoopClosureUseLaser': True,  # 激光辅助闭环（更稳定）
            'Optimizer': 'ceres',  # 用 ceres 优化器（比 g2o 稳定，需安装 ceres-solver）
            
            # 5. 里程计与 ICP 权重配置（信任轮式里程计，减少视觉干扰）
            'OdomSigmaX': 0.05,  # X方向噪声（值越大，对里程计信任度越低）
            'OdomSigmaY': 0.05,  # Y方向噪声
            'OdomSigmaTheta': 0.01,  # 旋转噪声（轮式里程计旋转更可靠，设小）
            'ICP/Iterations': 50,  # 增加 ICP 迭代次数，提高激光配准精度
            'ICP/InlierRatio': 0.5,  # 降低 ICP 内点门槛，更容易匹配
            
            # 6. 时间同步（解决视觉/激光时间戳不一致）
            'approx_sync': True,  # 允许近似同步（默认关闭，必须启用）
            'sync_queue_size': 20,  # 增大同步队列
        }
    ]

    # 话题重映射（保持与传感器发布话题一致）
    remappings = [
        ('odom', '/odom_combined'),  # 与上面 odom_topic 对应（若改 /odom，这里同步改）
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    return LaunchDescription([
        # 打印日志（方便调试）
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # 启动参数（仿真时间控制）
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # 启动 rtabmap 节点（删除了 gmapping 冲突节点）
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']  # -d 启用调试模式，终端会输出关键信息（如闭环、配准状态）
        ),

        # 可选：启动 rtabmap RViz 配置（若需要默认可视化）
        # Node(
        #     package='rtabmap_rviz',
        #     executable='rtabmap_rviz',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # )
    ])
