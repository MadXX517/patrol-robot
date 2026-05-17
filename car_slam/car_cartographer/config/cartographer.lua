include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,  -- 使用 MAP_BUILDER 模块
    trajectory_builder = TRAJECTORY_BUILDER,  -- 使用 TRAJECTORY_BUILDER 模块
    map_frame = "map",  -- 地图坐标系的名称
    tracking_frame = "base_link",  -- 机器人基础坐标系的名称
    published_frame = "odom_combined",  -- 机器人发布里程计信息的坐标系
    odom_frame = "odom_combined",  -- 里程计坐标系的名称
    provide_odom_frame = false,  -- 是否由 Cartographer 提供里程计坐标系
    publish_frame_projected_to_2d = true,  -- 将发布的位姿投影到2D平面
    use_odometry = true,  -- 是否使用外部里程计数据
    use_nav_sat = false,  -- 是否使用卫星导航数据
    use_landmarks = false,  -- 是否使用地标进行定位
    num_laser_scans = 1,  -- 激光扫描传感器的数量
    num_multi_echo_laser_scans = 0,  -- 多回波激光扫描传感器的数量
    num_subdivisions_per_laser_scan = 1,  -- 每次激光扫描的子扫描数
    num_point_clouds = 0,  -- 点云传感器的数量
    lookup_transform_timeout_sec = 0.2,  -- 查找坐标变换的超时时间
    submap_publish_period_sec = 0.3,  -- 发布子地图的时间间隔
    pose_publish_period_sec = 5e-3,  -- 发布位姿的时间间隔
    trajectory_publish_period_sec = 30e-3,  -- 发布轨迹的时间间隔
    rangefinder_sampling_ratio = 1.,  -- 激光雷达数据的采样比例
    odometry_sampling_ratio = 1.,  -- 里程计数据的采样比例
    fixed_frame_pose_sampling_ratio = 1.,  -- 固定坐标系位姿数据的采样比例
    imu_sampling_ratio = 1.,  -- IMU 数据的采样比例
    landmarks_sampling_ratio = 1.,
  }
  
  -- 设置 Map Builder 使用 2D 模式
  MAP_BUILDER.use_trajectory_builder_2d = true

-- 设置 2D 轨迹构建器参数
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 每次扫描积累的激光数据量
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 是否使用 IMU 数据
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- 激光扫描的最小范围
TRAJECTORY_BUILDER_2D.max_range = 5.0  -- 激光扫描的最大范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0  -- 缺失数据的射线长度
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- 体素滤波器的大小
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) --设置运动过滤器的最大角度变化为 0.1 弧度，可以减少频繁发布的位姿更新，从而减少计算负担

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 自适应体素滤波器的最大长度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200  -- 自适应体素滤波器的最小点数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.0  -- 自适应体素滤波器的最大范围

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9  -- 回环闭合自适应体素滤波器的最大长度
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100  -- 回环闭合自适应体素滤波器的最小点数
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.0  -- 回环闭合自适应体素滤波器的最大范围

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关性扫描匹配
-- NOTE(2026-05-17): 收紧 RT-CSM 搜索窗解决转向时 10-20° 瞬间误匹配。
-- 原值 linear=0.1m / angular=30° 过大，转向时 odom 外推误差 + 走廊对称纹理
-- 会让 RT-CSM 跳到搜索窗内的局部最优误匹配（10-20° 偏移正好落在 30° 窗内）。
-- 8° 足以覆盖 10Hz 间隔内任何合理的 odom 外推角误差。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05  -- 实时相关性扫描匹配的线性搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(8.0)  -- 实时相关性扫描匹配的角度搜索窗口

-- NOTE(2026-05-17): 提高 ceres 旋转/平移权重以更信任运动先验，抑制转向时跳变。
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0  -- Ceres 扫描匹配的平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0  -- Ceres 扫描匹配的旋转权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4  -- Ceres 解算器的线程数

POSE_GRAPH.optimization_problem.huber_scale = 1e1  -- Huber 损失函数的尺度
POSE_GRAPH.optimize_every_n_nodes = 35  -- 每隔多少个节点优化一次 50
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 约束构建器的采样比例
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 约束构建器的最大约束距离
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 约束构建器的最小得分
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 全局定位的最小得分
POSE_GRAPH.matcher_translation_weight = 5e2  -- 匹配器的平移权重
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 匹配器的旋转权重
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10  -- Ceres 解算器的最大迭代次数
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4  -- Ceres 解算器的线程数
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0  -- 快速相关性扫描匹配的线性搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.0)  -- 快速相关性扫描匹配的角度搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- 快速相关性扫描匹配的分支和界限深度


return options
