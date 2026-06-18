include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,  -- 使用 MAP_BUILDER 模块
    trajectory_builder = TRAJECTORY_BUILDER,  -- 使用 TRAJECTORY_BUILDER 模块
    map_frame = "map",  -- 地图坐标系的名称
    -- NOTE(2026): 开 use_imu_data 后 cartographer 要求 IMU 与 tracking_frame 重合
    -- (sensor_bridge 检查平移<1e-5)。base_link→imu_link 偏 ~0.12m 会致 FATAL,
    -- 故 tracking_frame 用 imu_link。published_frame 仍 odom_combined,对外位姿经 TF 精确换算,地图/机器人位姿参考不变。
    tracking_frame = "imu_link",  -- 机器人基础坐标系的名称
    published_frame = "odom_combined",  -- 机器人发布里程计信息的坐标系
    odom_frame = "odom_combined",  -- 里程计坐标系的名称
    provide_odom_frame = false,  -- 是否由 Cartographer 提供里程计坐标系
    publish_frame_projected_to_2d = true,  -- 将发布的位姿投影到2D平面
    use_odometry = true,  -- 是否使用外部里程计数据
    use_nav_sat = false,  -- 是否使用卫星导航数据
    use_landmarks = false,  -- 是否使用地标进行定位
    num_laser_scans = 1,  -- 激光扫描传感器的数量
    num_multi_echo_laser_scans = 0,  -- 多回波激光扫描传感器的数量
    -- NOTE(2026): 去畸变(unwarp)开关。=1 时整帧100ms扫描当成单一时刻刚性点云,不做帧内运动补偿,
    -- 转动时一帧内车转 ω×0.1s(0.4rad/s→2.3°)畸变烤进每帧,×距离=远墙横向涂抹(5m处~20cm)变厚糊歪。
    -- 切成10片(各10ms)各自用 IMU+odom 先验去畸变。须与下方 num_accumulated_range_data=10 配套
    -- (把10子片重新累积成整帧再匹配,否则在稀疏子片上匹配反而变差)。
    num_subdivisions_per_laser_scan = 10,  -- 帧内切10片做去畸变(原1=不去畸变)
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
-- NOTE(2026): 与 num_subdivisions_per_laser_scan=10 配套:雷达每帧被切成10个子片,
-- 这里累积满10片(=一整圈扫描)各自去畸变后合并成一帧完整点云再做扫描匹配,
-- 保持匹配频率仍约10Hz(每整圈一次),但匹配前畸变已逐片校正。两参数必须一起改。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10  -- 累积10子片成整帧再匹配(配合去畸变,原1)
-- NOTE(2026): 开启 IMU 直喂 cartographer。原 false 时旋转先验只来自 odom_combined,
-- 转向时先验滞后/有误,配合高 rotation_weight(=200)会把小角度误差锁进地图,长时间累积偏转。
-- 陀螺已断电重标定(零偏 ≈0),imu_link TF 与 /imu/data(50Hz,含重力)就绪。
-- ImuTracker 高频积分陀螺 + 用重力持续纠正朝向,给扫描匹配一个稳的旋转先验,抑制累积偏转。
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 是否使用 IMU 数据
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- 激光扫描的最小范围
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- 激光扫描的最大范围(2026:5.0→12.0,原5m把16m雷达砍太狠,地图困在小气泡里 explore 无远方前沿可探;提到12m放开探索范围,远距噪声靠去畸变+missing_data_ray_length=1.0 抑制)
-- NOTE(2026): 远处细节消失(薄墙近距精准,远离一会被擦)。第二轮加码(第一轮 hit=0.62 不够):
-- 三管齐下抗擦除。hit_probability 0.62→0.65(命中更牢);
-- miss_probability 0.49→0.495(关键纠正:miss 越低擦除越狠,往0.5提则每次穿过的擦除减半);
-- missing_data_ray_length 3.0→1.0(真凶之一:无回波光束会沿射线把最远3m画成自由空间,
-- 远离薄墙时擦掉它;缩到1m限制这种远距离擦除)。过高会让移走障碍残留更久,均属温和。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65  -- 命中权重(默认0.55)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.495  -- 穿过擦除强度,往0.5提=擦得轻(默认0.49)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0  -- 缺失数据的射线长度(默认/原3.0,缩短限制远距擦除)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- 体素滤波器的大小
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) --设置运动过滤器的最大角度变化为 0.1 弧度，可以减少频繁发布的位姿更新，从而减少计算负担

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 自适应体素滤波器的最大长度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200  -- 自适应体素滤波器的最小点数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.0  -- 自适应体素滤波器的最大范围

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9  -- 回环闭合自适应体素滤波器的最大长度
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100  -- 回环闭合自适应体素滤波器的最小点数
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.0  -- 回环闭合自适应体素滤波器的最大范围

-- NOTE(2026 实验): 量化诊断发现转动时里程计先验稳(0.2cm)但 SLAM 游走(map->odom 摆 5-8°/Y 7-11cm),
-- 且误差不随转速正比放大 -> 非运动畸变,是 RT-CSM 暴力搜索在 ±5° 窗里逐帧跳不同相关峰。
-- 现 IMU 零偏已标定、odom 平移先验准,RT-CSM 救场作用已不需,反而引入游走。先关掉只靠 Ceres 精修。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- 关 RT-CSM(原true),靠准的先验+Ceres
-- NOTE(2026-05-17): 收紧 RT-CSM 搜索窗解决转向时 10-20° 瞬间误匹配。
-- 原值 linear=0.1m / angular=30° 过大，转向时 odom 外推误差 + 走廊对称纹理
-- 会让 RT-CSM 跳到搜索窗内的局部最优误匹配（10-20° 偏移正好落在 30° 窗内）。
-- 8° 足以覆盖 10Hz 间隔内任何合理的 odom 外推角误差。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05  -- 实时相关性扫描匹配的线性搜索窗口
-- NOTE(2026 第二轮): 第一轮 rotation_delta_cost_weight=1e0 后仍有少量跳变,继续加码:
-- 窗 8°→5°(硬上限:IMU 先验已准,跳变物理上限到 5°),cost weight 1e0→1e1(软惩罚再×10)。
-- 软硬双管:窗限制最大跳幅,权重压制窗内错误峰。仍有跳再降窗或升权重。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)  -- 实时相关性扫描匹配的角度搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e1  -- 惩罚旋转偏离先验(默认1e-1,二轮加到1e1)

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
