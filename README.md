# 基于飞凌 ELF 2 RK3588开发板的双模式军警巡逻辅助机器人

本仓库是 RK3588 巡逻小车的 ROS2 工作空间源码目录，围绕移动巡逻平台实现底盘控制、相机/雷达接入、YOLO/RKNN 视觉识别、人体跟随、云台追踪、手势控制、语音交互、建图导航、事件记录、钉钉告警、LLM 报告生成和网页操作台。

推荐日常使用入口是 `car_web` 网页操作台：它把多个 ROS2 功能包包装到浏览器页面里，负责统一启动、停止和避免节点竞争。各功能包仍然可以单独运行和排查，根 README 只做仓库总入口，具体参数以各子包 README 和 launch 文件为准。

## 系统架构

```text
硬件与基础链路
  car_base / OrbbecSDK_ROS2 / depend
  -> 底盘串口、深度相机、雷达、IMU、URDF、EKF、web_video_server

感知与行为
  car_yolo
  -> YOLO/RKNN 目标检测，发布 /car_yolo/object_detect 与 /result_img

  car_patrol
  -> 人体跟随、云台追踪、手势控制，使用 /cmd_vel、/ik_states、/patrol/command

  car_nav2 / car_slam / car_rviz2
  -> Cartographer/SLAM、Nav2、地图保存、定位导航、RViz 可视化

交互与上报
  car_report
  -> 订阅 YOLO 结果，生成事件 JSONL、截图和 Markdown 报告

  car_notify
  -> 订阅 /car_report/event，通过钉钉 webhook 推送 markdown 告警

  car_voice / car_llm
  -> 语音触发、语音播报、ASR/LLM/TTS 参数与大模型能力

统一操作层
  car_web
  -> 浏览器操作台，包装遥控、视频、功能中心、跟随、手势、语音、建图导航、报告和通知
```

## 功能总览

| 功能 | 主要包 | 说明 |
| --- | --- | --- |
| 底盘与传感器基础 | `car_base` | 启动底盘串口、深度相机、雷达、IMU、URDF、EKF 等基础节点 |
| 网页视频流 | `depend/web_video_server`、`car_web` | 将 ROS 图像话题以 MJPEG 方式在浏览器查看 |
| YOLO/RKNN 识别 | `car_yolo` | 端侧目标检测，输出结构化目标和带框图像 |
| 事件记录与报告 | `car_report` | 把识别结果转成事件日志、截图和 LLM 巡逻报告 |
| 钉钉告警 | `car_notify` | 订阅事件 JSON，按阈值和冷却规则推送钉钉机器人 |
| 人体跟随 | `car_patrol` | 锁定目标人，结合视觉和距离信息控制底盘保持跟随 |
| 云台追踪 | `car_patrol` | 底盘不动时用云台跟踪目标 |
| 手势控制 | `car_patrol` | 通过手势识别发送跟随、追踪、停止、重锁等高层指令 |
| 语音交互 | `car_voice`、`car_llm` | 支持语音触发、语音播报和大模型对话/控制能力 |
| 建图导航 | `car_slam`、`car_nav2` | 支持建图、地图保存、定位、目标点导航和巡航基础 |
| Web 统一操作 | `car_web` | 统一管理 core、跟随/巡检、建图导航、语音、告警和报告 |
| 机器人模型与仿真 | `car_urdf`、`car_moveit`、`car_rviz2` | URDF、MoveIt、RViz、Gazebo/可视化相关配置 |
| 视觉公共库 | `car_vision` | PID、坐标变换、机械臂逆解、手势模型等公共模块，被 `car_patrol` 等复用 |
| 自定义消息 | `car_msg`、`depend/interfaces` | 蜂鸣器、超声、目标检测等 ROS2 消息接口 |

## 快速开始

进入工作空间根目录，不是在 `src` 目录里构建：

```bash
cd ~/Desktop/ROS2/SRC_20260427
```

第一次运行或代码改动后构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果只改了某几个包，可以按需选择构建，例如：

```bash
colcon build --symlink-install --packages-select car_web car_report car_notify
source install/setup.bash
```

启动网页操作台：

```bash
ros2 launch car_web car_web.launch.py
```

浏览器访问：

```text
http://RK3588_IP:8000
```

例如小车 IP 是 `192.168.0.102`：

```text
http://192.168.0.102:8000
```

`192.168.0.102` 是在开发时所用路由器的 DHCP 设置里，把小车网卡 MAC 地址和这个 IP 绑定后得到的固定地址，只在那台路由器下有效。换了路由器或网络后，小车会重新获取地址，需要在新路由器的管理页面里给小车重新做一次 MAC 与 IP 的绑定（也就是静态 DHCP 分配或地址保留），或者在小车上用 `ip addr` 查到当前 IP 后再访问。

如果已经配置自动 source，可以不手动执行 `source install/setup.bash`。如果出现 `package 'xxx' not found`，先 source；仍找不到时重新 build。

## Web 操作台

`car_web` 是推荐的统一入口，详细说明见 [car_web/README.md](car_web/README.md)。

网页后端会自动或按按钮启动相关 ROS2 进程：

- core：底盘串口、深度相机、`web_video_server`。
- 跟随/巡检：普通遥控、YOLO 识别、事件记录、钉钉、报告、人体跟随、云台追踪、手势控制、语音播报。
- 建图导航：建图、保存地图、加载地图、定位、导航、巡航点管理。
- 安全控制：停止、底盘复位、云台回中、紧急报警、低电量状态显示。

启动 `car_web` 后，不要再手工重复启动相机、底盘串口、`web_video_server`、YOLO 或 Nav2 相关 launch，避免设备、端口和话题竞争。需要拆开排查时，先在网页点“全部停止”，或关闭 `car_web` 终端。

## 常用单模块命令

### 基础链路

只启动底盘串口、深度相机和网页视频服务：

```bash
ros2 launch car_web car_web_core.launch.py
```

检查相机图像：

```bash
ros2 topic info -v /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
```

浏览器查看视频服务首页：

```text
http://RK3588_IP:8080/
```

普通相机流：

```text
http://RK3588_IP:8080/stream?topic=/camera/color/image_raw&type=mjpeg
```

### YOLO 识别与事件记录

启动 YOLO + 事件记录：

```bash
ros2 launch car_report car_report_yolo.launch.py yolo_pub_result_img:=true
```

查看识别结果和事件：

```bash
ros2 topic echo /car_yolo/object_detect
ros2 topic echo /car_report/event
```

查看带框图像：

```text
http://RK3588_IP:8080/stream?topic=/result_img&type=mjpeg
```

### 钉钉与 LLM 报告

密钥均通过环境变量提供，代码中不保存。运行前在小车上设置（建议写入 `~/.bashrc`）：

```bash
export DASHSCOPE_API_KEY="<阿里云百炼 API Key>"          # car_llm / car_voice / car_report
export DINGTALK_WEBHOOK_URL="https://oapi.dingtalk.com/robot/send?access_token=<你的token>"  # car_notify / car_report / car_web
```

也可在 launch 时用 `api_key:=...`、`webhook_url:=...` 临时覆盖。

单独测试钉钉 webhook：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```

单独测试 LLM API：

```bash
ros2 run car_report report_generator --api-test
```

已有事件日志后生成报告：

```bash
ros2 run car_report report_generator --mode text
ros2 run car_report report_generator --mode vision --max-images 3
```

### 人体跟随、云台追踪和手势

人体跟随：

```bash
ros2 launch car_patrol human_follow.launch.py mode:=follow auto_start:=true
```

云台追踪：

```bash
ros2 launch car_patrol gimbal_track.launch.py auto_start:=true
```

手势控制：

```bash
ros2 launch car_patrol gesture_command.launch.py
```

常用检查：

```bash
ros2 topic echo /person_follow/state
ros2 topic echo /gesture_command/gesture
ros2 topic echo /patrol/command
```

### 语音

启动语音助手：

```bash
ros2 launch car_voice voice.launch.py enable_tts:=true enable_asr:=true
```

常用话题：

```bash
ros2 topic echo /voice/announce
ros2 topic echo /voice/control
```

### 建图导航

启动 Nav2/SLAM 组合入口：

```bash
ros2 launch car_nav2 car_nav2.launch.py
```

保存地图：

```bash
ros2 launch car_nav2 save_map.launch.py
```

常用检查：

```bash
ros2 topic echo /map
ros2 topic echo /scan
ros2 topic echo /odom_combined
ros2 action list
```

## 目录结构

| 目录 | 作用 |
| --- | --- |
| `car_base/` | 底盘串口、相机、雷达、URDF、IMU、EKF 等硬件基础 |
| `car_yolo/` | YOLO/RKNN 目标检测节点、模型配置和识别结果发布 |
| `car_patrol/` | 人体跟随、云台追踪、手势识别和巡逻控制 |
| `car_report/` | 视觉事件记录、截图留证、Markdown 报告生成 |
| `car_notify/` | 钉钉机器人事件推送 |
| `car_web/` | 浏览器操作台和后端进程管理 |
| `car_voice/` | 语音助手节点，负责语音触发和语音播报 |
| `car_llm/` | LLM/ASR/TTS 参数和大模型相关能力 |
| `car_nav2/` | Nav2 参数、地图、导航启动入口 |
| `car_slam/` | 建图包：`car_cartographer`（本项目使用的 Cartographer 2D）和 `car_gmapping`（备用） |
| `car_rviz2/` | RViz 可视化配置和启动入口 |
| `car_vision/` | 视觉公共库（`pid`、`common`、`transform`、`arm_ik_sdk`、MediaPipe 手势模型）及 RViz 多点导航 `nav2_waypoints`；其余颜色/AR/二维码等节点为厂商模板自带，本项目未使用 |
| `car_app/` | 厂商模板自带的机械臂应用示例（颜色抓取、码垛等），本项目未使用 |
| `car_keyboard/` | 键盘遥控节点，调试时可替代网页遥控 |
| `car_urdf/`、`car_moveit/` | 机器人模型、仿真和机械臂/MoveIt 相关配置 |
| `car_msg/`、`depend/interfaces/` | 自定义 ROS2 消息接口 |
| `depend/` | 第三方 ROS2 依赖，如 `web_video_server`、雷达、TEB、Explore 等 |
| `OrbbecSDK_ROS2/` | Orbbec/Astra 深度相机相关驱动包 |
| `yeabot_tools/` | 辅助工具脚本 |

## 常见排查

查看当前节点：

```bash
ros2 node list
```

查看关键话题：

```bash
ros2 topic list | grep -E "camera|image|yolo|report|patrol|gesture|voice|scan|map|cmd_vel"
```

相机被占用时，通常是之前的相机节点没有退出干净，或手工启动的 launch 与 `car_web` 重复。先关闭相关终端，再检查：

```bash
ps aux | grep -E "astra_camera|web_video_server|yolo_detect|car_base|car_web" | grep -v grep
```

如果网页没有画面，先直接打开 `web_video_server` 的视频直链，判断是网页嵌入问题还是 ROS 图像话题没有输出。

如果 YOLO 有框但事件少，优先检查 `min_score`、`target_classes`、`cooldown_sec`。如果画面框太多，优先检查 YOLO 的置信度阈值和类别过滤配置。

如果语音、报告或钉钉失败，先确认小车网络可访问对应云服务，再分别运行 `--api-test` 或 `--webhook-test`。

## 运行注意

- 每次新增 ROS2 包、修改 `setup.py`、`package.xml` 或 launch 文件后，都要重新 `colcon build`。
- 推荐使用 `--symlink-install`，方便 Python、HTML、CSS、JS 等源码改动快速生效。
- `car_web` 启动时会按进程名清理残留的功能节点（相机、底盘串口、YOLO、事件记录、跟随、手势、钉钉、语音），停止导航时会清理 Cartographer/Nav2/rosbridge/explore。清理按名称匹配，手动启动的同名节点也会被结束，需要单独调试时不要同时运行 `car_web`。
- 相机、底盘串口、8080 视频端口、YOLO 节点和 Nav2 栈都不建议重复启动。
- 遥控和跟随测试前确认周围安全，必要时先架空轮子。
- API Key、Webhook 等敏感配置不写入代码，统一通过环境变量提供，不要提交到仓库。
