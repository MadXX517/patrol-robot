# 巡逻小车 ROS2 项目

本仓库是 RK3588 小车的 ROS2 工作空间源码，包含底盘、相机、YOLO/RKNN 识别、视觉事件记录、GLM 报告、钉钉通知和网页操作台。

## 最常用流程

进入工作空间根目录，不是 `src` 目录：

```bash
cd ~/Desktop/ROS2/SRC_20260427
```

第一次运行或代码有改动后构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

如果只改了网页操作台，可以只构建 `car_web`：

```bash
colcon build --symlink-install --packages-select car_web
source install/setup.bash
```

启动网页操作台：

```bash
ros2 launch car_web car_web.launch.py
```

浏览器打开：

```text
http://RK3588_IP:8000
```

打开网页后，不需要再手动敲 `car_report`、`car_notify`、`web_video_server` 等命令。普通操作、识别记录、钉钉通知、遥控、急停和报告生成都在网页按钮里完成。

如果同学已经配置了自动 source，可以不手动执行 `source install/setup.bash`。如果出现 `package 'car_web' not found`，先执行 source；还不行就重新 build。

## 网页操作台

`car_web` 是浏览器 GUI 包：

- 普通操作：启动底盘串口、深度相机、网页视频服务，显示 `/camera/color/image_raw` 前方画面。
- 识别记录：启动 `car_report_yolo.launch.py yolo_pub_result_img:=true`，显示 `/result_img`，画面带 YOLO 置信框。
- 钉钉通知：在识别记录模式下启动 `car_notify`，订阅 `/car_report/event` 后推送钉钉。
- 遥控：网页按钮通过后端发布 `/cmd_vel`。
- 急停：后端连续发布零速度到 `/cmd_vel`。
- 报告：调用 `car_report report_generator` 生成文本报告或图文报告。

启动 Web 后不要再同时手工启动这些容易竞争的命令：

```bash
ros2 launch car_base car_app.launch.py
ros2 launch car_report car_report_yolo.launch.py
ros2 launch car_notify dingtalk_notify.launch.py
ros2 run web_video_server web_video_server
```

这些节点由网页后端统一管理，重复启动可能导致相机、底盘串口、端口或 YOLO 节点冲突。

## 主要目录

- `car_base/`：底盘串口、相机、雷达、URDF、EKF 等硬件基础。
- `car_yolo/`：YOLO/RKNN 识别，发布 `/car_yolo/object_detect` 和可选 `/result_img`。
- `car_report/`：把 YOLO 结果记录为事件，保存 JSONL、截图，并生成 GLM 巡逻报告。
- `car_notify/`：订阅 `/car_report/event`，通过钉钉机器人推送告警。
- `car_web/`：网页操作台，包装遥控、视频、事件记录、通知和报告生成。
- `car_llm/`：语音和大模型控制相关代码。
- `car_vision/`、`car_app/`：视觉任务、跟随、颜色识别、手势、导航辅助等功能。
- `depend/`、`OrbbecSDK_ROS2/`：第三方 ROS2 依赖和相机相关包。

## 常见检查

查看相机图像话题：

```bash
ros2 topic hz /camera/color/image_raw
```

查看 YOLO 识别结果：

```bash
ros2 topic echo /car_yolo/object_detect
```

查看事件记录输出：

```bash
ros2 topic echo /car_report/event
```

单独测试 GLM API：

```bash
ros2 run car_report report_generator --api-test
```

单独测试钉钉 webhook：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```

## 运行注意

- 每次新增 ROS2 包、改 `setup.py`、改 `package.xml`、改 launch 文件后，都要重新 `colcon build`。
- `--symlink-install` 下，很多 Python、HTML、CSS、JS 改动能直接反映，但上车演示前仍建议重新 build 一次。
- 网页操作台 v1 只接遥控、急停、视觉事件、钉钉通知和报告；导航、SLAM、语音、机械臂后续稳定后再接入。
- 小车遥控有风险，测试时先架空轮子或确保周围安全。
