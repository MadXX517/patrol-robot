# 巡逻小车 ROS2 项目

本仓库是 RK3588 巡逻小车的 ROS2 工作空间源码。当前网页操作台已经可视化包装了这些功能：

- `car_base`：底盘串口、深度相机、网页视频服务。
- `car_yolo`：YOLO/RKNN 目标识别和 `/result_img` 置信框画面。
- `car_report`：视觉事件记录、截图保存、LLM 文本/图文报告生成。
- `car_notify`：订阅视觉事件并通过钉钉机器人推送告警。

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

例如小车 IP 是 `192.168.0.102`：

```text
http://192.168.0.102:8000
```

打开网页后，普通前方画面会自动启动。后续普通操作、功能中心、识别记录、钉钉通知、遥控、底盘复位、云台控制和报告生成都在网页按钮里完成。

如果同学已经配置了自动 source，可以不手动执行 `source install/setup.bash`。如果出现 `package 'car_web' not found`，先执行 source；还不行就重新 build。

## 模块单独测试

如果网页里某一块不正常，先在网页点“全部停止”，或者关掉 `car_web` 终端，再按下面拆开测，避免相机、底盘串口、8080 端口或 YOLO 节点竞争。

终端 1：只启动基础链路，包括底盘串口、相机和 `web_video_server`：

```bash
ros2 launch car_web car_web_core.launch.py
```

终端 2：检查普通相机是否有发布者和帧率：

```bash
ros2 topic info -v /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
```

浏览器：先打开视频服务首页，看它列出了哪些图像话题：

```text
http://RK3588_IP:8080/
```

浏览器：直接看普通相机流：

```text
http://RK3588_IP:8080/stream?topic=/camera/color/image_raw&type=mjpeg
```

终端 3：在终端 1 保持运行时，只启动 YOLO + report 识别记录链路：

```bash
ros2 launch car_report camp_security_yolo.launch.py
```

终端 4：检查 YOLO 画面和事件是否有输出：

```bash
ros2 topic hz /result_img
ros2 topic echo /car_report/event
```

浏览器：直接看带置信框的 YOLO 画面：

```text
http://RK3588_IP:8080/stream?topic=/result_img&type=mjpeg
```

减少画面杂框主要调 `yolo_conf_thres` 和 `target_classes`，减少事件/通知主要调 `min_score`、`cooldown_sec`。比赛警戒入口会把 YOLO 输出收敛到人员和包类目标，事件语义再归并为“人员闯入 / 遗留物”。

看 YOLO 和 report 是否真的有输出：

```bash
ros2 topic echo /car_yolo/object_detect
ros2 topic echo /car_report/event
```

单独测试 LLM API：

```bash
ros2 run car_report report_generator --api-test
```

单独测试钉钉 webhook：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```
## 网页操作台

详细说明见：`car_web/README.md`。

`car_web` 的后端会代替终端启动和关闭相关节点。启动 Web 后，不要再同时手工启动这些容易竞争的命令：

```bash
ros2 launch car_base car_app.launch.py
ros2 launch car_base car_camera.launch.py
ros2 launch car_report car_report_yolo.launch.py
ros2 launch car_notify dingtalk_notify.launch.py
ros2 run web_video_server web_video_server
```

重复启动可能导致相机、底盘串口、8080 视频端口或 YOLO 节点冲突。

## 主要目录

- `car_base/`：底盘串口、相机、雷达、URDF、EKF 等硬件基础。
- `car_yolo/`：YOLO/RKNN 识别，发布 `/car_yolo/object_detect` 和可选 `/result_img`。
- `car_report/`：把 YOLO 结果记录为事件，保存 JSONL、截图，并生成 LLM 巡逻报告。
- `car_notify/`：订阅 `/car_report/event`，通过钉钉机器人推送告警。
- `car_web/`：网页操作台，包装遥控、视频、功能中心、事件记录、通知和报告生成。
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

单独测试 LLM API：

```bash
ros2 run car_report report_generator --api-test
```

单独测试钉钉 webhook：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```

## 运行注意

- 每次新增 ROS2 包、改 `setup.py`、改 `package.xml`、改 launch 文件后，都要重新 `colcon build`。
- 推荐继续使用 `--symlink-install`；网页静态文件已兼容这种构建方式。
- 只改 Python、HTML、CSS、JS 时，`--symlink-install` 下通常能直接反映；上车演示前仍建议重新 build 一次。
- 网页操作台 v1 只接遥控、底盘复位、云台控制、普通视频、YOLO 事件记录、钉钉通知和报告；导航、SLAM、语音、机械臂后续稳定后再接入。
- 小车遥控有风险，测试时先架空轮子或确保周围安全。



