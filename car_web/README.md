# car_web 网页操作台

`car_web` 是 RK3588 小车的浏览器操作台。它不重新实现底盘、相机、YOLO、报告和钉钉逻辑，只是在网页里包装已有 ROS2 功能，点击按钮等价于后端替你执行对应命令。

当前包装的功能包：

- `car_base`：启动底盘串口、深度相机和 `web_video_server`。
- `car_yolo`：启动 YOLO/RKNN 识别，输出 `/car_yolo/object_detect` 和 `/result_img`。
- `car_report`：订阅 YOLO 结果，生成事件 JSON、截图和 Markdown 报告。
- `car_notify`：订阅 `/car_report/event`，推送钉钉 markdown 告警。

## 构建和启动

进入工作空间根目录：

```bash
cd ~/Desktop/ROS2/SRC_20260427
```

代码有改动后构建：

```bash
colcon build --symlink-install --packages-select car_web
source install/setup.bash
```

如果改了 `car_report`、`car_notify`、`car_yolo` 或底层依赖，建议全量构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动网页后端：

```bash
ros2 launch car_web car_web.launch.py
```

浏览器访问：

```text
http://RK3588_IP:8000
```

例如：

```text
http://192.168.0.102:8000
```

## 模块单独测试

如果网页内嵌画面没有图，先点页面里的“打开视频直链”和“视频服务首页”。如果直链也没有图，再关掉 `car_web` 或点击“全部停止”，按模块拆开测试。

终端 1：只启动基础链路，包括底盘串口、相机和 `web_video_server`：

```bash
ros2 launch car_web car_web_core.launch.py
```

终端 2：检查普通相机是否有发布者和帧率：

```bash
ros2 topic info -v /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
```

浏览器：先打开视频服务首页，确认 `web_video_server` 能看到哪些图像话题：

```text
http://RK3588_IP:8080/
```

浏览器：直接看普通相机流：

```text
http://RK3588_IP:8080/stream?topic=/camera/color/image_raw&type=mjpeg
```

终端 3：在终端 1 保持运行时，只启动 YOLO + report 识别记录链路：

```bash
ros2 launch car_report car_report_yolo.launch.py yolo_pub_result_img:=true yolo_conf_thres:=0.5 min_score:=0.5
```

终端 4：检查 YOLO 画面和 report 事件是否有输出：

```bash
ros2 topic hz /result_img
ros2 topic echo /car_report/event
```

浏览器：直接看 YOLO 置信框画面：

```text
http://RK3588_IP:8080/stream?topic=/result_img&type=mjpeg
```

减少画面杂框主要调 `yolo_conf_thres`，减少事件/通知主要调 `min_score`、`target_classes`、`cooldown_sec`。`target_classes` 只过滤事件和通知，不会改变 `/result_img` 上已经画出的 YOLO 框。

检查两个核心话题：

```bash
ros2 topic echo /car_yolo/object_detect
ros2 topic echo /car_report/event
```

API 和 webhook 单测：

```bash
ros2 run car_report report_generator --api-test
ros2 run car_notify dingtalk_notifier --webhook-test
```
## 启动关系

网页后端启动后会自动启动基础链路：

```text
car_web_core.launch.py
  -> car_base/launch/base_serial.launch.py
  -> car_base/launch/car_camera.launch.py camera_type:=depth
  -> web_video_server
```

因此打开网页后应默认显示普通前方画面 `/camera/color/image_raw`。

点击“功能列表”只会打开同页的功能中心，不会立即启动新节点。功能卡片进入详情页后，再点击“启动功能”才会执行对应 ROS2 launch。

点击“识别记录”后会启动：

```text
car_report_yolo.launch.py yolo_pub_result_img:=true
  -> car_yolo/yolo_detect
  -> car_report/event_recorder
```

此时网页画面切到 `/result_img`，也就是带 YOLO 置信框的画面。

点击“启动钉钉”后会启动：

```text
car_notify/launch/dingtalk_notify.launch.py cooldown_sec:=60
```

钉钉依赖 `/car_report/event`，所以必须先处于识别记录模式。

## 功能中心

当前功能中心已经接入：

- 视觉巡逻：启动 `car_report_yolo.launch.py`，显示 `/result_img` 置信框画面，记录 `/car_report/event`。

当前只保留入口、暂不启动节点的功能：

- 自主巡线
- 姿态检测
- 颜色追踪
- 雷达控制
- 语音控制

切换到一个真正启动的功能前，后端会先关闭上一个功能由 Web 自己启动的 report/notify 等节点，避免竞争。core 是共享基础链路，功能切换时默认保留；只有“关闭基础”或“全部停止”才关闭 core。

## 按钮含义

- 普通操作：保持 core 运行，关闭识别和钉钉，画面回到 `/camera/color/image_raw`。
- 关闭基础：先关闭识别和钉钉，再关闭 core；同时发送零速度。
- 识别记录：确保 core 已运行，再启动 YOLO 和事件记录，画面切到 `/result_img`。
- 关闭识别：关闭 report 和 notify，core 保持运行，画面回到普通相机。
- 全部停止：关闭 Web 自己启动的 core、report、notify，并发送零速度。
- 功能列表：打开同页功能中心，点击功能卡片只看详情，不直接启动。
- 底盘复位：连续发布零速度到 `/cmd_vel`，相当于轮子/底盘停车复位；它不控制云台，也不关闭节点。
- 云台回中：通过 `/ik_states` 设置 `joint0=0`、`joint3=1.2`，让摄像头云台回到中位。
- 左看/右看：通过 `/ik_states` 微调 `joint0`。
- 上看/下看：通过 `/ik_states` 微调 `joint3`。如果实车方向和按钮文字相反，后续只需要交换加减方向。
- 启动钉钉/停止钉钉：只控制钉钉通知节点。
- 文本报告：调用 `report_generator --mode text`，只把事件摘要发给 `car_llm` 的 `llm_model`。
- 图文报告：调用 `report_generator --mode vision --max-images 3`，把事件摘要和最多 3 张截图发给 `car_llm` 的 `vision_model`。

## 遥控逻辑

方向键是“按住才动”：

- 按住前进/后退/左转/右转时，前端持续请求 `/api/drive`。
- 松开、移出按钮、切换窗口或页面失焦时，前端发送停止。
- 后端用 ROS timer 每 100ms 发布 `/cmd_vel`，超过 `drive_timeout` 没收到新指令会自动发布零速度。

“停止”和“底盘复位”的区别：

- 停止：普通停车，发送一次零速度。
- 底盘复位：连续多次发送零速度，不关闭相机、YOLO 或网页节点。

## 视频显示

网页通过 `web_video_server` 显示 MJPEG：

- 普通操作：`/camera/color/image_raw`
- 识别记录：`/result_img`

视频区域右上角有“打开视频直链”。如果页面内没有图，先点这个直链判断是网页嵌入问题，还是 `web_video_server`/ROS 图像话题没有输出。

视频区只保留一行诊断提示，会显示 8080 是否连上、当前图像话题是否有发布者。更详细的图像话题列表请点“视频服务首页”。

## 注意事项

启动 `car_web` 后，不要再手动启动这些命令，避免相机、底盘串口、8080 端口或 YOLO 节点竞争：

```bash
ros2 launch car_base car_app.launch.py
ros2 launch car_base car_camera.launch.py
ros2 launch car_report car_report_yolo.launch.py
ros2 launch car_notify dingtalk_notify.launch.py
ros2 run web_video_server web_video_server
```

`car_web` 只停止它自己启动的进程，不会主动杀掉你手动启动的其他 ROS2 节点。如果你已经手动启动过冲突节点，建议先关掉相关终端，再重新启动 `car_web`。

## 常用排查

看基础节点是否有相机输出：

```bash
ros2 topic hz /camera/color/image_raw
```

看 YOLO 是否有识别结果：

```bash
ros2 topic echo /car_yolo/object_detect
```

看 report 是否发布事件：

```bash
ros2 topic echo /car_report/event
```

看底盘复位是否发布零速度：

```bash
ros2 topic echo /cmd_vel
```

看云台回中/上下左右微调是否发布关节消息：

```bash
ros2 topic echo /ik_states
```

`/ik_states` 的 `position` 应包含 7 个值：前 6 个是关节角，最后 1 个是动作时间。左看/右看会改变 `joint0`，上看/下看会改变 `joint3`，云台回中会把 `joint0` 设为 `0`、`joint3` 设为约 `1.2`。

单独测试 LLM API：

```bash
ros2 run car_report report_generator --api-test
```

单独测试钉钉 webhook：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```



