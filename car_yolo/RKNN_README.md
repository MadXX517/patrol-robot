# car_yolo 的 RKNN / rkNPU 推理说明

这个包现在支持两种 YOLOv5 推理后端：

- `backend:=rknn`：使用 Rockchip RKNN Runtime，在 RK 板子的 NPU 上推理。当前 launch 默认使用这个后端。
- `backend:=torch`：保留原来的 PyTorch YOLOv5 推理路径，只作为临时回退或对照测试使用。

注意：在小车板子上运行程序，不等于一定用了 NPU。只有加载 `.rknn` 模型，并且代码走 `RKNNLite` 推理，才算使用 rkNPU。

## 1. 上板前必须准备的文件

仓库现在只有 `.pt` 权重，RKNN Runtime 不能直接运行 `.pt`。上车前需要先把模型转换成 `.rknn`，并放到：

```bash
car_yolo/config/traffic_640n_7.rknn
```

如果你是在 ROS2 工作区里 `colcon build` 后运行，模型会被安装到 `install/car_yolo/share/car_yolo/config/`。因此有两种做法：

```bash
# 做法 A：先把 .rknn 放到 src 里，再重新 build
cp traffic_640n_7.rknn ~/patrol_ws/src/car_yolo/config/
cd ~/patrol_ws
colcon build --symlink-install

# 做法 B：已经 build 过时，直接把 .rknn 放到安装后的 share 目录
cp traffic_640n_7.rknn ~/patrol_ws/install/car_yolo/share/car_yolo/config/
```

如果 RKNN 输出没有类别名，还需要准备类别文件，每行一个类别名：

```bash
car_yolo/config/traffic_640n_7.names
```

也可以启动时手动传类别名：

```bash
ros2 run car_yolo yolo_detect --ros-args -p class_names:="class0,class1,class2"
```

## 2. 板子环境检查

进入小车板子后，先确认 ROS2 环境和工作区环境已经 source：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/patrol_ws/install/setup.bash
```

确认 RKNN Runtime 可用：

```bash
python3 -c "from rknnlite.api import RKNNLite; print('RKNNLite OK')"
```

如果这里报 `ModuleNotFoundError`，说明板子没有安装 `rknn-toolkit-lite2`，YOLO NPU 节点会启动失败。

确认模型文件存在：

```bash
ls -lh ~/patrol_ws/install/car_yolo/share/car_yolo/config/traffic_640n_7.rknn
```

确认摄像头话题后面会出现：

```bash
ros2 topic list | grep camera
```

## 3. 推荐调试顺序

不要一上来就启动全车。建议按下面顺序逐层调试：

1. 只启动相机。
2. 确认相机图像话题有数据。
3. 单独启动 YOLO NPU 节点。
4. 确认 YOLO 识别结果话题有数据。
5. 再启动事件记录、自动驾驶或全车 bringup。

这样出问题时能快速判断是相机、模型、NPU Runtime、YOLO 后处理，还是下游节点的问题。

## 4. 只启动相机

深度相机默认使用：

```bash
ros2 launch car_base car_camera.launch.py camera_type:=depth
```

如果临时使用普通 USB 摄像头：

```bash
ros2 launch car_base car_camera.launch.py camera_type:=usb
```

深度相机代码里默认彩色图像话题一般是：

```bash
/camera/color/image_raw
```

USB 摄像头 launch 里发布的是：

```bash
/usb_cam/image_raw
```

确认相机是否真的在发图：

```bash
ros2 topic list | grep image
ros2 topic hz /camera/color/image_raw
```

如果你用的是 USB 摄像头，把上一条换成：

```bash
ros2 topic hz /usb_cam/image_raw
```

也可以查看图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

如果板子没有桌面环境，就先用 `ros2 topic hz` 判断是否有帧率。

## 5. 单独启动 YOLO NPU 节点

相机确认有图后，再开一个终端，source 环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/patrol_ws/install/setup.bash
```

使用默认 NPU launch：

```bash
ros2 launch car_yolo car_yolo.launch.py
```

等价的显式启动方式：

```bash
ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=rknn \
  -p model:=traffic_640n_7 \
  -p rknn_model:=traffic_640n_7.rknn \
  -p image_topic:=/camera/color/image_raw \
  -p img_size:=640 \
  -p conf_thres:=0.25 \
  -p iou_thres:=0.45 \
  -p npu_core:=auto
```

如果你用的是 USB 摄像头，需要改图像话题：

```bash
ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=rknn \
  -p model:=traffic_640n_7 \
  -p rknn_model:=traffic_640n_7.rknn \
  -p image_topic:=/usb_cam/image_raw
```

如果板子接了显示器，并且想看标注图窗口：

```bash
ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=rknn \
  -p model:=traffic_640n_7 \
  -p rknn_model:=traffic_640n_7.rknn \
  -p image_topic:=/camera/color/image_raw \
  -p show_result:=true
```

如果没有显示器，不要开 `show_result`，否则 `cv2.imshow` 可能导致窗口相关错误。可以改为发布标注图：

```bash
ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=rknn \
  -p model:=traffic_640n_7 \
  -p rknn_model:=traffic_640n_7.rknn \
  -p image_topic:=/camera/color/image_raw \
  -p pub_result_img:=true
```

## 6. 检查 YOLO 输出

YOLO 节点会发布两个主要话题：

```bash
/car_yolo/yolo_result
/car_yolo/object_detect
```

查看结构化识别结果：

```bash
ros2 topic echo /car_yolo/object_detect
```

查看是否有发布频率：

```bash
ros2 topic hz /car_yolo/object_detect
ros2 topic hz /car_yolo/yolo_result
```

如果开启了 `pub_result_img:=true`，可以查看标注图：

```bash
ros2 topic hz /car_yolo/result_img
ros2 run rqt_image_view rqt_image_view
```

## 7. 判断是否真的用了 NPU

启动日志里应该看到类似：

```text
YOLO backend=rknn model=.../traffic_640n_7.rknn
```

如果看到：

```text
YOLO backend=torch
```

那就是走了 PyTorch 回退，不是 NPU。

还可以观察 CPU 占用：

```bash
top
```

如果 Python 进程 CPU 长时间很高，说明可能没有真正走 NPU，或者后处理太重。NPU 推理正常时，CPU 占用通常会明显低于纯 PyTorch CPU 推理。

## 8. 启动 YOLO + 事件记录

如果只想验证“识别到目标后能不能记录事件”，启动：

```bash
ros2 launch car_report car_report_yolo.launch.py
```

这个 launch 会启动：

- `car_yolo/yolo_detect`
- `car_report/event_recorder`

但它不会启动相机。所以需要先在另一个终端启动相机：

```bash
ros2 launch car_base car_camera.launch.py camera_type:=depth
```

如果要显式指定 NPU 参数：

```bash
ros2 launch car_report car_report_yolo.launch.py \
  yolo_backend:=rknn \
  yolo_model:=traffic_640n_7 \
  yolo_rknn_model:=traffic_640n_7.rknn \
  image_topic:=/camera/color/image_raw
```

查看事件：

```bash
ros2 topic echo /car_report/event
```

## 9. 启动自动驾驶 + YOLO

当前自动驾驶和 YOLO 的组合 launch 是：

```bash
ros2 launch car_vision driver_yolo.launch.py
```

这个 launch 会启动：

- `car_yolo/yolo_detect`
- `car_vision/driver`

注意：它本身不启动相机、底盘、雷达。所以正式跑自动驾驶前，通常还要先启动基础硬件：

```bash
ros2 launch car_base car_base.launch.py
```

然后另一个终端启动：

```bash
ros2 launch car_vision driver_yolo.launch.py
```

如果只是调 YOLO，不建议一开始就用这个组合 launch。先用“只启动相机 + 单独启动 YOLO”的方式确认 NPU 识别稳定。

## 10. 启动全车基础硬件

全车基础硬件 launch：

```bash
ros2 launch car_base car_base.launch.py
```

它会启动：

- 底盘串口：`car_base/launch/base_serial.launch.py`
- 相机：`car_base/launch/car_camera.launch.py`
- 雷达：`car_base/launch/car_lidar.launch.py`
- 机器人模型：`robot_mode_description.launch.py`
- IMU 滤波：`imu_filter_madgwick_node`
- EKF：`robot_localization/ekf_node`

调试 YOLO 时，如果你只需要摄像头，不需要一开始就启动这个全套。

## 11. 常见问题

### 找不到 `.rknn` 模型

报错类似：

```text
RKNN model not found
```

处理：

```bash
ls -lh ~/patrol_ws/install/car_yolo/share/car_yolo/config/
```

确认 `traffic_640n_7.rknn` 在安装后的 `config` 目录里。

### 找不到 `rknnlite`

报错类似：

```text
ModuleNotFoundError: No module named 'rknnlite'
```

处理：在 RK 板子上安装 Rockchip 的 `rknn-toolkit-lite2`，直到下面命令成功：

```bash
python3 -c "from rknnlite.api import RKNNLite; print('OK')"
```

### 没有识别结果

按顺序检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /car_yolo/object_detect
```

如果相机没有帧率，先修相机。  
如果相机有帧率但 YOLO 没输出，检查 `.rknn` 模型、类别文件、置信度阈值和启动日志。

### 没有显示窗口

无桌面环境时不要用：

```bash
-p show_result:=true
```

改用：

```bash
-p pub_result_img:=true
```

然后通过话题查看标注图。

## 12. CPU 回退对照

如果怀疑 RKNN 后处理或模型转换有问题，可以临时用 CPU 路径做对照：

```bash
ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=torch \
  -p device:=cpu \
  -p model:=traffic_640n_7 \
  -p image_topic:=/camera/color/image_raw
```

这个命令只用于排查，不是 issue 的最终目标。
