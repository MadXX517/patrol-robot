# car_yolo RKNN / rkNPU 部署记录

本文记录 `car_yolo` 从原来的 PyTorch CPU 推理切换到 RKNN / rkNPU 推理的完整流程。当前已在 ELF 2 / RK3588 小车上跑通：

```text
相机 -> YOLOv5s RKNN/NPU -> /car_yolo/object_detect
-> car_report 事件记录 + 截图留证
-> /car_report/event
-> car_notify 钉钉机器人推送
```

已验证的钉钉事件包含 `person` 类别、置信度、bbox、截图大小和截图路径，说明推理、事件记录、截图保存、消息通知链路已经闭环。

## 1. 当前结论

- 小车芯片平台：`rk3588`
- 板端运行库：`rknn-toolkit-lite2` / RKNN Runtime `2.1.0`
- 模型转换工具：RKNN Toolkit2 `2.3.2`
- 当前基线模型：`yolov5s.rknn`
- 可选交通模型：`traffic_640n_7.rknn`
- 当前默认启动策略：无 GUI，`show_result=false`，`pub_result_img=false`

注意：`yolov5s` 是 COCO 80 类模型，用来复现原 CPU 版本的 `person/chair/laptop` 等通用检测效果。`traffic_640n_7` 是 7 类交通场景模型，类别是 `Keep_Straight_Sign`、`Turn_Right_Sign`、`Parking_Sign`、`Sidewalk_Sign`、`Crossing`、`Green_Light`、`Turn_R`，不能用于复现 `person` 检测。

## 2. 文件说明

源码中保留 `.pt` 权重：

```text
car_yolo/config/yolov5s.pt
car_yolo/config/traffic_640n_7.pt
```

转换后需要得到：

```text
car_yolo/config/yolov5s.rknn
car_yolo/config/traffic_640n_7.rknn
```


`yolov5s` 的 COCO 类别名已经内置在 `car_yolo/car_yolo/rknn_yolov5.py`，不需要额外 `.names` 文件。自定义模型如果没有内置类别名，需要准备：

```text
car_yolo/config/<model>.names
```

每行一个类别名，顺序必须和训练时的 class id 一致。

## 3. 代码入口

核心运行节点：

```text
car_yolo/car_yolo/yolo_detect.py
```

RKNN 后端：

```text
car_yolo/car_yolo/rknn_yolov5.py
```

模型转换脚本：

```text
car_yolo/tools/export_yolov5_to_onnx.py
car_yolo/tools/convert_onnx_to_rknn.py
```

常用 launch：

```text
car_yolo/launch/car_yolo.launch.py
car_report/launch/car_report_yolo.launch.py
car_notify/launch/dingtalk_notify.launch.py
```

## 4. 虚拟机转换环境

推荐用 Ubuntu 22.04 虚拟机，Python 3.10。

创建环境：

```bash
python3 -m venv ~/rknn-venv
source ~/rknn-venv/bin/activate
python -m pip install --upgrade pip
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5 onnx==1.16.1 onnxsim==0.4.36
```

安装 RKNN Toolkit2。解压 `rknn-toolkit2` 后，Python 3.10 + x86_64 Ubuntu 使用：

```bash
pip install /home/leo/Desktop/rknn-toolkit2-master/rknn-toolkit2/packages/x86_64/rknn_toolkit2-2.3.2-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

验证：

```bash
python -c "from rknn.api import RKNN; print('RKNN Toolkit2 OK')"
python -c "import onnx; print(onnx.__version__, hasattr(onnx, 'mapping'))"
```

`onnx` 建议固定在 `1.16.1`。如果使用 `onnx 1.21.0`，RKNN Toolkit2 2.3.2 可能报：

```text
AttributeError: module 'onnx' has no attribute 'mapping'
```

如果 `torchvision` 报 `operator torchvision::nms does not exist`，让版本匹配：

```bash
pip uninstall -y torchvision
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple torchvision==0.19.0
```

因为 RKNN Toolkit2 2.3.2 会安装 `torch 2.4.0`，对应 `torchvision 0.19.0`。

## 5. 转换 yolov5s.rknn

在虚拟机里进入能看到 `car_yolo/` 的目录：

```bash
cd ~/Desktop/patrol-robot-Develop
source ~/rknn-venv/bin/activate
```

导出 ONNX：

```bash
python car_yolo/tools/export_yolov5_to_onnx.py \
  --weights car_yolo/config/yolov5s.pt \
  --output car_yolo/config/yolov5s.onnx \
  --img-size 640 \
  --opset 12
```

转换 RKNN：

```bash
python car_yolo/tools/convert_onnx_to_rknn.py \
  --onnx car_yolo/config/yolov5s.onnx \
  --output car_yolo/config/yolov5s.rknn \
  --target-platform rk3588 \
  --quantized-dtype w8a8
```

校验文件头：

```bash
ls -lh car_yolo/config/yolov5s.rknn
head -c 32 car_yolo/config/yolov5s.rknn | xxd
```

有效 RKNN 文件头应该看到：

```text
524b 4e4e
RKNN
```

## 6. 转换 traffic_640n_7.rknn

交通 7 类模型转换命令：

```bash
python car_yolo/tools/export_yolov5_to_onnx.py \
  --weights car_yolo/config/traffic_640n_7.pt \
  --output car_yolo/config/traffic_640n_7.onnx \
  --img-size 640 \
  --opset 12

python car_yolo/tools/convert_onnx_to_rknn.py \
  --onnx car_yolo/config/traffic_640n_7.onnx \
  --output car_yolo/config/traffic_640n_7.rknn \
  --target-platform rk3588 \
  --quantized-dtype w8a8
```

读取 `.pt` 中的类别名：

```bash
python - <<'PY'
import sys
import torch
import yolov5
from pathlib import Path

yolov5_root = Path(yolov5.__file__).resolve().parent
sys.path.insert(0, str(yolov5_root))

ckpt = torch.load('car_yolo/config/traffic_640n_7.pt', map_location='cpu')
model = ckpt.get('model') or ckpt.get('ema')
names = getattr(model, 'names', None) or ckpt.get('names')

if isinstance(names, dict):
    names = [names[i] for i in sorted(names)]

print(names)
with open('car_yolo/config/traffic_640n_7.names', 'w', encoding='utf-8') as f:
    f.write('\n'.join(names) + '\n')
PY
```

## 7. 传模型到小车

从虚拟机传到小车（`192.168.0.102` 是开发时在路由器里按 MAC 绑定的小车 IP，更换路由器后需重新绑定或替换为实际 IP，见 [根 README](../README.md#快速开始)）：

```bash
scp car_yolo/config/yolov5s.rknn elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/yolov5s.rknn
scp car_yolo/config/traffic_640n_7.rknn elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/traffic_640n_7.rknn
```

小车上校验：

```bash
cd ~/Desktop/ROS2/SRC_20260427
ls -lh src/car_yolo/config/yolov5s.rknn
head -c 32 src/car_yolo/config/yolov5s.rknn | xxd
```

如果文件头全是 `00`，说明传输后的模型文件损坏，需要重新传。有效文件头必须包含 `RKNN`。

构建同步到 `install`：

```bash
cd ~/Desktop/ROS2/SRC_20260427
rm -rf build/car_yolo install/car_yolo
colcon build --symlink-install --packages-select car_yolo
source install/setup.bash
head -c 32 install/car_yolo/share/car_yolo/config/yolov5s.rknn | xxd
```

## 8. 小车运行 yolov5s RKNN 闭环

终端 1，启动相机：

```bash
cd ~/Desktop/ROS2/SRC_20260427
source install/setup.bash
ros2 launch car_base car_camera.launch.py camera_type:=depth
```

终端 2，启动 YOLOv5s RKNN + 事件记录 + 截图保存：

```bash
cd ~/Desktop/ROS2/SRC_20260427
source install/setup.bash

ros2 launch car_report car_report_yolo.launch.py \
  yolo_backend:=rknn \
  yolo_model:=yolov5s \
  yolo_rknn_model:=yolov5s.rknn \
  image_topic:=/camera/color/image_raw \
  yolo_show_result:=false \
  yolo_pub_result_img:=false \
  min_score:=0.5 \
  save_image:=true
```

终端 3，启动钉钉通知：

```bash
cd ~/Desktop/ROS2/SRC_20260427
source install/setup.bash

ros2 launch car_notify dingtalk_notify.launch.py \
  event_topic:=/car_report/event \
  keyword:=巡逻告警 \
  min_score:=0.5 \
  dry_run:=false
```

终端 4，观察事件：

```bash
cd ~/Desktop/ROS2/SRC_20260427
source install/setup.bash
ros2 topic echo /car_report/event
```

成功事件示例应包含：

```text
目标：person
置信度：0.82
截图大小：134988 bytes
截图路径：/home/elf/Desktop/ROS2/SRC_20260427/install/car_report/share/car_report/data/events/images/...
来源话题：/car_yolo/object_detect
```

## 9. 巡逻场景过滤建议

`yolov5s` 会识别 COCO 80 类，但比赛巡逻不需要 `mouse/chair/laptop` 等全部类别。建议启动事件记录时只推送巡逻相关目标：

```bash
target_classes:=person,car,motorcycle,bicycle,backpack,suitcase
```

示例：

```bash
ros2 launch car_report car_report_yolo.launch.py \
  yolo_backend:=rknn \
  yolo_model:=yolov5s \
  yolo_rknn_model:=yolov5s.rknn \
  image_topic:=/camera/color/image_raw \
  yolo_show_result:=false \
  yolo_pub_result_img:=false \
  min_score:=0.6 \
  save_image:=true \
  target_classes:=person,car,motorcycle,bicycle,backpack,suitcase
```

下一步应把单帧 `object_detected` 升级为巡逻事件：

- `person` -> `person_detected` 或 `intrusion_detected`
- `backpack/suitcase` -> `suspicious_package`
- `car/motorcycle/bicycle` -> `vehicle_detected`
- 连续 N 秒出现同一目标 -> `loitering` 或 `long_stay`

## 10. 常见问题

### 10.1 没有钉钉消息

先看 YOLO 和事件话题是否有输出：

```bash
ros2 topic echo /car_yolo/object_detect
ros2 topic echo /car_report/event
```

如果 `/car_report/event` 有输出但钉钉没收到，再检查 `car_notify` 的 webhook、关键词和网络。

### 10.2 没有截图路径

启动 `car_report_yolo.launch.py` 时必须使用：

```bash
save_image:=true
```

如果设置为 `false`，事件中会显示：

```text
截图大小：0 bytes
截图路径：无
```

### 10.3 OpenCV / Qt / xcb 报错

板子无桌面显示时不要开启 GUI：

```bash
yolo_show_result:=false
yolo_pub_result_img:=false
```

`show_result=true` 会触发 `cv2.imshow`，无显示环境时可能报：

```text
qt.qpa.xcb: could not connect to display
```

### 10.4 invalid RKNN_MAGIC

如果出现：

```text
parseRKNN: invalid RKNN_MAGIC
Invalid RKNN format
```

检查模型文件头：

```bash
head -c 32 src/car_yolo/config/yolov5s.rknn | xxd
```

有效文件应显示 `RKNN`。如果全是 `00`，重新从虚拟机传模型到小车。

### 10.5 Runtime 和 Toolkit 版本不一致

当前已见到：

```text
RKNN Model version: 2.3.2 not match with rknn runtime version: 2.1.0
```

目前模型可加载并运行，先记录为风险。如果后续出现推理异常、结果不稳定或崩溃，应升级板端 runtime 到 2.3.x，或用 Toolkit2 2.1.x 重新转换模型。
