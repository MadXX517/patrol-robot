# car_yolo 其他 PT 模型转 RKNN 操作手册

本文用于后续把 `car_yolo/config/` 目录下其他 `.pt` 模型继续转换成 `.rknn`，并部署到 RK3588 小车上运行。

当前目录里的模型文件：

```text
car_yolo/config/best.pt
car_yolo/config/num.pt
car_yolo/config/traffic.pt
car_yolo/config/traffic_640n_7.pt
car_yolo/config/yolov5s.pt
```

已经转换并验证过：

```text
car_yolo/config/yolov5s.rknn
car_yolo/config/traffic_640n_7.rknn
```

## 1. 什么时候需要转模型

只要你想让某个 `.pt` 模型走 RK NPU 推理，就需要转换：

```text
xxx.pt -> xxx.onnx -> xxx.rknn
```

例如：

```text
best.pt -> best.onnx -> best.rknn
num.pt -> num.onnx -> num.rknn
traffic.pt -> traffic.onnx -> traffic.rknn
```

`.pt` 是 PyTorch/CPU 路径使用的权重，板端 RKNN Runtime 不能直接运行 `.pt`。

## 2. 保留的虚拟机环境

如果后续还要转模型，建议保留虚拟机中的：

```text
~/rknn-venv
~/Desktop/rknn-toolkit2-master
```

每次转换前进入虚拟机终端：

```bash
source ~/rknn-venv/bin/activate
```

确认工具可用：

```bash
python -c "from rknn.api import RKNN; print('RKNN Toolkit2 OK')"
python -c "import onnx; print(onnx.__version__, hasattr(onnx, 'mapping'))"
```

建议版本状态：

```text
Python 3.10
RKNN Toolkit2 2.3.2
onnx 1.16.1
onnxsim 0.4.36
torch 2.4.0
torchvision 0.19.0
```

如果 `onnx` 是 `1.21.0`，转换可能报：

```text
AttributeError: module 'onnx' has no attribute 'mapping'
```

修复：

```bash
pip uninstall -y onnx onnxsim onnx-simplifier
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple onnx==1.16.1 onnxsim==0.4.36
```

如果 `torchvision` 报：

```text
RuntimeError: operator torchvision::nms does not exist
```

修复：

```bash
pip uninstall -y torchvision
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple torchvision==0.19.0
```

## 3. 准备代码目录

在虚拟机里进入能看到 `car_yolo/` 的目录，例如：

```bash
cd ~/Desktop/patrol-robot-Develop
source ~/rknn-venv/bin/activate
```

确认转换脚本和模型存在：

```bash
ls car_yolo/tools/export_yolov5_to_onnx.py
ls car_yolo/tools/convert_onnx_to_rknn.py
ls car_yolo/config/*.pt
```

## 4. 单个模型转换流程

下面用变量写法，后续换模型时只改 `MODEL`。

### 4.1 设置模型名

例如转换 `best.pt`：

```bash
MODEL=best
```

转换 `num.pt`：

```bash
MODEL=num
```

转换 `traffic.pt`：

```bash
MODEL=traffic
```

转换 `traffic_640n_7.pt`：

```bash
MODEL=traffic_640n_7
```

转换 `yolov5s.pt`：

```bash
MODEL=yolov5s
```

### 4.2 导出 ONNX

```bash
python car_yolo/tools/export_yolov5_to_onnx.py \
  --weights car_yolo/config/${MODEL}.pt \
  --output car_yolo/config/${MODEL}.onnx \
  --img-size 640 \
  --opset 12
```

确认：

```bash
ls -lh car_yolo/config/${MODEL}.onnx
```

### 4.3 转换 RKNN

小车是 RK3588，所以平台固定为：

```text
--target-platform rk3588
```

转换命令：

```bash
python car_yolo/tools/convert_onnx_to_rknn.py \
  --onnx car_yolo/config/${MODEL}.onnx \
  --output car_yolo/config/${MODEL}.rknn \
  --target-platform rk3588 \
  --quantized-dtype w8a8
```

确认：

```bash
ls -lh car_yolo/config/${MODEL}.rknn
head -c 32 car_yolo/config/${MODEL}.rknn | xxd
```

有效 `.rknn` 文件头应该包含：

```text
524b 4e4e
RKNN
```

如果文件头全是 `00`，说明文件损坏，不能上车运行。

## 5. 一次性转换多个模型

如果要把还没转的几个模型都转换，可以执行：

```bash
cd ~/Desktop/patrol-robot-Develop
source ~/rknn-venv/bin/activate

for MODEL in best num traffic; do
  echo "=== export ${MODEL}.pt to ONNX ==="
  python car_yolo/tools/export_yolov5_to_onnx.py \
    --weights car_yolo/config/${MODEL}.pt \
    --output car_yolo/config/${MODEL}.onnx \
    --img-size 640 \
    --opset 12

  echo "=== convert ${MODEL}.onnx to RKNN ==="
  python car_yolo/tools/convert_onnx_to_rknn.py \
    --onnx car_yolo/config/${MODEL}.onnx \
    --output car_yolo/config/${MODEL}.rknn \
    --target-platform rk3588 \
    --quantized-dtype w8a8

  echo "=== check ${MODEL}.rknn ==="
  ls -lh car_yolo/config/${MODEL}.rknn
  head -c 32 car_yolo/config/${MODEL}.rknn | xxd
done
```

如果某个模型失败，先单独转那个模型，方便看完整报错。

## 6. 类别名处理

`yolov5s` 是 COCO 80 类，代码已经内置类别名，不需要 `.names` 文件。

其他自定义模型建议生成同名 `.names` 文件：

```text
car_yolo/config/best.names
car_yolo/config/num.names
car_yolo/config/traffic.names
car_yolo/config/traffic_640n_7.names
```

从 `.pt` 里尝试读取类别名：

```bash
MODEL=best

python - <<'PY'
import os
import sys
import torch
import yolov5
from pathlib import Path

model_name = os.environ.get('MODEL')
if not model_name:
    raise RuntimeError('Please set MODEL first, for example: MODEL=best')

yolov5_root = Path(yolov5.__file__).resolve().parent
sys.path.insert(0, str(yolov5_root))

pt_path = f'car_yolo/config/{model_name}.pt'
names_path = f'car_yolo/config/{model_name}.names'

ckpt = torch.load(pt_path, map_location='cpu')
model = ckpt.get('model') or ckpt.get('ema')
names = getattr(model, 'names', None) or ckpt.get('names')

if isinstance(names, dict):
    names = [names[i] for i in sorted(names)]

print(names)
if not names:
    raise RuntimeError(f'No class names found in {pt_path}')

with open(names_path, 'w', encoding='utf-8') as f:
    f.write('\n'.join(names) + '\n')

print(f'wrote {names_path}')
PY
```

然后查看：

```bash
cat car_yolo/config/${MODEL}.names
```

如果 `.pt` 里读不到类别名，就必须根据训练数据手动写 `.names`，每行一个类别，顺序要和训练时一致。

## 7. 传到小车

单个模型传输：

```bash
MODEL=best

scp car_yolo/config/${MODEL}.rknn \
  elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/${MODEL}.rknn
```

如果有 `.names`：

```bash
scp car_yolo/config/${MODEL}.names \
  elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/${MODEL}.names
```

多个模型传输：

```bash
for MODEL in best num traffic; do
  scp car_yolo/config/${MODEL}.rknn \
    elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/${MODEL}.rknn

  if [ -f car_yolo/config/${MODEL}.names ]; then
    scp car_yolo/config/${MODEL}.names \
      elf@192.168.0.102:/home/elf/Desktop/ROS2/SRC_20260427/src/car_yolo/config/${MODEL}.names
  fi
done
```

## 8. 小车端构建

在小车 SSH 终端：

```bash
cd ~/Desktop/ROS2/SRC_20260427
source /opt/ros/$ROS_DISTRO/setup.bash

rm -rf build/car_yolo install/car_yolo
colcon build --symlink-install --packages-select car_yolo
source install/setup.bash
```

校验：

```bash
MODEL=best
ls -lh install/car_yolo/share/car_yolo/config/${MODEL}.rknn
head -c 32 install/car_yolo/share/car_yolo/config/${MODEL}.rknn | xxd
```

必须看到 `RKNN` 文件头。

## 9. 小车端运行指定模型

只启动 YOLO 节点：

```bash
MODEL=best

ros2 run car_yolo yolo_detect --ros-args \
  -p backend:=rknn \
  -p model:=${MODEL} \
  -p rknn_model:=${MODEL}.rknn \
  -p image_topic:=/camera/color/image_raw \
  -p show_result:=false \
  -p pub_result_img:=false
```

启动 YOLO + 事件记录 + 截图：

```bash
MODEL=best

ros2 launch car_report car_report_yolo.launch.py \
  yolo_backend:=rknn \
  yolo_model:=${MODEL} \
  yolo_rknn_model:=${MODEL}.rknn \
  image_topic:=/camera/color/image_raw \
  yolo_show_result:=false \
  yolo_pub_result_img:=false \
  min_score:=0.5 \
  save_image:=true
```

如果是巡逻告警场景，建议过滤目标：

```bash
target_classes:=person,car,motorcycle,bicycle,backpack,suitcase
```

例如 `yolov5s`：

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

## 10. 可删除和可保留内容

后续还要继续转模型，建议保留：

```text
~/rknn-venv
~/Desktop/rknn-toolkit2-master
```

可以删除缓存和中间文件：

```bash
rm -rf ~/.cache/pip
rm -rf ~/.cache/torch
rm -f ~/Desktop/patrol-robot-Develop/car_yolo/config/*.onnx
rm -f ~/Desktop/*.zip
sudo apt clean
df -h
```

`*.onnx` 是中间文件，`.rknn` 已生成并验证后可以删除 `.onnx` 释放空间。
