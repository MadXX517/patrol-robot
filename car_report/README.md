# car_report

视觉识别事件记录与 GLM 巡逻报告模块，负责把 YOLO 识别结果转成可保存、可订阅、可总结的巡逻事件。

对应 `plan.md` 中的这些目标：

- `2. 与竞赛考核指标对齐 / 边缘AI`：事件发生后进行结构化摘要，再调用云侧 LLM API 生成报告。
- `4.2 独立巡航模式 / 报告生成与推送`：生成事件级报告和班次级报告。
- `5. 关键技术路线 / 通信与报告`：结构化 JSON -> LLM API -> Markdown/PDF 日报。
- `8. 分阶段实施计划 / 第9-10周`：接入 LLM API 生成事件报告与班次总结。

## 做什么

`car_report` 不做实时识别算法本身，而是复用 `car_yolo` 的识别结果：

- 从 `/car_yolo/object_detect` 接收 YOLO 结构化目标结果。
- 从 `/camera/color/image_raw` 接收相机原图，用于事件截图留证。
- 按置信度、类别过滤和冷却时间生成事件。
- 保存事件 JSONL、事件截图，并可调用 BigModel `GLM-4V-Flash` 生成 Markdown 巡逻报告。

当前默认运行数据目录为 `car_report/data/`。源码直接运行时写入源码目录下的 `car_report/data/`；ROS2 构建安装后默认写入已安装包的 `share/car_report/data/`。运行生成的事件、图片和报告会被源码目录下的 `.gitignore` 忽略，不会污染提交；也可以通过 `output_dir` 参数改到指定目录。

## 输入

事件记录节点 `event_recorder` 接收两个 ROS2 话题：

- `/car_yolo/object_detect`
  - 类型：`interfaces/msg/ObjectsInfo`
  - 来源：`car_yolo/car_yolo/yolo_detect.py`
  - 内容：`objects` 数组，每个对象为 `interfaces/msg/ObjectInfo`
  - 对象字段：`class_name`、`box`、`score`、`width`、`height`

- `/camera/color/image_raw`
  - 类型：`sensor_msgs/msg/Image`
  - 用途：保存事件截图

报告生成命令 `report_generator` 读取事件日志：

- 默认路径：`car_report/data/events/events_YYYYMMDD.jsonl`
- 格式：JSONL，一行一个 JSON 事件

事件示例：

```json
{"time":"2026-05-11T21:30:12+08:00","event_type":"object_detected","class_name":"person","score":0.87,"bbox":[120,80,300,420],"image_width":640,"image_height":480,"image_path":"car_report/data/events/images/20260511_213012_person.jpg","image_bytes":123456,"source_topic":"/car_yolo/object_detect"}
```

## 输出

- `car_report/data/events/events_YYYYMMDD.jsonl`
  - 事件日志，JSONL 格式，一行一个事件。
- `car_report/data/events/images/*.jpg`
  - 事件截图，默认限制单张小于 5MB。
- `/car_report/event`
  - ROS2 话题，类型 `std_msgs/msg/String`，内容为事件 JSON 字符串。
- `car_report/data/reports/report_YYYYMMDD_HHMMSS.md`
  - GLM 生成的 Markdown 巡逻报告。

## 格式用途

| 数据 | 格式 | 主要读者 | 用途 |
| --- | --- | --- | --- |
| `/car_yolo/object_detect` | `interfaces/msg/ObjectsInfo` | 机器 | `event_recorder` 读取 YOLO 结构化识别结果 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 机器 | `event_recorder` 保存截图 |
| `events_YYYYMMDD.jsonl` | JSONL | 机器为主，人也可查 | `report_generator` 和后续通知/检索模块读取 |
| `events/images/*.jpg` | JPG | 人为主，机器也可复核 | 人工留证，`--mode vision` 时给 GLM 图片理解 |
| `/car_report/event` | `std_msgs/msg/String`，内容为 JSON | 机器 | `car_notify` 等下游节点实时订阅 |
| `reports/report_*.md` | Markdown | 人 | 比赛展示、巡逻记录、人工复核 |

## 大模型使用边界

- `event_recorder` 阶段不调用大模型，只负责记录结构化事件和截图。
- `report_generator --mode text` 使用文本对话：只把事件 JSON 摘要发给 GLM，不发送图片。
- `report_generator --mode vision` 使用图片理解：在事件摘要外，额外发送最多 `--max-images` 张事件截图给 GLM 复核画面。

API Key 不再写死在代码中。运行前设置环境变量 `BIGMODEL_API_KEY` 或 `ZHIPUAI_API_KEY`，也可以通过 `report_service` 的 `api_key` 参数或 `report_generator --api-key` 临时覆盖。

## ROS2 环境运行

只启动事件记录，适合相机和 YOLO 已经由其他 launch 启动的情况：

```bash
ros2 launch car_report car_report.launch.py
```

启动 YOLO + 事件记录，适合只想验证识别事件记录链路的情况：

```bash
ros2 launch car_report car_report_yolo.launch.py
```

`car_report_yolo.launch.py` 默认使用 `car_yolo` 当前 RKNN 链路：`yolo_backend=rknn`、`yolo_model=yolov5s`。`yolo_rknn_model` 默认留空，由 `car_yolo` 自动使用 `<yolo_model>.rknn`。如果需要网页查看带置信框的 `/result_img`，启动时加 `yolo_pub_result_img:=true`。

如果在小车桌面环境直接打开 OpenCV 检测窗口：

```bash
ros2 launch car_report car_report_yolo.launch.py yolo_show_result:=true
```

单独测试 GLM API 连通性：

```bash
export BIGMODEL_API_KEY="你的Key"
ros2 run car_report report_generator --api-test
```

已有事件日志后，生成文本报告：

```bash
ros2 run car_report report_generator --mode text
```

已有事件日志和截图后，生成带截图复核的多模态报告：

```bash
ros2 run car_report report_generator --mode vision --max-images 3
```

## launch 边界

本仓库其他模块也同时存在“单节点 run”和“组合 launch”两类入口：

- `car_yolo` 提供单个 YOLO 检测节点。
- `car_vision/launch/driver_yolo.launch.py` 组合 YOLO 和自动驾驶节点。
- `car_base/launch/car_base.launch.py` 组合底盘、相机、雷达、URDF、EKF 等硬件基础节点。

本模块新增的 `car_report_yolo.launch.py` 只组合 `car_yolo + car_report`，不启动相机、底盘、导航、Web 或自动驾驶，避免和其他 launch 竞争。最终比赛的一键启动应在收尾阶段单独做总 bringup。

通知节点建议由更上层的总 launch 按参数条件拉起，而不是写死进每个业务 launch。ROS2 launch 可以用 `enable_dingtalk_notify` 这类参数配合 `IfCondition` 决定是否启动 `car_notify`，这样平时调试不推送，演示时再打开 webhook 通知。

## 主要参数

`car_report.launch.py` 和 `car_report_yolo.launch.py` 都支持：

- `image_topic`：相机图像话题，默认 `/camera/color/image_raw`。
- `output_dir`：事件、截图、报告输出目录，默认 `car_report/data`。
- `min_score`：最低置信度，默认 `0.5`；节点内最小按 `0.0` 处理。
- `cooldown_sec`：同一类别事件冷却时间，默认 `5.0` 秒；节点内最小按 `0.0` 秒处理。
- `target_classes`：只记录指定类别，逗号分隔；默认空字符串表示记录全部类别。
- `save_image`：是否保存事件截图，默认 `true`。
- `max_image_bytes`：截图最大字节数，默认 `5242880`，即 5MB；节点内最小按 `1` byte 处理。
- `jpeg_quality`：截图初始 JPEG 质量，默认 `85`，节点内限制在 `30` 到 `95`。

`car_report.launch.py` 额外支持：

- `objects_topic`：识别结果话题，默认 `/car_yolo/object_detect`。

`car_report_yolo.launch.py` 额外支持：

- `yolo_backend`：YOLO 推理后端，默认 `rknn`；需要走 PyTorch 时可设为 `torch`。
- `yolo_device`：PyTorch 后端的推理设备，默认 `cpu`；`yolo_backend=rknn` 时不使用。
- `yolo_model`：`car_yolo/config` 下的模型名，默认 `yolov5s`。
- `yolo_rknn_model`：RKNN 模型文件名或绝对路径，默认空字符串；留空时由 `car_yolo` 自动使用 `<yolo_model>.rknn`。
- `yolo_show_result`：是否弹出 YOLO 显示窗口，默认 `false`。
- `yolo_pub_result_img`：是否发布 YOLO 标注图 `/result_img`，默认 `false`；网页查看置信框时设为 `true`。

`report_generator` 参数：

- `--output-dir`：事件和报告目录，默认 `car_report/data`。
- `--events-file`：指定某个 `events_YYYYMMDD.jsonl`；不填则读取最新事件文件。
- `--mode`：`text` 或 `vision`，默认 `text`。
- `--max-images`：图片理解模式最多发送几张图，默认 `3`。
- `--max-image-bytes`：发送给 GLM 的单张图片最大字节数，默认 `5242880`。
- `--model`：BigModel 模型 ID，默认 `glm-4v-flash`。
- `--endpoint`：BigModel 对话补全接口。
- `--api-key`：BigModel API Key；不填时读取 `BIGMODEL_API_KEY` 或 `ZHIPUAI_API_KEY`。
- `--temperature`：生成随机性，默认 `0.2`。
- `--timeout`：HTTP 超时时间，默认 `60` 秒。

注册入口在 `car_report/setup.py` 的 `console_scripts`：

- `event_recorder = car_report.event_recorder:main`
- `report_generator = car_report.report_generator:main`
- `report_service = car_report.report_service:main`

构建并 source 工作区后，下面两个命令应能找到对应入口：

```bash
ros2 run car_report event_recorder
ros2 run car_report report_generator
```

## 已验证与运行注意

已在小车 ROS2 环境验证：

- `car_report_yolo.launch.py` 可启动 `car_yolo + event_recorder`。
- `/car_yolo/object_detect` 可被 `event_recorder` 接收并转成 `/car_report/event`。
- 事件 JSONL 和截图可正常生成。
- `report_generator` 的 GLM API 调用已验证可用。

运行时仍需按现场情况关注：

- `/camera/color/image_raw` 是否持续有相机图像。
- RKNN 模型文件是否和 `yolo_model` 对应。
- 截图保存和压缩对实时性能的影响。
- 小车网络是否能稳定访问 BigModel API。
