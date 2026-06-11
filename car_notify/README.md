# car_notify

钉钉机器人事件推送模块。

对应 `plan.md` 中的这些目标：

- `1. 项目定位`：实现联动告警、远程接管与事件报告中的告警闭环。
- `3. 方案总体架构 / 云端服务`：告警消息网关，可用企业微信机器人、钉钉机器人、MQTT/HTTP 接口。
- `4.4 功能设计 / 报告生成与推送`：事件级报告、班次级报告需要配套消息推送入口。
- `5. 关键技术路线 / 通信与报告`：实时链路中的 HTTP/MQTT 控制与事件通道。
- `8. 分阶段实施计划 / 第9-10周`：打通消息推送（企业微信/钉钉/短信网关任选一种）。
- `9. 验收指标 / 巡航与异常识别`：端到端告警时延，从事件触发到消息推送。

## 做什么

`car_notify` 不做识别、不做事件记录，也不生成 GLM 报告。它只负责把 `car_report` 已经发布出来的事件推送到钉钉群。

- 订阅 `/car_report/event`。
- 解析 `car_report` 事件 JSON。
- 按置信度、目标类别、冷却时间过滤。
- 使用钉钉自定义机器人 webhook 发送 markdown 告警消息。

## 输入

默认订阅：

- 话题：`/car_report/event`
- 类型：`std_msgs/msg/String`
- 内容：JSON 字符串
- 来源：`car_report/car_report/event_recorder.py`

事件字段沿用 `car_report`：

```json
{"time":"2026-05-11T21:30:12+08:00","event_type":"object_detected","class_name":"person","score":0.87,"bbox":[120,80,300,420],"image_width":640,"image_height":480,"image_path":"car_report/data/events/images/test.jpg","image_bytes":123456,"source_topic":"/car_yolo/object_detect"}
```

## 输出

输出到钉钉自定义机器人 webhook，消息类型为 markdown：

- `msgtype`: `markdown`
- `markdown.title`: 包含关键词，例如 `巡逻告警 - object_detected`
- `markdown.text`: 包含关键词、时间、事件类型、目标、置信度、位置框、图像尺寸、截图大小、截图本地路径、来源话题和人工复核提示；没有置信度的事件显示 `未记录`

v1 不上传图片。钉钉 webhook 的 markdown 不能直接发送本地文件，当前只推送截图路径；后续如果接入图床、内网 Web 服务或文件服务，再把路径改成可访问链接。

## 格式用途

| 数据 | 格式 | 主要读者 | 用途 |
| --- | --- | --- | --- |
| `/car_report/event` | `std_msgs/msg/String`，内容为 JSON | 机器 | `car_notify` 实时订阅 |
| 钉钉请求体 | JSON，`msgtype=markdown` | 机器 | 钉钉机器人 webhook 接收 |
| 钉钉群消息 | Markdown 渲染文本 | 人 | 值守人员查看告警摘要 |
| 截图路径 | 本地路径字符串 | 人为主 | 提醒现场留证位置，不直接上传图片 |

## 钉钉配置

钉钉群里添加“自定义机器人”，安全设置选择“关键词”，关键词填：

```text
巡逻告警
```

webhook 不再写死在代码中。推荐运行前设置环境变量：

```bash
export DINGTALK_WEBHOOK_URL="钉钉机器人webhook完整地址"
```

`car_notify/launch/dingtalk_notify.launch.py` 默认传空 `webhook_url`，节点会自动回退到环境变量 `DINGTALK_WEBHOOK_URL`。如果 launch 传入 `webhook_url` 参数，则会覆盖环境变量。

默认关键词是 `巡逻告警`，代码会把它同时放进 markdown `title` 和正文，避免关键词校验拦截。要换机器人，可以设置环境变量，也可以启动时用参数覆盖：

```bash
ros2 launch car_notify dingtalk_notify.launch.py webhook_url:="钉钉机器人webhook完整地址" keyword:=巡逻告警
```

官方文档：

- 自定义机器人接入：https://open.dingtalk.com/document/group/custom-robot-access
- 自定义机器人安全设置：https://open.dingtalk.com/document/dingstart/customize-robot-security-settings

## ROS2 环境运行

先启动识别和事件记录：

```bash
ros2 launch car_report car_report_yolo.launch.py
```

`car_report_yolo.launch.py` 默认会按 `car_yolo` 当前配置启动识别。`car_notify` 只依赖 `/car_report/event`，不关心 YOLO 使用 RKNN 还是 torch，也不依赖网页流 `/result_img`。

如果在小车桌面环境直接打开 OpenCV 检测窗口：

```bash
ros2 launch car_report car_report_yolo.launch.py yolo_show_result:=true
```

再启动钉钉推送：

```bash
ros2 launch car_notify dingtalk_notify.launch.py
```

单独测试钉钉 webhook 连通性：

```bash
ros2 run car_notify dingtalk_notifier --webhook-test
```

ROS2 真实运行链路是：

```text
car_report/event_recorder.py
  -> 发布 /car_report/event，std_msgs/String，内容是事件 JSON
car_notify/dingtalk_notifier.py
  -> 订阅 /car_report/event
  -> JSON 解析
  -> min_score / target_classes / cooldown_sec 过滤
  -> 组装钉钉 markdown
  -> 请求 webhook
```

ROS2 环境里如果只想确认订阅和过滤逻辑，不请求钉钉：

```bash
ros2 launch car_notify dingtalk_notify.launch.py dry_run:=true
```

## launch 边界

`dingtalk_notify.launch.py` 只启动推送节点，不启动：

- `car_report`
- `car_yolo`
- 相机
- 底盘
- 导航
- Web 服务
- `/result_img` 网页流

这样不会和其他功能包竞争，也不会受 YOLO 后端选择影响。后续总 launch 可以用参数控制是否启用通知，例如设计一个 `enable_dingtalk_notify` 参数：为 `true` 时启动 `car_notify`，为 `false` 时不启动。这样平时调试不打扰钉钉群，比赛演示再打开。

## 参数

`dingtalk_notify.launch.py` / ROS2 节点参数：

- `event_topic`：事件输入话题，默认 `/car_report/event`。
- `webhook_url`：钉钉机器人 webhook，launch 默认空；为空时使用 `dingtalk_notifier.py` 中写死的 `DEFAULT_WEBHOOK_URL`，启动时可覆盖。
- `keyword`：关键词，默认 `巡逻告警`，需要和钉钉机器人安全设置一致。
- `cooldown_sec`：同一类别推送冷却时间，默认 `5.0` 秒。
- `min_score`：最低推送置信度，默认 `0.5`；只对带 `score` 字段的事件生效，没有 `score` 的告警类事件不会因此被误过滤。
- `target_classes`：只推送指定类别，逗号分隔；默认空字符串表示全部类别。匹配 `class_name`，如果事件没有 `class_name` 则匹配 `event_type`。
- `timeout`：HTTP 超时时间，默认 `10.0` 秒；节点内最小按 `1.0` 秒处理，避免错误参数导致请求立即失败。
- `dry_run`：只打印 markdown，不请求 webhook，默认 `false`。

注册入口在 `car_notify/setup.py` 的 `console_scripts`：

- `dingtalk_notifier = car_notify.dingtalk_notifier:main`

构建并 source 工作区后，下面命令应能找到对应入口：

```bash
ros2 run car_notify dingtalk_notifier
```

## 已验证与运行注意

已在小车 ROS2 环境验证：

- `car_notify` 可订阅 `/car_report/event`。
- `/car_report/event -> dingtalk_notifier -> 钉钉 webhook` 链路可正常推送。
- `dry_run` 可用于只看将发送的 markdown 内容。

运行时仍需按现场情况关注：

- 钉钉机器人关键词是否和 `keyword` 参数一致。
- 小车网络是否能访问钉钉 webhook。
- `min_score`、`target_classes`、`cooldown_sec` 是否符合演示节奏，避免刷屏。
