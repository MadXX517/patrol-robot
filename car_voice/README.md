# car_voice

巡逻机器人语音交互桥接模块，采用“固定安全口令 + 大模型自然语言”的混合路线。

## 功能

- C21 语音模块串口唤醒：默认 `/dev/aibox`，波特率 `115200`。
- 固定口令优先解析：急停、停止运动、基础移动、雷达警卫/跟随/避障、视觉跟踪、生成巡逻报告。
- 大模型兜底解析：ASR 文本不命中固定口令时，调用 DashScope/OpenAI-compatible 接口生成白名单 JSON 指令。
- TTS 播报：订阅 `/car_voice/say`，可用 `qwen-tts` 播报执行结果。
- 调试入口：发布文本到 `/car_voice/mock_text` 可绕过硬件和 ASR，直接测试语音控制链路。

## 环境变量

```bash
export DASHSCOPE_API_KEY="你的阿里云百炼Key"
export BIGMODEL_API_KEY="你的智谱BigModel Key"      # 只有生成 car_report 报告时需要
export DINGTALK_WEBHOOK_URL="你的钉钉机器人webhook" # 只有推送告警时需要
```

## 启动

先启动底盘、雷达、相机等基础节点，再启动语音模块：

```bash
ros2 launch car_voice car_voice.launch.py
```

不上硬件时可关闭串口和 ASR，只测文本到控制：

```bash
ros2 launch car_voice car_voice.launch.py enable_serial:=false enable_asr:=false enable_tts:=false
```

然后另开终端发布模拟语音：

```bash
ros2 topic pub --once /car_voice/mock_text std_msgs/msg/String "{data: '开始警卫'}"
ros2 topic pub --once /car_voice/mock_text std_msgs/msg/String "{data: '向前进30厘米'}"
ros2 topic pub --once /car_voice/mock_text std_msgs/msg/String "{data: '急停'}"
ros2 topic pub --once /car_voice/mock_text std_msgs/msg/String "{data: '生成巡逻报告'}"
```

## 话题

- `/car_voice/text`：ASR 原始文本，`std_msgs/String`。
- `/car_voice/command`：白名单 JSON 指令，`std_msgs/String`。
- `/car_voice/status`：状态 JSON，`std_msgs/String`。
- `/car_voice/say`：TTS 播报文本，`std_msgs/String`。
- `/car_voice/mock_text`：调试输入文本，`std_msgs/String`。

## 支持的 JSON 指令

```json
{"intent":"emergency_stop"}
{"intent":"move","direction":"forward","distance_m":0.3}
{"intent":"move","direction":"turn_left","angle_deg":45}
{"intent":"set_lidar_mode","mode":3}
{"intent":"set_color_follow","action":"start"}
{"intent":"generate_report"}
{"intent":"status"}
```

`move.direction` 可选：`forward`、`backward`、`left`、`right`、`turn_left`、`turn_right`。V1 不支持语音导航点位。
