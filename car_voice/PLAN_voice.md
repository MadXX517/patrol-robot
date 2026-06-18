# 语音播报 + 语音触发 集成计划

## 目标
给小车各功能加上**语音播报**(状态变化时说出来)与**语音触发**(说一句话就能开关功能),
并在 Web 控制台提供开关(参照现有"手势控制 / 雷达"的独立开关模式)。

## 现状与可复用资产(已勘查)
- `car_llm/src/text2voice_class.py` → `TTSPlayer(api_key, voice, ...).say(text)`:
  qwen-tts 流式播放,阻塞;干净可复用,**直接 import 用**。
- `car_llm/src/voice2text_class.py` → `RealTimeASR(api_key, result_queue, ...)`:
  paraformer 实时识别,后台线程 + 门控聆听 + 播报时静音防回授;**直接 import 用**。
- 现成解耦播报通道:`/tts_node/tts_text`(String),ros_control 已订阅转 TTS。
- dashboard 已订阅全部要播报的信号:
  - `/car_report/event`(EVENT_TOPIC,YOLO 识别事件)→ event_callback
  - `/patrol/...state`(PATROL_STATE_TOPIC)→ patrol_state_callback
  - `/gesture_command/gesture`(GESTURE_STATE_TOPIC)→ gesture_state_callback
  - 电压(POWER_VOLTAGE_TOPIC)→ battery_callback
- dashboard 是唯一控制中枢,HTTP API 覆盖全部功能(/api/mode/*, /api/all/stop,
  /api/features/start|stop, /api/patrol/command, /api/gesture/*, /api/lidar/* 等)。
- 板端:dashscope + pyaudio 已装;扬声器 nau8822(card1);麦克风 USB CODEC(card3)
  或 ASTRA(card4);`/dev/aibox`→ttyUSB1 在。

## 架构决策(已定:复用 car_llm 的 LLM 栈)
- **复用 car_llm 整套语音栈**,而非自写关键词映射。理由:`LLMCommandParser` 是
  **schema 无关**的——只从 LLM 流式输出抽 `message`(喂 TTS)和 `step` JSON(动作),
  `step` 含义完全由 yaml prompt 定义、由消费端解释。原 JSON 就是占位符,可自由改。
- **零改动复用**:`text2voice_class.TTSPlayer`、`voice2text_class.RealTimeASR`、
  `chat_model_class.LLMCommandParser`、`llm_main.py` 的多进程+唤醒词("小星")+防回授架构。
- **只需新增两样**:
  1. 新 yaml prompt(如 `chat_prompt_patrol.yaml`):把 `function` 枚举换成我们的功能
     (见下表),`message` 字段保留(LLM 生成自然回执,直接播报)。
  2. 新执行器 `web_control.py`(替换 `ros_control.py`):消费 `step` → **requests.post
     dashboard HTTP API**(而非驱动机械臂)。dashboard 是唯一控制中枢,复用全部端点。
- **播报双通道**:
  - 对话回执:LLM 的 `message` 字段 → TTS(车自己会说"好的,开始巡逻")。
  - 状态播报:dashboard 状态变化 → publish `/voice/announce` → voice 进程 TTS。
    (两者最终都进同一个 tts_queue,串行播放,避免抢扬声器。)
- 固定参数(已定):唤醒词=**小星**;麦克风=**USB CODEC(card3)**;外放=nau8822(card1)。
- dashboard 像管 gesture/lidar 一样管理 voice 进程:start/stop + 句柄 + 兜底清理 +
  启动自清(复用 RESIDUAL_PATTERNS 机制)。

## 语音命令映射(LLM 输出 step.function → web_control 执行器 → dashboard API)
新 yaml prompt 里 `function` 枚举(LLM 负责把自然语言归类到这些,带容错):
| function(LLM 输出) | 自然语言示例 | 执行器动作(POST dashboard API) |
|---------------------|-------------|-------------------------------|
| start_patrol | 开始巡逻 / 去巡逻 | /api/mode/report |
| start_follow | 跟着我 / 跟上来 | /api/features/start {human_follow} |
| start_track | 看着我 / 盯住我 | /api/features/start {gimbal_track} |
| stop_all | 停下 / 别动 / 都停了 | /api/all/stop |
| relock | 重新锁定 / 锁定我 | /api/patrol/command {relock} |
| gesture_on/off | 开/关手势控制 | /api/gesture/start \| stop |
| lidar_on/off | 开/关雷达 | /api/lidar/start \| stop |
| gen_report | 生成报告 / 拍照记录 | /api/report/generate |
| camera_move | 摄像头左/右/上/下看 | /api/servo/camera_* |
| chassis_reset | 底盘复位 / 回正 | /api/chassis/reset |
| chat | (闲聊/自我介绍/无动作) | 仅播报 LLM 的 message,不发命令 |
- LLM 的 `message` 字段始终播报(回执/应答/闲聊),`step` 命中才发命令。
- 多步指令(如"先巡逻再跟随")LLM 会拆成多个 step,执行器顺序 POST。
- 未识别为任何 function → LLM 归为 chat,仅对话不误触发。

## 播报范围(已定)
事件类(状态变化触发,dashboard → /voice/announce):
- [x] A. 功能启停:"巡逻已开启" / "已全部停止" / "视觉巡逻已停止"
- [x] B. 跟随模式切换:"开始跟随你" / "切换到云台追踪" / "已停止跟随"
- [x] C. 手势唤醒/触发:"手势已唤醒" / "收到手势指令:跟随"
- [x] D. YOLO 识别事件:"检测到行人" / "发现可疑目标"(同类 N 秒去抖)
- [x] E. 电量低告警:"电量不足,请及时充电"(阈值触发,只播一次)
- [x] F. 报告生成完成:"巡逻报告已生成"
- [ ] G. 雷达开关 —— **不做**
- [x] H. 目标丢失/重锁:"目标丢失,正在重新锁定" / "已锁定目标"
- [x] I. 系统就绪:开机/节点起好后"系统就绪,我是巡逻机器人小星"
对话类(语音触发自带,LLM message 字段,无需额外开发):
- [x] 命令回执 / 唤醒应答 / 闲聊自我介绍(随 LLM 输出)

## 阶段拆解(基于复用 car_llm)
### 阶段 0:基础验证(0.5d)
- [ ] 板端选定 USB CODEC(card3)为麦、nau8822(card1)为外放,测 TTSPlayer.say
      与 RealTimeASR 收音(显式传 input/output_device_index,确认无回授)
- [ ] 确认 API Key(沿用 car_llm 默认或独立配置)

### 阶段 1:新 prompt + web 执行器(1.5d)
- [ ] 写 `chat_prompt_patrol.yaml`:function 枚举=上表,message 保留;context 写功能说明
- [ ] 写 `web_control.py`(替换 ros_control):消费 step → requests.post dashboard API;
      失败降级(投 tts_queue 播报"操作失败")
- [ ] 改 `llm_main.py` 入口(或复制为 `voice_main.py`):用新 yaml + web_control 执行器,
      去掉机械臂/串口 aibox 依赖中与控制无关的部分(aibox 按钮可选保留)

### 阶段 2:car_voice 包 + 状态播报订阅(1d)
- [ ] 新建 car_voice 包(package.xml/setup.py/launch),依赖 car_llm(import 复用类)
- [ ] voice 节点额外订阅 `/voice/announce` → tts_queue(与对话回执共用 TTS,串行)
- [ ] 参数:api_key/wake_word=小星/tts_voice/asr_model/input_device_index=card3/
      output_device_index=card1/dashboard_url/enable_tts/enable_asr
- [ ] launch + entry_point,板端 `ros2 launch car_voice voice.launch.py` 跑通

### 阶段 3:dashboard 播报源 + 进程管理(1d)
- [ ] 加 /voice/announce 发布器 + announce(text) 去抖封装
- [ ] 按勾选的 A–I,在 event/patrol_state/gesture_state/battery/各 start_*/stop_* 注入播报
- [ ] start_voice/stop_voice + 句柄;RESIDUAL_PATTERNS 增 voice 进程特征;启动自清覆盖
- [ ] status 暴露 tts_on / asr_on(两个独立子开关);
      /api/voice/tts/start|stop 与 /api/voice/asr/start|stop
- [ ] **进程拆分**:播报与触发可独立开关。两种实现取一:
      (a) 一个 voice 进程内部按 enable_tts/enable_asr 标志开关子功能(省进程,推荐);
      (b) 两个进程分别管。**默认 (a)**:进程常驻,dashboard 通过参数/信号切子开关。
- [ ] **与 car_llm 互斥**:dashboard 保证 voice 与 car_llm 不同时起(抢麦+抢 /dev/aibox)

### 阶段 4:Web 前端(0.5d)
- [ ] **两个独立开关**:"语音播报"(TTS)/ "语音触发"(ASR),参照 gestureToggleBtn/lidarToggleBtn
- [ ] 播报开关控状态播报+对话回执;触发开关控麦克风聆听。可只开其一
- [ ] 状态回显(最近播报 / 最近识别指令,可选)
- [ ] 验证全链路:开触发 → 说"小星,开始巡逻" → 应答并启动;开播报 → 状态变化有语音

## 风险与注意
- **音频设备**:麦=USB CODEC(card3)、外放=nau8822(card1),**必须显式指定
  input/output_device_index**,否则 pyaudio 选错默认设备。先用 `arecord -D plughw:3`
  / `aplay -D plughw:1` 实测确认设备号稳定(USB 重插可能变号)。
- **回授死循环**:TTS 播报时门控麦(两个类已支持);状态播报与对话回执共用 tts_queue
  串行,避免并发抢扬声器。
- **与 car_llm 并存**:二者抢麦 + 抢 /dev/aibox,dashboard 层做**互斥**(起 voice 前
  确保 car_llm 没跑,反之亦然)。
- **联网依赖**:TTS/ASR/LLM 都走阿里云,断网降级(失败仅日志/简短提示,不阻塞控制)。
  LLM 命令有 ~1-2s 云端往返延迟(用户已接受走 LLM)。
- **CPU**:ASR 常开 + LLM 调用,RK3568 上与 YOLO/相机共存需测压。
- **进程残留**:严格纳入 dashboard 兜底清理,防麦克风/串口句柄泄漏。

## 决策记录(已定)
1. 语音命令:**复用 car_llm 的 LLM 栈**(新 prompt + web_control 执行器)
2. 唤醒词:**小星**
3. 播报范围:**A–F + H + I**(不含 G 雷达)
4. 麦克风:**USB CODEC(card3)**;外放:**nau8822(card1)**
5. 包结构:**独立 car_voice 包**(import 复用 car_llm 的类)
6. Web 开关:**拆"语音播报"/"语音触发"两个**(默认单进程内子开关实现)

## 开工前唯一待确认
- API Key:沿用 car_llm 默认 key,还是用独立的?(不影响动工,可先用默认)

