# car_patrol 下一步开发计划:手势控制 + Web 深度集成

> 状态:规划中 | 重建于电量排查会话后,基于当前代码现状重新制定
> 目标:在 car_patrol 包内接入手势控制(巡航功能切换:自主巡航/智能跟随),并与 car_web 控制端深度结合。

## 0. 背景与定位

巡逻辅助机器人(双模式)的"随行模式"已基本完成:`person_follow.py` 实现了智能跟随(follow)
与云台追踪(track_only),含麦轮全向移动、雷达避障、丢失搜索。下一步是让操作员能用
**手势**就地切换/控制机器人行为,并让 **Web 端**成为统一的态势与控制中枢。

## 1. 现状盘点(已有资产)

### car_patrol/person_follow.py
- 统一命令入口:订阅 `/patrol/command` (std_msgs/String),`_dispatch()` 处理:
  - `follow`/`start` → 智能跟随;`track_only`/`gimbal_track` → 仅云台;`stop`/`pause` → 停;
    `relock` → 重新锁定;`gimbal_home` → 云台回中
- 状态发布:`~/state`(即 `/person_follow/state`,JSON String):mode/running/locked/front_min 等
- 服务:`~/enter ~/exit ~/set_running ~/relock`(Trigger/SetBool)
- 模式参数:`mode`(follow|track_only)、`machine_type`(Mec|Ack)

### car_vision/hand_gesture.py(关键可复用资产,但需改造)
- MediaPipe HandLandmarker,识别 fist/one/two/three/four/five/six
- 当前问题:① 直接发 `/cmd_vel` 开环动作(前进/后退/转),非"模式切换";
  ② 带 `cv2.imshow` GUI(板端 headless 不可用);③ 与 person_follow 抢 `/cmd_vel` 控制权

### car_web(已具备深度集成基础)
- 后端 dashboard_server.py:FEATURES 列表(已含 `pose_detect` 预留入口,available=False)
- 发布 `/patrol/command`,订阅 `/person_follow/state`、`/PowerVoltage`
- HTTP API:`/api/features/start|stop`、`/api/patrol/command`、`/api/patrol/relock`、`/api/drive` 等
- 前端 app.js:功能面板、relock 按钮、巡逻状态显示、电量徽标

## 2. 核心设计决策

### 决策 A:手势 = 高层"指令",不是底层"动作"
当前 hand_gesture.py 用手势直接发 cmd_vel 速度,会与 person_follow 抢 `/cmd_vel`,且只是遥控
而非"巡航功能切换"。新设计:**手势节点只发布高层命令到 `/patrol/command`**,由 person_follow
统一裁决运动。手势不直接碰 `/cmd_vel`,彻底避免控制权冲突。

手势→命令映射(初版,可调):
| 手势 | 命令 | 含义 |
|------|------|------|
| five(张开手掌) | `follow` | 进入智能跟随(随行模式) |
| fist(握拳) | `stop` | 停止/待机 |
| one(食指) | `track_only` | 仅云台追踪 |
| two(食指+中指) | `relock` | 重新锁定目标 |
| (预留)three/four | `patrol_auto` 等 | 后续接自主巡航/巡线 |

> **v2 重大修订(误触发根治 + 复杂手势)**:见文末「附录:手势 v2 设计」。
> v1 的"常驻监听 + fist=stop"被废弃(手自然握拳进画面即误停机)。
> v2 改为**状态机(唤醒+指令两段式)+ 双手组合 + 动态手势**。

### 决策 B:新建独立手势节点,放 car_patrol 包内
不改 car_vision/hand_gesture.py(它服务于机械臂等其它场景)。在 car_patrol 内新建
`gesture_command.py`:复用 hand_gesture.py 的 hand_angle()/h_gesture() 算法(可抽到共用模块或
直接移植),**去掉 imshow GUI**(板端 headless),**去掉 cmd_vel 直驱**,改为发 `/patrol/command`。
- 防误触发:同一手势需连续稳定 N 帧(沿用现有 count>10 防抖),且加**命令去抖**(同命令冷却 2-3s)
- 标注图发布到 `~/image_result`(供 web 视频流查看手势识别效果),不开窗口

### 决策 C:谁来运行 YOLO/相机?——共用相机,手势与跟随可并存
person_follow 用 `/camera/color/image_raw` + YOLO 人体框;手势节点也订阅同一彩色话题跑 MediaPipe。
两者共享相机,互不抢。手势节点常驻轻量运行,识别到指令就切换 person_follow 的模式。
- 注意算力:MediaPipe Hand + YOLO 同时跑在 RK3568,需实测帧率。若吃紧,手势节点降帧
  (每 2-3 帧处理一次)或做成"按需启停"(web 开关)。

### 决策 D:Web 集成——把手势节点纳入 FEATURES + 状态可视化
- `pose_detect` 预留入口改造为"手势控制"功能:available=True,start 时启动手势节点 launch
- 新增手势状态显示:手势节点发布 `~/gesture`(当前识别手势+最近触发命令),web 订阅并展示
- web 功能面板:自主巡航/智能跟随切换按钮(本质是发 `/patrol/command`,与手势同入口)
- 手势识别画面接入 web 视频流(`~/image_result`)

## 3. 实施阶段(每阶段可独立验证)

### 阶段 1:手势命令节点(car_patrol/gesture_command.py)
- [x] 移植 hand_angle()/h_gesture() 手势算法(去 GUI、去 cmd_vel)
- [x] 订阅 `/camera/color/image_raw`,MediaPipe 识别,发布 `/patrol/command`
- [x] 手势→命令映射(决策 A 表),含连续帧防抖 + 同命令冷却
- [x] 发布 `~/gesture` 状态、`~/result_img` 标注图
- [x] setup.py 注册 entry_point、加 launch 文件(gesture_command.launch.py)
- [ ] 验证:桌面无底盘下,做手势看 `/patrol/command` 是否正确发出(ros2 topic echo)

### 阶段 1.5:Web 实时显示(随阶段1一并做,便于实时观察)
- [x] car_web FEATURES:pose_detect → gesture_control(available=True)
- [x] dashboard start_gesture/stop_gesture:起 gesture+follow(track_only)节点,视频切手势画面
- [x] 订阅 /gesture_command/gesture,status 暴露 gesture_state
- [x] 前端 app.js renderGestureState + index.html 手势状态行
- [ ] 验证:web 启动"手势控制" → 视频看到手势识别 + 状态实时回显

### 阶段 2:手势 + 跟随联调(板端)
- [ ] 手势节点 + person_follow 同时跑,共用相机
- [ ] 实测 MediaPipe + YOLO 并行帧率,不足则降帧/按需启停
- [ ] 验证:five→进入跟随、fist→停、one→云台追踪、two→relock 全链路生效
- [ ] 带底盘前先 track_only 验证(底盘零速安全),再带底盘

### 阶段 3:Web 后端集成(car_web/dashboard_server.py)
- [x] `pose_detect` → "手势控制":available=True,加 gesture_command 的 launch 启停逻辑
- [x] 订阅 `~/gesture`,在 `/api/status` 暴露当前手势+最近命令(gesture_state)
- [x] start_gesture 拉起 core+human_follow(track_only)+gesture 全链路
- [x] 修 `patrol_command` 的 cmd 未定义 bug;`/api/patrol/command` 通用命令端点
- [ ] 验证:web 点击启动手势控制 → 节点起 → 做手势 → 状态回显

### 阶段 4:Web 前端可视化(car_web/web/app.js + index.html)
- [x] 功能面板"手势控制"卡片可点击启停(功能中心动态渲染)
- [x] 手势状态徽标(v2:休眠/已唤醒/已触发 + 手数 + 最近命令)
- [x] 手势识别画面接入视频流切换(GESTURE_RESULT_TOPIC)
- [x] 模式切换命令按钮(智能跟随/云台追踪/停止/重锁 → /api/patrol/command)
- [ ] 验证:web 端完整呈现手势识别 + 模式切换 + 状态

## 4. 风险与注意

- **算力**:MediaPipe Hand + YOLOv5(RKNN)+ 跟随控制同跑在 RK3568,是最大不确定项。
  阶段 2 必须先实测帧率,再决定是否降帧/按需启停手势节点。
- **控制权**:严守"手势只发命令、不碰 cmd_vel",person_follow 是唯一的 /cmd_vel 发布者
  (web 手动遥控 /api/drive 除外,且二者互斥由 web 协调)。
- **误触发安全**:手势触发模式切换前必须防抖;尤其 `follow`(会让底盘动)要更严格的确认帧数。
- **headless**:板端无显示器,手势节点严禁 cv2.imshow(现有 hand_gesture.py 的坑)。
- **camera_topic**:确认板端实际彩色话题名(person_follow 用 `/camera/color/image_raw`,
  Orbbec/Astra 相机,启动脚本里已固定)。

## 5. 运行环境备忘(见 memory)
- 启动脚本持久放 `~/follow_scripts/`(/tmp 重启清空)
- 雷达每次开机手动起 `ros2 launch car_base car_lidar.launch.py`
- 远程 pkill 用 `pkill -9 -f 'pers[o]n_follow'` 防自杀;启动与验证分两次 ssh
- 电量:`/PowerVoltage` 原始字节 × 0.178 = 伏特(3 串锂电 12.6/9.9V)
- 桌面安全测试:track_only 模式底盘强制零速

---

## 附录:手势 v2 设计(误触发根治 + 复杂手势)

### 问题
v1 把 `stop` 绑在 `fist`(握拳),而握拳是手自然放松下垂时最接近的形态;且 v1 常驻监听
(任何手进画面稳定够帧即触发)。结果:手一进画面就误触发停机。安全命令绑在最易无意做出的
手势上,设计反了。

### 状态机(核心:唤醒+指令两段式)
三态,默认休眠,手随意进画面不触发任何命令:
```
SLEEP(休眠) --挥手唤醒--> ARMED(接收指令) --做一个命令手势--> 执行 --> 回 SLEEP
                              ARMED 超时(无操作 ~8s) --> 回 SLEEP
任何状态 --双手张开五指--> EMERGENCY_STOP(立即停,然后回 SLEEP)
```
- SLEEP:只检测「挥手唤醒」和「双手急停」,其余一律忽略。彻底解决误触发。
- ARMED:接收命令手势(静态/双手/动态),做完一个即回 SLEEP;web/标注图显示醒目"已唤醒"。
- 急停常驻:双手同时张开五指,任何状态立即 `stop`,既可靠又不会无意做出。

### 命令词汇(v2)
唤醒:挥手(左右摆动)。急停:双手张开五指(常驻)。
ARMED 态命令:
| 类型 | 手势 | 命令 |
|------|------|------|
| 静态单手 | one(食指) | track_only |
| 静态单手 | five(张开) | follow |
| 静态双手 | 双手 two(各比2) | relock |
| 动态 | 左右滑动 | (预留)切换目标/转向 |
| 动态 | 上下滑动 | (预留)云台俯仰 |
| 动态 | 画圈 | (预留)patrol_auto 自主巡航 |
注:`stop` 移出单手静态,只保留「双手急停」常驻,根除误触发。

### 双手识别
MediaPipe HandLandmarker num_hands=2,handedness 区分左右手。双手组合扩展词汇
(决策:双手组合)。计算量翻倍,需实测帧率。

### 动态手势检测
缓存最近 ~1.5s 的手腕/指尖关键点轨迹(deque,按时间戳而非帧数,适应 4-18Hz 波动帧率):
- 挥手:x 方向往复反转 ≥N 次
- 左右/上下滑动:单向位移超阈值 + 主轴判定
- 画圈:轨迹角度累积约 360°
**风险**:彩色帧率波动大(实测 4~18Hz),低帧率下动态识别不稳;按时间戳归一化可缓解,
但仍需实测调参。先静态+双手急停跑通(根治误触发),动态手势作增量。

### 实现拆分(gesture_command.py v2)
1. [x] 重映射 + 状态机骨架(SLEEP/ARMED) + 挥手唤醒 + 双手急停 → 先解决误触发
2. [x] 双手组合命令(双手 two → relock)
3. [x] 动态手势(左右滑/画圈,占位映射)→ 待实测帧率调参
4. [x] web 显示状态机当前态(休眠/已唤醒 + 倒计时)
   - [ ] 板端实测验证:挥手唤醒、各命令、急停、动态手势识别率与帧率
