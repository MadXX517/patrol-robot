#!/usr/bin/env python3
# encoding: utf-8
# 手势命令节点 v2:状态机(唤醒+指令两段式) + 双手组合 + 动态手势
# -> 发布高层命令到 /patrol/command(由 person_follow 统一裁决运动)
# 设计见 car_patrol/PLAN_gesture_web.md 附录「手势 v2 设计」。
# 核心:默认休眠,手随意进画面不触发;挥手唤醒后才接收一个命令;双手张开=常驻急停。
import enum
import math
import time
import queue
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2


def _vector_2d_angle(v1, v2):
    """两向量夹角(度),-180~180。"""
    d = np.linalg.norm(v1) * np.linalg.norm(v2)
    if d == 0:
        return 0.0
    cos = v1.dot(v2) / d
    sin = np.cross(v1, v2) / d
    return float(np.degrees(np.arctan2(sin, cos)))


def hand_angle(landmarks):
    """各手指弯曲角度 [thumb, index, middle, ring, pink]。landmarks: 21x2 像素坐标。"""
    a = []
    a.append(_vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2]))
    a.append(_vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8]))
    a.append(_vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12]))
    a.append(_vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16]))
    a.append(_vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20]))
    return [abs(x) for x in a]


def h_gesture(angle_list):
    """由各指弯曲角度判别静态手势字符串。"""
    thr = 65.0
    thr_thumb = 53.0
    thr_s = 49.0
    a = angle_list
    g = "none"
    if (a[0] > thr_thumb) and (a[1] > thr) and (a[2] > thr) and (a[3] > thr) and (a[4] > thr):
        g = "fist"
    elif (a[0] < thr_s) and (a[1] < thr_s) and (a[2] > thr) and (a[3] > thr) and (a[4] > thr):
        g = "one"
    elif (a[0] > 5) and (a[1] < thr_s) and (a[2] > thr) and (a[3] > thr) and (a[4] > thr):
        g = "one"
    elif (a[0] > thr_thumb) and (a[1] < thr_s) and (a[2] < thr_s) and (a[3] > thr) and (a[4] > thr):
        g = "two"
    elif (a[0] > thr_thumb) and (a[1] < thr_s) and (a[2] < thr_s) and (a[3] < thr_s) and (a[4] > thr):
        g = "three"
    elif (a[0] > thr_thumb) and (a[1] < thr_s) and (a[2] < thr_s) and (a[3] < thr_s) and (a[4] < thr_s):
        g = "four"
    elif (a[0] < thr_s) and (a[1] < thr_s) and (a[2] < thr_s) and (a[3] < thr_s) and (a[4] < thr_s):
        g = "five"
    elif (a[0] < thr_s) and (a[1] > thr) and (a[2] > thr) and (a[3] > thr) and (a[4] > thr):
        g = "thumbup"   # 大拇指伸直,其余四指弯曲(姿势锁,用于摇动手势)
    return g


class FSM(enum.Enum):
    SLEEP = 0      # 休眠:只检测挥手唤醒 + 双手急停,其余忽略(根治误触发)
    ARMED = 1      # 已唤醒:接收一个命令手势,执行后进 CONFIRM
    CONFIRM = 2    # 确认期:顶栏显示刚触发的指令,保持数秒,再回 SLEEP


class MotionTracker:
    """缓存单手腕部轨迹(按时间戳),识别动态手势:挥手/左右滑/上下滑/画圈。
    归一化坐标(0~1),适应波动帧率。"""
    def __init__(self, window_s=1.5):
        self.window_s = window_s
        self.buf = deque()  # (t, x, y)

    def reset(self):
        self.buf.clear()

    def push(self, t, x, y):
        self.buf.append((t, x, y))
        while self.buf and (t - self.buf[0][0]) > self.window_s:
            self.buf.popleft()

    def _xs_ys(self):
        xs = [p[1] for p in self.buf]
        ys = [p[2] for p in self.buf]
        return xs, ys

    def is_wave(self, min_reversals=3, min_amp=0.04):
        """挥手:x 方向往复反转 >= min_reversals 次。"""
        if len(self.buf) < 6:
            return False
        xs, _ = self._xs_ys()
        # 计算方向反转次数(忽略微小抖动)
        dirs = []
        for i in range(1, len(xs)):
            dx = xs[i] - xs[i - 1]
            if abs(dx) > min_amp / 4:
                dirs.append(1 if dx > 0 else -1)
        reversals = sum(1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1])
        amp = (max(xs) - min(xs)) if xs else 0
        return reversals >= min_reversals and amp >= min_amp

    def _spans(self):
        xs, ys = self._xs_ys()
        ex = (max(xs) - min(xs)) if xs else 0.0
        ey = (max(ys) - min(ys)) if ys else 0.0
        path = 0.0
        for i in range(1, len(xs)):
            path += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        return ex, ey, path

    def _reversals(self, vals, min_step):
        dirs = []
        for i in range(1, len(vals)):
            d = vals[i] - vals[i - 1]
            if abs(d) > min_step:
                dirs.append(1 if d > 0 else -1)
        return sum(1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1])

    def free_motion(self, min_path=0.45, min_span=0.12):
        """自由大幅挥动(滑动/画圈合并):累计行程够长且包围盒够大。不看形状。"""
        if len(self.buf) < 6:
            return False
        ex, ey, path = self._spans()
        return path >= min_path and max(ex, ey) >= min_span

    def shake_axis(self, min_reversals=2, min_span=0.06, dom=1.5):
        """姿势锁摇动:返回 'updown'/'leftright'/None。
        主轴往复反转够次数,且主轴展开明显大于副轴(dom 倍)。"""
        if len(self.buf) < 6:
            return None
        xs, ys = self._xs_ys()
        ex, ey, _ = self._spans()
        rx = self._reversals(xs, min_span / 3)
        ry = self._reversals(ys, min_span / 3)
        # 左右摇:x 主轴
        if ex >= min_span and ex >= dom * ey and rx >= min_reversals:
            return 'leftright'
        # 上下摇:y 主轴
        if ey >= min_span and ey >= dom * ex and ry >= min_reversals:
            return 'updown'
        return None

    def debug_stats(self):
        """调参用:返回当前轨迹中间量。"""
        if len(self.buf) < 2:
            return 'n=%d' % len(self.buf)
        xs, ys = self._xs_ys()
        ex, ey, path = self._spans()
        rx = self._reversals(xs, 0.02)
        ry = self._reversals(ys, 0.02)
        return ('n=%d ex=%.2f ey=%.2f path=%.2f rx=%d ry=%d'
                % (len(self.buf), ex, ey, path, rx, ry))


# 命令冷却(秒):同命令短时间不重复下发
CMD_COOLDOWN = 3.0


class GestureCommandNode(Node):
    def __init__(self):
        super().__init__('gesture_command')

        # ---- 参数 ----
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('confirm_frames', 6)        # 静态命令稳定帧
        self.declare_parameter('armed_timeout', 15.0)      # ARMED 无操作超时回 SLEEP
        self.declare_parameter('confirm_hold', 3.0)        # 触发后确认期显示时长
        self.declare_parameter('cmd_cooldown', CMD_COOLDOWN)
        self.declare_parameter('process_every', 1)         # 每 N 帧处理一次(算力门控)
        self.declare_parameter('detect_width', 480)        # 喂 MediaPipe 前缩到此宽度(0=不缩)
        self.declare_parameter('flip', False)              # 画面镜像
        self.declare_parameter('publish_result', True)
        self.declare_parameter('enable_dynamic', True)     # 是否启用动态手势(滑动/画圈)
        self.declare_parameter('model_path', '')
        self.color_topic = self.get_parameter('color_topic').value
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.armed_timeout = float(self.get_parameter('armed_timeout').value)
        self.confirm_hold = float(self.get_parameter('confirm_hold').value)
        self.cmd_cooldown = float(self.get_parameter('cmd_cooldown').value)
        self.process_every = max(1, int(self.get_parameter('process_every').value))
        self.detect_width = int(self.get_parameter('detect_width').value)
        self.flip = bool(self.get_parameter('flip').value)
        self.publish_result = bool(self.get_parameter('publish_result').value)
        self.enable_dynamic = bool(self.get_parameter('enable_dynamic').value)

        # ---- 状态 ----
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.frame_idx = 0
        self.fsm = FSM.SLEEP
        self.armed_since = 0.0
        self.confirm_since = 0.0
        self.confirm_label = ''
        self.last_static = 'none'
        self.static_count = 0
        self.last_cmd = ''
        self.last_cmd_t = 0.0
        self.tracker = MotionTracker(window_s=1.5)   # 主手(用于唤醒/动态)
        self.last_action_label = ''                  # 给 web 显示的最近动作
        self._dbg_t = 0.0                            # 调试打印节流

        # ---- MediaPipe HandLandmarker(双手) ----
        model_path = self._resolve_model_path()
        self.get_logger().info('hand_landmarker model: %s' % model_path)
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options, min_hand_detection_confidence=0.3, num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(options)

        # ---- ROS 接口 ----
        self.pub_cmd = self.create_publisher(String, '/patrol/command', 5)
        self.pub_gesture = self.create_publisher(String, '~/gesture', 5)
        self.pub_result = self.create_publisher(Image, '~/result_img', 1)
        self.create_subscription(Image, self.color_topic, self._image_cb, 1)

        threading.Thread(target=self._proc_loop, daemon=True).start()
        self.get_logger().info(
            '\033[1;32mgesture_command v2 启动: topic=%s 动态=%s\033[0m'
            % (self.color_topic, self.enable_dynamic))

    def _resolve_model_path(self):
        """查找 hand_landmarker.task:参数 > car_patrol 包内 > car_vision 已安装路径。"""
        p = self.get_parameter('model_path').value
        if p and Path(p).exists():
            return p
        cands = [Path(__file__).parent / 'mediapipe' / 'model' / 'hand_landmarker.task']
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory('car_vision'))
            cands.append(share / 'mediapipe' / 'model' / 'hand_landmarker.task')
        except Exception:
            pass
        cands.append(Path.home() / 'Desktop' / 'ROS2' / 'SRC_20260427' / 'src' / 'car_vision'
                     / 'car_vision' / 'mediapipe' / 'model' / 'hand_landmarker.task')
        for c in cands:
            if Path(c).exists():
                return str(c)
        raise FileNotFoundError(
            'hand_landmarker.task 未找到,请用 -p model_path:=<路径> 指定。尝试: %s'
            % ', '.join(str(c) for c in cands))

    def _image_cb(self, msg):
        try:
            rgb = np.asarray(self.bridge.imgmsg_to_cv2(msg, 'rgb8'), dtype=np.uint8)
        except Exception as e:
            self.get_logger().warn('imgmsg fail: %s' % e)
            return
        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        self.image_queue.put(rgb)

    # ===================== 处理主循环 =====================
    def _proc_loop(self):
        while self.running:
            try:
                rgb = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            self.frame_idx += 1
            if self.flip:
                rgb = cv2.flip(rgb, 1)
            t = time.time()

            hands = []
            annotated = rgb
            if self.frame_idx % self.process_every == 0:
                try:
                    # 缩图再喂 MediaPipe:CPU 大降。归一化坐标不受尺寸影响,精度不变。
                    if self.detect_width and rgb.shape[1] > self.detect_width:
                        sc = self.detect_width / float(rgb.shape[1])
                        small = cv2.resize(rgb, (self.detect_width, int(rgb.shape[0] * sc)))
                    else:
                        small = rgb
                    hands, annotated = self._detect(small, small.copy())
                except Exception as e:
                    self.get_logger().warn('detect fail: %s' % e)

            action = self._run_fsm(hands, t)
            if action:
                self.last_action_label = action
            self._publish_gesture_state(hands, action)
            if self.publish_result:
                self._publish_result(annotated, hands, action)

    def _detect(self, rgb, annotated):
        """返回 (hands, annotated)。hands: list of dict{label, static, wrist(x,y norm), tip(x,y norm)}。"""
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.detector.detect(mp_image)
        h, w, _ = annotated.shape
        hands = []
        for idx, lm in enumerate(result.hand_landmarks):
            proto = landmark_pb2.NormalizedLandmarkList()
            proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=p.x, y=p.y, z=p.z) for p in lm])
            mp.solutions.drawing_utils.draw_landmarks(
                annotated, proto, mp.solutions.hands.HAND_CONNECTIONS,
                mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                mp.solutions.drawing_styles.get_default_hand_connections_style())
            pts = np.array([[p.x * w, p.y * h] for p in lm])
            static = h_gesture(hand_angle(pts))
            label = 'unknown'
            try:
                label = result.handedness[idx][0].category_name  # 'Left'/'Right'
            except Exception:
                pass
            hands.append({
                'label': label,
                'static': static,
                'wrist': (float(lm[0].x), float(lm[0].y)),     # 归一化
                'tip': (float(lm[8].x), float(lm[8].y)),       # 食指尖归一化
            })
        return hands, annotated

    # ===================== 状态机 =====================
    def _both_open(self, hands):
        """双手张开五指 = 常驻急停。"""
        opens = [hh for hh in hands if hh['static'] == 'five']
        return len(opens) >= 2

    def _fire(self, cmd, action_label):
        """下发命令到 /patrol/command(带冷却)。返回是否真正下发。"""
        now = time.time()
        if cmd == self.last_cmd and (now - self.last_cmd_t) < self.cmd_cooldown:
            return False
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)
        self.last_cmd = cmd
        self.last_cmd_t = now
        self.get_logger().info('FIRE %s (%s)' % (cmd, action_label))
        return True

    def _enter_confirm(self, cmd, action_label):
        """触发命令并进入 CONFIRM 确认期(顶栏显示数秒再回 SLEEP)。"""
        fired = self._fire(cmd, action_label)
        self.fsm = FSM.CONFIRM
        self.confirm_since = time.time()
        self.tracker.reset()
        self.static_count = 0
        if fired:
            self.confirm_label = '%s -> %s' % (action_label, cmd)
        else:
            # 冷却中没真正下发,也给确认期提示,避免立刻重触发
            self.confirm_label = '%s (cooldown)' % cmd
        return self.confirm_label

    def _run_fsm(self, hands, t):
        """返回本帧产生的动作标签字符串(用于显示),无则 ''。"""
        # --- 急停:任何状态,常驻最高优先级 ---
        if self._both_open(hands):
            return self._enter_confirm('stop', 'EMERGENCY_STOP')

        # 主手:取第一只手用于唤醒/动态轨迹
        main = hands[0] if hands else None
        if main is not None:
            self.tracker.push(t, main['wrist'][0], main['wrist'][1])
        else:
            self.tracker.reset()

        if self.fsm == FSM.CONFIRM:
            return self._fsm_confirm(t)
        if self.fsm == FSM.SLEEP:
            return self._fsm_sleep(main)
        return self._fsm_armed(hands, main, t)

    def _fsm_confirm(self, t):
        """确认期:保持显示触发的指令,到时回 SLEEP。期间忽略其它手势。"""
        if (t - self.confirm_since) > self.confirm_hold:
            self.fsm = FSM.SLEEP
            self.confirm_label = ''
            return ''
        return self.confirm_label

    def _fsm_sleep(self, main):
        """休眠态:只认挥手唤醒。其余一切忽略 -> 根治误触发。"""
        if main is not None and self.tracker.is_wave():
            self.fsm = FSM.ARMED
            self.armed_since = time.time()
            self.static_count = 0
            self.last_static = 'none'
            self.tracker.reset()
            self.get_logger().info('WAKE (wave)')
            return 'WAKE'
        return ''

    def _fsm_armed(self, hands, main, t):
        """已唤醒态:接收一个命令(静态/双手/动态),执行后进 CONFIRM;超时回 SLEEP。"""
        if (t - self.armed_since) > self.armed_timeout:
            self.fsm = FSM.SLEEP
            self.get_logger().info('ARMED timeout -> SLEEP')
            return 'TIMEOUT'

        # 调参调试:ARMED 时按节流打印轨迹中间量(~每0.5s一次)
        if main is not None and (t - self._dbg_t) > 0.5:
            self._dbg_t = t
            self.get_logger().info('DBG %s %s' % (main['static'], self.tracker.debug_stats()))

        # 1) 双手组合:双手各比 two -> relock
        twos = [hh for hh in hands if hh['static'] == 'two']
        if len(twos) >= 2:
            return self._enter_confirm('relock', 'both_two')

        # 2) 动态手势(姿势锁+运动,可关)。靠当前姿势分流,天然互斥。
        if self.enable_dynamic and main is not None:
            g = main['static']
            if g == 'thumbup':
                # 大拇指 + 上下摇 / 左右摇
                ax = self.tracker.shake_axis()
                if ax == 'updown':
                    return self._enter_confirm('thumb_updown', 'thumbup_updown')
                if ax == 'leftright':
                    return self._enter_confirm('thumb_leftright', 'thumbup_leftright')
            elif g == 'five':
                # 张开手 + 大幅自由挥动(原滑动/画圈合并为一个)
                if self.tracker.free_motion():
                    return self._enter_confirm('free_motion', 'open_freemotion')

        # 3) 静态单手命令(需稳定帧)。follow 绑 four,避免挥手后张开手误触发。
        g = main['static'] if main is not None else 'none'
        if g == self.last_static and g != 'none':
            self.static_count += 1
        else:
            self.static_count = 0
        self.last_static = g
        if self.static_count >= self.confirm_frames:
            cmd = {'one': 'track_only', 'four': 'follow'}.get(g)
            if cmd:
                return self._enter_confirm(cmd, 'static_%s' % g)
        return ''

    # ===================== 发布 =====================
    def _publish_gesture_state(self, hands, action):
        st = {FSM.ARMED: 'armed', FSM.CONFIRM: 'confirm'}.get(self.fsm, 'sleep')
        statics = '+'.join(hh['static'] for hh in hands) if hands else 'none'
        nhands = len(hands)
        msg = String()
        msg.data = ('{"state": "%s", "hands": %d, "statics": "%s", '
                    '"action": "%s", "last_cmd": "%s"}'
                    % (st, nhands, statics, action or '', self.last_cmd))
        self.pub_gesture.publish(msg)

    def _publish_result(self, rgb, hands, action):
        try:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            now = time.time()
            # 状态横幅:CONFIRM(绿,显示触发的指令) / ARMED(橙+倒计时) / SLEEP(灰)
            if self.fsm == FSM.CONFIRM:
                left = max(0.0, self.confirm_hold - (now - self.confirm_since))
                color = (0, 200, 0)
                label = 'DONE: %s  (%.0fs)' % (self.confirm_label, left)
            elif self.fsm == FSM.ARMED:
                left = max(0.0, self.armed_timeout - (now - self.armed_since))
                color = (0, 200, 255)
                label = 'ARMED (do a command)  %.0fs' % left
            else:
                color = (160, 160, 160)
                label = 'SLEEP (wave to wake)'
            cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 40), color, -1)
            cv2.putText(bgr, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            # 手势信息
            statics = ' | '.join('%s:%s' % (hh['label'][:1], hh['static']) for hh in hands)
            cv2.putText(bgr, 'hands: %s' % (statics or '-'), (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if self.last_cmd:
                cv2.putText(bgr, 'last_cmd: %s' % self.last_cmd, (10, 98),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            out = cv2.resize(bgr, (640, 360))
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(out, 'bgr8'))
        except Exception as e:
            self.get_logger().warn('publish result fail: %s' % e)


def main():
    rclpy.init()
    node = GestureCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
