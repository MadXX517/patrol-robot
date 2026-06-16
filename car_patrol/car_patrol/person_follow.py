#!/usr/bin/env python3
# encoding: utf-8
# car_patrol / person_follow
# 人体跟随 + 云台追踪:YOLO 选人 -> NanoTrack 锁定保持(轻量 re-ID) -> 深度/框测距
# -> 底盘按安全距离尾随(不冲撞) + 云台 pitch/yaw 追踪。
#
# 模式(参数 mode 或 /patrol/command):
#   follow      : 底盘尾随(角度对中 + 距离 PID) + 云台 pitch 保持人入框
#   track_only  : 底盘不动,仅云台 yaw+pitch 追踪
#
# 统一指令入口 /patrol/command(std_msgs/String):
#   follow / follow_on / start  -> 进入 follow 并开始
#   track_only / gimbal_track   -> 进入 track_only 并开始
#   stop / follow_off           -> 停止(底盘清零)
#   relock                      -> 重新锁定目标
#   gimbal_home                 -> 云台回正
import os
import json
import math
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

import car_vision.pid as pid
import car_vision.common as common
import car_vision.arm_ik_sdk as arm_ik_sdk
from interfaces.msg import ObjectsInfo

# 云台固定姿态(face_tracking 同款):[yaw, -0.930, 1.6, pitch, 0, 0.801]
GIMBAL_FIXED = [-0.930, 1.6, 0.0, 0.801]   # j1,j2,j4,j5
YAW_LIMIT = (-1.0, 1.0)      # joint0
PITCH_LIMIT = (0.8, 1.6)     # joint3
YAW_HOME = 0.0
PITCH_HOME = 1.2


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follow')

        # ---------------- 参数 ----------------
        self.declare_parameter('mode', 'follow')                 # follow | track_only
        self.declare_parameter('machine_type', 'Mec')            # Mec | Ack
        self.declare_parameter('follow_distance', 1.2)           # 期望跟随距离(m)
        self.declare_parameter('safe_distance', 0.8)             # 安全停车距离(m)
        self.declare_parameter('max_lin', 0.25)                  # 最大线速度
        self.declare_parameter('max_ang', 0.8)                   # 最大角速度
        self.declare_parameter('lost_timeout', 2.0)              # 丢失多久后停
        self.declare_parameter('auto_start', False)              # 启动即开始(便于单测)
        self.declare_parameter('select_policy', 'center')        # center | largest
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('yolo_topic', '/car_yolo/object_detect')
        self.declare_parameter('use_lidar_safety', True)

        self.mode = self.get_parameter('mode').value
        self.machine_type = self.get_parameter('machine_type').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.select_policy = self.get_parameter('select_policy').value
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        yolo_topic = self.get_parameter('yolo_topic').value
        self.use_lidar_safety = bool(self.get_parameter('use_lidar_safety').value)

        # ---------------- 状态 ----------------
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.running = bool(self.get_parameter('auto_start').value)
        self.locked = False
        self.relock_request = False
        self.target_present = False
        self.last_seen = 0.0
        self.last_box = None            # (x,y,w,h) 当前帧颜色图坐标
        self.target_hist = None
        self.frame_count = 0

        self.latest_color = None        # bgr np
        self.latest_color_stamp = 0.0
        self.latest_depth = None        # uint16 mm np
        self.latest_persons = []        # [(x1,y1,x2,y2,score), ...]
        self.latest_persons_stamp = 0.0
        self.front_min = float('inf')   # 雷达正前方最近距离

        self.gimbal_yaw = YAW_HOME
        self.gimbal_pitch = PITCH_HOME
        self.last_gimbal_t = 0.0

        # 跟踪以 YOLO person 框为准,IoU + 颜色直方图做帧间关联(不用易漂移的 NanoTrack)

        # PID
        self.pid_ang = pid.PID(0.0035, 0.0, 0.0005)     # 角度对中(基于像素)
        self.pid_lin = pid.PID(0.6, 0.0, 0.05)          # 距离(基于米)
        self.pid_gyaw = pid.PID(0.0020, 0.0, 0.0)       # 云台 yaw(像素)
        self.pid_gpitch = pid.PID(0.0020, 0.0, 0.0)     # 云台 pitch(像素)

        # 云台
        self.arm = arm_ik_sdk.ArmControl(self)

        # ---------------- 通信 ----------------
        sensor_qos = qos_profile_sensor_data
        self.create_subscription(Image, color_topic, self._color_cb, sensor_qos)
        self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(ObjectsInfo, yolo_topic, self._yolo_cb, 1)
        if self.use_lidar_safety:
            scan_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(String, '/patrol/command', self._cmd_cb, 1)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_result = self.create_publisher(Image, '~/result_img', 1)
        self.pub_state = self.create_publisher(String, '~/state', 1)

        self.create_service(Trigger, '~/enter', self._enter_srv)
        self.create_service(Trigger, '~/exit', self._exit_srv)
        self.create_service(SetBool, '~/set_running', self._set_running_srv)
        self.create_service(Trigger, '~/relock', self._relock_srv)

        self._gimbal_home()
        self.create_timer(1.0 / 20.0, self._loop)   # 20Hz 主循环(视频更顺;检测仍受 YOLO ~14Hz 限制)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info('\033[1;32mperson_follow 启动: mode=%s\033[0m' % self.mode)

    # ===================== 工具 =====================
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _hist(self, bgr, box):
        # box=(x,y,w,h) 取躯干中心区域的 HS 直方图作外观签名
        x, y, w, h = [int(v) for v in box]
        cx0 = x + int(w * 0.2); cx1 = x + int(w * 0.8)
        cy0 = y + int(h * 0.15); cy1 = y + int(h * 0.6)
        H, W = bgr.shape[:2]
        cx0 = _clamp(cx0, 0, W - 1); cx1 = _clamp(cx1, 1, W)
        cy0 = _clamp(cy0, 0, H - 1); cy1 = _clamp(cy1, 1, H)
        if cx1 <= cx0 or cy1 <= cy0:
            return None
        roi = bgr[cy0:cy1, cx0:cx1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    # ===================== 回调 =====================
    def _color_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn('color cvt fail: %s' % e)
            return
        with self.lock:
            self.latest_color = img
            self.latest_color_stamp = self._now()

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        with self.lock:
            self.latest_depth = depth

    def _yolo_cb(self, msg):
        persons = []
        for obj in msg.objects:
            if obj.class_name == 'person' and len(obj.box) == 4:
                x1, y1, x2, y2 = obj.box
                persons.append((int(x1), int(y1), int(x2), int(y2), float(obj.score)))
        with self.lock:
            self.latest_persons = persons
            self.latest_persons_stamp = self._now()

    def _scan_cb(self, msg):
        # 取正前方 ±20° 扇区最近距离
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        ang_min = msg.angle_min
        inc = msg.angle_increment
        half = math.radians(20)
        fmin = float('inf')
        for i in range(n):
            a = ang_min + i * inc
            # 归一到 [-pi,pi]
            a = (a + math.pi) % (2 * math.pi) - math.pi
            if abs(a) <= half:
                r = ranges[i]
                if math.isfinite(r) and r > 0.05 and r < fmin:
                    fmin = r
        self.front_min = fmin

    def _cmd_cb(self, msg):
        self._dispatch(msg.data.strip().lower())

    def _dispatch(self, cmd):
        if cmd in ('follow', 'follow_on', 'start'):
            with self.lock:
                self.mode = 'follow'; self.running = True; self.relock_request = True
            self.get_logger().info('cmd: follow on')
        elif cmd in ('track_only', 'gimbal_track', 'track'):
            with self.lock:
                self.mode = 'track_only'; self.running = True; self.relock_request = True
            self.get_logger().info('cmd: track_only')
        elif cmd in ('stop', 'follow_off', 'pause'):
            with self.lock:
                self.running = False
            self._publish_zero_vel()
            self.get_logger().info('cmd: stop')
        elif cmd == 'relock':
            with self.lock:
                self.relock_request = True
            self.get_logger().info('cmd: relock')
        elif cmd == 'gimbal_home':
            self._gimbal_home()

    # ===================== 服务 =====================
    def _enter_srv(self, req, resp):
        self._gimbal_home()
        resp.success = True; resp.message = 'entered'
        return resp

    def _exit_srv(self, req, resp):
        with self.lock:
            self.running = False
        self._publish_zero_vel(); self._gimbal_home()
        resp.success = True; resp.message = 'exited'
        return resp

    def _set_running_srv(self, req, resp):
        with self.lock:
            self.running = bool(req.data)
            if self.running:
                self.relock_request = True
        if not req.data:
            self._publish_zero_vel()
        resp.success = True; resp.message = 'running=%s' % self.running
        return resp

    def _relock_srv(self, req, resp):
        with self.lock:
            self.relock_request = True
        resp.success = True; resp.message = 'relock requested'
        return resp

    # ===================== 选目标/锁定 =====================
    def _sane_persons(self, persons, img_w, img_h):
        # 过滤明显不合理的框:整幅宽、几乎整幅、低分。避免 YOLO 偶发超大/合并框。
        out = []
        for (x1, y1, x2, y2, score) in persons:
            w = x2 - x1; h = y2 - y1
            if w <= 6 or h <= 6:
                continue
            if score < 0.4:
                continue
            # 不再按框大小剔除:接受任意大小/比例的 person(含近全屏、手臂前伸、被边缘截断)
            out.append((x1, y1, x2, y2, score))
        return out

    def _pick_target(self, img_w, img_h, persons):
        if not persons:
            return None
        if self.select_policy == 'largest':
            best = max(persons, key=lambda p: (p[2] - p[0]) * (p[3] - p[1]))
        else:  # center
            cx0 = img_w / 2.0
            best = min(persons, key=lambda p: abs((p[0] + p[2]) / 2.0 - cx0))
        x1, y1, x2, y2, _ = best
        return (x1, y1, x2 - x1, y2 - y1)

    def _reacquire(self, bgr, persons):
        # 用颜色直方图在 YOLO person 中找回同一人
        if not persons or self.target_hist is None:
            return None
        best_box, best_score = None, 0.0
        for (x1, y1, x2, y2, _) in persons:
            box = (x1, y1, x2 - x1, y2 - y1)
            h = self._hist(bgr, box)
            if h is None:
                continue
            s = cv2.compareHist(self.target_hist, h, cv2.HISTCMP_CORREL)
            if s > best_score:
                best_score, best_box = s, box
        if best_box is not None and best_score > 0.5:
            return best_box
        return None

    @staticmethod
    def _iou(a, b):
        ax1, ay1, aw, ah = a; ax2, ay2 = ax1 + aw, ay1 + ah
        bx1, by1, bx2, by2, _ = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = iw * ih
        ua = aw * ah + (bx2 - bx1) * (by2 - by1) - inter
        return inter / ua if ua > 0 else 0.0

    # ===================== 测距 =====================
    def _distance_from_depth(self, box, color_shape):
        depth = self.latest_depth
        if depth is None:
            return None
        ch, cw = color_shape[:2]
        dh, dw = depth.shape[:2]
        x, y, w, h = box
        cx = x + w / 2.0; cy = y + h * 0.45   # 取偏上(躯干)更稳
        dx = int(cx * dw / cw); dy = int(cy * dh / ch)
        dx = _clamp(dx, 1, dw - 2); dy = _clamp(dy, 1, dh - 2)
        patch = depth[dy - 1:dy + 2, dx - 1:dx + 2].astype(np.float32).flatten()
        patch = patch[(patch > 200) & (patch < 8000)]   # 0.2~8m 有效
        if patch.size == 0:
            return None
        return float(np.median(patch)) / 1000.0

    # ===================== 主循环 =====================
    def _loop(self):
        with self.lock:
            running = self.running
            bgr = None if self.latest_color is None else self.latest_color.copy()
            persons = list(self.latest_persons)
            persons_fresh = (self._now() - self.latest_persons_stamp) < 1.0
            relock = self.relock_request
            self.relock_request = False
            mode = self.mode
        if bgr is None:
            return
        img_h, img_w = bgr.shape[:2]

        if not running:
            self._draw_and_publish(bgr, None, None, 'idle')
            return

        fresh_persons = self._sane_persons(persons, img_w, img_h) if persons_fresh else []

        # --- 锁定/重锁:从合理的 person 框中选目标 ---
        if relock or not self.locked:
            tgt = self._pick_target(img_w, img_h, fresh_persons)
            if tgt is not None:
                tgt = self._sanitize_box(tgt, img_w, img_h)
                self.target_hist = self._hist(bgr, tgt)
                self.last_box = tgt
                self.locked = True
                self.target_present = True
                self.last_seen = self._now()
                self.frame_count = 0
                self.get_logger().info('locked target box=%s' % str(tgt))
            else:
                self._publish_zero_vel()
                self._draw_and_publish(bgr, None, None, 'searching')
                return

        # --- 跟踪:按"中心距离"关联(对快速移动+低帧率宽容,且省 CPU) ---
        matched = None
        if fresh_persons:
            lb = self.last_box
            lcx = lb[0] + lb[2] / 2.0
            lcy = lb[1] + lb[3] / 2.0

            def center_dist(p):
                pcx = (p[0] + p[2]) / 2.0
                pcy = (p[1] + p[3]) / 2.0
                return ((pcx - lcx) ** 2 + (pcy - lcy) ** 2) ** 0.5
            best = min(fresh_persons, key=center_dist)
            if center_dist(best) < 0.35 * img_w:   # 中心位移门限
                matched = (best[0], best[1], best[2] - best[0], best[3] - best[1])
        if matched is not None:
            nb = self._sanitize_box(matched, img_w, img_h)
            # EMA 平滑(仅连续跟踪时)
            if self.last_box is not None and self.target_present:
                a = 0.6
                ob = self.last_box
                nb = (int(a * nb[0] + (1 - a) * ob[0]), int(a * nb[1] + (1 - a) * ob[1]),
                      int(a * nb[2] + (1 - a) * ob[2]), int(a * nb[3] + (1 - a) * ob[3]))
            self.last_box = nb
            self.target_present = True
            self.last_seen = self._now()
            self.frame_count += 1
        else:
            self.target_present = False

        # --- 控制 ---
        if not self.target_present:
            if self._now() - self.last_seen > self.lost_timeout:
                self._publish_zero_vel()
            self._draw_and_publish(bgr, self.last_box, None, 'lost')
            return

        box = self.last_box
        bx, by, bw, bh = box
        cx = bx + bw / 2.0
        cy = by + bh / 2.0
        dist = self._distance_from_depth(box, bgr.shape)

        twist = Twist()
        if mode == 'follow':
            # 角度对中
            self.pid_ang.SetPoint = img_w / 2.0
            self.pid_ang.update(cx)
            ang = common.set_range(self.pid_ang.output, -self.max_ang, self.max_ang)
            if abs(cx - img_w / 2.0) < img_w * 0.04:
                ang = 0.0
            # 距离跟随
            lin = 0.0
            if dist is not None:
                self.pid_lin.SetPoint = self.follow_distance
                self.pid_lin.update(dist)
                lin = common.set_range(-self.pid_lin.output, -self.max_lin, self.max_lin)
                if abs(dist - self.follow_distance) < 0.12:
                    lin = 0.0
            else:
                # 无深度: 用框高占比作距离代理(越大越近)
                ratio = bh / float(img_h)
                err = 0.45 - ratio      # 目标占比 0.45
                lin = common.set_range(err * 1.2, -self.max_lin, self.max_lin)
            # 安全: 太近 / 雷达前方有障碍 -> 禁止前进
            too_close = (dist is not None and dist < self.safe_distance)
            lidar_block = (self.use_lidar_safety and self.front_min < self.safe_distance)
            if too_close or lidar_block:
                lin = min(lin, 0.0)
            twist.linear.x = float(lin)
            twist.angular.z = float(ang)
            self.pub_vel.publish(twist)
            # 云台仅 pitch 保持人垂直入框
            self._gimbal_track(cx, cy, img_w, img_h, yaw_active=False)
        else:  # track_only
            self._publish_zero_vel()
            self._gimbal_track(cx, cy, img_w, img_h, yaw_active=True)

        self._draw_and_publish(bgr, box, dist, mode)

    def _sanitize_box(self, box, img_w, img_h):
        x, y, w, h = box
        x = _clamp(int(x), 0, img_w - 2)
        y = _clamp(int(y), 0, img_h - 2)
        w = _clamp(int(w), 1, img_w - x)
        h = _clamp(int(h), 1, img_h - y)
        return (x, y, w, h)

    # ===================== 云台 =====================
    # 云台控制参数(本云台仅 yaw 左右,无俯仰)
    GIMBAL_PERIOD = 0.28   # 发令周期(秒):转到位再发下一条,防过冲
    GIMBAL_DZ = 0.13       # 死区(归一化):目标进入画面中央±13%即停,防小幅震荡
    GIMBAL_GAIN = 0.28     # 比例增益(略降,减小过冲)
    GIMBAL_STEP = 0.07     # 每条指令最大步进(rad ≈ 4°)
    GIMBAL_DUR_MS = 260    # 舵机运动时长,与周期匹配

    def _gimbal_track(self, cx, cy, img_w, img_h, yaw_active):
        if not yaw_active:
            return
        now = self._now()
        if now - self.last_gimbal_t < self.GIMBAL_PERIOD:   # 限频:转到位再发下一条
            return
        ex = (cx - img_w / 2.0) / (img_w / 2.0)   # 归一化水平误差 [-1,1]
        if abs(ex) <= self.GIMBAL_DZ:
            return
        step = _clamp(self.GIMBAL_GAIN * ex, -self.GIMBAL_STEP, self.GIMBAL_STEP)
        # 目标在右(ex>0)→ yaw 增大右转(实测方向)
        self.gimbal_yaw = _clamp(self.gimbal_yaw + step, *YAW_LIMIT)
        self.last_gimbal_t = now
        self._gimbal_publish()

    def _gimbal_publish(self):
        self.arm.set_steer([self.gimbal_yaw, GIMBAL_FIXED[0], GIMBAL_FIXED[1],
                            self.gimbal_pitch, GIMBAL_FIXED[2], GIMBAL_FIXED[3]], self.GIMBAL_DUR_MS)

    def _gimbal_home(self):
        self.gimbal_yaw = YAW_HOME
        self.gimbal_pitch = PITCH_HOME
        self.arm.set_steer([YAW_HOME, GIMBAL_FIXED[0], GIMBAL_FIXED[1],
                            PITCH_HOME, GIMBAL_FIXED[2], GIMBAL_FIXED[3]], 500)

    # ===================== 输出 =====================
    def _publish_zero_vel(self):
        self.pub_vel.publish(Twist())

    def _draw_and_publish(self, bgr, box, dist, status):
        try:
            if box is not None:
                x, y, w, h = [int(v) for v in box]
                color = (0, 255, 0) if self.target_present else (0, 165, 255)
                cv2.rectangle(bgr, (x, y), (x + w, y + h), color, 2)
                cv2.circle(bgr, (int(x + w / 2), int(y + h / 2)), 4, (0, 255, 255), -1)
            txt = 'mode=%s status=%s' % (self.mode, status)
            if dist is not None:
                txt += ' d=%.2fm' % dist
            cv2.putText(bgr, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 降到 640x360 再发布:省 CPU、视频传输更流畅(检测仍用原分辨率)
            out = cv2.resize(bgr, (640, 360))
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(out, 'bgr8'))
        except Exception as e:
            self.get_logger().warn('publish result fail: %s' % e)

    def _publish_state(self):
        with self.lock:
            st = {
                'running': self.running,
                'mode': self.mode,
                'locked': self.locked,
                'target_present': self.target_present,
                'front_min': None if not math.isfinite(self.front_min) else round(self.front_min, 2),
            }
            if self.last_box is not None:
                st['box'] = [int(v) for v in self.last_box]
        msg = String(); msg.data = json.dumps(st, ensure_ascii=False)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_vel.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
