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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
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
        self.declare_parameter('follow_distance', 2.0)           # 期望跟随距离(m)
        self.declare_parameter('safe_distance', 0.8)             # 安全停车距离(m,离人太近)
        self.declare_parameter('lidar_safe_distance', 0.45)      # 雷达避障触发距离(m,与离人距离解耦)
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
        self.lidar_safe_distance = float(self.get_parameter('lidar_safe_distance').value)
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

        self.latest_color_msg = None    # 最新彩色原始消息(懒反序列化)
        self.latest_color_stamp = 0.0
        self.latest_depth = None        # uint16 mm np
        self.latest_persons = []        # [(x1,y1,x2,y2,score), ...]
        self.latest_persons_stamp = 0.0
        self.front_min = float('inf')   # 雷达正前方最近距离

        self.gimbal_yaw = YAW_HOME          # 当前实际下发的 yaw(20Hz 平滑插值后)
        self.gimbal_yaw_target = YAW_HOME   # 控制决策算出的目标 yaw(8.5Hz 检测率更新)
        self.gimbal_pitch = PITCH_HOME
        self.last_gimbal_t = 0.0
        self.prev_ex = 0.0           # 云台 yaw 上次归一化误差(算微分阻尼)
        self.gimbal_ex_rate = 0.0    # 误差变化率(EMA 平滑)
        self.gimbal_last_proc_stamp = 0.0   # 上次处理的检测时间戳(按检测门控)
        self.base_prev_gy = 0.0      # 上次底盘控制用的 gimbal_yaw(算微分阻尼)
        self.base_gy_rate = 0.0      # gimbal_yaw 变化率(EMA 平滑)
        self.base_last_t = 0.0       # 上次底盘控制时间
        self.search_dir = 0.0        # 丢目标时搜索旋转方向(基于人最后所在侧)
        self.dist_filt = None        # 平滑后的距离(抑制深度间歇导致的前进顿挫)
        self.dist_lost_t = 0.0       # 上次拿到有效深度的时间

        # 跟踪以 YOLO person 框为准,IoU + 颜色直方图做帧间关联(不用易漂移的 NanoTrack)

        # PID
        self.pid_ang = pid.PID(0.0035, 0.0, 0.0005)     # 角度对中(基于像素)
        self.pid_lin = pid.PID(0.6, 0.0, 0.05)          # 距离(基于米)
        self.pid_gyaw = pid.PID(0.0020, 0.0, 0.0)       # 云台 yaw(像素)
        self.pid_gpitch = pid.PID(0.0020, 0.0, 0.0)     # 云台 pitch(像素)

        # 云台
        self.arm = arm_ik_sdk.ArmControl(self)

        # ---------------- 通信 ----------------
        # 回调组:让图像反序列化、雷达处理、主循环在多线程执行器下并行,互不阻塞
        cb_color = MutuallyExclusiveCallbackGroup()
        cb_depth = MutuallyExclusiveCallbackGroup()
        cb_scan = MutuallyExclusiveCallbackGroup()
        cb_loop = MutuallyExclusiveCallbackGroup()
        cb_misc = ReentrantCallbackGroup()
        sensor_qos = qos_profile_sensor_data
        self.create_subscription(Image, color_topic, self._color_cb, sensor_qos, callback_group=cb_color)
        self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos, callback_group=cb_depth)
        self.create_subscription(ObjectsInfo, yolo_topic, self._yolo_cb, 1, callback_group=cb_misc)
        if self.use_lidar_safety:
            scan_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos, callback_group=cb_scan)
        self.create_subscription(String, '/patrol/command', self._cmd_cb, 1, callback_group=cb_misc)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_result = self.create_publisher(Image, '~/result_img', 1)
        self.pub_state = self.create_publisher(String, '~/state', 1)

        self.create_service(Trigger, '~/enter', self._enter_srv)
        self.create_service(Trigger, '~/exit', self._exit_srv)
        self.create_service(SetBool, '~/set_running', self._set_running_srv)
        self.create_service(Trigger, '~/relock', self._relock_srv)

        self._gimbal_home()
        self.create_timer(1.0 / 20.0, self._loop, callback_group=cb_loop)   # 20Hz 主循环
        self.create_timer(0.5, self._publish_state, callback_group=cb_misc)
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
        # 懒反序列化:仅存最新原始消息,真正转 cv2 放到 20Hz 主循环里按需做一次,
        # 避免 30Hz 全速反序列化 2.7MB 大图(其中 1/3 帧主循环根本用不到)
        with self.lock:
            self.latest_color_msg = msg
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
        # 取车头正前方 ±20° 扇区最近距离(numpy 向量化)。
        # 注意:本车雷达 0° 指向车尾(见 scan_angle_filter),车头前方对应
        # 雷达角度 ±pi,故前方扇区是 |ang| >= pi-20°,而非 |ang| <= 20°。
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        half = math.radians(20)
        idx = np.arange(n, dtype=np.float32)
        ang = msg.angle_min + idx * msg.angle_increment
        ang = (ang + math.pi) % (2 * math.pi) - math.pi      # 归一到 [-pi,pi]
        valid = (np.abs(ang) >= (math.pi - half)) & np.isfinite(ranges) & (ranges > 0.05)
        sector = ranges[valid]
        self.front_min = float(sector.min()) if sector.size else float('inf')

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
        elif cmd in ('speed_up', 'speed_down', 'speed_normal') or cmd.startswith('speed_set:'):
            self._adjust_speed(cmd)
        elif cmd.startswith('lidar_safe:'):
            self._adjust_lidar_safe(cmd)

    def _adjust_speed(self, cmd):
        """运行时调跟随线速度上限。钳在 0.1~0.45,normal=0.25,步进 0.05。"""
        with self.lock:
            if cmd == 'speed_up':
                self.max_lin = min(0.45, round(self.max_lin + 0.05, 2))
            elif cmd == 'speed_down':
                self.max_lin = max(0.1, round(self.max_lin - 0.05, 2))
            elif cmd == 'speed_normal':
                self.max_lin = 0.25
            elif cmd.startswith('speed_set:'):
                try:
                    self.max_lin = common.set_range(float(cmd.split(':', 1)[1]), 0.1, 0.45)
                except (ValueError, IndexError):
                    return
        self.get_logger().info('cmd: %s -> max_lin=%.2f' % (cmd, self.max_lin))

    def _adjust_lidar_safe(self, cmd):
        """运行时调雷达避障触发距离。钳在 0.2~1.0。"""
        try:
            val = float(cmd.split(':', 1)[1])
        except (ValueError, IndexError):
            return
        with self.lock:
            self.lidar_safe_distance = common.set_range(val, 0.2, 1.0)
        self.get_logger().info('cmd: %s -> lidar_safe=%.2f' % (cmd, self.lidar_safe_distance))

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
            color_msg = self.latest_color_msg
            persons = list(self.latest_persons)
            persons_fresh = (self._now() - self.latest_persons_stamp) < 1.0
            relock = self.relock_request
            self.relock_request = False
            mode = self.mode
        if color_msg is None:
            return
        # 持锁外反序列化:每次主循环(20Hz)产生独立新数组,无需再 .copy()
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn('color cvt fail: %s' % e)
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
            twist = Twist()
            if fresh_persons:
                # 画面里还有人 → 立刻重锁最居中者继续跟(应对快速走过/绕后重现);本帧先停,下帧接管
                self.locked = False
                self.relock_request = True
            elif mode == 'follow' and (self._now() - self.last_seen) < self.lost_timeout \
                    and self.search_dir != 0.0:
                # 完全没人:原地朝人最后所在方向旋转找回(只转不前进=安全)
                twist.angular.z = float(self.search_dir * self.SEARCH_ANG)
            self.pub_vel.publish(twist)
            status = 'searching' if (twist.angular.z != 0.0 or fresh_persons) else 'lost'
            self._draw_and_publish(bgr, self.last_box, None, status)
            return

        box = self.last_box
        bx, by, bw, bh = box
        cx = bx + bw / 2.0
        cy = by + bh / 2.0
        dist = self._distance_from_depth(box, bgr.shape)

        twist = Twist()
        if mode == 'follow':
            # === B 架构 + 坦克炮塔稳定:相机世界朝向只由云台追踪决定,底盘转动被前馈抵消 ===
            now_b = self._now()
            dt_b = now_b - self.base_last_t
            if not (0.0 < dt_b < 0.5):
                dt_b = 0.05
            self.base_last_t = now_b
            ex = (cx - img_w / 2.0) / (img_w / 2.0)   # 人水平像素误差 = 相机世界指向误差

            # 1) 云台跟踪步进:仅新检测时,响应人的世界指向误差
            #    (底盘转动已被前馈补偿,故云台不会和底盘耦合)
            track_step = 0.0
            stamp = self.latest_persons_stamp
            if stamp != self.gimbal_last_proc_stamp:
                self.gimbal_last_proc_stamp = stamp
                if abs(ex) > self.GIMBAL_DZ:
                    track_step = _clamp(self.GIMBAL_KP * ex, -self.GIMBAL_STEP, self.GIMBAL_STEP)

            # 2) 底盘 angular.z:PD 跟随,把 gimbal_yaw 缓慢卸载回正前方
            gy = self.gimbal_yaw
            raw_rate = (gy - self.base_prev_gy) / dt_b
            self.base_gy_rate = 0.6 * self.base_gy_rate + 0.4 * raw_rate
            self.base_prev_gy = gy
            if abs(gy) <= self.BASE_YAW_DZ:
                ang = 0.0
            else:
                ctrl = gy + self.BASE_YAW_KD * self.base_gy_rate
                ang = common.set_range(-self.BASE_YAW_K * ctrl, -self.max_ang, self.max_ang)

            # 3) 炮塔稳定前馈:底盘这一拍转 ang*dt,给云台叠加同量反向补偿,
            #    净 gimbal_yaw += ang*dt(ang<0 右转时 yaw 减小)→ 底盘旋转不改变相机世界朝向。
            #    硬钳位单次增量,结构性杜绝大跳。
            delta = _clamp(track_step + ang * dt_b, -self.GIMBAL_MAX_DELTA, self.GIMBAL_MAX_DELTA)
            new_yaw = _clamp(gy + delta, *YAW_LIMIT)
            if abs(new_yaw - self.gimbal_yaw) > 1e-4:
                self.gimbal_yaw = new_yaw
                # 底盘在动(前馈流~20Hz)用短时长衔接;仅追踪(~8.5Hz)用长时长更平滑
                self._gimbal_publish(60 if ang != 0.0 else self.GIMBAL_DUR_MS)

            # 记录人最后所在侧(丢失时朝此方向搜索旋转);深度平滑抑制前进顿挫
            self.search_dir = -1.0 if ex > 0 else 1.0
            if dist is not None:
                self.dist_filt = dist if self.dist_filt is None else 0.5 * self.dist_filt + 0.5 * dist
                self.dist_lost_t = now_b
            elif self.dist_filt is not None and (now_b - self.dist_lost_t) > 0.6:
                self.dist_filt = None       # 深度久缺 → 放弃旧值,转用框高代理
            d = self.dist_filt

            # 4) 接近速度 v(>0 前进靠近):距离环 或 无深度时框高占比代理
            v = 0.0
            if d is not None:
                self.pid_lin.SetPoint = self.follow_distance
                self.pid_lin.update(d)
                v = -self.pid_lin.output
                if abs(d - self.follow_distance) < 0.12:
                    v = 0.0
            else:
                ratio = bh / float(img_h)        # 越大越近
                # 无深度回退:框高占比代理距离。目标占比≈与 follow_distance 反比标定
                # (0.45 占比≈1.2m,故 2.0m≈0.27);仅深度久缺时启用,正常走深度环。
                target_ratio = 0.45 * 1.2 / self.follow_distance
                v = (target_ratio - ratio) * 1.2
            # 安全:太近 / 雷达前方障碍 → 不再靠近(允许后退)
            too_close = (d is not None and d < self.safe_distance)
            lidar_block = (self.use_lidar_safety and self.front_min < self.lidar_safe_distance)
            if too_close or lidar_block:
                v = min(v, 0.0)
            v = common.set_range(v, -self.max_lin, self.max_lin)

            # 5) 麦轮全向:把接近速度按人方位角(gimbal_yaw)分解成前后 x + 横移 y,
            #    人在侧面时直接斜向/横移过去,不必先转身 → 更快、更直接、用上麦轮特性。
            if self.machine_type == 'Mec':
                vx = v * math.cos(gy)
                vy = -v * math.sin(gy)   # 人在右(gy>0)→ 向右横移(REP103 右为 -y)
            else:  # Ack:只能前后,云台偏太多时先转底盘对准再走
                vx = 0.0 if abs(gy) > self.BASE_FWD_GATE else v
                vy = 0.0
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(ang)
            self.pub_vel.publish(twist)
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
    # 下游:ik_states→STM32→串行总线舵机,position[6]=运动时长 DUR_MS,舵机在 DUR_MS 内
    # 自己平滑插值到目标角。核心思路:让舵机硬件做插值,软件按检测率发令即可。
    #   DUR_MS ≈ 检测间隔(~120ms)是关键:舵机在每段内连续移动、下条指令到时刚好到位
    #   → 不早停(消卡顿)、不滞后累积(消过冲)。DUR<间隔→早停顿挫;DUR>间隔→滞后累积过冲。
    # 不用软件 20Hz 插值:高频发令(2.4x 包量)+短 DUR 反复重启舵机轨迹 → 总线压力+偶发大跳。
    GIMBAL_PERIOD = 0.05   # 发令最小间隔(秒)安全上限,实际由"新检测"门控(~8.5Hz)
    GIMBAL_DZ = 0.05       # 死区(归一化):仅防正中心像素抖
    GIMBAL_KP = 0.09       # 比例增益:g≈0.17 仍稳;比 0.06 更快更灵敏(平滑靠 DUR_MS 保证)
    GIMBAL_STEP = 0.12     # 单次最大步进(rad)安全上限
    GIMBAL_DUR_MS = 140    # 运动时长略大于检测间隔(~118ms):下条指令到时舵机仍在移动→平滑改向
                           # 而非先停再起,消除段边界速度脉动(卡顿)。配小 KP 滞后累积可忽略,不致过冲。

    # follow 模式 B 架构(云台主导、底盘跟随)参数:
    # 底盘 angular.z 跟随云台朝向把 gimbal_yaw 拉回正前方(慢于云台→解耦不抢、不耦合震荡)。
    # 符号:云台朝右 gimbal_yaw>0 → 底盘右转 angular.z<0,故 ang = -K*gimbal_yaw(已对 PID 符号核验)。
    BASE_YAW_K = 0.72      # gimbal_yaw→angular.z 增益:大角度时底盘转更快跟上(提速)
    BASE_YAW_KD = 0.0      # 底盘微分阻尼:炮塔稳定前馈已断开耦合环,纯比例卸载即一阶稳定,
                           # 故先设 0。若底盘机械惯性致过头再加(gimbal_yaw 无视觉延迟,微分干净)。
    GIMBAL_MAX_DELTA = 0.15  # 单次云台命令最大增量(rad):结构性杜绝"大跳",安全网
    BASE_YAW_DZ = 0.15     # 云台在正前方 ±0.15rad(~9°)内底盘不转:近中心不追残余小角度→抑过冲
    BASE_FWD_GATE = 0.5    # (保留)Ack 模式才用:云台偏太多时禁止前进。Mec 用横移不需要。
    SEARCH_ANG = 0.9       # 丢目标时原地搜索旋转角速度(rad/s),朝人最后所在方向找回
    # follow 模式云台略缩幅(底盘做转向主力),但保持足够响应(太小会滞后致底盘转过头)。
    # track_only 不缩放(scale=1.0),保持已定型手感。
    FOLLOW_GIMBAL_SCALE = 1.0

    def _gimbal_track(self, cx, cy, img_w, img_h, yaw_active, gain_scale=1.0):
        if not yaw_active:
            return
        # 门控到检测率,一检测一发令:低发令率不压总线(消大跳),DUR≈间隔让舵机连续平滑。
        stamp = self.latest_persons_stamp
        if stamp == self.gimbal_last_proc_stamp:
            return
        now = self._now()
        if now - self.last_gimbal_t < self.GIMBAL_PERIOD:   # 安全限频
            return
        self.gimbal_last_proc_stamp = stamp
        ex = (cx - img_w / 2.0) / (img_w / 2.0)   # 归一化水平误差 [-1,1]
        if abs(ex) <= self.GIMBAL_DZ:
            return
        # 纯比例欠阻尼:每步只朝中心移一小部分(< 真实角误差)→ 延迟下单调逼近不过冲。
        # 目标在右(ex>0)→ yaw 增大右转(实测方向)。DUR≈间隔→指令≈实际,无累积过冲。
        # gain_scale<1(follow 模式):云台幅度变小,底盘做转向主力 → 解耦防耦合震荡。
        step = _clamp(self.GIMBAL_KP * gain_scale * ex, -self.GIMBAL_STEP, self.GIMBAL_STEP)
        self.gimbal_yaw = _clamp(self.gimbal_yaw + step, *YAW_LIMIT)
        self.last_gimbal_t = now
        self._gimbal_publish()

    def _gimbal_publish(self, dur_ms=None):
        dur = self.GIMBAL_DUR_MS if dur_ms is None else dur_ms
        self.arm.set_steer([self.gimbal_yaw, GIMBAL_FIXED[0], GIMBAL_FIXED[1],
                            self.gimbal_pitch, GIMBAL_FIXED[2], GIMBAL_FIXED[3]], dur)

    def _gimbal_home(self):
        self.gimbal_yaw = YAW_HOME
        self.gimbal_yaw_target = YAW_HOME
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
                'max_lin': round(self.max_lin, 2),
                'front_min': None if not math.isfinite(self.front_min) else round(self.front_min, 2),
            }
            if self.last_box is not None:
                st['box'] = [int(v) for v in self.last_box]
        msg = String(); msg.data = json.dumps(st, ensure_ascii=False)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollow()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node.pub_vel.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
