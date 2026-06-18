#!/usr/bin/env python3
# encoding: utf-8
import json
import mimetypes
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import tf2_ros


CAMERA_TOPIC = '/camera/color/image_raw'
RESULT_TOPIC = '/result_img'
EVENT_TOPIC = '/car_report/event'
CMD_VEL_TOPIC = '/cmd_vel'
WEB_VIDEO_PORT = 8080
IK_TOPIC = '/ik_states'
JOINT_STATES_TOPIC = '/joint_states'
PERSON_FOLLOW_RESULT_TOPIC = '/person_follow/result_img'
GESTURE_RESULT_TOPIC = '/gesture_command/result_img'
GESTURE_STATE_TOPIC = '/gesture_command/gesture'
PATROL_CMD_TOPIC = '/patrol/command'
ROBOT_POSE_TOPIC = '/robot_pose'   # dashboard 由 TF(map->base_link)重发,供网页画机器人
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
MAPS_DIR = os.path.expanduser('~/maps')   # 地图存放目录(map_saver_cli 输出 .pgm+.yaml)


def yaw_to_quaternion(yaw):
    """平面偏航角 -> 四元数 (x,y,z,w)。"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_to_yaw(x, y, z, w):
    """四元数 -> 平面偏航角(弧度)。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# 兜底清理用的进程特征(按 launch/可执行名匹配进程组)。dashboard 自身命令行是
# `ros2 run car_web dashboard_server`,不含下列任一特征,绝不会误杀自己。
# web_video_server 不在此列(保留视频流不中断)。
RESIDUAL_PATTERNS = [
    'astra_camera_node', 'car_camera.launch', 'base_serial.launch',
    'car_web_core.launch', 'yolo_detect', 'car_report_yolo.launch',
    'camp_security_yolo.launch',
    'person_follow', 'human_follow.launch', 'gesture_command',
    'dingtalk_notify', 'gimbal_track.launch',
]
LIDAR_PATTERNS = ['car_lidar.launch', 'rplidar', 'scan_angle_filter']
VOICE_PATTERNS = ['voice_main', 'car_voice', 'voice.launch']
# 建图导航相关进程特征。nav 启动会经 car_base.launch 拉起底盘/雷达/相机/EKF,
# 故清理时连同 LIDAR_PATTERNS 一起;rosbridge/explore 为导航模式专属。
NAV_PATTERNS = [
    'car_nav2.launch', 'cartographer_node', 'cartographer_occupancy_grid',
    'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'lifecycle_manager', 'map_server',
    'amcl', 'explore', 'rosbridge_websocket', 'imu_filter_madgwick',
    'ekf_node', 'robot_state_publisher', 'car_base_node',
]
PATROL_STATE_TOPIC = '/person_follow/state'
POWER_VOLTAGE_TOPIC = '/PowerVoltage'
JOINT_NAMES = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
JOINT_NAME_TO_INDEX = {
    'joint0': 0,
    'joint1': 1,
    'joint2': 2,
    'joint3': 3,
    'joint4': 4,
    'joint5': 5,
    'arm_0_joint': 0,
    'arm_1_joint': 1,
    'arm_2_joint': 2,
    'arm_3_joint': 3,
    'arm_4_joint': 4,
    'arm_5_1_joint': 5,
}
SERVO_STEP_RAD = 0.2617993878  # 15 degrees
SERVO_LIMIT_RAD = 1.5707963268  # 90 degrees
SERVO_DURATION_MS = 500.0
CAMERA_YAW_INDEX = 0
CAMERA_PITCH_INDEX = 3
CAMERA_YAW_CENTER = 0.0
CAMERA_PITCH_CENTER = 1.2
CAMERA_PITCH_MIN = 0.8
CAMERA_PITCH_MAX = 1.6
FEATURES = [
    {
        'id': 'visual_patrol',
        'name': 'YOLO识别',
        'description': '启动 YOLO 识别、事件记录和置信框视频。',
        'available': True,
    },
    {
        'id': 'human_follow',
        'name': '人体跟随',
        'description': '锁定一人并保持安全距离尾随,云台同步追踪(car_patrol)。',
        'available': True,
    },
    {
        'id': 'gimbal_track',
        'name': '云台追踪',
        'description': '底盘不动,仅云台锁定追踪目标人(car_patrol)。',
        'available': True,
    },
]


class DashboardError(RuntimeError):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def get_web_dir():
    try:
        return Path(get_package_share_directory('car_web')) / 'web'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'web'


class DashboardNode(Node):
    def __init__(self):
        super().__init__('car_web_dashboard')

        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('drive_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)
        # 横移(麦轮 linear.y)最大速度,与前进同量级
        self.declare_parameter('max_lateral_speed', 0.2)
        # 旋转死区下限:实测 <0.35rad/s 时电机落入 PWM 死区,四轮走停不同步,
        # 角速度瞬时在 0~0.44 间剧烈抖动,麦轮耦合成前后左右平移晃动,伤建图精度。
        # 实测拐点:0.30→抖动占比17%且仍掉到0;0.35→减半到8%且不再掉0;0.40 收益饱和。
        # 故取 0.35。仅作用于 dashboard 遥控(_publish_twist),不碰 nav2 控制器,不影响导航对准精度。
        self.declare_parameter('min_angular_speed', 0.35)
        # 电量:/PowerVoltage 为串口单字节 ADC 值,线性过原点标定。
        # 实测:万用表 11.06V 对应原始值≈62 -> scale≈0.178(原始字节本身有 ±2 噪声)。
        # full/empty 为 3 串锂电满/空电压(伏特),用于估算百分比。
        self.declare_parameter('voltage_scale', 0.178)
        self.declare_parameter('voltage_full', 12.6)
        self.declare_parameter('voltage_empty', 9.9)

        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.drive_timeout = max(0.1, float(self.get_parameter('drive_timeout').value))
        self.max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self.max_lateral_speed = max(0.0, float(self.get_parameter('max_lateral_speed').value))
        self.min_angular_speed = max(0.0, float(self.get_parameter('min_angular_speed').value))
        self.voltage_scale = float(self.get_parameter('voltage_scale').value)
        self.voltage_full = float(self.get_parameter('voltage_full').value)
        self.voltage_empty = float(self.get_parameter('voltage_empty').value)
        self.battery_raw = None
        self.battery_stamp = 0.0

        self.web_dir = get_web_dir()
        self.lock = threading.RLock()
        self.processes = {}
        self.current_mode = 'idle'
        self.ui_mode = 'follow'           # 'follow'(跟随/巡检) | 'nav'(建图导航),两大模式互斥
        self.nav_state = 'idle'           # idle|mapping|exploring|localizing|navigating|cruising
        self.nav_feedback = ''            # 导航反馈(距目标/状态文字)
        self.current_map = ''             # 当前加载/在建的地图名        self.current_video_topic = CAMERA_TOPIC
        self.active_feature = ''
        self.last_error = ''
        self.events = []
        self.report_job = None
        self.last_drive_time = 0.0
        self.motion_active = False
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.target_lateral = 0.0
        self.current_joints = [0.0] * len(JOINT_NAMES)
        self.have_joint_state = False

        self.patrol_state = ''
        self.gesture_state = ''
        self.gesture_on = False           # 手势控制为独立叠加开关,不占 active_feature
        self.lidar_on = False             # 雷达独立开关(供 follow 避障)
        self.tts_on = False               # 语音播报开关
        self.asr_on = False               # 语音触发(麦克风聆听)开关
        # 播报去抖状态
        self._last_announce = {}          # key -> 单调时间戳
        self._battery_warned = False      # 电量低只播一次,回升复位
        self._prev_patrol_state = ''      # 巡逻/跟随状态沿变化检测
        self._prev_gesture_state = ''

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 5)
        self.servo_pub = self.create_publisher(JointState, IK_TOPIC, 5)
        self.patrol_cmd_pub = self.create_publisher(String, PATROL_CMD_TOPIC, 5)
        self.voice_announce_pub = self.create_publisher(String, '/voice/announce', 10)
        self.voice_control_pub = self.create_publisher(String, '/voice/control', 5)
        self.create_subscription(String, EVENT_TOPIC, self.event_callback, 20)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.joint_state_callback, 10)
        self.create_subscription(String, PATROL_STATE_TOPIC, self.patrol_state_callback, 5)
        self.create_subscription(Float32, POWER_VOLTAGE_TOPIC, self.battery_callback, 10)
        self.create_subscription(String, GESTURE_STATE_TOPIC, self.gesture_state_callback, 5)
        self.create_timer(0.1, self.drive_timer_callback)

        # TF -> /robot_pose 重发:网页画机器人需要 map 系下位姿,但建图时位姿只在 TF。
        # 仅 nav 模式下查 map->base_link,失败静默(follow 模式无 map 帧)。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_pose_pub = self.create_publisher(PoseStamped, ROBOT_POSE_TOPIC, 5)
        # amcl 初始位姿:加载地图进定位时,把建图最后的车位姿发给 amcl,
        # 否则 amcl 锚在地图原点(0,0,0),车不在原点就定位错位。
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 5)
        self.last_robot_pose = None       # (x, y, yaw) in map frame, 供回起点/打点用
        self.home_pose = None             # 进建图那刻记录的起点 (x, y, yaw)
        # 导航调速:TEB controller_server 的 FollowPath 速度上限,按 scale 整体缩放。
        # 基准取自 car_teb.yaml(max_vel_x=0.18 等)。运行时 ros2 param set 动态生效。
        self.nav_speed_base = {'x': 0.18, 'back': 0.10, 'theta': 0.4}
        self.nav_speed_scale = 1.0        # 0.4~2.0,网页加减档
        # NavigateToPose action 客户端:canvas 点击导航 / 回起点。非阻塞发目标,
        # 回调更新 nav_state/nav_feedback。goal handle 存下供取消。
        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_goal_handle = None
        # 取消 /navigate_to_pose 上的全部目标(含 explore_lite 自己发的,我们无 handle)。
        # explore 停止后 bt_navigator 仍持最后 frontier 目标在跑,必须按服务端 cancel-all 清掉。
        self._navpose_cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        # FollowWaypoints action 客户端:多点巡航。车依次到访点列表,可取消。
        self.cruise_action = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._cruise_goal_handle = None
        self._cruise_total = 0
        self.create_timer(0.2, self.publish_robot_pose)

        self.http_server = ThreadingHTTPServer(
            (self.http_host, self.http_port),
            DashboardRequestHandler
        )
        self.http_server.dashboard = self
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name='car_web_http',
            daemon=True
        )
        self.http_thread.start()

        self.get_logger().info(
            'car_web dashboard started: http://%s:%d web_dir=%s'
            % (self.http_host, self.http_port, str(self.web_dir))
        )

        # 注意:core 的自启移到 main() 的「启动自清」之后。否则 __init__ 在此起 core,
        # 紧接着 main() 的 force_cleanup_residuals 会按 car_web_core.launch 特征把刚起的
        # core 杀掉(启动顺序竞态:core 偶尔逃过、偶尔被杀,表现为 core 时有时无)。

    def start_core_safe(self):
        """启动自清之后再调:起 core 并吞掉异常,记 last_error。"""
        try:
            self.start_core()
        except DashboardError as exc:
            self.last_error = str(exc)
            self.get_logger().error(str(exc))

    def event_callback(self, msg):
        try:
            parsed = json.loads(msg.data)
        except Exception:
            parsed = {'raw': msg.data}

        with self.lock:
            self.events.append(parsed)
            self.events = self.events[-50:]

        # 识别事件播报(同类 8 秒去抖,标签作 key)
        label = ''
        if isinstance(parsed, dict):
            label = str(parsed.get('label') or parsed.get('class')
                        or parsed.get('name') or '').strip()
        if label:
            self.announce('检测到%s' % label, key='event:%s' % label, min_interval=8.0)

    def joint_state_callback(self, msg):
        positions = list(msg.position)
        if not positions:
            return

        updated = False
        with self.lock:
            if msg.name:
                for name, position in zip(msg.name, positions):
                    index = JOINT_NAME_TO_INDEX.get(name)
                    if index is not None:
                        self.current_joints[index] = float(position)
                        updated = True
            else:
                for index in range(min(len(positions), len(self.current_joints))):
                    self.current_joints[index] = float(positions[index])
                    updated = True

            if updated:
                self.have_joint_state = True

    def drive_timer_callback(self):
        with self.lock:
            if not self.motion_active:
                return

            expired = time.monotonic() - self.last_drive_time > self.drive_timeout
            if expired:
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.target_lateral = 0.0
                self.motion_active = False

            linear = self.target_linear
            angular = self.target_angular
            lateral = self.target_lateral

        self._publish_twist(linear, angular, lateral)

    def publish_robot_pose(self):
        """0.2s 一次:查 map->base_link,重发为 /robot_pose(PoseStamped),并缓存
        last_robot_pose。仅 nav 模式查(其他模式无 map 帧,查必失败)。"""
        with self.lock:
            if self.ui_mode != 'nav':
                return
        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:
            return
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        with self.lock:
            self.last_robot_pose = (tr.x, tr.y, yaw)
        msg = PoseStamped()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = MAP_FRAME
        msg.pose.position.x = tr.x
        msg.pose.position.y = tr.y
        msg.pose.orientation = q
        self.robot_pose_pub.publish(msg)

    def _publish_initialpose_async(self, pose):
        """后台线程:等 amcl 的 /initialpose 订阅连上再发(避免刚启动 discovery
        竞态丢消息),连发几次。pose=(x,y,yaw)。amcl 收到后重置粒子滤波到此位姿,
        发出正确的 map->odom 修正,车无需移动即已正确定位。"""
        x, y, yaw = pose
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = MAP_FRAME
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 中等不确定度,让 amcl 用扫描微调
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.068  # yaw (~15°)
        # 等订阅者(amcl)连上,最多 20s(amcl 起身较慢)
        deadline = time.monotonic() + 20.0
        while self.initialpose_pub.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                self.get_logger().warn('initialpose: amcl 未在20s内订阅,放弃自动定位')
                return
            time.sleep(0.3)
        # 连上后稍等并连发 3 次确保被接收
        time.sleep(0.5)
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            time.sleep(0.4)
        self.get_logger().info(
            'initialpose 已发: x=%.3f y=%.3f yaw=%.1f°' % (x, y, math.degrees(yaw)))

    def status(self):
        with self.lock:
            processes = {
                name: self._process_state(name)
                for name in ('core', 'report', 'notify', 'follow', 'gesture', 'lidar',
                             'nav', 'rosbridge', 'explore')
            }
            report_job = dict(self.report_job) if self.report_job else None
            core_running = self._is_running('core')
            report_running = self._is_running('report')
            current_video_topic = self.current_video_topic

        video_diagnostics = self.video_diagnostics(current_video_topic)

        with self.lock:
            return {
                'ok': True,
                'mode': self.current_mode,
                'ui_mode': self.ui_mode,
                'nav_state': self.nav_state,
                'nav_feedback': self.nav_feedback,
                'current_map': self.current_map,
                'nav_speed_scale': self.nav_speed_scale,
                'active_feature': self.active_feature,
                'video_topic': current_video_topic,
                'video_url': self.video_url(current_video_topic),
                'stream_hint': self.video_hint(current_video_topic, video_diagnostics),
                'web_video_port': WEB_VIDEO_PORT,
                'video_diagnostics': video_diagnostics,
                'processes': processes,
                'events_count': len(self.events),
                'last_event': self.events[-1] if self.events else None,
                'report_job': report_job,
                'last_error': self.last_error,
                'can_drive': core_running,
                'can_report': report_running,
                'patrol_state': self.patrol_state,
                'gesture_state': self.gesture_state,
                'gesture_on': self.gesture_on,
                'lidar_on': self.lidar_on,
                'tts_on': self.tts_on,
                'asr_on': self.asr_on,
                'battery': self.battery_info(),
            }

    def get_events(self):
        with self.lock:
            return {'ok': True, 'events': list(reversed(self.events))}

    def get_features(self):
        with self.lock:
            active_feature = self.active_feature
        return {'ok': True, 'features': FEATURES, 'active_feature': active_feature}

    def start_feature(self, payload):
        feature_id = str(payload.get('id', '')).strip()
        feature = next((item for item in FEATURES if item['id'] == feature_id), None)
        if feature is None:
            raise DashboardError('未知功能：%s' % feature_id, 404)
        if not feature.get('available'):
            raise DashboardError('%s 还只是预留入口，后续接入对应功能包。' % feature['name'], 501)
        if feature_id == 'visual_patrol':
            return self.start_report_mode()
        if feature_id == 'human_follow':
            return self.start_patrol('human_follow', 'follow')
        if feature_id == 'gimbal_track':
            return self.start_patrol('gimbal_track', 'track_only')
        raise DashboardError('%s 未配置启动逻辑。' % feature['name'], 501)

    def stop_feature(self):
        with self.lock:
            active_feature = self.active_feature
        if active_feature == 'visual_patrol':
            return self.stop_report(update_mode=True)
        if active_feature in ('human_follow', 'gimbal_track'):
            return self.stop_patrol()
        if not active_feature:
            return self.status()
        with self.lock:
            self.active_feature = ''
        return self.status()

    def start_patrol(self, feature_id, mode):
        with self.lock:
            active_feature = self.active_feature
        if active_feature and active_feature not in ('human_follow', 'gimbal_track'):
            self.stop_feature()
        self.start_core()
        # 人体跟随底盘会动,需雷达避障;云台追踪底盘不动,无需雷达
        if feature_id == 'human_follow' and not self._is_running('lidar'):
            self.start_lidar()
        self.start_process('follow', self.follow_command(mode))
        with self.lock:
            self.current_mode = feature_id
            self.current_video_topic = PERSON_FOLLOW_RESULT_TOPIC
            self.active_feature = feature_id
        return self.status()

    def stop_patrol(self):
        self.publish_patrol_command('stop')
        self.stop_process('follow')
        self.publish_stop()
        # 跟随专用的避障雷达随跟随一起停(雷达=跟随起、跟随停,避免空转耗电)
        self.stop_lidar()
        with self.lock:
            self.current_video_topic = CAMERA_TOPIC
            self.current_mode = 'manual' if self._is_running('core') else 'idle'
            self.active_feature = ''
            self.patrol_state = ''
        return self.status()

    def start_gesture(self):
        """手势控制 = 纯输入设备,独立叠加开关,不占 active_feature。
        只起 core(手势节点需要相机)+ gesture 节点,不带任何跟随。
        手势命令发到 /patrol/command;若想让命令生效,另外开"人体跟随/云台追踪"
        起 person_follow 作执行方即可。手势与追踪彻底解耦。"""
        self.start_core()
        self.start_process('gesture', self.gesture_command())
        with self.lock:
            self.gesture_on = True
            self.current_video_topic = GESTURE_RESULT_TOPIC
        return self.status()

    def stop_gesture(self):
        """只停手势节点,不动 follow(跟随是独立功能,可能仍需运行)。"""
        self.stop_process('gesture')
        with self.lock:
            self.gesture_on = False
            # 视频切回:若跟随在跑则看跟随画面,否则看相机
            if self._is_running('follow'):
                self.current_video_topic = PERSON_FOLLOW_RESULT_TOPIC
            else:
                self.current_video_topic = CAMERA_TOPIC
        return self.status()

    def patrol_command(self, payload):
        cmd = str(payload.get('cmd', '')).strip()
        if not cmd:
            raise DashboardError('缺少 cmd 参数。', 400)
        self.publish_patrol_command(cmd)
        return {'ok': True, 'cmd': cmd}

    def publish_patrol_command(self, cmd):
        msg = String()
        msg.data = str(cmd)
        self.patrol_cmd_pub.publish(msg)

    def patrol_speed(self, payload):
        """跟随调速:payload {action:'up'|'down'|'normal'} 或 {value:0.x}。
        转成 person_follow 的 speed_* / speed_set: 命令。"""
        payload = payload or {}
        value = payload.get('value')
        if value is not None:
            self.publish_patrol_command('speed_set:%s' % value)
            return {'ok': True, 'cmd': 'speed_set:%s' % value}
        action = str(payload.get('action', '')).strip().lower()
        cmd = {'up': 'speed_up', 'down': 'speed_down', 'normal': 'speed_normal'}.get(action)
        if not cmd:
            raise DashboardError('speed 参数无效(需 action=up/down/normal 或 value)。', 400)
        self.publish_patrol_command(cmd)
        return {'ok': True, 'cmd': cmd}

    def start_lidar(self):
        """冷启动雷达(rplidar + scan_angle_filter)。供人体跟随避障用。"""
        self.start_process('lidar', self.lidar_command())
        with self.lock:
            self.lidar_on = True
        return self.status()

    def stop_lidar(self):
        self.stop_process('lidar')
        # 兜底清掉可能脱离句柄的雷达残留
        self.force_cleanup_residuals_lidar_only()
        with self.lock:
            self.lidar_on = False
        return self.status()

    def force_cleanup_residuals_lidar_only(self):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pat in LIDAR_PATTERNS:
                cpat = '[' + pat[0] + ']' + pat[1:]
                try:
                    out = subprocess.run(['pgrep', '-f', cpat],
                                         capture_output=True, text=True, timeout=3)
                except Exception:
                    continue
                for pid_s in out.stdout.split():
                    try:
                        os.killpg(os.getpgid(int(pid_s)), sig)
                    except (ProcessLookupError, ValueError, PermissionError):
                        pass
            if sig == signal.SIGTERM:
                time.sleep(1.5)

    def patrol_state_callback(self, msg):
        with self.lock:
            self.patrol_state = msg.data
        self._announce_patrol_transition(msg.data)

    def gesture_state_callback(self, msg):
        with self.lock:
            self.gesture_state = msg.data
        self._announce_gesture_transition(msg.data)

    # ===================== 语音播报 / 语音触发 =====================
    def announce(self, text, key=None, min_interval=0.0):
        """发布一句播报到 /voice/announce(voice 节点决定是否真的出声)。
        key+min_interval 用于去抖:同 key 在间隔内只播一次。"""
        text = (text or '').strip()
        if not text:
            return
        if key is not None and min_interval > 0:
            now = time.monotonic()
            last = self._last_announce.get(key, 0.0)
            if now - last < min_interval:
                return
            self._last_announce[key] = now
        msg = String()
        msg.data = text
        self.voice_announce_pub.publish(msg)

    def _publish_voice_control(self, cmd):
        msg = String()
        msg.data = cmd
        self.voice_control_pub.publish(msg)

    def start_voice(self):
        """拉起 car_voice 语音助手进程(若未在跑),默认播报+触发都开。"""
        if not self._is_running('voice'):
            self.start_process('voice', self.voice_command())
        with self.lock:
            self.tts_on = True
            self.asr_on = True
        return self.status()

    def stop_voice(self):
        self.stop_process('voice')
        self.force_cleanup_residuals_voice_only()
        with self.lock:
            self.tts_on = False
            self.asr_on = False
        return self.status()

    def force_cleanup_residuals_voice_only(self):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pat in VOICE_PATTERNS:
                cpat = '[' + pat[0] + ']' + pat[1:]
                try:
                    out = subprocess.run(['pgrep', '-f', cpat],
                                         capture_output=True, text=True, timeout=3)
                except Exception:
                    continue
                for pid_s in out.stdout.split():
                    try:
                        os.killpg(os.getpgid(int(pid_s)), sig)
                    except (ProcessLookupError, ValueError, PermissionError):
                        pass
            if sig == signal.SIGTERM:
                time.sleep(1.5)

    def set_voice_toggle(self, which, on):
        """which: 'tts'(播报) 或 'asr'(触发)。进程未跑时先拉起。
        先更新权威状态再拉起进程,使新进程开机自查 /api/status 时读到正确开关。"""
        with self.lock:
            if which == 'tts':
                self.tts_on = on
            elif which == 'asr':
                self.asr_on = on
            stop_proc = (not self.tts_on and not self.asr_on)
        running = self._is_running('voice')
        if on and not running:
            self.start_process('voice', self.voice_command())
        elif running:
            # 进程已在跑:发实时开关信号(订阅已就绪,无竞态)
            self._publish_voice_control('%s_%s' % (which, 'on' if on else 'off'))
        if stop_proc:
            self.stop_process('voice')
        return self.status()

    def _announce_patrol_transition(self, state):
        """巡逻/跟随状态沿变化时播报。
        去抖用**稳定相位**(running+mode+locked+target_present)比较,而非整条 JSON
        ——JSON 含 front_min/box 每 0.5s 都变,直接比串会每 2s 重播一次。"""
        try:
            s = json.loads(state) if state else {}
        except (ValueError, TypeError):
            s = {}
        if not s:
            return
        running = bool(s.get('running'))
        mode = s.get('mode')
        locked = bool(s.get('locked'))
        present = bool(s.get('target_present'))
        phase = '%s|%s|%s|%s' % (running, mode, locked, present)
        prev = self._prev_patrol_state
        self._prev_patrol_state = phase
        if phase == prev:
            return
        # 按稳定字段决定播报,running 优先(停止时 mode 仍可能是 follow)
        if not running:
            text = '已停止'
        elif mode == 'track_only':
            text = '切换到云台追踪'
        elif locked and present:
            text = '已锁定目标，开始跟随你' if mode == 'follow' else '已锁定目标'
        else:
            text = '目标丢失，正在重新锁定'
        self.announce(text, key='patrol', min_interval=2.0)

    def _announce_gesture_transition(self, state):
        prev = self._prev_gesture_state
        self._prev_gesture_state = state or ''
        if not state or state == prev:
            return
        low = str(state).lower()
        if 'wake' in low or '唤醒' in state:
            self.announce('手势已唤醒', key='gesture', min_interval=1.5)
        elif 'follow' in low or '跟随' in state:
            self.announce('收到手势指令，开始跟随', key='gesture', min_interval=1.5)

    def battery_callback(self, msg):
        with self.lock:
            self.battery_raw = float(msg.data)
            self.battery_stamp = time.monotonic()
            voltage = self.battery_raw * self.voltage_scale
            full = self.voltage_full
            empty = self.voltage_empty
        # 电量低告警:跌破阈值播一次,回升 1V 以上复位
        if full > empty:
            pct = (voltage - empty) / (full - empty) * 100.0
            if pct <= 15 and not self._battery_warned:
                self._battery_warned = True
                self.announce('电量不足，请及时充电')
            elif pct >= 25:
                self._battery_warned = False

    def battery_info(self):
        with self.lock:
            raw = self.battery_raw
            stamp = self.battery_stamp
            scale = self.voltage_scale
            full = self.voltage_full
            empty = self.voltage_empty
        if raw is None:
            return {'available': False}
        fresh = (time.monotonic() - stamp) < 15.0
        voltage = raw * scale
        if full > empty:
            percent = (voltage - empty) / (full - empty) * 100.0
        else:
            percent = 0.0
        percent = max(0, min(100, int(round(percent))))
        if percent <= 15:
            level = 'critical'
        elif percent <= 35:
            level = 'low'
        else:
            level = 'ok'
        return {
            'available': True,
            'fresh': fresh,
            'voltage': round(voltage, 1),
            'percent': percent,
            'level': level,
            'raw': round(raw, 1),
        }

    def start_manual_mode(self):
        self.start_core()
        self.stop_report(update_mode=False)
        self.publish_stop()
        with self.lock:
            self.current_mode = 'manual'
            self.current_video_topic = CAMERA_TOPIC
            self.active_feature = ''
        return self.status()

    def start_report_mode(self):
        with self.lock:
            active_feature = self.active_feature
        if active_feature and active_feature != 'visual_patrol':
            self.stop_feature()

        self.start_core()
        self.start_process('report', self.report_command())
        with self.lock:
            self.current_mode = 'report'
            self.current_video_topic = RESULT_TOPIC
            self.active_feature = 'visual_patrol'
        self.announce('巡逻已开启')
        return self.status()

    def start_core(self):
        self.start_process('core', self.core_command())
        with self.lock:
            if self.current_mode == 'idle':
                self.current_mode = 'manual'
                self.current_video_topic = CAMERA_TOPIC
        return self.status()

    def stop_report(self, update_mode=True):
        self.stop_process('notify')
        self.stop_process('report')
        self.publish_stop()
        if update_mode:
            with self.lock:
                self.current_video_topic = CAMERA_TOPIC
                self.current_mode = 'manual' if self._is_running('core') else 'idle'
                if self.active_feature == 'visual_patrol':
                    self.active_feature = ''
        else:
            with self.lock:
                if self.active_feature == 'visual_patrol':
                    self.active_feature = ''
        return self.status()

    def stop_core(self):
        self.stop_process('notify')
        self.stop_process('report')
        self.stop_process('gesture')
        self.stop_process('follow')
        self.publish_stop(repeat=5)
        self.stop_process('core')
        with self.lock:
            self.current_mode = 'idle'
            self.current_video_topic = CAMERA_TOPIC
            self.active_feature = ''
            self.gesture_on = False
        return self.status()

    def stop_all(self):
        self.announce('已全部停止')
        result = self.stop_core()
        # 兜底:清掉任何句柄外的残留(孤儿/上次实例起的),含雷达与导航栈
        self.stop_process('lidar')
        self.stop_process('explore')
        self.stop_process('nav')
        self.stop_process('rosbridge')
        self.force_cleanup_residuals(include_lidar=True, include_nav=True)
        with self.lock:
            self.lidar_on = False
            self.nav_state = 'idle'
            self.nav_feedback = ''
        return result

    def set_ui_mode(self, mode):
        """切换两大顶层模式。互斥:nav 与 follow 抢 /cmd_vel,切换前先停掉对方所有
        /cmd_vel owner。本阶段只做编排骨架;nav 子进程在后续阶段接入。"""
        mode = 'nav' if str(mode) == 'nav' else 'follow'
        with self.lock:
            cur = self.ui_mode
        if mode == cur:
            return self.status()
        if mode == 'nav':
            # 进 nav:停跟随系(follow/gesture)+core(其 car_base 会与 nav 的 car_base 抢串口),
            # 清零 /cmd_vel,起 rosbridge。nav 模式看地图画布,不需 web_video。
            self.publish_patrol_command('stop')
            self.stop_process('follow')
            self.stop_process('gesture')
            self.stop_process('report')
            self.stop_process('notify')
            self.stop_process('lidar')
            self.publish_stop(repeat=5)
            self.stop_process('core')
            # core 的 base_serial 与 nav 的 car_base 都开底盘串口,必须确保 core 完全退出
            self.force_cleanup_residuals(include_lidar=True)
            with self.lock:
                self.gesture_on = False
                self.lidar_on = False
                self.active_feature = ''
                self.ui_mode = 'nav'
                self.current_mode = 'nav'
            self.start_process('rosbridge', self.rosbridge_command())
        else:
            # 回 follow:停整个导航栈,清零 /cmd_vel,重启 core 恢复视频/遥控
            self.stop_nav()
            with self.lock:
                self.ui_mode = 'follow'
                self.current_mode = 'idle'   # 复位,让 start_core 提升为 manual
            self.start_core()
        return self.status()

    def stop_nav(self):
        """停建图/导航全栈(nav/explore/rosbridge)并清残留,/cmd_vel 清零。"""
        self.stop_process('explore')
        self.stop_process('nav')
        self.stop_process('rosbridge')
        self.publish_stop(repeat=5)
        self.force_cleanup_residuals(include_nav=True)
        with self.lock:
            self.nav_state = 'idle'
            self.nav_feedback = ''
        return self.status()

    def _require_nav_mode(self):
        with self.lock:
            if self.ui_mode != 'nav':
                raise DashboardError('请先切到"建图导航"模式。', 409)

    def start_mapping(self, explore=False):
        """开始建图:起 cartographer(carto_slam:=true)。explore=True 再叠加
        explore_lite 自动前沿探索;否则等用户遥控开车手动建图。"""
        self._require_nav_mode()
        if self._is_running('nav'):
            raise DashboardError('导航栈已在运行,请先停止再建图。', 409)
        self.start_process('nav', self.nav_command(carto_slam=True))
        with self.lock:
            self.nav_state = 'exploring' if explore else 'mapping'
            self.current_map = ''
            # cartographer 起图时把 map 原点锚在车当前位姿,故起点=(0,0,0)
            self.home_pose = (0.0, 0.0, 0.0)
            self.nav_speed_scale = 1.0
        if explore:
            # 给 cartographer/nav2 起身时间(也覆盖 car_base 陀螺静止标定窗口)
            time.sleep(8.0)
            # 先原地转一圈扫全周建初始图:车面对墙时正前 180° 雷达只看到墙,
            # 不转身则身后大片开阔区始终是"未知"且雷达(后半被滤)看不到,
            # explore 会误判"无前沿"提前停。转一圈让前向雷达扫遍四周,喂出真前沿。
            self.announce('原地扫描建初始图')
            self._spin_in_place(angular_speed=0.4, revolutions=1.0)
            time.sleep(1.0)
            self.start_process('explore', self.explore_command())
        self.announce('开始自动建图' if explore else '开始手动建图')
        return self.status()

    def _cancel_all_navpose_goals(self):
        """取消 /navigate_to_pose 服务端的全部活动目标(空 goal_id + 零时间戳 = cancel-all)。
        explore_lite 被杀后,它发给 bt_navigator 的最后 frontier 目标仍在执行,
        controller_server 继续吐 /cmd_vel 把车拽回去,必须从服务端清掉,不靠本地 handle。"""
        try:
            if not self._navpose_cancel_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('navpose cancel service 未就绪,跳过 cancel-all')
                return
            req = CancelGoal.Request()  # 默认空 goal_info → 取消该 action 全部目标
            self._navpose_cancel_cli.call_async(req)
        except Exception as exc:
            self.get_logger().warn('cancel-all navpose 失败: %s' % exc)

    def stop_explore(self):
        """只停自动探索,保留建图(可转手动继续补图)。

        绝不调 force_cleanup_residuals:nav 栈经 car_base.launch 拉起的相机
        (astra_camera_node/car_camera.launch/base_serial.launch)在 RESIDUAL_PATTERNS 里,
        而它们与 cartographer/nav2 同属 nav launch 的进程组;force_cleanup 用 killpg 杀组
        会把整个 nav 栈连根带走(地图丢失)。explore 是独立进程组,_cleanup_explore_residual
        只 pgrep '[e]xplore' 并 killpg,精准清 explore 自己,不碰 nav。
        """
        self.stop_process('explore')
        self._cleanup_explore_residual()
        # explore 进程死了但 bt_navigator 仍在跑它发的最后目标:取消服务端全部目标,
        # 再清零 /cmd_vel,否则手动接管会被 nav2 控制器持续拽回最后 frontier。
        self._cancel_all_navpose_goals()
        self.publish_stop(repeat=3)
        with self.lock:
            if self.nav_state == 'exploring':
                self.nav_state = 'mapping'
        return self.status()

    def _cleanup_explore_residual(self):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                out = subprocess.run(['pgrep', '-f', '[e]xplore'],
                                     capture_output=True, text=True, timeout=3)
            except Exception:
                break
            for pid_s in out.stdout.split():
                try:
                    os.killpg(os.getpgid(int(pid_s)), sig)
                except (ProcessLookupError, ValueError, PermissionError):
                    pass
            if sig == signal.SIGTERM:
                time.sleep(1.5)

    def save_map(self, payload):
        """用 map_saver_cli 把当前 /map 存成 ~/maps/<name>.{pgm,yaml}。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('未在建图,无法保存地图。', 409)
        name = self._safe_map_name(payload.get('name', ''))
        os.makedirs(MAPS_DIR, exist_ok=True)
        stem = os.path.join(MAPS_DIR, name)
        try:
            res = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', stem, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            raise DashboardError('保存地图超时(10s 内未收到 /map)。', 504)
        if not os.path.exists(stem + '.yaml'):
            raise DashboardError('保存失败:%s' % (res.stderr or res.stdout)[-200:], 500)
        with self.lock:
            self.current_map = name
        self.announce('地图已保存:%s' % name)
        return {'ok': True, 'name': name, 'maps': self._list_map_names()}

    @staticmethod
    def _safe_map_name(name):
        name = str(name or '').strip()
        if not name:
            name = 'map'
        # 只留字母数字下划线连字符中文,挡路径穿越
        keep = []
        for ch in name:
            if ch.isalnum() or ch in '_-' or '一' <= ch <= '鿿':
                keep.append(ch)
        cleaned = ''.join(keep)[:40]
        return cleaned or 'map'

    @staticmethod
    def _list_map_names():
        try:
            files = os.listdir(MAPS_DIR)
        except OSError:
            return []
        return sorted(f[:-5] for f in files if f.endswith('.yaml'))

    def list_maps(self):
        return {'ok': True, 'maps': self._list_map_names(), 'current_map': self.current_map}

    def load_map(self, payload):
        """加载已存地图进入定位导航(slam:=False map:=<yaml>)。

        若当前正建图(cartographer 在跑),停它前先抓车在 map 中的当前位姿,
        amcl 起来后自动补发为 /initialpose,使车无缝定位(否则 amcl 锚在原点)。
        冷加载(无 last_robot_pose)时跳过,沿用 amcl 默认原点,待用户手动设位姿。"""
        self._require_nav_mode()
        name = self._safe_map_name(payload.get('name', ''))
        yaml_path = os.path.join(MAPS_DIR, name + '.yaml')
        if not os.path.exists(yaml_path):
            raise DashboardError('地图不存在:%s' % name, 404)
        # 停栈前抓当前位姿(建图连续流程下=车真实所在;加载同一张图坐标系一致)
        with self.lock:
            seed_pose = self.last_robot_pose
        self.stop_process('explore')
        self.stop_process('nav')
        self._cleanup_explore_residual()
        # 保活 rosbridge:本函数常在已处 nav 模式下被调(点地图列表),浏览器已连 :9090
        # 并订阅了 /map。若杀掉 rosbridge,浏览器断连重连后会错过 map_server 的 latched
        # /map(transient_local 只发一次)→ 画布空白。保活后,订阅持续,新 map_server
        # 起来发布 /map 时既有订阅者即可收到,画布正常刷新。
        self.force_cleanup_residuals(include_nav=True, preserve_rosbridge=True)
        time.sleep(1.0)
        self.start_process('nav', self.nav_command(carto_slam=False, map_path=yaml_path))
        with self.lock:
            self.nav_state = 'localizing'
            self.current_map = name
            self.home_pose = (0.0, 0.0, 0.0)
            self.nav_speed_scale = 1.0   # 新 controller_server 用 yaml 默认速度
        if seed_pose is not None:
            # 后台等 amcl 订阅连上再发,不阻塞 HTTP 响应
            threading.Thread(target=self._publish_initialpose_async,
                             args=(seed_pose,), daemon=True).start()
            self.announce('已加载地图:%s,正在定位' % name)
        else:
            self.announce('已加载地图:%s(请在地图上点设初始位姿)' % name)
        return self.status()

    def _send_nav_goal(self, x, y, yaw, label):
        """构造 NavigateToPose 目标并异步发送。HTTP 线程不阻塞:等 server 就绪
        (短超时)后 send_goal_async,回调里更新 nav_state/feedback、存 goal handle。"""
        if not self.nav_action.wait_for_server(timeout_sec=3.0):
            raise DashboardError('导航 action 服务未就绪(nav2 未起齐)。', 503)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        with self.lock:
            self.nav_state = 'navigating'
            self.nav_feedback = label
        fut = self.nav_action.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        fut.add_done_callback(self._nav_goal_response_cb)
        self.announce(label)

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            with self.lock:
                self.nav_feedback = '导航目标被拒绝'
                self.nav_state = 'localizing' if self.current_map else 'mapping'
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, msg):
        try:
            dist = msg.feedback.distance_remaining
        except AttributeError:
            return
        with self.lock:
            self.nav_feedback = '距目标 %.2f m' % dist

    def _nav_result_cb(self, future):
        status = future.result().status
        self._nav_goal_handle = None
        say = None
        with self.lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.nav_feedback = '已到达目标'
                say = '已到达'
            elif status == GoalStatus.STATUS_CANCELED:
                self.nav_feedback = '导航已取消'
            else:
                self.nav_feedback = '导航未完成(状态 %d)' % status
                say = '没能到达目标'
            self.nav_state = 'localizing' if self.current_map else 'mapping'
        if say:
            self.announce(say)

    def nav_speed(self, payload):
        """导航调速:payload {action:'up'|'down'|'normal'} 或 {scale:0.x}。
        按 scale 整体缩放 TEB 的 max_vel_x/back/theta,ros2 param set 到 controller_server 即时生效。"""
        self._require_nav_mode()
        payload = payload or {}
        with self.lock:
            scale = self.nav_speed_scale
        if payload.get('scale') is not None:
            try:
                scale = float(payload['scale'])
            except (TypeError, ValueError):
                raise DashboardError('scale 无效。', 400)
        else:
            action = str(payload.get('action', '')).strip().lower()
            if action == 'up':
                scale += 0.2
            elif action == 'down':
                scale -= 0.2
            elif action == 'normal':
                scale = 1.0
            else:
                raise DashboardError('调速参数无效(需 action=up/down/normal 或 scale)。', 400)
        scale = max(0.4, min(2.0, round(scale, 2)))
        b = self.nav_speed_base
        params = {
            'max_vel_x': round(b['x'] * scale, 3),
            'max_vel_x_backwards': round(b['back'] * scale, 3),
            'max_vel_theta': round(b['theta'] * scale, 3),
        }
        # 先更新状态并立即返回,显示瞬间刷新;3 次 ros2 param set(每次 CLI 启动 ~0.4s,
        # 同步跑会让按钮卡 ~1s)丢后台线程,车速随后跟上(肉眼无感)。
        with self.lock:
            self.nav_speed_scale = scale
            self.nav_feedback = '导航限速 ×%.1f (%.2f m/s)' % (scale, params['max_vel_x'])
        threading.Thread(target=self._apply_nav_speed, args=(params,), daemon=True).start()
        self.announce('导航限速 %.1f 倍' % scale)
        return self.status()

    def _apply_nav_speed(self, params):
        """后台把 TEB 速度上限 param set 到 controller_server(FollowPath 插件)。"""
        for key, val in params.items():
            try:
                subprocess.run(
                    ['ros2', 'param', 'set', '/controller_server',
                     'FollowPath.%s' % key, str(val)],
                    timeout=5, capture_output=True)
            except Exception as exc:
                self.get_logger().warn('nav_speed param set %s 失败: %s' % (key, exc))

    def set_initial_pose(self, payload):
        """手动设 amcl 初始位姿(网页点图拖朝向)。{x,y,yaw}。需在定位中。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('导航栈未运行,无法设初始位姿。', 409)
        try:
            x = float(payload.get('x'))
            y = float(payload.get('y'))
            yaw = float(payload.get('yaw', 0.0) or 0.0)
        except (TypeError, ValueError):
            raise DashboardError('位姿参数无效。', 400)
        # amcl 已订阅(定位中),后台线程发(等订阅+连发3次),不阻塞 HTTP
        threading.Thread(target=self._publish_initialpose_async,
                         args=((x, y, yaw),), daemon=True).start()
        with self.lock:
            self.nav_feedback = '已设初始位姿 (%.2f, %.2f)' % (x, y)
        self.announce('已设初始位姿')
        return self.status()

    def goto(self, payload):
        """canvas 点击单点导航。{x,y,yaw?} in map frame。需 nav 栈在跑。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('导航栈未运行,请先建图或加载地图。', 409)
        try:
            x = float(payload.get('x'))
            y = float(payload.get('y'))
        except (TypeError, ValueError):
            raise DashboardError('目标坐标无效。', 400)
        yaw = float(payload.get('yaw', 0.0) or 0.0)
        self._send_nav_goal(x, y, yaw, '导航至 (%.2f, %.2f)' % (x, y))
        return self.status()

    # ---- 命名点(预设打点):存车当前位姿,关联当前地图,供网页/语音按名导航 ----
    def _points_path(self, map_name):
        """命名点存 ~/maps/<地图名>.points.json,每张图独立一组。"""
        return os.path.join(MAPS_DIR, self._safe_map_name(map_name) + '.points.json')

    def _load_points(self, map_name):
        if not map_name:
            return {}
        try:
            with open(self._points_path(map_name), 'r', encoding='utf-8') as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_points(self, map_name, points):
        os.makedirs(MAPS_DIR, exist_ok=True)
        with open(self._points_path(map_name), 'w', encoding='utf-8') as f:
            json.dump(points, f, ensure_ascii=False, indent=2)

    def list_points(self, payload=None):
        """列当前地图的命名点。"""
        with self.lock:
            map_name = self.current_map
        pts = self._load_points(map_name)
        items = [{'name': k, 'x': v.get('x'), 'y': v.get('y'), 'yaw': v.get('yaw', 0.0)}
                 for k, v in sorted(pts.items())]
        return {'ok': True, 'map': map_name, 'points': items}

    def save_point(self, payload):
        """把车当前位姿存为命名点。{name}。需 nav 模式且已知当前位姿。"""
        self._require_nav_mode()
        name = self._safe_map_name((payload or {}).get('name'))
        if not name or name == 'map':
            raise DashboardError('点名无效,请输入名字。', 400)
        with self.lock:
            map_name = self.current_map
            pose = self.last_robot_pose
        if not map_name:
            raise DashboardError('未加载地图,无法存点。', 409)
        if pose is None:
            raise DashboardError('当前位姿未知(定位未就绪),稍候再存。', 409)
        pts = self._load_points(map_name)
        pts[name] = {'x': round(pose[0], 4), 'y': round(pose[1], 4), 'yaw': round(pose[2], 4)}
        self._save_points(map_name, pts)
        self.announce('已存点:%s' % name)
        return self.list_points()

    def delete_point(self, payload):
        """删命名点。{name}。"""
        self._require_nav_mode()
        name = str((payload or {}).get('name', '')).strip()
        with self.lock:
            map_name = self.current_map
        pts = self._load_points(map_name)
        if name in pts:
            del pts[name]
            self._save_points(map_name, pts)
        return self.list_points()

    def goto_point(self, payload):
        """按名导航到命名点。{name}。供网页按钮 + 后续语音"去xxx点"复用。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('导航栈未运行,请先建图或加载地图。', 409)
        name = str((payload or {}).get('name', '')).strip()
        with self.lock:
            map_name = self.current_map
        pts = self._load_points(map_name)
        p = pts.get(name)
        if not p:
            raise DashboardError('没有名为「%s」的点。' % name, 404)
        self._send_nav_goal(float(p['x']), float(p['y']), float(p.get('yaw', 0.0)),
                            '前往 %s' % name)
        return self.status()

    def nav_home(self, payload=None):
        """回到起点。home_pose 为建图/加载时记录的原点 (0,0,0)。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('导航栈未运行,无法回起点。', 409)
        with self.lock:
            home = self.home_pose
        if home is None:
            raise DashboardError('未记录起点(尚未建图/加载地图)。', 409)
        self._send_nav_goal(home[0], home[1], home[2], '正在回到起点')
        return self.status()

    def cancel_nav(self, payload=None):
        """取消当前导航目标。"""
        self._require_nav_mode()
        handle = self._nav_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        cruise = self._cruise_goal_handle
        if cruise is not None:
            cruise.cancel_goal_async()
        with self.lock:
            self.nav_feedback = '已取消导航'
            if self.nav_state in ('navigating', 'cruising'):
                self.nav_state = 'localizing' if self.current_map else 'mapping'
        self.announce('已取消导航')
        return self.status()

    def cruise(self, payload):
        """多点巡航:payload {points:[{x,y,yaw?},...]}。车依次到访,经 FollowWaypoints。"""
        self._require_nav_mode()
        if not self._is_running('nav'):
            raise DashboardError('导航栈未运行,请先建图或加载地图。', 409)
        pts = (payload or {}).get('points') or []
        if len(pts) < 1:
            raise DashboardError('巡航点为空,请先在地图上点几个点。', 400)
        poses = []
        for i, p in enumerate(pts):
            try:
                x = float(p.get('x'))
                y = float(p.get('y'))
            except (TypeError, ValueError):
                raise DashboardError('第 %d 个巡航点坐标无效。' % (i + 1), 400)
            yaw = float(p.get('yaw', 0.0) or 0.0)
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = x
            ps.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)
        if not self.cruise_action.wait_for_server(timeout_sec=3.0):
            raise DashboardError('巡航 action 服务未就绪(nav2 未起齐)。', 503)
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        with self.lock:
            self.nav_state = 'cruising'
            self.nav_feedback = '巡航 %d 点:前往第 1 点' % len(poses)
            self._cruise_total = len(poses)
        fut = self.cruise_action.send_goal_async(
            goal, feedback_callback=self._cruise_feedback_cb)
        fut.add_done_callback(self._cruise_goal_response_cb)
        self.announce('开始巡航 %d 个点' % len(poses))
        return self.status()

    def _cruise_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            with self.lock:
                self.nav_feedback = '巡航目标被拒绝'
                self.nav_state = 'localizing' if self.current_map else 'mapping'
            return
        self._cruise_goal_handle = handle
        handle.get_result_async().add_done_callback(self._cruise_result_cb)

    def _cruise_feedback_cb(self, msg):
        try:
            idx = msg.feedback.current_waypoint
        except AttributeError:
            return
        with self.lock:
            total = getattr(self, '_cruise_total', 0)
            self.nav_feedback = '巡航中:前往第 %d/%d 点' % (idx + 1, total)

    def _cruise_result_cb(self, future):
        result = future.result()
        status = result.status
        self._cruise_goal_handle = None
        say = None
        with self.lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                missed = list(getattr(result.result, 'missed_waypoints', []) or [])
                self.nav_feedback = ('巡航完成(漏 %d 点)' % len(missed)) if missed else '巡航完成'
                say = '巡航完成'
            elif status == GoalStatus.STATUS_CANCELED:
                self.nav_feedback = '巡航已取消'
            else:
                self.nav_feedback = '巡航未完成(状态 %d)' % status
                say = '巡航没能完成'
            self.nav_state = 'localizing' if self.current_map else 'mapping'
        if say:
            self.announce(say)

    def start_notify(self):
        with self.lock:
            report_running = self._is_running('report')
            mode = self.current_mode
        if mode != 'report' or not report_running:
            raise DashboardError('请先启动识别记录模式，再启动钉钉通知。', 409)

        self.start_process('notify', self.notify_command())
        return self.status()

    def stop_notify(self):
        self.stop_process('notify')
        return self.status()

    def drive(self, payload):
        with self.lock:
            # follow 模式靠 core 的 car_base 收 /cmd_vel;nav 模式靠 nav 的 car_base。
            # 两者都接 /cmd_vel,故只要其一在跑即可遥控(手动建图需要 nav 模式遥控)。
            can_drive = self._is_running('core') or (
                self.ui_mode == 'nav' and self._is_running('nav'))
        if not can_drive:
            raise DashboardError('底盘未运行，不能遥控。请先启动底盘+相机,或在导航模式开始建图/加载地图。', 409)

        linear = float(payload.get('linear', 0.0))
        angular = float(payload.get('angular', 0.0))
        lateral = float(payload.get('lateral', 0.0))
        linear = self._clamp(linear, -self.max_linear_speed, self.max_linear_speed)
        angular = self._clamp(angular, -self.max_angular_speed, self.max_angular_speed)
        lateral = self._clamp(lateral, -self.max_lateral_speed, self.max_lateral_speed)

        with self.lock:
            self.target_linear = linear
            self.target_angular = angular
            self.target_lateral = lateral
            self.last_drive_time = time.monotonic()
            self.motion_active = (abs(linear) > 1e-6 or abs(angular) > 1e-6
                                  or abs(lateral) > 1e-6)

        if not self.motion_active:
            self.publish_stop()

        return {
            'ok': True,
            'linear': linear,
            'angular': angular,
            'lateral': lateral,
        }

    def estop(self):
        self.publish_stop(repeat=8)
        return {'ok': True, 'message': '急停已发送'}

    def chassis_reset(self):
        self.publish_stop(repeat=8)
        return {'ok': True, 'message': '底盘已复位，已连续发送零速度'}

    def servo_reset(self):
        self.publish_camera_joints({
            CAMERA_YAW_INDEX: CAMERA_YAW_CENTER,
            CAMERA_PITCH_INDEX: CAMERA_PITCH_CENTER,
        })
        return {'ok': True, 'message': '云台已回中'}

    def camera_left(self):
        with self.lock:
            target = self.current_joints[CAMERA_YAW_INDEX] - SERVO_STEP_RAD
        self.publish_camera_joint(CAMERA_YAW_INDEX, target, -SERVO_LIMIT_RAD, SERVO_LIMIT_RAD)
        return {'ok': True, 'message': '摄像头左转'}

    def camera_right(self):
        with self.lock:
            target = self.current_joints[CAMERA_YAW_INDEX] + SERVO_STEP_RAD
        self.publish_camera_joint(CAMERA_YAW_INDEX, target, -SERVO_LIMIT_RAD, SERVO_LIMIT_RAD)
        return {'ok': True, 'message': '摄像头右转'}

    def camera_up(self):
        with self.lock:
            target = self.current_joints[CAMERA_PITCH_INDEX] + SERVO_STEP_RAD
        self.publish_camera_joint(CAMERA_PITCH_INDEX, target, CAMERA_PITCH_MIN, CAMERA_PITCH_MAX)
        return {'ok': True, 'message': '摄像头上看'}

    def camera_down(self):
        with self.lock:
            target = self.current_joints[CAMERA_PITCH_INDEX] - SERVO_STEP_RAD
        self.publish_camera_joint(CAMERA_PITCH_INDEX, target, CAMERA_PITCH_MIN, CAMERA_PITCH_MAX)
        return {'ok': True, 'message': '摄像头下看'}

    def publish_camera_joint(self, joint_index, angle_rad, lower, upper):
        self.publish_camera_joints({
            int(joint_index): self._clamp(float(angle_rad), lower, upper),
        })

    def publish_camera_joints(self, updates):
        if not self._is_running('core'):
            raise DashboardError('基础节点未运行，不能控制云台。请先启动底盘+相机或YOLO识别。', 409)

        with self.lock:
            if not self.have_joint_state:
                raise DashboardError('尚未收到 /joint_states，稍等底盘节点发布关节状态后再控制云台。', 409)

            joints = list(self.current_joints)
            if len(joints) < len(JOINT_NAMES):
                joints.extend([0.0] * (len(JOINT_NAMES) - len(joints)))
            for joint_index, angle in updates.items():
                if 0 <= int(joint_index) < len(JOINT_NAMES):
                    joints[int(joint_index)] = float(angle)
            self.current_joints = joints[:len(JOINT_NAMES)]
            positions = list(self.current_joints)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions + [SERVO_DURATION_MS]
        self.servo_pub.publish(msg)

    def publish_stop(self, repeat=1):
        with self.lock:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.target_lateral = 0.0
            self.last_drive_time = time.monotonic()
            self.motion_active = False
        for _ in range(max(1, int(repeat))):
            self._publish_twist(0.0, 0.0, 0.0)

    def _publish_twist(self, linear, angular, lateral=0.0):
        angular = float(angular)
        # 旋转死区补偿:非零但低于死区下限的角速度抬到下限(保号),跨过电机静摩擦。
        # 极小值(<0.02)视为停止意图,不抬,避免松手余量被放大成持续旋转。
        if 0.02 < abs(angular) < self.min_angular_speed:
            angular = self.min_angular_speed if angular > 0 else -self.min_angular_speed
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = float(lateral)
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def _spin_in_place(self, angular_speed=0.4, revolutions=1.0):
        """原地慢转扫描建初始地图(供 explore 用)。直接发 /cmd_vel,不走
        drive_timer(避开其超时逻辑)。车前方 180° 雷达转一圈即可扫全周环境
        (后方被 scan_angle_filter 滤掉不影响),给 explore_lite 喂出满地图前沿。
        启动 explore 前调用,此时 nav2 控制器无 goal 不发 /cmd_vel,无抢占。"""
        with self.lock:
            self.motion_active = False  # 防 drive_timer 干扰
        duration = revolutions * 2.0 * math.pi / max(0.1, abs(angular_speed))
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publish_twist(0.0, angular_speed)
            time.sleep(0.1)
        self.publish_stop(repeat=5)

    def generate_report(self, payload):
        mode = str(payload.get('mode', 'text')).strip().lower()
        if mode not in ('text', 'vision'):
            raise DashboardError('报告模式只能是 text 或 vision。')

        with self.lock:
            if self.report_job and self.report_job.get('running'):
                raise DashboardError('已有报告生成任务正在运行。', 409)

            job = {
                'running': True,
                'mode': mode,
                'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'returncode': None,
                'stdout': '',
                'stderr': '',
            }
            self.report_job = job

        command = ['ros2', 'run', 'car_report', 'report_generator', '--mode', mode]
        if mode == 'vision':
            command.extend(['--max-images', str(int(payload.get('max_images', 3)))])

        thread = threading.Thread(
            target=self._run_report_job,
            args=(command, job),
            name='car_web_report_job',
            daemon=True
        )
        thread.start()
        return {'ok': True, 'report_job': dict(job)}

    def get_report_job(self):
        with self.lock:
            return {'ok': True, 'report_job': dict(self.report_job) if self.report_job else None}

    def _run_report_job(self, command, job):
        try:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding='utf-8',
                errors='replace'
            )
            stdout, stderr = proc.communicate()
            with self.lock:
                job['running'] = False
                job['returncode'] = proc.returncode
                job['stdout'] = stdout[-4000:]
                job['stderr'] = stderr[-4000:]
                job['finished_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
                if proc.returncode != 0:
                    self.last_error = '报告生成失败，请查看 report_job.stderr。'
            self.announce('巡逻报告已生成' if proc.returncode == 0 else '报告生成失败')
        except Exception as exc:
            with self.lock:
                job['running'] = False
                job['returncode'] = -1
                job['stderr'] = str(exc)
                job['finished_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
                self.last_error = str(exc)

    @staticmethod
    def core_command():
        return ['ros2', 'launch', 'car_web', 'car_web_core.launch.py']

    @staticmethod
    def report_command():
        return [
            'ros2', 'launch', 'car_report', 'camp_security_yolo.launch.py',
        ]

    @staticmethod
    def notify_command():
        return [
            'ros2', 'launch', 'car_notify', 'dingtalk_notify.launch.py',
            'cooldown_sec:=60',
            'min_score:=0.5'
        ]

    @staticmethod
    def follow_command(mode):
        return [
            'ros2', 'launch', 'car_patrol', 'human_follow.launch.py',
            'mode:=%s' % mode,
            'auto_start:=true'
        ]

    @staticmethod
    def gesture_command():
        return [
            'ros2', 'launch', 'car_patrol', 'gesture_command.launch.py',
        ]

    @staticmethod
    def lidar_command():
        return ['ros2', 'launch', 'car_base', 'car_lidar.launch.py']

    @staticmethod
    def voice_command():
        return ['ros2', 'launch', 'car_voice', 'voice.launch.py']

    @staticmethod
    def rosbridge_command():
        # 发导航 action goal 需在新线程,否则阻塞 rosbridge 主线程。
        return [
            'ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml',
            'send_action_goals_in_new_thread:=true',
            'call_services_in_new_thread:=true',
        ]

    @staticmethod
    def nav_command(carto_slam, map_path=None):
        """carto_slam=True → 建图(cartographer);False → 加载地图定位(amcl)。

        注意:car_nav2.launch.py 只认 slam(True=建图/cartographer,False=定位/amcl),
        且其 include 的 car_base.launch.py 把 ekf 设为 UnlessCondition(carto_slam)。
        cartographer 依赖 ekf 发 /odom_combined 话题与 odom_combined->base_link TF,
        所以建图时**绝不能**传 carto_slam:=true(会全局生效关掉 ekf,导致永远不出图)。
        carto_slam 走默认 false 即可,ekf 两种模式都正常启动。
        """
        cmd = ['ros2', 'launch', 'car_nav2', 'car_nav2.launch.py']
        if carto_slam:
            cmd += ['slam:=True']
        else:
            cmd += ['slam:=False']
            if map_path:
                cmd += ['map:=%s' % map_path]
        return cmd

    @staticmethod
    def explore_command():
        # use_sim_time 必须显式传 false:explore.launch.py 默认 "true",真机无 /clock
        # 发布者时 explore_node 时间冻结在 0,定时器/TF 查询全失效 -> 机器人不动。
        # params.yaml 已设 return_to_init:true、costmap_topic:/global_costmap/costmap。
        return ['ros2', 'launch', 'explore_lite', 'explore.launch.py',
                'use_sim_time:=false']

    @staticmethod
    def video_url(topic):
        return '/stream?topic=%s&type=mjpeg' % str(topic or CAMERA_TOPIC)

    def video_diagnostics(self, current_topic):
        current_topic = current_topic or CAMERA_TOPIC
        return {
            'web_video_reachable': self._web_video_reachable(),
            'core_running': self._is_running('core'),
            'camera_publishers': self._publisher_count(CAMERA_TOPIC),
            'result_publishers': self._publisher_count(RESULT_TOPIC),
            'current_topic_publishers': self._publisher_count(current_topic),
        }

    @staticmethod
    def video_hint(current_topic, diagnostics):
        if not diagnostics.get('web_video_reachable'):
            return '8080 视频服务未连接；先确认 core 正在运行，或点击视频服务首页排查。'
        if current_topic == RESULT_TOPIC and diagnostics.get('result_publishers', 0) <= 0:
            return '8080 已连接，但 /result_img 暂无发布者；先启动识别记录。'
        if current_topic == CAMERA_TOPIC and diagnostics.get('camera_publishers', 0) <= 0:
            return '8080 已连接，但相机话题暂无发布者；检查 car_camera 是否启动。'
        return '8080 已连接，当前视频话题发布者 %d 个。' % diagnostics.get('current_topic_publishers', 0)

    def _publisher_count(self, topic):
        try:
            return len(self.get_publishers_info_by_topic(topic))
        except Exception:
            return 0

    @staticmethod
    def _web_video_reachable():
        try:
            with socket.create_connection(('127.0.0.1', WEB_VIDEO_PORT), timeout=0.2):
                return True
        except OSError:
            return False

    def start_process(self, name, command):
        with self.lock:
            if self._is_running(name):
                return

            old_proc = self.processes.get(name)
            if old_proc is not None and old_proc.poll() is not None:
                self.processes.pop(name, None)

            self.get_logger().info('starting %s: %s' % (name, ' '.join(command)))
            try:
                kwargs = {}
                if os.name == 'nt':
                    kwargs['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
                else:
                    kwargs['preexec_fn'] = os.setsid
                proc = subprocess.Popen(command, **kwargs)
            except FileNotFoundError as exc:
                self.last_error = '找不到 ros2 命令，请确认 ROS2 环境已 source。'
                raise DashboardError(self.last_error, 500) from exc
            except Exception as exc:
                self.last_error = str(exc)
                raise DashboardError(str(exc), 500) from exc

            self.processes[name] = proc
            self.last_error = ''

    def stop_process(self, name):
        with self.lock:
            proc = self.processes.get(name)
        if proc is None:
            return
        if proc.poll() is not None:
            with self.lock:
                self.processes.pop(name, None)
            return

        self.get_logger().info('stopping %s' % name)
        try:
            if os.name == 'nt':
                proc.terminate()
            else:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                proc.kill()
            else:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)
        except ProcessLookupError:
            pass
        finally:
            with self.lock:
                self.processes.pop(name, None)

    def shutdown(self):
        self.stop_all()
        self.http_server.shutdown()
        self.http_server.server_close()

    def force_cleanup_residuals(self, include_lidar=False, include_voice=False,
                                include_nav=False, preserve_rosbridge=False):
        """按进程特征兜底清理残留(孤儿/非本实例起的功能节点)。
        先 SIGTERM 再 SIGKILL 整个进程组。用字符类化的 pattern 避免误杀。
        注意:voice 默认不清(stop_all 不应杀语音助手);仅启动自清/显式停语音时清。
        nav 残留(cartographer/nav2 各 server/rosbridge/explore)极易留孤儿,停导航时必清。
        preserve_rosbridge:加载地图等场景下保活 rosbridge,避免浏览器 websocket 断开、
        重连后错过 map_server 的 latched /map(transient_local)导致画布空白。"""
        patterns = list(RESIDUAL_PATTERNS)
        if include_lidar:
            patterns += LIDAR_PATTERNS
        if include_voice:
            patterns += VOICE_PATTERNS
        if include_nav:
            patterns += NAV_PATTERNS + LIDAR_PATTERNS
        if preserve_rosbridge:
            patterns = [p for p in patterns if p != 'rosbridge_websocket']
        killed = []
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pat in patterns:
                # [x]xx 字符类:防止 pgrep 匹配到自己的命令行
                cpat = '[' + pat[0] + ']' + pat[1:]
                try:
                    out = subprocess.run(['pgrep', '-f', cpat],
                                         capture_output=True, text=True, timeout=3)
                except Exception:
                    continue
                for pid_s in out.stdout.split():
                    try:
                        pid = int(pid_s)
                        os.killpg(os.getpgid(pid), sig)
                        killed.append(pid)
                    except (ProcessLookupError, ValueError, PermissionError):
                        pass
            if sig == signal.SIGTERM:
                time.sleep(2.5)
        if killed:
            self.get_logger().info('force_cleanup killed pids: %s'
                                    % sorted(set(killed)))
        return len(set(killed))

    def _is_running(self, name):
        proc = self.processes.get(name)
        return proc is not None and proc.poll() is None

    def _process_state(self, name):
        proc = self.processes.get(name)
        if proc is None:
            return {'running': False, 'returncode': None}
        return {
            'running': proc.poll() is None,
            'returncode': proc.poll(),
            'pid': proc.pid,
        }

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = 'car_web/0.2'

    def do_GET(self):
        try:
            path = urlsplit(self.path).path
            if path == '/':
                self.serve_file('index.html')
            elif path == '/api/status':
                self.send_json(self.server.dashboard.status())
            elif path == '/api/events':
                self.send_json(self.server.dashboard.get_events())
            elif path == '/api/features':
                self.send_json(self.server.dashboard.get_features())
            elif path == '/api/report/job':
                self.send_json(self.server.dashboard.get_report_job())
            elif path == '/api/nav/maps':
                self.send_json(self.server.dashboard.list_maps())
            elif path == '/api/nav/points':
                self.send_json(self.server.dashboard.list_points())
            elif path.startswith('/static/'):
                self.serve_file(path[len('/static/'):])
            else:
                self.send_json({'ok': False, 'error': 'not found'}, 404)
        except Exception as exc:
            self.handle_error(exc)

    def do_POST(self):
        try:
            payload = self.read_json()
            dashboard = self.server.dashboard
            if self.path == '/api/mode/manual':
                result = dashboard.start_manual_mode()
            elif self.path == '/api/uimode':
                result = dashboard.set_ui_mode(payload.get('mode', 'follow'))
            elif self.path == '/api/nav/stop':
                result = dashboard.stop_nav()
            elif self.path == '/api/nav/mapping/start':
                result = dashboard.start_mapping(explore=bool(payload.get('explore')))
            elif self.path == '/api/nav/explore/stop':
                result = dashboard.stop_explore()
            elif self.path == '/api/nav/map/save':
                result = dashboard.save_map(payload)
            elif self.path == '/api/nav/map/load':
                result = dashboard.load_map(payload)
            elif self.path == '/api/nav/goto':
                result = dashboard.goto(payload)
            elif self.path == '/api/nav/point/save':
                result = dashboard.save_point(payload)
            elif self.path == '/api/nav/point/delete':
                result = dashboard.delete_point(payload)
            elif self.path == '/api/nav/point/goto':
                result = dashboard.goto_point(payload)
            elif self.path == '/api/nav/home':
                result = dashboard.nav_home(payload)
            elif self.path == '/api/nav/cancel':
                result = dashboard.cancel_nav(payload)
            elif self.path == '/api/nav/initialpose':
                result = dashboard.set_initial_pose(payload)
            elif self.path == '/api/nav/speed':
                result = dashboard.nav_speed(payload)
            elif self.path == '/api/nav/cruise':
                result = dashboard.cruise(payload)
            elif self.path == '/api/mode/report':
                result = dashboard.start_report_mode()
            elif self.path == '/api/core/stop':
                result = dashboard.stop_core()
            elif self.path == '/api/report/stop':
                result = dashboard.stop_report()
            elif self.path == '/api/all/stop':
                result = dashboard.stop_all()
            elif self.path == '/api/notify/start':
                result = dashboard.start_notify()
            elif self.path == '/api/notify/stop':
                result = dashboard.stop_notify()
            elif self.path == '/api/drive':
                result = dashboard.drive(payload)
            elif self.path == '/api/estop':
                result = dashboard.estop()
            elif self.path == '/api/chassis/reset':
                result = dashboard.chassis_reset()
            elif self.path == '/api/report/generate':
                result = dashboard.generate_report(payload)
            elif self.path == '/api/features/start':
                result = dashboard.start_feature(payload)
            elif self.path == '/api/features/stop':
                result = dashboard.stop_feature()
            elif self.path == '/api/servo/reset':
                result = dashboard.servo_reset()
            elif self.path == '/api/servo/camera_left':
                result = dashboard.camera_left()
            elif self.path == '/api/servo/camera_right':
                result = dashboard.camera_right()
            elif self.path == '/api/servo/camera_up':
                result = dashboard.camera_up()
            elif self.path == '/api/servo/camera_down':
                result = dashboard.camera_down()
            elif self.path == '/api/patrol/command':
                result = dashboard.patrol_command(payload)
            elif self.path == '/api/patrol/speed':
                result = dashboard.patrol_speed(payload)
            elif self.path == '/api/patrol/relock':
                dashboard.publish_patrol_command('relock')
                result = {'ok': True, 'message': '已发送重新锁定'}
            elif self.path == '/api/gesture/start':
                result = dashboard.start_gesture()
            elif self.path == '/api/gesture/stop':
                result = dashboard.stop_gesture()
            elif self.path == '/api/lidar/start':
                result = dashboard.start_lidar()
            elif self.path == '/api/lidar/stop':
                result = dashboard.stop_lidar()
            elif self.path == '/api/voice/start':
                result = dashboard.start_voice()
            elif self.path == '/api/voice/stop':
                result = dashboard.stop_voice()
            elif self.path == '/api/voice/tts/start':
                result = dashboard.set_voice_toggle('tts', True)
            elif self.path == '/api/voice/tts/stop':
                result = dashboard.set_voice_toggle('tts', False)
            elif self.path == '/api/voice/asr/start':
                result = dashboard.set_voice_toggle('asr', True)
            elif self.path == '/api/voice/asr/stop':
                result = dashboard.set_voice_toggle('asr', False)
            else:
                result = {'ok': False, 'error': 'not found'}
                self.send_json(result, 404)
                return
            self.send_json(result)
        except Exception as exc:
            self.handle_error(exc)

    def read_json(self):
        length = int(self.headers.get('Content-Length', '0') or '0')
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode('utf-8')
        return json.loads(raw) if raw.strip() else {}

    def serve_file(self, relative_path):
        safe_path = unquote(urlsplit(relative_path).path).replace('\\', '/')
        parts = [part for part in safe_path.split('/') if part]
        if safe_path.startswith('/') or '..' in parts:
            self.send_json({'ok': False, 'error': 'invalid path'}, 403)
            return

        requested = self.server.dashboard.web_dir.joinpath(*parts)
        if not requested.exists() or not requested.is_file():
            self.send_json({'ok': False, 'error': 'not found'}, 404)
            return

        content_type = mimetypes.guess_type(str(requested))[0] or 'application/octet-stream'
        data = requested.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def handle_error(self, exc):
        if isinstance(exc, DashboardError):
            self.send_json({'ok': False, 'error': str(exc)}, exc.status)
            return
        self.server.dashboard.last_error = str(exc)
        self.send_json({'ok': False, 'error': str(exc)}, 500)

    def log_message(self, fmt, *args):
        self.server.dashboard.get_logger().info(fmt % args)


def main():
    rclpy.init()
    node = DashboardNode()
    # 启动自清:清掉上次崩溃/重启遗留的功能节点(不含雷达,雷达由用户/跟随按需起)
    try:
        n = node.force_cleanup_residuals(include_lidar=False, include_voice=True)
        node.get_logger().info('startup cleanup removed %d residual process(es)' % n)
    except Exception as exc:
        node.get_logger().warn('startup cleanup failed: %s' % exc)
    # 自清之后再起 core,避免被上面的 cleanup 误杀(car_web_core.launch 在 RESIDUAL_PATTERNS)
    node.start_core_safe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
