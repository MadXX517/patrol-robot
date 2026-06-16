#!/usr/bin/env python3
# encoding: utf-8
import json
import mimetypes
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
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


CAMERA_TOPIC = '/camera/color/image_raw'
RESULT_TOPIC = '/result_img'
EVENT_TOPIC = '/car_report/event'
CMD_VEL_TOPIC = '/cmd_vel'
WEB_VIDEO_PORT = 8080
IK_TOPIC = '/ik_states'
JOINT_STATES_TOPIC = '/joint_states'
PERSON_FOLLOW_RESULT_TOPIC = '/person_follow/result_img'
PATROL_CMD_TOPIC = '/patrol/command'
PATROL_STATE_TOPIC = '/person_follow/state'
POWER_VOLTAGE_TOPIC = '/PowerVoltage'
PROCESS_NAMES = ('core', 'report', 'notify', 'voice', 'follow')
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
        'name': '视觉巡逻',
        'description': '启动 YOLO 识别、事件记录和置信框视频。',
        'available': True,
    },
    {
        'id': 'line_follow',
        'name': '自主巡线',
        'description': '预留接口，后续接 car_vision 或 car_app 巡线。',
        'available': False,
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
    {
        'id': 'pose_detect',
        'name': '姿态检测',
        'description': '预留接口，后续接姿态/手势识别。',
        'available': False,
    },
    {
        'id': 'color_track',
        'name': '颜色追踪',
        'description': '预留接口，后续接颜色识别和追踪。',
        'available': False,
    },
    {
        'id': 'radar_control',
        'name': '雷达控制',
        'description': '预留接口，后续接雷达避障或导航。',
        'available': False,
    },
    {
        'id': 'voice_control',
        'name': '语音控制',
        'description': '启动 car_llm 语音识别、大模型解析、TTS 播报和机器人控制。',
        'available': True,
    },
]

FEATURE_STARTERS = {
    'visual_patrol': 'start_report_mode',
    'voice_control': 'start_voice_control',
}

FEATURE_STOPPERS = {
    'visual_patrol': 'stop_report',
    'voice_control': 'stop_voice',
}

PATROL_FEATURE_MODES = {
    'human_follow': 'follow',
    'gimbal_track': 'track_only',
}


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
        # 电量:/PowerVoltage 为串口单字节 ADC 值,线性过原点标定。
        # 实测:万用表 11.06V 对应原始值≈62 -> scale≈0.178(原始字节本身有 ±2 噪声)。
        # full/empty 为 3 串锂电满/空电压(伏特),用于估算百分比。
        self.declare_parameter('voltage_scale', 0.178)
        self.declare_parameter('voltage_full', 12.6)
        self.declare_parameter('voltage_empty', 9.9)
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('llm_wake_word', '小星')

        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.drive_timeout = max(0.1, float(self.get_parameter('drive_timeout').value))
        self.max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self.voltage_scale = float(self.get_parameter('voltage_scale').value)
        self.voltage_full = float(self.get_parameter('voltage_full').value)
        self.voltage_empty = float(self.get_parameter('voltage_empty').value)
        self.battery_raw = None
        self.battery_stamp = 0.0
        self.llm_api_key = str(self.get_parameter('llm_api_key').value or '').strip()
        self.llm_wake_word = str(self.get_parameter('llm_wake_word').value or '小星').strip() or '小星'

        self.web_dir = get_web_dir()
        self.lock = threading.RLock()
        self.processes = {}
        self.current_mode = 'idle'
        self.current_video_topic = CAMERA_TOPIC
        self.active_features = set()
        self.voice_mode = ''
        self.last_error = ''
        self.events = []
        self.report_job = None
        self.last_drive_time = 0.0
        self.motion_active = False
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_joints = [0.0] * len(JOINT_NAMES)
        self.have_joint_state = False

        self.patrol_state = ''

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 5)
        self.servo_pub = self.create_publisher(JointState, IK_TOPIC, 5)
        self.patrol_cmd_pub = self.create_publisher(String, PATROL_CMD_TOPIC, 5)
        self.create_subscription(String, EVENT_TOPIC, self.event_callback, 20)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.joint_state_callback, 10)
        self.create_subscription(String, PATROL_STATE_TOPIC, self.patrol_state_callback, 5)
        self.create_subscription(Float32, POWER_VOLTAGE_TOPIC, self.battery_callback, 10)
        self.create_timer(0.1, self.drive_timer_callback)

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
                self.motion_active = False

            linear = self.target_linear
            angular = self.target_angular

        self._publish_twist(linear, angular)

    def status(self):
        with self.lock:
            self._reconcile_active_features_locked()
            processes = {
                name: self._process_state(name)
                for name in PROCESS_NAMES
            }
            report_job = dict(self.report_job) if self.report_job else None
            core_running = self._is_running('core')
            report_running = self._is_running('report')
            voice_running = self._is_running('voice')
            follow_running = self._is_running('follow')
            voice_mode = self.voice_mode
            current_video_topic = self.current_video_topic

        video_diagnostics = self.video_diagnostics(current_video_topic)

        with self.lock:
            return {
                'ok': True,
                'mode': self.current_mode,
                'active_feature': self._primary_active_feature(),
                'active_features': sorted(self.active_features),
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
                'can_drive': core_running and not (voice_running and voice_mode == 'control') and not follow_running,
                'can_report': report_running,
                'patrol_state': self.patrol_state,
                'battery': self.battery_info(),
                'voice_mode': voice_mode,
                'voice_wake_word': self.llm_wake_word,
                'voice_configured': True,
            }

    def get_events(self):
        with self.lock:
            return {'ok': True, 'events': list(reversed(self.events))}

    def get_features(self):
        with self.lock:
            self._reconcile_active_features_locked()
            active_features = sorted(self.active_features)
        return {
            'ok': True,
            'features': FEATURES,
            'active_feature': active_features[0] if len(active_features) == 1 else '',
            'active_features': active_features,
        }

    def start_feature(self, payload):
        feature_id = str(payload.get('id', '')).strip()
        feature = next((item for item in FEATURES if item['id'] == feature_id), None)
        if feature is None:
            raise DashboardError('未知功能：%s' % feature_id, 404)
        if not feature.get('available'):
            raise DashboardError('%s 还只是预留入口，后续接入对应功能包。' % feature['name'], 501)
        patrol_mode = PATROL_FEATURE_MODES.get(feature_id)
        if patrol_mode:
            return self.start_patrol(feature_id, patrol_mode)
        
        starter_name = FEATURE_STARTERS.get(feature_id)
        if starter_name:
            return getattr(self, starter_name)()
        raise DashboardError('%s 未配置启动逻辑。' % feature['name'], 501)

    def stop_feature(self, payload=None):
        feature_id = ''
        if isinstance(payload, dict):
            feature_id = str(payload.get('id', '')).strip()
        with self.lock:
            active_features = set(self.active_features)
            if not feature_id and len(active_features) == 1:
                feature_id = next(iter(active_features))
        if feature_id in PATROL_FEATURE_MODES:
            return self.stop_patrol()

        stopper_name = FEATURE_STOPPERS.get(feature_id)
        if stopper_name:
            return getattr(self, stopper_name)(update_mode=True)
        
        if not active_features:
            return self.status()
        raise DashboardError('请指定要停止的功能。', 400)

    def start_patrol(self, feature_id, mode):
        self.start_core()
        self.stop_report(update_mode=False)
        self.stop_voice(update_mode=False)
        self.stop_process('follow')
        self.start_process('follow', self.follow_command(mode))
        with self.lock:
            self.current_mode = feature_id
            self.current_video_topic = PERSON_FOLLOW_RESULT_TOPIC
            self._remove_active_feature('human_follow')
            self._remove_active_feature('gimbal_track')
            self._add_active_feature(feature_id)
        return self.status()

    def stop_patrol(self, update_mode=True):
        self.publish_patrol_command('stop')
        self.stop_process('follow')
        self.publish_stop()
        with self.lock:
            if update_mode:
                self.current_video_topic = CAMERA_TOPIC
                self.current_mode = 'manual' if self._is_running('core') else 'idle'
            self._remove_active_feature('human_follow')
            self._remove_active_feature('gimbal_track')
            self.patrol_state = ''
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

    def patrol_state_callback(self, msg):
        with self.lock:
            self.patrol_state = msg.data

    def battery_callback(self, msg):
        with self.lock:
            self.battery_raw = float(msg.data)
            self.battery_stamp = time.monotonic()

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
        self.stop_patrol(update_mode=False)
        self.publish_stop()
        with self.lock:
            self.current_mode = 'manual'
            self.current_video_topic = CAMERA_TOPIC
        return self.status()

    def start_report_mode(self):
        self.start_core()
        self.stop_patrol(update_mode=False)
        self.start_process('report', self.report_command())
        with self.lock:
            self.current_mode = 'report'
            self.current_video_topic = RESULT_TOPIC
            self._add_active_feature('visual_patrol')
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
                self._remove_active_feature('visual_patrol')
        else:
            with self.lock:
                self._remove_active_feature('visual_patrol')
        return self.status()

    def start_voice_chat(self):
        return self.start_voice_mode(voice_mode='chat', enable_ros_control=False)

    def start_voice_control(self):
        return self.start_voice_mode(voice_mode='control', enable_ros_control=True)

    def start_voice_mode(self, voice_mode='control', enable_ros_control=True):
        if voice_mode not in ('chat', 'control'):
            raise DashboardError('语音模式只能是 chat 或 control。')

        self.start_core()
        if enable_ros_control:
            self.stop_patrol(update_mode=False)
        self.stop_process('voice')
        self.publish_stop(repeat=3)
        self.start_process('voice', self.voice_command(enable_ros_control=enable_ros_control))
        with self.lock:
            if self.current_mode == 'idle':
                self.current_mode = 'manual'
                self.current_video_topic = CAMERA_TOPIC
            self._add_active_feature('voice_control')
            self.voice_mode = voice_mode
        return self.status()

    def stop_voice(self, update_mode=True):
        self.stop_process('voice')
        self.publish_stop(repeat=3)
        if update_mode:
            with self.lock:
                self.voice_mode = ''
                self._remove_active_feature('voice_control')
                if not self._is_running('report'):
                    self.current_video_topic = CAMERA_TOPIC
                    self.current_mode = 'manual' if self._is_running('core') else 'idle'
        else:
            with self.lock:
                self.voice_mode = ''
                self._remove_active_feature('voice_control')
        return self.status()

    def voice_safe_stop(self):
        self.stop_voice(update_mode=True)
        self.publish_stop(repeat=8)
        return self.status()

    def stop_core(self):
        self.stop_process('notify')
        self.stop_process('report')
        self.stop_process('voice')
        self.stop_process('follow')
        self.publish_stop(repeat=5)
        self.stop_process('core')
        with self.lock:
            self.current_mode = 'idle'
            self.current_video_topic = CAMERA_TOPIC
            self.active_features.clear()
            self.voice_mode = ''
            self.patrol_state = ''
        return self.status()

    def stop_all(self):
        return self.stop_core()

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
        linear = float(payload.get('linear', 0.0))
        angular = float(payload.get('angular', 0.0))
        linear = self._clamp(linear, -self.max_linear_speed, self.max_linear_speed)
        angular = self._clamp(angular, -self.max_angular_speed, self.max_angular_speed)

        with self.lock:
            can_drive = self._is_running('core')
            voice_running = self._is_running('voice')
            voice_control_running = voice_running and self.voice_mode == 'control'
            follow_running = self._is_running('follow')
        if not can_drive:
            raise DashboardError('基础节点未运行，不能遥控。请先启动普通操作或识别记录。', 409)
        if voice_control_running and (abs(linear) > 1e-6 or abs(angular) > 1e-6):
            raise DashboardError('语音控制运行中，已禁止手动遥控。请先停止语音控制。', 409)
        if follow_running and (abs(linear) > 1e-6 or abs(angular) > 1e-6):
            raise DashboardError('人体跟随运行中，已禁止手动遥控。请先停止跟随功能。', 409)

        with self.lock:
            self.target_linear = linear
            self.target_angular = angular
            self.last_drive_time = time.monotonic()
            self.motion_active = abs(linear) > 1e-6 or abs(angular) > 1e-6

        if not self.motion_active:
            self.publish_stop()

        return {
            'ok': True,
            'linear': linear,
            'angular': angular,
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
            raise DashboardError('基础节点未运行，不能控制云台。请先启动普通操作或识别记录。', 409)
        with self.lock:
            voice_control_running = self._is_running('voice') and self.voice_mode == 'control'
            follow_running = self._is_running('follow')
        if voice_control_running:
            raise DashboardError('语音控制运行中，已禁止手动云台控制。请先停止语音控制。', 409)
        if follow_running:
            raise DashboardError('人体跟随运行中，已禁止手动云台控制。请先停止跟随功能。', 409)

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
            self.last_drive_time = time.monotonic()
            self.motion_active = False
        for _ in range(max(1, int(repeat))):
            self._publish_twist(0.0, 0.0)

    def _publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

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
            'ros2', 'launch', 'car_report', 'car_report_yolo.launch.py',
            'yolo_pub_result_img:=true',
            'yolo_conf_thres:=0.5',
            'min_score:=0.5'
        ]

    @staticmethod
    def notify_command():
        return [
            'ros2', 'launch', 'car_notify', 'dingtalk_notify.launch.py',
            'cooldown_sec:=60',
            'min_score:=0.5'
        ]

    def voice_command(self, enable_ros_control=True):
        command = [
            'ros2', 'launch', 'car_llm', 'car_llm.launch.py',
            'enable_ros_control:=%s' % ('true' if enable_ros_control else 'false'),
            'mic_trigger:=continuous',
            'start_base:=false',
            'wake_word:=%s' % self.llm_wake_word,
        ]
        if self.llm_api_key:
            command.insert(4, 'api_key:=%s' % self.llm_api_key)
        return command

    @staticmethod
    def follow_command(mode):
        return [
            'ros2', 'launch', 'car_patrol', 'human_follow.launch.py',
            'mode:=%s' % mode,
            'auto_start:=true'
        ]

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

            self.get_logger().info('starting %s: %s' % (name, ' '.join(self._redact_command(command))))
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

    def _add_active_feature(self, feature_id):
        self.active_features.add(feature_id)

    def _remove_active_feature(self, feature_id):
        self.active_features.discard(feature_id)

    def _primary_active_feature(self):
        if len(self.active_features) == 1:
            return next(iter(self.active_features))
        return ''

    def _reconcile_active_features_locked(self):
        if not self._is_running('report'):
            self.active_features.discard('visual_patrol')
        if not self._is_running('voice'):
            self.active_features.discard('voice_control')
            self.voice_mode = ''
        if not self._is_running('follow'):
            self.active_features.discard('human_follow')
            self.active_features.discard('gimbal_track')

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _redact_command(command):
        redacted = []
        for item in command:
            if str(item).startswith('api_key:='):
                redacted.append('api_key:=***')
            else:
                redacted.append(item)
        return redacted


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
                result = dashboard.stop_feature(payload)
            elif self.path == '/api/voice/start_chat':
                result = dashboard.start_voice_chat()
            elif self.path == '/api/voice/start_control':
                result = dashboard.start_voice_control()
            elif self.path == '/api/voice/stop':
                result = dashboard.stop_voice()
            elif self.path == '/api/voice/safe_stop':
                result = dashboard.voice_safe_stop()
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
            elif self.path == '/api/patrol/relock':
                dashboard.publish_patrol_command('relock')
                result = {'ok': True, 'message': '已发送重新锁定'}
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
