# web_control.py
# car_voice 的命令执行器 + 播报订阅器(替代 car_llm/ros_control.py 的机械臂执行器)。
# 作为 ROS 节点运行:
#   - 订阅 /voice/announce(String):收到文本 -> 投入 tts_queue 播报(状态播报通道)
#   - 订阅 /voice/control(String):tts_on/tts_off/asr_on/asr_off,供 dashboard 实时切开关
#   - 消费 ros_control_queue 里的 step(LLM 解析出的功能命令)-> POST dashboard HTTP API
# 设计:不碰机械臂/底盘,所有控制都转成对 dashboard 既有 HTTP 端点的调用,复用单一控制中枢。

import json
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# LLM function -> (HTTP method 用 POST, 路径, 可选 json body)
# body 为 None 表示空 POST;camera_move 的方向在运行时拼路径。
FUNCTION_ROUTES = {
    'start_patrol':  ('/api/mode/report', None),
    'start_follow':  ('/api/features/start', {'id': 'human_follow'}),
    'start_track':   ('/api/features/start', {'id': 'gimbal_track'}),
    'stop_all':      ('/api/all/stop', None),
    'relock':        ('/api/patrol/relock', None),
    'follow_faster': ('/api/patrol/speed', {'action': 'up'}),
    'follow_slower': ('/api/patrol/speed', {'action': 'down'}),
    'follow_normal': ('/api/patrol/speed', {'action': 'normal'}),
    'gesture_on':    ('/api/gesture/start', None),
    'gesture_off':   ('/api/gesture/stop', None),
    'lidar_on':      ('/api/lidar/start', None),
    'lidar_off':     ('/api/lidar/stop', None),
    'gen_report':    ('/api/report/generate', None),
    'chassis_reset': ('/api/chassis/reset', None),
    # ---- 导航类(阶段6)----
    'enter_nav':     ('/api/uimode', {'mode': 'nav'}),
    'nav_home':      ('/api/nav/home', None),
    'nav_cancel':    ('/api/nav/cancel', None),
}

# camera_move 方向 -> dashboard 端点
CAMERA_DIR_ROUTES = {
    'left':  '/api/servo/camera_left',
    'right': '/api/servo/camera_right',
    'up':    '/api/servo/camera_up',
    'down':  '/api/servo/camera_down',
    'reset': '/api/servo/reset',
}


class WebControl(Node):
    """LLM step -> dashboard HTTP API;并桥接 /voice/announce、/voice/control。"""

    def __init__(self, ros_control_queue=None, tts_queue=None,
                 control_queue=None, dashboard_url='http://127.0.0.1:8000',
                 name='voice_web_control'):
        super().__init__(name)
        self.action_queue = ros_control_queue   # LLM 解析出的 step(multiprocessing.Queue)
        self.tts_queue = tts_queue               # 要播报的文本(送回主进程 TTS)
        self.control_queue = control_queue       # tts/asr 开关信号(送回主进程)
        self.dashboard_url = dashboard_url.rstrip('/')

        # 状态播报:任何节点发文本到 /voice/announce -> 播报
        self.create_subscription(String, '/voice/announce', self.announce_callback, 10)
        # 实时开关:dashboard 发 tts_on/tts_off/asr_on/asr_off
        self.create_subscription(String, '/voice/control', self.control_callback, 5)
        self.get_logger().info('voice web_control 就绪, dashboard=%s' % self.dashboard_url)

    def announce_callback(self, msg):
        text = (msg.data or '').strip()
        if text and self.tts_queue is not None:
            self.tts_queue.put(text)

    def control_callback(self, msg):
        cmd = (msg.data or '').strip()
        if cmd and self.control_queue is not None:
            self.control_queue.put(cmd)

    def _post(self, path, body=None):
        url = self.dashboard_url + path
        try:
            r = requests.post(url, json=body, timeout=8)
            ok = (r.status_code == 200)
            self.get_logger().info('POST %s -> %s' % (path, r.status_code))
            # 失败时尽量取出后端的中文 error,供语音播报具体原因
            err = None
            if not ok:
                try:
                    err = (r.json() or {}).get('error')
                except Exception:
                    err = None
            return ok, err
        except Exception as exc:
            self.get_logger().warn('POST %s 失败: %s' % (path, exc))
            return False, None

    def execute_step(self, step):
        """把一个 LLM step 翻译成 dashboard HTTP 调用。"""
        func = step.get('function', '')
        params = step.get('parameters', {}) or {}

        if func == 'camera_move':
            direction = str(params.get('dir', '')).strip().lower()
            path = CAMERA_DIR_ROUTES.get(direction)
            if not path:
                self.get_logger().warn('camera_move 未知方向: %s' % direction)
                return
            self._post(path)
            return

        # goto_point:LLM 从口语里抽出地点名(如"去客厅"->name=客厅),按名导航。
        if func == 'goto_point':
            name = str(params.get('name', '')).strip()
            if not name:
                if self.tts_queue is not None:
                    self.tts_queue.put('你要去哪里呢')
                return
            ok, err = self._post('/api/nav/point/goto', {'name': name})
            if not ok and self.tts_queue is not None:
                self.tts_queue.put(err or ('没找到%s这个点' % name))
            return

        route = FUNCTION_ROUTES.get(func)
        if route is None:
            self.get_logger().warn('未知 function: %s' % func)
            return
        path, body = route
        ok, err = self._post(path, body)
        if not ok and self.tts_queue is not None:
            self.tts_queue.put(err or '操作没成功，请稍后再试')

    def analyze_communication(self, data):
        """兼容 LLMCommandParser 投进来的对象:{'step': {...}} 或裸 step。"""
        try:
            if isinstance(data, str):
                data = json.loads(data)
            step = data.get('step', data) if isinstance(data, dict) else None
            if isinstance(step, dict) and 'function' in step:
                self.get_logger().info('执行 step: %s' % step.get('function'))
                self.execute_step(step)
        except Exception as exc:
            self.get_logger().warn('解析 step 出错: %s' % exc)

    def loop(self):
        if self.action_queue is not None and self.action_queue.qsize() > 0:
            self.analyze_communication(self.action_queue.get())
        rclpy.spin_once(self, timeout_sec=0.05)
