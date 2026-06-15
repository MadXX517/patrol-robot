#!/usr/bin/env python3
# encoding: utf-8
import json
import mimetypes
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


CAMERA_TOPIC = '/camera/color/image_raw'
RESULT_TOPIC = '/result_img'
EVENT_TOPIC = '/car_report/event'
CMD_VEL_TOPIC = '/cmd_vel'


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

        self.http_host = str(self.get_parameter('http_host').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.drive_timeout = max(0.1, float(self.get_parameter('drive_timeout').value))
        self.max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))

        self.web_dir = get_web_dir()
        self.lock = threading.RLock()
        self.processes = {}
        self.current_mode = 'idle'
        self.current_video_topic = CAMERA_TOPIC
        self.last_error = ''
        self.events = []
        self.report_job = None
        self.last_drive_time = 0.0
        self.motion_active = False

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 5)
        self.create_subscription(String, EVENT_TOPIC, self.event_callback, 20)
        self.create_timer(0.1, self.drive_watchdog)

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
            'car_web dashboard started: http://%s:%d'
            % (self.http_host, self.http_port)
        )

    def event_callback(self, msg):
        try:
            parsed = json.loads(msg.data)
        except Exception:
            parsed = {'raw': msg.data}

        with self.lock:
            self.events.append(parsed)
            self.events = self.events[-50:]

    def drive_watchdog(self):
        with self.lock:
            should_stop = (
                self.motion_active and
                time.monotonic() - self.last_drive_time > self.drive_timeout
            )
        if should_stop:
            self.publish_stop()

    def status(self):
        with self.lock:
            processes = {
                name: self._process_state(name)
                for name in ('core', 'report', 'notify')
            }
            report_job = dict(self.report_job) if self.report_job else None
            return {
                'ok': True,
                'mode': self.current_mode,
                'video_topic': self.current_video_topic,
                'web_video_port': 8080,
                'processes': processes,
                'events_count': len(self.events),
                'last_event': self.events[-1] if self.events else None,
                'report_job': report_job,
                'last_error': self.last_error,
            }

    def get_events(self):
        with self.lock:
            return {'ok': True, 'events': list(reversed(self.events))}

    def start_manual_mode(self):
        self.start_process('core', self.core_command())
        self.stop_process('notify')
        self.stop_process('report')
        self.publish_stop()
        with self.lock:
            self.current_mode = 'manual'
            self.current_video_topic = CAMERA_TOPIC
        return self.status()

    def start_report_mode(self):
        self.start_process('core', self.core_command())
        self.start_process('report', self.report_command())
        with self.lock:
            self.current_mode = 'report'
            self.current_video_topic = RESULT_TOPIC
        return self.status()

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

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        with self.lock:
            self.last_drive_time = time.monotonic()
            self.motion_active = abs(linear) > 1e-6 or abs(angular) > 1e-6

        return {
            'ok': True,
            'linear': linear,
            'angular': angular,
        }

    def estop(self):
        self.publish_stop(repeat=5)
        return {'ok': True, 'message': '急停已发送'}

    def publish_stop(self, repeat=1):
        twist = Twist()
        for _ in range(max(1, int(repeat))):
            self.cmd_pub.publish(twist)
        with self.lock:
            self.last_drive_time = time.monotonic()
            self.motion_active = False

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
            'yolo_pub_result_img:=true'
        ]

    @staticmethod
    def notify_command():
        return [
            'ros2', 'launch', 'car_notify', 'dingtalk_notify.launch.py',
            'cooldown_sec:=60'
        ]

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
        self.publish_stop(repeat=5)
        for name in ('notify', 'report', 'core'):
            self.stop_process(name)
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

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = 'car_web/0.1'

    def do_GET(self):
        try:
            if self.path == '/' or self.path.startswith('/?'):
                self.serve_file('index.html')
            elif self.path == '/api/status':
                self.send_json(self.server.dashboard.status())
            elif self.path == '/api/events':
                self.send_json(self.server.dashboard.get_events())
            elif self.path == '/api/report/job':
                self.send_json(self.server.dashboard.get_report_job())
            elif self.path.startswith('/static/'):
                self.serve_file(self.path[len('/static/'):])
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
            elif self.path == '/api/notify/start':
                result = dashboard.start_notify()
            elif self.path == '/api/notify/stop':
                result = dashboard.stop_notify()
            elif self.path == '/api/drive':
                result = dashboard.drive(payload)
            elif self.path == '/api/estop':
                result = dashboard.estop()
            elif self.path == '/api/report/generate':
                result = dashboard.generate_report(payload)
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
        web_dir = self.server.dashboard.web_dir.resolve()
        requested = (web_dir / relative_path).resolve()
        try:
            requested.relative_to(web_dir)
        except ValueError:
            self.send_json({'ok': False, 'error': 'invalid path'}, 403)
            return

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
