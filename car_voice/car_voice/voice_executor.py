#!/usr/bin/env python3
# encoding: utf-8
import json
import math
import queue
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from interfaces.srv import SetInt64
from std_srvs.srv import SetBool

from .command_parser import duration_for_move, validate_command


class VoiceExecutor(Node):
    def __init__(self):
        super().__init__('voice_executor')

        self.declare_parameter('command_topic', '/car_voice/command')
        self.declare_parameter('status_topic', '/car_voice/status')
        self.declare_parameter('say_topic', '/car_voice/say')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('lateral_speed', 0.10)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('stop_repeat_count', 8)
        self.declare_parameter('stop_repeat_interval_sec', 0.05)
        self.declare_parameter('service_timeout_sec', 1.5)
        self.declare_parameter('enable_report', True)
        self.declare_parameter('enable_tts', True)

        self.command_topic = self.get_parameter('command_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.stop_repeat_count = max(
            1, int(self.get_parameter('stop_repeat_count').value)
        )
        self.stop_repeat_interval_sec = max(
            0.01, float(self.get_parameter('stop_repeat_interval_sec').value)
        )
        self.service_timeout_sec = max(
            0.1, float(self.get_parameter('service_timeout_sec').value)
        )
        self.enable_report = bool(self.get_parameter('enable_report').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self.say_pub = self.create_publisher(
            String,
            self.get_parameter('say_topic').value,
            10,
        )
        self.create_subscription(String, self.command_topic, self.command_callback, 10)

        self.lidar_mode_client = self.create_client(
            SetInt64,
            'lidar_control/set_running',
        )
        self.color_enter_client = self.create_client(Trigger, 'color_follow/enter')
        self.color_exit_client = self.create_client(Trigger, 'color_follow/exit')
        self.color_running_client = self.create_client(
            SetBool,
            'color_follow/set_running',
        )
        self.report_client = self.create_client(
            Trigger,
            '/car_report/generate_report',
        )

        self.commands = queue.Queue()
        self.stop_requested = threading.Event()
        self.shutdown_requested = threading.Event()
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.publish_status('ready', 'voice_executor started')

    def command_callback(self, msg):
        try:
            command = validate_command(msg.data)
        except (ValueError, json.JSONDecodeError, TypeError) as exc:
            self.publish_status('rejected', str(exc), raw=msg.data)
            self.say('这个语音指令暂不支持。')
            return

        if command['intent'] == 'emergency_stop':
            self.stop_requested.set()
            self.publish_zero_velocity_burst()
        self.commands.put(command)
        self.publish_status('queued', 'command queued', command=command)

    def worker_loop(self):
        while not self.shutdown_requested.is_set() and rclpy.ok():
            try:
                command = self.commands.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.execute_command(command)
            except Exception as exc:
                self.publish_status('error', str(exc), command=command)
                self.say('语音指令执行失败。')

    def execute_command(self, command):
        intent = command['intent']
        if intent == 'emergency_stop':
            self.emergency_stop()
        elif intent == 'move':
            self.move(command)
        elif intent == 'set_lidar_mode':
            self.set_lidar_mode(command['mode'])
        elif intent == 'set_color_follow':
            self.set_color_follow(command['action'])
        elif intent == 'generate_report':
            self.generate_report()
        elif intent == 'status':
            self.publish_status('ready', '语音模块在线')
            self.say('语音模块在线，可以接收巡逻指令。')
        else:
            raise ValueError(f'unsupported intent: {intent}')

    def emergency_stop(self):
        self.stop_requested.set()
        self.publish_status('running', 'emergency stop')
        self.stop_known_modes()
        self.publish_zero_velocity_burst()
        self.say('已急停。')
        self.publish_status('done', 'emergency stop completed')

    def stop_known_modes(self):
        self.call_lidar_mode(0)
        self.call_color_running(False)

    def move(self, command):
        self.stop_requested.clear()
        self.stop_known_modes()

        twist = Twist()
        direction = command['direction']
        if direction == 'forward':
            twist.linear.x = abs(self.linear_speed)
        elif direction == 'backward':
            twist.linear.x = -abs(self.linear_speed)
        elif direction == 'left':
            twist.linear.y = abs(self.lateral_speed)
        elif direction == 'right':
            twist.linear.y = -abs(self.lateral_speed)
        elif direction == 'turn_left':
            twist.angular.z = abs(self.angular_speed)
        elif direction == 'turn_right':
            twist.angular.z = -abs(self.angular_speed)
        else:
            raise ValueError(f'unsupported direction: {direction}')

        duration = duration_for_move(
            command,
            self.linear_speed,
            self.lateral_speed,
            self.angular_speed,
        )
        self.publish_status(
            'running',
            f'move {direction} for {duration:.2f}s',
            command=command,
        )
        deadline = time.time() + duration
        while time.time() < deadline and not self.stop_requested.is_set():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())

        if self.stop_requested.is_set():
            self.publish_status('interrupted', 'move interrupted', command=command)
            self.say('运动已停止。')
        else:
            self.publish_status('done', 'move completed', command=command)

    def set_lidar_mode(self, mode):
        self.publish_status('running', f'set lidar mode {mode}')
        ok, message = self.call_lidar_mode(mode)
        if ok:
            names = {0: '停止', 1: '避障', 2: '跟随', 3: '警卫'}
            self.say(f'雷达{names.get(mode, "模式")}已设置。')
            self.publish_status('done', f'lidar mode set to {mode}')
        else:
            self.say('雷达模式设置失败。')
            self.publish_status('error', message)

    def set_color_follow(self, action):
        self.publish_status('running', f'set color follow {action}')
        if action == 'start':
            self.call_trigger(self.color_enter_client, 'color_follow/enter')
            ok, message = self.call_color_running(True)
            if ok:
                self.say('视觉目标跟踪已开启。')
                self.publish_status('done', 'color follow started')
            else:
                self.say('视觉目标跟踪开启失败。')
                self.publish_status('error', message)
        else:
            self.call_color_running(False)
            self.call_trigger(self.color_exit_client, 'color_follow/exit')
            self.say('视觉目标跟踪已停止。')
            self.publish_status('done', 'color follow stopped')

    def generate_report(self):
        if not self.enable_report:
            self.publish_status('rejected', 'report generation disabled')
            self.say('报告生成功能未启用。')
            return
        self.publish_status('running', 'generate report')
        request = Trigger.Request()
        ok, message = self.call_service(
            self.report_client,
            request,
            '/car_report/generate_report',
        )
        if ok:
            self.say('巡逻报告已生成。')
            self.publish_status('done', message or 'report generated')
        else:
            self.say('巡逻报告生成失败。')
            self.publish_status('error', message)

    def call_lidar_mode(self, mode):
        request = SetInt64.Request()
        request.data = int(mode)
        return self.call_service(
            self.lidar_mode_client,
            request,
            'lidar_control/set_running',
        )

    def call_color_running(self, running):
        request = SetBool.Request()
        request.data = bool(running)
        return self.call_service(
            self.color_running_client,
            request,
            'color_follow/set_running',
        )

    def call_trigger(self, client, name):
        return self.call_service(client, Trigger.Request(), name)

    def call_service(self, client, request, name):
        if not client.wait_for_service(timeout_sec=min(0.5, self.service_timeout_sec)):
            return False, f'service unavailable: {name}'

        future = client.call_async(request)
        deadline = time.time() + self.service_timeout_sec
        while time.time() < deadline and not future.done() and rclpy.ok():
            time.sleep(0.02)

        if not future.done():
            return False, f'service timeout: {name}'
        result = future.result()
        if result is None:
            return False, f'service failed: {name}'
        success = bool(getattr(result, 'success', True))
        message = str(getattr(result, 'message', ''))
        return success, message

    def publish_zero_velocity_burst(self):
        zero = Twist()
        for _ in range(self.stop_repeat_count):
            self.cmd_vel_pub.publish(zero)
            time.sleep(self.stop_repeat_interval_sec)

    def publish_status(self, state, message, **extra):
        payload = {
            'time': datetime.now().astimezone().isoformat(timespec='seconds'),
            'state': state,
            'message': message,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        if state == 'error':
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def say(self, text):
        if not self.enable_tts:
            return
        msg = String()
        msg.data = str(text)
        self.say_pub.publish(msg)

    def destroy_node(self):
        self.shutdown_requested.set()
        self.stop_requested.set()
        self.publish_zero_velocity_burst()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
