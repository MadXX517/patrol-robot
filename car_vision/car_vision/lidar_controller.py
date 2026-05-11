#!/usr/bin/env python3
# encoding: utf-8
# 雷达避障，跟随功能(lidar obstacle avoidance and lidar following function)
import os
import math
import time
import rclpy
import threading
import numpy as np
import pandas as pd
import car_vision.pid as pid
import car_vision.common as common
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from interfaces.srv import SetInt64, SetFloat64List
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from car_msg.msg import Beep

CAR_WIDTH = 0.4  # meter
MAX_SCAN_ANGLE = 180  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of Lidar. The covered part is always eliminated)

class LidarController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.threshold = 0.5  # meters
        self.scan_angle = math.radians(90)  # radians
        self.speed = 0.2
        self.last_act = 0
        self.timestamp = 0
        self.angle_data = []
        # pid参数
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        self.lock = threading.RLock()
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)  # 底盘控制(chassis control)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.declare_parameter('machine_type', 'Mec', ParameterDescriptor(name='machine_type', description='Mec,Ack'))
        self.machine_type = self.get_parameter('machine_type').value #麦轮：Mec,阿克曼：Ack
        self.declare_parameter('running_mode', 0, ParameterDescriptor(name='running_mode', description='0 close, 1 obstacle avoidance, 2 Lidar following, 3 Lidar guarding'))
        self.running_mode = self.get_parameter('running_mode').value #0关闭，1避障，2跟随，3警卫(0 close, 1 obstacle avoidance, 2 Lidar following, 3 Lidar guarding)
    
    def beep_open(self):
        msg = Beep()
        msg.times = 2
        msg.on_time = 0.3
        msg.off_time = 0.2
        self.beep_pub.publish(msg)

    def lidar_callback(self, lidar_data):
        # 雷达订阅回调(Lidar subscription callback)
        twist = Twist()
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
        right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)
        with self.lock:
            # 根据设定取数据 (get the data according to the settings)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])
            if self.machine_type != 'Ack':
                if self.running_mode == 1 and self.timestamp <= time.time():
                    left_nonzero = left_range.nonzero()
                    right_nonzero = right_range.nonzero()
                    left_nonan = np.isfinite(left_range[left_nonzero])
                    right_nonan = np.isfinite(right_range[right_nonzero])
                    min_dist_left_ = left_range[left_nonzero][left_nonan]
                    min_dist_right_ = right_range[right_nonzero][right_nonan]
                    # 取左右最近的距离(Take the nearest distance left and right)
                    if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                        min_dist_left = min_dist_left_.min()
                        min_dist_right = min_dist_right_.min()
                        if min_dist_left <= self.threshold and min_dist_right > self.threshold:  # 左侧有障碍(there is obstacle at left)
                            twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            max_angle = math.radians(90)  # 转90度(turn 90 degree)
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = -w
                            if self.last_act != 0 and self.last_act != 1:
                                twist.angular.z = w
                            self.last_act = 1
                            self.pub_vel.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # 两侧都有障碍
                            twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = w
                            self.last_act = 3
                            self.pub_vel.publish(twist)
                            # 转180度
                            self.timestamp = time.time() + (math.radians(180) / w / 2)
                        elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # 右侧有障碍(there is obstacles at right)
                            twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            max_angle = math.radians(90)  # 转90度 turn 90 degrees
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = w
                            if self.last_act != 0 and self.last_act != 2:
                                twist.angular.z = -w
                            self.last_act = 2
                            self.pub_vel.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        else:  # 没有障碍(none obstacle)
                            self.last_act = 0
                            twist.linear.x = self.speed  # 直行
                            self.pub_vel.publish(twist)
                elif self.running_mode == 2:
                    # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                    ranges = np.append(right_range[::-1], left_range)
                    self.get_logger().info(str(ranges))
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 0:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                        # self.get_logger().info(str(dist_))
                        # self.get_logger().info(str([dist, angle]))
                        if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                            self.pid_yaw.update(-angle)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0) 
                        else:
                            self.pid_yaw.clear()

                        if dist < self.threshold and abs(0.2 - dist) > 0.02:  # 控制前后(control the front and back)
                            self.pid_dist.update(self.threshold / 2 - dist)
                            twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                        else:
                            self.pid_dist.clear()
                        if abs(twist.angular.z) < 0.008:
                            twist.angular.z = 0.0
                        if abs(twist.linear.x) < 0.05:
                            twist.linear.x = 0.0
                        self.pub_vel.publish(twist)
                elif self.running_mode == 3:
                    # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                    ranges = np.append(right_range[::-1], left_range)
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 1:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        
                        angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                        if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                            self.pid_yaw.update(-angle)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                            threading.Thread(target=self.beep_open).start()
                        else:
                            self.pid_yaw.clear()
                        if abs(twist.angular.z) < 0.008:
                            twist.angular.z = 0.0
                        self.pub_vel.publish(twist)
            else:
                if self.running_mode == 1 and self.timestamp <= time.time():
                    left_nonzero = left_range.nonzero()
                    right_nonzero = right_range.nonzero()
                    left_nonan = np.isfinite(left_range[left_nonzero])
                    right_nonan = np.isfinite(right_range[right_nonzero])
                    min_dist_left_ = left_range[left_nonzero][left_nonan]
                    min_dist_right_ = right_range[right_nonzero][right_nonan]
                    # 取左右最近的距离(Take the nearest distance left and right)
                    if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                        min_dist_left = min_dist_left_.min()
                        min_dist_right = min_dist_right_.min()
                        if min_dist_left <= self.threshold and min_dist_right > self.threshold:  # 左侧有障碍(there is obstacle at left)
                            twist.linear.x = self.speed / 2  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            max_angle = math.radians(90)  # 转90度(turn 90 degree)
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = -w
                            if self.last_act != 0 and self.last_act != 1:
                                twist.angular.z = w
                            self.last_act = 1
                            self.pub_vel.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # 两侧都有障碍
                            twist.linear.x = self.speed / 2  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = w
                            self.last_act = 3
                            self.pub_vel.publish(twist)
                            # 转180度
                            self.timestamp = time.time() + (math.radians(180) / w / 2)
                        elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # 右侧有障碍(there is obstacles at right)
                            twist.linear.x = self.speed / 2  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                            max_angle = math.radians(90)  # 转90度 turn 90 degrees
                            w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                            twist.angular.z = w
                            if self.last_act != 0 and self.last_act != 2:
                                twist.angular.z = -w
                            self.last_act = 2
                            self.pub_vel.publish(twist)
                            self.timestamp = time.time() + (max_angle / w / 2)
                        else:  # 没有障碍(none obstacle)
                            self.last_act = 0
                            twist.linear.x = self.speed  # 直行
                            self.pub_vel.publish(twist)
                elif self.running_mode == 2:
                    # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                    ranges = np.append(right_range[::-1], left_range)
                    self.get_logger().info(str(ranges))
                    nonzero = ranges.nonzero()
                    nonan = np.isfinite(ranges[nonzero])
                    dist_ = ranges[nonzero][nonan]
                    if len(dist_) > 0:
                        dist = dist_.min()
                        min_index = list(ranges).index(dist)
                        angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                        # self.get_logger().info(str(dist_))
                        # self.get_logger().info(str([dist, angle]))
                        if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                            self.pid_yaw.update(-angle)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0) 
                        else:
                            self.pid_yaw.clear()

                        if dist < self.threshold and abs(0.2 - dist) > 0.02:  # 控制前后(control the front and back)
                            self.pid_dist.update(self.threshold / 2 - dist)
                            twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                        else:
                            self.pid_dist.clear()
                        if abs(twist.angular.z) < 0.008:
                            twist.angular.z = 0.0
                        if abs(twist.linear.x) < 0.05:
                            twist.linear.x = 0.0
                        self.pub_vel.publish(twist)

def main():
    node = LidarController('lidar_app')
    try:
        rclpy.spin(node)  # 循环等待ROS2退出
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C pressed, shutting down...')
        # 发布停止指令
        twist = Twist()
        node.pub_vel.publish(twist)
        # 销毁节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
