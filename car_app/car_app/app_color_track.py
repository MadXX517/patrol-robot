#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪(color tracking)，支持红、绿、蓝三种颜色选择

import os
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
import car_vision.pid as pid
import car_vision.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from car_vision.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

# 定义红、绿、蓝三种颜色的LAB和RGB值
COLORS = {
    "red": {
        "lab": (50, 80, 60),
        "rgb": (255, 0, 0),
        "name": "red"
    },
    "green": {
        "lab": (50, -80, 60),
        "rgb": (0, 255, 0),
        "name": "green"
    },
    "blue": {
        "lab": (50, 0, -60),
        "rgb": (0, 0, 255),
        "name": "blue"
    }
}

class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.003, 0.0, 0.0)
        self.pid_dist = pid.PID(0.003, 0.0, 0.00)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color["lab"], color["rgb"]
        self.yaw_limit = [-1.000, 1.000]
        self.pitch_limit = [0.800, 1.600]
        self.yaw = 0
        self.pitch = 1.200
        self.endpoint = None
        self.weight_sum = 1.0
        self.x_stop = 320
        self.y_stop = 300
        self.pro_size = (320, 180)

    def __call__(self, image, result_image, threshold):
        h, w = image.shape[:2]
        image = cv2.resize(image, self.pro_size)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间(convert RGB to LAB space)
        image = cv2.GaussianBlur(image, (5, 5), 5)

        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(binarization)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除面积过小的轮廓
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / self.pro_size[0] * w
            y = y / self.pro_size[1] * h
            r = r / self.pro_size[0] * w

            cv2.circle(result_image, (self.x_stop, self.y_stop), 5, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), self.target_rgb, 2)
            vx = 0
            vw = 0
            if abs(y - self.y_stop) > 20:
                self.pid_dist.update(y - self.y_stop)
                self.pitch = common.set_range(self.pitch-self.pid_dist.output*0.1, 0.8, 1.6)
            else:
                self.pid_dist.clear()
            if abs(x - self.x_stop) > 20:
                self.pid_yaw.update(x - self.x_stop)
                self.yaw = common.set_range(self.yaw-self.pid_yaw.output*0.1, -1.0, 1.0)
            else: 
                self.pid_yaw.clear()

        return result_image, (self.pitch, self.yaw)

class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.Arm_controller=arm_ik_sdk.ArmControl()
        self.debug = False
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.5
        self.dist_threshold = 0.3
        self.current_color = None  # 当前追踪的颜色
        self.lock = threading.RLock()
        self.image_sub_d = None
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        
        self.enter_srv = self.create_service(Trigger, 'color_follow/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, 'color_follow/exit', self.exit_srv_callback)
        self.set_running_srv = self.create_service(SetBool, 'color_follow/set_running', self.set_running_srv_callback)
        self.set_target_color_srv = self.create_service(SetPoint, 'color_follow/set_target_color', self.set_target_color_srv_callback)
        self.get_target_color_srv = self.create_service(Trigger, 'color_follow/set_red', self.set_target_color_red_callback)  # 修改服务
        self.set_threshold_srv = self.create_service(SetFloat64, 'color_follow/set_threshold', self.set_threshold_srv_callback)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.result_publisher = self.create_publisher(Image, 'color_follow/image_result',  1)
        
        if self.debug:
            threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start, support red/green/blue tracking')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)
            if  not self.set_callback:
                self.set_callback = True
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        rclpy.shutdown()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking enter')
        with self.lock:
            self.is_running = False
            self.threshold = 0.5
            self.tracker = None
            self.color_picker = None
            self.dist_threshold = 0.3
            self.current_color = None
            time.sleep(2)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
            time.sleep(1)
            if self.image_sub_d is None:
                self.image_sub_d = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
            if self.image_sub is None:
                self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking exit')
        try:
            if self.image_sub_d is not None:
                self.destroy_subscription(self.image_sub_d)
                self.image_sub_d = None
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        with self.lock:
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.threshold = 0.5
            self.dist_threshold = 0.3
            self.current_color = None
            self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
            time.sleep(1)
        response.success = True
        response.message = "exit"
        return response    
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())
    
    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color by pixel')
        with self.lock:
            x, y = request.data.x, request.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(request.data, 10)
            self.pub_vel.publish(Twist())
        response.success = True
        response.message = "set_target_color"
        return response
    
    def set_target_color_red_callback(self, request, response):
        """red_track"""
        self.get_logger().info(f'\033[1;32mSet target color to: red\033[0m')
        
        with self.lock:
            self.is_running = True
            self.current_color = COLORS[0]
            self.tracker = ObjectTracker(self.current_color, self)
            self.color_picker = None
            response.success = True
            response.message = f"Set target color to red"
            self.get_logger().info(f"Target color set to red, LAB: {self.current_color['lab']}, RGB: {self.current_color['rgb']}")
        
        return response
    
    def set_target_color_green_callback(self, request, response):
        """green_track"""
        self.get_logger().info(f'\033[1;32mSet target color to: green\033[0m')
        
        with self.lock:
            self.is_running = True
            self.current_color = COLORS[1]
            self.tracker = ObjectTracker(self.current_color, self)
            self.color_picker = None
            response.success = True
            response.message = f"Set target color to green"
            self.get_logger().info(f"Target color set to green, LAB: {self.current_color['lab']}, RGB: {self.current_color['rgb']}")
        
        return response
    
    def set_target_color_blue_callback(self, request, response):
        """blue_track"""
        self.get_logger().info(f'\033[1;32mSet target color to: blue\033[0m')
        
        with self.lock:
            self.is_running = True
            self.current_color = COLORS[2]
            self.tracker = ObjectTracker(self.current_color, self)
            self.color_picker = None
            response.success = True
            response.message = f"Set target color to blue"
            self.get_logger().info(f"Target color set to blue, LAB: {self.current_color['lab']}, RGB: {self.current_color['rgb']}")
        
        return response
    
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_running')
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.pub_vel.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response
    
    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'threshold')
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)
        with self.lock:
            # 如果设置了颜色拾取器，优先使用拾取器
            if self.color_picker is not None:
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.tracker = ObjectTracker(target_color, self)
                    self.get_logger().info("target color: {}".format(target_color))
            else:
                # 否则使用预设的颜色追踪器
                if self.tracker is not None:
                    try:
                        result_image, p_y = self.tracker(rgb_image, result_image, self.threshold)
                        if self.is_running:
                            self.Arm_controller.set_steer([p_y[1], -0.930, 1.6, p_y[0], 0, 0.801], 0)
                        else:
                            self.tracker.pid_dist.clear()
                            self.tracker.pid_yaw.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        img_h, img_w = result_image.shape[:2]
        center_x, center_y = img_w // 2, img_h // 2
        cross_length = 20  # 十字准星长度
        cv2.line(result_image, (center_x - cross_length, center_y), 
                    (center_x + cross_length, center_y), (0, 0, 255), 2)  # 水平线（红色）
        cv2.line(result_image, (center_x, center_y - cross_length), 
                    (center_x, center_y + cross_length), (0, 0, 255), 2)  # 垂直线（红色）
        
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(result_image)
    
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()