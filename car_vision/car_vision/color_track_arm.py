#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪(color tracking)
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

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



class ArmColorTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.003, 0.0, 0.0)
        self.pid_dist = pid.PID(0.003, 0.0, 0.00)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.yaw_limit = [-1.000, 1.000]
        self.pitch_limit = [0.800, 1.600]
        self.yaw = 0
        self.pitch = 1.200
        self.endpoint = None
        self.weight_sum = 1.0
        self.x_stop = 320
        self.y_stop = 240
        self.pro_size = (320, 240)

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
        # _imshow_fit('mask', cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
        # cv2.waitKey(1)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积(calculate the area of each contour)
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓(Exclude contours with area too small)
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
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (self.target_rgb[0],
                                                                               self.target_rgb[1],
                                                                               self.target_rgb[2]), 2)
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

class ArmColorTrackerNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = True
        self.threshold = 0.5
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

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
            _imshow_fit("result", result)
            if  not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(Set a callback function for mouse click event)
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        rclpy.shutdown()
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())
    
    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
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

    def image_callback(self, ros_image):
        # 将ros格式(rgb)转为opencv的rgb格式(convert RGB format of ROS to that of OpenCV)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)  # 显示结果用的画面(the image used for display the result)
        with self.lock:
            # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取(color picker and object tracking are mutually exclusive. If the color picker exists, start picking colors)
            if self.color_picker is not None:  # 拾取器存在(color pick exists)
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.tracker = ArmColorTracker(target_color, self)
                    self.get_logger().info("target color: {}".format(target_color))
            else:
                if self.tracker is not None:
                    try:
                        result_image, p_y = self.tracker(rgb_image, result_image, self.threshold)
                        if p_y is not None:
                            self.Arm_controller.set_steer([p_y[1], -0.930, 1.6, p_y[0], 0, 0.801], 0)
                    except Exception as e:
                        self.get_logger().error(str(e))
        
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(result_image)
    
        # 将opencv的格式(bgr)转为ros的rgb格式(convert BGR format of OpenCV to RGB format of ROS)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

def main():
    node = ArmColorTrackerNode('arm_color_track')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



