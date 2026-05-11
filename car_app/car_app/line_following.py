#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
import os
import cv2
import math
import rclpy
import queue
import threading
import numpy as np
import car_vision.pid as pid
import car_vision.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from car_vision.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Image, LaserScan
from interfaces.srv import SetPoint, SetFloat64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        self.rois = ((0.9, 0.92, 0, 1, 0.7), (0.78, 0.80, 0, 1, 0.2), (0.66, 0.68, 0, 1, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        w = w + 200
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
        return result_image, deflection_angle

class LineFollowingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.name = name
        self.set_callback = False
        self.is_running = True
        self.color_picker = None
        self.follower = None
        self.pid = pid.PID(2.0, 0.0, 0.01)
        self.empty = 0
        self.count = 0
        self.stop = False
        self.threshold = 0.5
        self.stop_threshold = 0.4
        self.lock = threading.RLock()
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        #self.camera_type = os.environ['DEPTH_CAMERA_TYPE']
        self.declare_parameter('machine_type', 'Mec', ParameterDescriptor(name='machine_type', description='Mec,Ack'))
        self.machine_type = self.get_parameter('machine_type').value #麦轮：Mec,阿克曼：Ack
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)  # 底盘控制(chassis control)
        self.result_publisher = self.create_publisher(Image, 'line_follow/image_result', 1)  # 图像处理结果发布(publish the image processing result)
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
            cv2.imshow("result", result)
            if not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(set a callback function for mouse click event)
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
        self.pub_vel.publish(Twist())
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
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
        with self.lock:
            x, y = request.data.x, request.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(request.data, 5)
                self.pub_vel.publish(Twist())
        response.success = True
        response.message = "set_target_color"
        return response
        
    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        result_image = np.copy(rgb_image)  # 显示结果用的画面 (the image used to display the result)
        with self.lock:
            # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
            if self.color_picker is not None:  # 拾取器存在(color picker exists)
                try:
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.follower = LineFollower(target_color, self)
                        self.get_logger().info("target color: {}".format(target_color))
                except Exception as e:
                    self.get_logger().error(str(e))
            else:
                twist = Twist()
                twist.linear.x = 0.12
                if self.follower is not None:
                    try:
                        result_image, deflection_angle = self.follower(rgb_image, result_image, self.threshold)
                        if deflection_angle is not None and self.is_running and not self.stop:
                            self.pid.update(deflection_angle)
                            twist.angular.z = common.set_range(-self.pid.output, -0.37, 0.37)
                            self.pub_vel.publish(twist)
                        elif self.stop:
                            self.pub_vel.publish(Twist())
                        else:
                            self.pid.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(result_image)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

def main():
    node = LineFollowingNode('line_following')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


