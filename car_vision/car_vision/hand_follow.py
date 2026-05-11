#!/usr/bin/env python3
# encoding: utf-8
import cv2
import enum
import time
import rclpy
import os
import queue
import threading
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from car_vision.common import vector_2d_angle, distance
from interfaces.msg import Points, PixelPosition
from mediapipe.framework.formats import landmark_pb2
from geometry_msgs.msg import Twist
from car_msg.msg import Beep
import car_vision.pid as pid
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world
from pathlib import Path

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (255, 255, 0) # yellow

def get_hand_landmarks(img_size, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the bending angle of each finger)
    :param landmarks: 手部关键点(the key points of the hand)
    :return: 各个手指的角度(the angle of each angle)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list

def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(Determine finger gestures by 2D features)
    :param angle_list: 各个手指弯曲的角度(the bending angle of each finger)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle = 65.
    thr_angle_thumb = 10.
    thr_angle_s = 49.
    gesture_str = "none"
    # print(angle_list[0], angle_list[1], angle_list[2], angle_list[3], angle_list[4])
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str

def draw_points(img, points, thickness=4, color=(200, 200, 0)):
    points = np.array(points).astype(dtype=int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

def get_palm_area(landmarks):
    # 选择代表手掌轮廓的关键点
    palm_landmark_indices = [0, 5, 9, 13, 17]
    palm_points = []
    for index in palm_landmark_indices:
        landmark = landmarks[index]
        x = int(landmark[0])
        y = int(landmark[1]) 
        palm_points.append([x, y])

    # 将关键点转换为 NumPy 数组
    palm_points = np.array(palm_points)

    # 使用 Shoelace 公式计算多边形的面积
    area = 0.5 * np.abs(np.dot(palm_points[:, 0], np.roll(palm_points[:, 1], 1)) -
                        np.dot(palm_points[:, 1], np.roll(palm_points[:, 0], 1)))
    return area

class pid_tracker:
    def __init__(self):
        self.pid_pitch = pid.PID(0.15, 0.0, 0.0)
        self.pid_yaw = pid.PID(0.15, 0.0, 0.009)
        self.detected_face = 0 
        self.pitch_limit = [-0.10,0.10]
        self.yaw_limit = [-0.50,0.50]
        self.yaw = 0.000
        self.pitch = 0.000
    
    def track(self,pos,width,height):
        c_x, c_y = pos
        dist_x = c_x / width
        dist_y = c_y / 5000

        if abs(dist_y - 1.0) > 0.10:
            self.pid_pitch.SetPoint = 1.0
            self.pid_pitch.update(dist_y) 
            self.pitch = self.pitch - self.pid_pitch.output
            if (self.pitch > self.pitch_limit[1]):
                self.pitch=self.pitch_limit[1]
            elif (self.pitch < self.pitch_limit[0]):
                self.pitch=self.pitch_limit[0]
        else:
            self.pid_pitch.clear()
            self.pitch=0

        if abs(dist_x - 0.5) > 0.03:
            self.pid_yaw.SetPoint = 0.5
            self.pid_yaw.update(dist_x)
            self.yaw=self.yaw + self.pid_yaw.output
            if (self.yaw > self.yaw_limit[1]):
                self.yaw=self.yaw_limit[1]
            elif (self.yaw < self.yaw_limit[0]):
                self.yaw=self.yaw_limit[0]
        else:
            self.pid_yaw.clear()
            self.yaw=0.0

        return (self.pitch, self.yaw)

class HandFollowNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.HandTracker=pid_tracker()
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.name = name
        self.start = True
        self.running = True
        self.image = None
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.last_tracker_point=None
        self.lose_count=0
        self.hand_area=0
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.count_miss = 0
        self.last_point = [0, 0]
        model_path = str(Path(__file__).parent / 'mediapipe' / 'model' / 'hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(base_options=base_options, min_hand_detection_confidence=0.2, num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(options)

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 5)
        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the result of image processing) 
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def beep_open(self):
        msg = Beep()
        msg.times = 2
        msg.on_time = 0.1
        msg.off_time = 0.1
        self.beep_pub.publish(msg)

    def image_proc(self):
        points_list = []
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            annotated_image = image.copy()
            twist=Twist()
            p_y=None
            if self.start:
                try:
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
                    detection_result = self.detector.detect(mp_image)
                    hand_landmarks_list = detection_result.hand_landmarks
                    handedness_list = detection_result.handedness
                    if len(hand_landmarks_list) > 0:
                        self.count_miss = 0
                        gesture = "none"
                        hand_center = [0, 0]
                        for idx in range(len(hand_landmarks_list)):
                            hand_landmarks = hand_landmarks_list[idx]
                            handedness = handedness_list[idx]

                            # Draw the hand landmarks.
                            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                            hand_landmarks_proto.landmark.extend([
                              landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
                            ])
                            mp.solutions.drawing_utils.draw_landmarks(
                              annotated_image,
                              hand_landmarks_proto,
                              mp.solutions.hands.HAND_CONNECTIONS,
                              mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                              mp.solutions.drawing_styles.get_default_hand_connections_style())
                            # Get the top left corner of the detected hand's bounding box.
                            height, width, _ = annotated_image.shape
                            p1 = hand_landmarks[0]
                            p2 = hand_landmarks[2]
                            p3 = hand_landmarks[9]
                            p4 = hand_landmarks[17]
                            hand_center = [int((p1.x + p2.x + p3.x + p4.x)/4*width), int((p1.y + p2.y + p3.y + p4.y)/4*height)]                            
                            cv2.circle(annotated_image, tuple(hand_center), 10, (255, 255, 0), -1)
                            landmarks = np.array([[landmark.x*width, landmark.y*height] for landmark in hand_landmarks])
                            self.hand_area=get_palm_area(landmarks)
                        p_y = self.HandTracker.track([hand_center[0],self.hand_area],width,height)
                        self.last_tracker_point=hand_center
                        self.lose_count=0
                    elif self.last_tracker_point and self.lose_count < 10:
                        # p_y = self.HandTracker.track(self.last_tracker_point,ros_image.width,ros_image.height)
                        self.lose_count+=1
                    if p_y is not None:
                        twist.linear.x= float(-p_y[0])
                        twist.angular.z=float(-p_y[1])
                        self.pub_vel.publish(twist)
                        self.last_p_y=p_y
                    else:
                        self.pub_vel.publish(Twist())
                except Exception as e:
                    print(e)
            else:
                time.sleep(0.01)
            _imshow_fit('annotated_image', cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) == ord('q'):  # 退出键q
                self.pub_vel.publish(Twist())
                self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
                self.destroy_node()
                rclpy.shutdown()
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "rgb8"))


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

def main():
    node = HandFollowNode('hand_trajectory')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
