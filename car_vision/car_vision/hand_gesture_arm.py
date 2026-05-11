#!/usr/bin/env python3
# coding: utf8

import os
import sys
import enum
import cv2
import time
import math
import numpy as np
import queue
import threading
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node
import rclpy
from car_msg.msg import Beep
from car_vision.common import vector_2d_angle, distance
from mediapipe.framework.formats import landmark_pb2
from geometry_msgs.msg import Twist
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera
import car_vision.pid as pid
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
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    # print(angle_list[0], angle_list[1], angle_list[2], angle_list[3], angle_list[4])
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
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
        gesture_str = "three"
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

def draw_points(img, points, thickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

def get_track_img(points):
    points = np.array(points).astype(dtype=np.int)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full([y_max - y_min + 100, x_max - x_min + 100, 1], 0, dtype=np.uint8)
    points = points - [x_min, y_min]
    points = points + [50, 50]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

class HandGestureNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.Arm_controller=arm_ik_sdk.ArmControl(self)  # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(3)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.start = True
        self.running = True
        self.image = None
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.count_miss = 0
        self.last_point = [0, 0]
        self.last_gesture = "none"
        model_path = str(Path(__file__).parent / 'mediapipe' / 'model' / 'hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(base_options=base_options, min_hand_detection_confidence=0.2, num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(options)

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.drawing = mp.solutions.drawing_utils
        #self.camera_type = os.environ['DEPTH_CAMERA_TYPE']
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
    
    def do_act(self, gesture):
        if gesture == "fist":
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0.6,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,-0.6,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801],duration=300)
            time.sleep(0.3)
        elif gesture == "five":
            self.Arm_controller.set_steer([0.6,-0.930,1.6,1.200,0,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([-0.6,-0.930,1.6,1.200,0,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801],duration=300)
            time.sleep(0.3)
        elif gesture == "one":
            self.Arm_controller.set_steer([0,-0.930,1.6,1.600,0,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([0,-0.930,1.6,0.800,0,0.801],duration=600)
            time.sleep(0.6)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801],duration=300)
            time.sleep(0.3)
        elif gesture == "two":
            self.Arm_controller.set_steer([0.0,-0.8,1.6,-0.8,0.0,0.0],duration=1500)
            time.sleep(1.5)
            self.Arm_controller.set_steer([0.0,-0.8,1.6,-0.8,0.0,-0.4],duration=700)
            time.sleep(0.7)
            self.Arm_controller.set_steer([0.0,-0.8,1.6,-0.8,0.0,0.4],duration=700)
            time.sleep(0.7)
            self.Arm_controller.set_steer([0.0,-0.8,1.6,-0.8,0.0,-0.4],duration=700)
            time.sleep(0.7)
            self.Arm_controller.set_steer([0.0,-0.8,1.6,-0.8,0.0,0.4],duration=1000)
            time.sleep(1.0)
            self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801],duration=1500)
            time.sleep(1.5)
        elif gesture == "three":
            for i in range(2):
                self.Arm_controller.set_steer([-0.4,-0.3,0.6,1.2,0.0,0.0],duration=500)
                time.sleep(0.5)
                self.Arm_controller.set_steer([0.4,-0.3,0.6,1.2,0.0,0.0],duration=500)
                time.sleep(0.5)
                self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801],duration=500)
        time.sleep(1)
        self.count = 0
        self.last_gesture = "none"
        self.state = State.NULL
        

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
            if self.start:
                try:
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
                    detection_result = self.detector.detect(mp_image)
                    hand_landmarks_list = detection_result.hand_landmarks
                    handedness_list = detection_result.handedness
                    if len(hand_landmarks_list) > 0:
                        self.count_miss = 0
                        gesture = "none"
                        index_finger_tip = [0, 0]
                        self.no_finger_timestamp = time.time()  # 记下当期时间，以便超时处理
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
                            landmarks = np.array([[landmark.x*width, landmark.y*height] for landmark in hand_landmarks])
                            angle_list = (hand_angle(landmarks))
                            gesture = (h_gesture(angle_list))
                            index_finger_tip = landmarks[8].tolist()
                            #cv2.circle(annotated_image, tuple(index_finger_tip), 10, (255, 255, 0), -1)
                        cv2.putText(annotated_image, "gesture:" + str(gesture), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                        if self.state != State.RUNNING:
                            if gesture == self.last_gesture and gesture != "none":
                                self.count += 1
                            else:
                                self.count = 0
                            if self.count > 10:
                                self.state = State.RUNNING
                                threading.Thread(target=self.do_act, args=(gesture, )).start()
                        else:
                            self.count = 0
                        self.last_gesture = gesture

                    else:
                        if self.state != State.NULL:
                            if time.time() - self.no_finger_timestamp > 2:
                                self.one_count = 0
                                self.points = [[0, 0],]
                                self.state = State.NULL
                                self.last_gesture = "none"
                except Exception as e:
                    print(e)
            else:
                time.sleep(0.01)
            _imshow_fit('annotated_image', cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
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
    node = HandGestureNode('hand_trajectory')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
