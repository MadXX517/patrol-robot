#!/usr/bin/env python3
# encoding: utf-8
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
import car_vision.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import car_vision.Kinematics as Kinematics
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, box_center, distance, pixels_to_world

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from normalized output of Mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(image corresponding to pixel coordinates)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

class FingerControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.drawing = mp.solutions.drawing_utils
        self.image_queue = queue.Queue(maxsize=2)
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.running = True
        self.fps = fps.FPS()  # fps计算器(FPS calculator)
        self.z_dis = 0.16
        self.y_dis = 500
        self.last_d = 0
        self.bridge = CvBridge()
        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(3)
        self.Arm_controller.set_steer([0.0,-1.0,2.0,0.8,0.0,0.0],duration=1500)
        time.sleep(1.0)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        threading.Thread(target=self.main, daemon=True).start()

    def main(self):
        while self.running:
            t1 = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image_flip = cv2.flip(image, 1)
            bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
            results = self.hand_detector.process(image_flip)
            if results is not None and results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        bgr_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                    try:
                        index_finger_tip = landmarks[8].tolist()
                        thumb_finger_tip = landmarks[4].tolist() 
                        cv2.circle(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.circle(bgr_image, (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.line(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), (0, 255, 255), 5)
                        
                        d = math.sqrt(math.pow(thumb_finger_tip[0] - index_finger_tip[0], 2) + math.pow(thumb_finger_tip[1] - index_finger_tip[1], 2)) 
                        
                        # print(d, d - self.last_d)
                        if abs(d - self.last_d) > 10:
                            if d -self.last_d > 0:
                                self.z_dis -= 0.1
                            else:
                                self.z_dis += 0.1
                            
                            if self.z_dis > 2.0:
                                self.z_dis = 2.0
                            if self.z_dis < 1.6:
                                self.z_dis = 1.6
                            self.Arm_controller.set_steer([0.0,-(self.z_dis/2.0),self.z_dis,self.z_dis/2.0,0.0,0.0],duration=50)
                            t2 = time.time()
                            t = t2 - t1
                            if t < 0.02:
                                time.sleep(0.02 - t)  
                        self.last_d = d
                    except BaseException as e:
                        self.get_logger().info('\033[1;32m%s\033[0m' % e)

            _imshow_fit(self.name, bgr_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press Q or Esc to quit)
                break

        rclpy.shutdown()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

def main():
    node = FingerControlNode('finger_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
