#!/usr/bin/env python3
# encoding: utf-8
import cv2
import time
import rclpy
import queue
import signal
import threading
import numpy as np
import faulthandler
import car_vision.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from car_msg.msg import Beep
import car_vision.arm_ik_sdk as arm_ik_sdk

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



faulthandler.enable()

mp_pose = mp.solutions.pose

LEFT_SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER
LEFT_ELBOW = mp_pose.PoseLandmark.LEFT_ELBOW
LEFT_WRIST = mp_pose.PoseLandmark.LEFT_WRIST
LEFT_HIP = mp_pose.PoseLandmark.LEFT_HIP

RIGHT_SHOULDER = mp_pose.PoseLandmark.RIGHT_SHOULDER
RIGHT_ELBOW = mp_pose.PoseLandmark.RIGHT_ELBOW
RIGHT_WRIST = mp_pose.PoseLandmark.RIGHT_WRIST
RIGHT_HIP = mp_pose.PoseLandmark.RIGHT_HIP

LEFT_KNEE = mp_pose.PoseLandmark.LEFT_KNEE
LEFT_ANKLE = mp_pose.PoseLandmark.LEFT_ANKLE

RIGHT_KNEE = mp_pose.PoseLandmark.RIGHT_KNEE
RIGHT_ANKLE = mp_pose.PoseLandmark.RIGHT_ANKLE

def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def height_cal(landmarks):
    y = []
    for i in landmarks:
        y.append(i[1])
    height = sum(y)/len(y)

    return height

class FallDownDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.7,
            min_detection_confidence=0.7)
        self.running = True
        self.fps = fps.FPS()  # fps计算器(FPS calculator)
        
        self.fall_down_count = []
        signal.signal(signal.SIGINT, self.shutdown)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)
        self.create_subscription(Image, '/camera/color/image_raw' , self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw' , self.image_callback, 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)  # 底盘控制(chassis control)

        self.pub_vel.publish(Twist())

        threading.Thread(target=self.main, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def shutdown(self, signum, frame):
        self.running = False

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)

    def beep_open(self):
        msg = Beep()
        msg.times = 2
        msg.on_time = 0.3
        msg.off_time = 0.2
        self.beep_pub.publish(msg)

    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks:
            landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
            h = height_cal(landmarks)
            if h > image.shape[:-2][0] - 120:
                self.fall_down_count.append(1)
            else:
                self.fall_down_count.append(0)
            if len(self.fall_down_count) == 3:
                count = sum(self.fall_down_count)
                
                self.fall_down_count = []
                if count <= 1:
                    print("sure")
                else:
                    print("fall down")
                    threading.Thread(target=self.beep_open).start()

            result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.drawing.draw_landmarks(
                result_image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS)
            return cv2.flip(result_image, 1)
        else:
            return image_flip

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
                result_image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            _imshow_fit(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press Q or Esc to exit)
                self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
                self.running = False

        rclpy.shutdown()

def main():
    node = FallDownDetectNode('fall_down_detect')
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
