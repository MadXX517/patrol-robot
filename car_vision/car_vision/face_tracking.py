#!/usr/bin/env python3
# encoding: utf-8
import os
import gc
import cv2
import time
import queue
import rclpy
import threading
import numpy as np
import mediapipe as mp
from typing import Tuple, Union
import car_vision.fps as fps
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import car_vision.pid as pid
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera,box_center,distance,point_remapped
from pathlib import Path

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



MARGIN = 10  # pixels
ROW_SIZE = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
FACE_TEXT_COLOR = (255, 255, 0)  # yellow
HANDEDNESS_TEXT_COLOR = (255, 255, 0) # yellow

class FaceTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.pid_yaw = pid.PID(0.25, 0.05, 0.02) # 0.25,0,03,0,02
        self.pid_pitch = pid.PID(0.25, 0.05, 0.02)
        self.detected_face = 0 
        self.yaw_limit = [-1.000,1.000]
        self.pitch_limit = [0.800,1.600] # [1.450,1.850]
        self.yaw = 0.000
        self.pitch = 1.200
        self.running = True
        self.bridge = CvBridge()
        model_path = str(Path(__file__).parent / 'mediapipe' / 'model' / 'detector.tflite')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
        self.detector = vision.FaceDetector.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
    def _normalized_to_pixel_coordinates(self,
        normalized_x: float, normalized_y: float, image_width: int,
        image_height: int) -> Union[None, Tuple[int, int]]:
        """Converts normalized value pair to pixel coordinates."""

    def visualize(self,image, detection_result) -> np.ndarray:
        """Draws bounding boxes and keypoints on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualize.
        Returns:
            Image with bounding boxes.
        """
        annotated_image = image.copy()
        height, width, _ = image.shape
        boxes = []

        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            # print(bbox.origin_x, bbox.origin_y, bbox.width, bbox.height)
            x_min = bbox.origin_x
            y_min = bbox.origin_y
            w = bbox.width
            h = bbox.height
            x_min, y_min = max(x_min, 0), max(y_min, 0)
            x_max, y_max = min(x_min + w, width), min(y_min + h, height)
            boxes.append((x_min, y_min, x_max, y_max))
            cv2.rectangle(annotated_image, start_point, end_point, FACE_TEXT_COLOR, 3)

            # Draw keypoints
            for keypoint in detection.keypoints:
                keypoint_px = self._normalized_to_pixel_coordinates(keypoint.x, keypoint.y,
                                                                width, height)
                color, thickness, radius = (0, 255, 255), 2, 2
                cv2.circle(annotated_image, keypoint_px, thickness, color, radius)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            category_name = '' if category_name is None else category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(annotated_image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, FACE_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

        return annotated_image,boxes

    def proc(self, source_image, detection_result):
        result_image, boxes = self.visualize(source_image, detection_result)
        o_h, o_w = source_image.shape[:2]

        if len(boxes) > 0:
            self.detected_face += 1 
            self.detected_face = min(self.detected_face, 20) # 让计数总是不大于20

            # 连续 5 帧识别到了人脸就开始追踪, 避免误识别
            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes] # 计算所有人脸的中心坐标
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # 计算所有人脸中心坐标到画面中心的距离
                face = min(zip(boxes, center, dist), key=lambda k: k[2]) # 找出到画面中心距离最小的人脸

                # 计算要追踪的人脸距画面中心的x轴距离(0~1)。
                c_x, c_y = face[1]
                dist_x = c_x / o_w
                dist_y = c_y / o_h

                if abs(dist_y - 0.5) > 0.01:
                    self.pid_pitch.SetPoint = 0.5
                    self.pid_pitch.update(dist_y) # 更新俯仰角 pid 控制器
                    self.yaw=self.yaw + self.pid_yaw.output
                    if (self.yaw > self.yaw_limit[1]):
                        self.yaw=self.yaw_limit[1]
                    elif (self.yaw < self.yaw_limit[0]):
                        self.yaw=self.yaw_limit[0]
                else:
                    self.pid_pitch.clear()

                if abs(dist_x - 0.5) > 0.01:
                    self.pid_yaw.SetPoint = 0.5
                    self.pid_yaw.update(dist_x) # 更新偏航角 pid 控制器
                    self.pitch = self.pitch - self.pid_pitch.output
                    if (self.pitch > self.pitch_limit[1]):
                        self.pitch=self.pitch_limit[1]
                    elif (self.pitch < self.pitch_limit[0]):
                        self.pitch=self.pitch_limit[0]
                else:
                    self.pid_yaw.clear()

        else: # 这里是没有识别到人脸的处理
            gc.collect()
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        return result_image, (self.pitch, self.yaw)
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            detection_result = self.detector.detect(mp_image)

            annotated_image, p_y = self.proc(image, detection_result)
            if p_y is not None:
                self.Arm_controller.set_steer([p_y[1],-0.930,1.6,p_y[0],0,0.801],0) #0,-0.930,1.178,1.650,0,0.801) # 0和3号舵机运动，即左右和上下，其他舵机保持不变
                print("p_y",p_y)
            self.fps.update()
            result_image = self.fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            _imshow_fit('face_tracking', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press Q or Esc to quit)
              break

        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    node = FaceTrackingNode('face_tracking')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()

