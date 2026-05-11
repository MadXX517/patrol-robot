#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from pathlib import Path
import apriltag
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from interfaces.srv import SetString
from car_vision.obj_loader import OBJ as obj_load
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
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



parent_dir = Path(__file__).parent
MODEL_PATH = parent_dir / 'models'

OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

AXIS = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])

MODELS_SCALE = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }

def draw_rectangle(img, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img

class ARNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        time.sleep(1)
        # 声明并获取模型参数(declare and get model parameter)
        self.declare_parameter('model_name', 'wolf')
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        # 验证模型参数有效性(validate model parameter)
        if model_name not in MODELS_SCALE and model_name != 'rectangle':
            raise ValueError(f"Invalid model name: {model_name}")

        self.target_model = model_name
        self.obj = None
        
        # 如果指定了3D模型则加载(load 3D model if specified)
        if self.target_model != 'rectangle' and self.target_model in MODELS_SCALE:
            self._load_3d_model()

        self.camera_intrinsic = np.matrix([[619.063979, 0, 302.560920],
                                         [0, 613.745352, 237.714934],
                                         [0, 0, 1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)
        self.create_subscription(CameraInfo, '/usb_cam/camera_info', self.camera_info_callback, 1)
        self.bridge = CvBridge()
        options = apriltag.DetectorOptions(families="tag36h11")
        self.tag_detector = apriltag.Detector(options)
        # self.tag_detector = apriltag("tag36h11")
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32mAR节点已启动\033[0m')

    def _load_3d_model(self):
        """加载3D模型资源"""
        obj = obj_load(MODEL_PATH / (self.target_model + '.obj'), swapyz=True)
        obj.faces = obj.faces[::-1]
        new_faces = []
        
        for face in obj.faces:
            face_vertices = face[0]
            points = []
            colors = []
            for vertex in face_vertices:
                data = obj.vertices[vertex - 1]
                points.append(data[:3])
                if self.target_model not in ['cow', 'wolf']:
                    colors.append(data[3:])
            
            # 应用缩放(apply scaling)
            scale_matrix = np.eye(3) * MODELS_SCALE[self.target_model]
            points = np.dot(np.array(points), scale_matrix)
            
            # 应用模型特定旋转(apply model-specific rotation)
            if self.target_model == 'bicycle':
                points = points - np.array([670, 350, 0])
                points = R.from_euler('z', 180, degrees=True).apply(points)
            elif self.target_model in ['fox', 'chair']:
                points = R.from_euler('z', -90, degrees=True).apply(points)
            
            # 处理颜色(process color)
            color = tuple(255 * np.array(colors[0])) if colors else None
            new_faces.append((points, color))
        
        self.obj = new_faces

    def get_node_state(self, request, response):
        response.success = True
        return response

    def camera_info_callback(self, msg):
        self.camera_intrinsic = np.array(msg.k).reshape(3, -1)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        result_image = np.copy(rgb_image)
        
        try:
            result_image = self._process_image(rgb_image, result_image)
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
        
        _imshow_fit("AR Result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

    def _process_image(self, rgb_input, output_image):
        if self.target_model is None:
            return output_image
            
        gray = cv2.cvtColor(rgb_input, cv2.COLOR_RGB2GRAY)
        detections = self.tag_detector.detect(gray)
        
        for detection in detections:
            tag_corners = detection.corners
            lb = tag_corners[3]  # 左下
            rb = tag_corners[2]  # 右下
            lt = tag_corners[0]  # 左上
            rt = tag_corners[1]  # 右上
            center = np.mean(tag_corners, axis=0)
            corners = np.array([lb, rb, lt, rt, center])
            # corners = np.array([
            #     detection['lb-rb-rt-lt'][0],  # lb
            #     detection['lb-rb-rt-lt'][1],  # rb
            #     detection['lb-rb-rt-lt'][3],  # lt 
            #     detection['lb-rb-rt-lt'][2],  # rt
            #     detection['center']
            # ])
            
            # 计算姿态估计(solve PnP)
            ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, 
                                           self.camera_intrinsic, 
                                           self.dist_coeffs)
            
            if self.target_model == 'rectangle':
                imgpts, _ = cv2.projectPoints(AXIS, rvecs, tvecs,
                                            self.camera_intrinsic, 
                                            self.dist_coeffs)
                output_image = draw_rectangle(output_image, imgpts)
            else:
                self._render_3d_model(output_image, rvecs, tvecs)
                
        return output_image

    def _render_3d_model(self, image, rvecs, tvecs):
        for points, color in self.obj:
            points = points.reshape(-1, 1, 3) / 100.0  # 单位转换(unit conversion)
            imgpts, _ = cv2.projectPoints(points, rvecs, tvecs,
                                        self.camera_intrinsic,
                                        self.dist_coeffs)
            imgpts = imgpts.astype(int)
            
            # 根据模型类型填充颜色(fill color based on model type)
            if self.target_model == 'cow':
                cv2.fillConvexPoly(image, imgpts, (0, 255, 255))
            elif self.target_model == 'wolf':
                cv2.fillConvexPoly(image, imgpts, (255, 255, 0))
            else:
                cv2.fillConvexPoly(image, imgpts, color)

def main():
    rclpy.init()
    node = ARNode('ar_app')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
