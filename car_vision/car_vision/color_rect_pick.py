#!/usr/bin/env python3
# coding=utf-8

import cv2
import rclpy
from rclpy.node import Node
import sys
import numpy as np
import threading
import math
import time
import queue
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
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



# 颜色范围定义
color_ranges = {
    "RED": [[9, 142, 141], [185, 183, 189]],
    "GREEN": [[30, 60, 128], [160, 128, 172]],
    "BLUE": [[10, 100, 50], [140, 180, 110]]
}

class ColorRectPick(Node):
    def __init__(self):
        super().__init__('color_rect_pick')
        self.moving = False
        self.endpoint = None
        self.enable_sortting = False
        self.imgpts = None
        self.center_imgpts = None
        self.roi = None
        self.pick_pitch = 80
        self.moving_step = 0
        self.count = 0

        self.K = None
        self.D = None
        self.camera_info_received = False 

        self.min_area = 500
        self.max_area = 7000

        self.target = None
        self.target_labels = {
            "RED": True,
            "GREEN": True,
            "BLUE": True,
        }

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2)
        self.Arm_controller.move_arm([0.15, 0.0, 0.13], 90.0)
        time.sleep(1)
        self.rgb_sub = self.create_subscription(
            Image,
            '/usb_cam/image_raw',
            self.camera_callback,
            10)
        self.rgb_sub

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/usb_cam/camera_info',
            self.camera_info_callback,
            10)
        self.info_sub  

        self.queue = queue.Queue(maxsize=1)
        # 初始化相机外参和识别区域
        self.white_area_center = np.array([[1, 0, 0, 0.199],
                                          [0, 1, 0,   0],
                                          [0, 0, 1,   0],
                                          [0, 0, 0,   1]], dtype=np.float64)
        
        self.white_area_cam = np.array([[1, 0, 0,     0],
                                       [0, 1, 0,     0],
                                       [0, 0, 1, 0.2140],
                                       [0, 0, 0,     1]], dtype=np.float64)

        # 添加定时器（例如每0.03秒调用一次 image_proc）
        self.create_timer(0.03, self.image_proc)

    def camera_callback(self, msg):
        if self.queue.empty():
            self.queue.put_nowait(msg)
    
    def initialize_after_camera_info(self):
        white_area_height = 0.118
        white_area_width = 0.210

        # 计算识别区域角点
        white_area_lt = np.matmul(self.white_area_center, xyz_euler_to_mat((white_area_height/2, white_area_width/2, 0.0), (0, 0, 0)))
        white_area_lb = np.matmul(self.white_area_center, xyz_euler_to_mat((-white_area_height/2, white_area_width/2, 0.0), (0, 0, 0)))
        white_area_rb = np.matmul(self.white_area_center, xyz_euler_to_mat((-white_area_height/2, -white_area_width/2, 0.0), (0, 0, 0)))
        white_area_rt = np.matmul(self.white_area_center, xyz_euler_to_mat((white_area_height/2, -white_area_width/2, 0.0), (0, 0, 0)))

        self.endpoint = self.get_endpoint()

        corners_cam = np.matmul(np.linalg.inv(np.matmul(self.endpoint, matrix_hand_to_cam)), 
                               [white_area_lt, white_area_lb, white_area_rb, white_area_rt, self.white_area_center])
        corners_cam = np.matmul(np.linalg.inv(self.white_area_cam), corners_cam)
        corners_cam = corners_cam[:, :3, 3:].reshape((-1, 3))

        tvec = np.array([-0.025, 0.003, 0.2140]).reshape((3, 1))
        rmat = np.array([[0, -1.0, 0],
                        [1.0, 0, 0],
                        [0, 0, 1.0]], dtype=np.float64)
        # 计算图像坐标
        center_imgpts, jac = cv2.projectPoints(corners_cam[-1:], rmat, tvec, self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)

        tvec, rmat = extristric_plane_shift(tvec, rmat, 0.015)
        self.extristric = (tvec, rmat)
        imgpts, jac = cv2.projectPoints(corners_cam[:-1], rmat, tvec, self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

        # 计算ROI区域
        x_min = max(0, min(p[0] for p in self.imgpts))
        x_max = min(640, max(p[0] for p in self.imgpts))
        y_min = max(0, min(p[1] for p in self.imgpts))
        y_max = min(480, max(p[1] for p in self.imgpts))
        self.roi = np.array([y_min, y_max, x_min, x_max])
        self.get_logger().info(f"ROI: {self.roi}")

    def get_endpoint(self):
        endpoint = self.Arm_controller.get_current_pose()
        return xyz_quat_to_mat([
            endpoint.position.x,
            endpoint.position.y,
            endpoint.position.z
        ], [
            endpoint.orientation.w,
            endpoint.orientation.x,
            endpoint.orientation.y,
            endpoint.orientation.z
        ])
        
    def point_remapped(self, point, now, new, data_type=float):
        x, y = point
        now_w, now_h = now
        new_w, new_h = new
        new_x = x * new_w / now_w
        new_y = y * new_h / now_h
        return data_type(new_x), data_type(new_y)

    def adaptive_threshold(self, gray_image):
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)
        _imshow_fit("BIN", binary)
        return binary

    def canny_proc(self, bgr_image):
        mask = cv2.Canny(bgr_image, 23, 51, 23, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))
        return mask

    def get_top_surface(self, rgb_image):
        image_scale = cv2.convertScaleAbs(rgb_image, alpha=2.5, beta=0)
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)
        image_gs = cv2.GaussianBlur(image_mb, (5, 5), 5)
        binary = self.adaptive_threshold(image_gs)
        mask = self.canny_proc(image_gs)
        mask1 = cv2.bitwise_and(binary, mask)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)
        return roi_image_mask

    def pick(self, position, rolll, color_name):
        self.moving = True
        x=A.position.x
        y=A.position.y
        z=A.position.z
        rolll=A.roll

        self.Arm_controller.move_arm([x,y,z+0.03],pitch=70,roll=rolll,duration=1500)
        time.sleep(0.7)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,duration=400)
        time.sleep(0.2)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,hand=-0.35,duration=500)
        time.sleep(0.2)
        self.Arm_controller.move_arm([x,y,z+0.07],pitch=70,hand=-0.35,duration=600)
        time.sleep(0.3)
        set_x=0.07
        set_y=-0.205
        set_z=0.03
        set_roll=-10
        if color_in_order[self.color_order] == color_ranges['RED']:
            set_x=0.06 # 0.08
            set_y=0.19
            set_roll=-21
            set_z=-0.01
        elif color_in_order[self.color_order] == color_ranges['GREEN']:
            set_x=-0.03
            set_y=0.19
            set_roll=-13
            set_z=-0.01
        elif color_in_order[self.color_order] == color_ranges['BLUE']:
            set_x=-0.13 # -0.02
            set_y=0.20
            set_roll=11
            set_z=-0.01

        self.Arm_controller.move_arm([set_x,set_y,0.10],pitch=70,roll=set_roll,hand=-0.35,duration=1300)
        time.sleep(0.6)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.35,duration=1000)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=0.800,duration=600)
        time.sleep(0.3)
        self.Arm_controller.move_arm([set_x,set_y,0.10],pitch=70,roll=-10,hand=0.800,duration=400)
        time.sleep(0.2)
        self.Arm_controller.move_arm([0.12,0.0,0.08],pitch=70.0,duration=1500)
        time.sleep(2.0)

        self.color_order+=1
        if (self.color_order < len(color_in_order)):
            self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
        else:
            self.color_order=0
            self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
        self.moving = False

    def image_proc(self):
        if self.queue.empty():
            return

        ros_image = self.queue.get(block=True)
        try:
            if ros_image.encoding == 'rgb8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 3).astype(np.uint8)
            elif ros_image.encoding == 'mono8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width).astype(np.uint8)
                # 如果需要RGB图像，可以将单通道图像复制到三个通道
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB)
            elif ros_image.encoding == 'yuv422_yuy2':
                yuv_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 2).astype(np.uint8)
                rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_YUY2)
            else:
                self.get_logger().error(f"Unsupported image encoding: {ros_image.encoding}")
                return

            result_image = np.copy(rgb_image)

            # 绘制识别区域
            if self.imgpts is not None:
                cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA)
                for p in self.imgpts:
                    cv2.circle(result_image, tuple(p), 8, (255, 0, 0), -1)
            
            if self.center_imgpts is not None:
                cv2.line(result_image, (self.center_imgpts[0]-10, self.center_imgpts[1]), 
                        (self.center_imgpts[0]+10, self.center_imgpts[1]), (255, 255, 0), 2)
                cv2.line(result_image, (self.center_imgpts[0], self.center_imgpts[1]-10), 
                        (self.center_imgpts[0], self.center_imgpts[1]+10), (255, 255, 0), 2)

            target_list = []
            index = 0

            if self.roi is not None and self.moving_step == 0:
                roi_area_mask = np.zeros((ros_image.height, ros_image.width), dtype=np.uint8)
                cv2.drawContours(roi_area_mask, [self.imgpts], -1, 255, cv2.FILLED)
                rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=roi_area_mask)
                roi_img = rgb_image[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
                
                roi_img = self.get_top_surface(roi_img)
                image_lab = cv2.cvtColor(roi_img, cv2.COLOR_RGB2LAB)

                for color_name in ['RED', 'GREEN', 'BLUE']:
                    color = color_ranges[color_name]
                    mask = cv2.inRange(image_lab, tuple(color[0]), tuple(color[1]))
                    
                    eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    
                    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    
                    for c in contours:
                        area = cv2.contourArea(c)
                        if self.min_area <= area <= self.max_area:
                            rect = cv2.minAreaRect(c)
                            (center_x, center_y), _, angle = rect
                            
                            # 转换回原始图像坐标
                            center_x += self.roi[2]
                            center_y += self.roi[0]
                            
                            corners = cv2.boxPoints(rect)
                            corners = np.int0(corners)
                            corners[:, 0] += self.roi[2]
                            corners[:, 1] += self.roi[0]
                            
                            cv2.circle(result_image, (int(center_x), int(center_y)), 8, (0, 0, 0), -1)
                            cv2.drawContours(result_image, [corners], -1, (0, 255, 255), 2)
                            
                            target_list.append([color_name, index, (center_x, center_y), angle])
                            index += 1

            if self.moving_step == 0 and target_list:
                selected_target = None
                for color in ['RED', 'GREEN', 'BLUE']:
                    if not self.target_labels[color]:
                        continue
                    for target in target_list:
                        if target[0] == color:
                            selected_target = target
                            break
                    if selected_target:
                        break
                if selected_target:
                    color_name, _, (cx, cy), angle = selected_target
                    if self.target and self.target[0] == color_name:
                        self.count += 1
                    else:
                        self.count = 0
                    self.target = selected_target

                    if self.count > 35:
                        # 坐标转换
                        projection_matrix = np.vstack((np.hstack((self.extristric[1], self.extristric[0])), [0, 0, 0, 1]))
                        position = pixels_to_world([(cx, cy)], self.K, projection_matrix)[0]
                        
                        position[0] = -position[0]
                        position[2] = 0.03  # 调整高度
                        
                        world_pose = np.matmul(self.white_area_center, xyz_euler_to_mat(position, (0, 0, 0)))
                        pose_t, _ = mat_to_xyz_euler(world_pose)
                        
                        # 修正坐标
                        pose_t[0] += 0.01
                        pose_t[1] = pose_t[1] * 0.91 + 0.01
                        
                        self.moving_step = 1
                        threading.Thread(target=self.pick, args=(pose_t, angle, color_name)).start()
                        
            _imshow_fit("color", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    color_rect_pick = ColorRectPick()
    rclpy.spin(color_rect_pick)
    color_rect_pick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
