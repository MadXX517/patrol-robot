#!/usr/bin/env python3
# coding=utf-8

import cv2
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import threading
from queue import Queue
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



class RGBDepthNode(Node):
    def __init__(self):
        super().__init__('rgb_depth_node')
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        # 创建 RGB 和深度图像的订阅者
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data)

        # 图像队列，用于存储同步的 RGB 和深度图像
        self.queue = Queue(maxsize=1)
        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()

        # 创建一个线程用于处理图像
        self.image_thread = threading.Thread(target=self.image_proc)
        self.image_thread.daemon = True
        self.image_thread.start()

    def rgb_callback(self, msg):
        """RGB 图像回调函数"""
        self.rgb_image = msg
        self.sync_images()

    def depth_callback(self, msg):
        """深度图像回调函数"""
        self.depth_image = msg
        self.sync_images()

    def sync_images(self):
        """同步 RGB 和深度图像"""
        if self.rgb_image is not None and self.depth_image is not None:
            if self.queue.empty():
                self.queue.put((self.rgb_image, self.depth_image))
                self.rgb_image = None
                self.depth_image = None

    def image_proc(self):
        """图像处理函数"""
        while rclpy.ok():
            try:
                if not self.queue.empty():
                    ros_rgb_image, ros_depth_image = self.queue.get(block=True)
                    rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, desired_encoding='bgr8')
                    depth_image = self.bridge.imgmsg_to_cv2(ros_depth_image, desired_encoding='16UC1')

                    h, w = depth_image.shape[:2]
                    depth = np.copy(depth_image).reshape((-1,))
                    depth[depth <= 0] = 0
                    sim_depth_image = np.clip(depth_image, 0, 4000).astype(np.float64)
                    sim_depth_image = sim_depth_image / 2000.0 * 255.0
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                    rgb_image = cv2.resize(rgb_image, (320, 240))
                    depth_color_map = cv2.resize(depth_color_map, (320, 240))
                    result_image = np.concatenate([rgb_image, depth_color_map], axis=1)
                    _imshow_fit("depth", result_image)

                    if cv2.waitKey(1) & 0xFF == 27:  # 退出键, 27=ESC
                        cv2.destroyAllWindows()
                        rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = RGBDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
