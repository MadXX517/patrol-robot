#!/usr/bin/env python3
# encoding: utf-8
import os
import queue
import threading
import cv2
import numpy as np
from pyzbar import pyzbar
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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




class QRCodeDetectNode(Node):
    def __init__(self):
        super().__init__('qrcode_detect_node')
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.running = True

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        if self.image_queue.full():
            self.image_queue.get()  # Discard the oldest frame
        self.image_queue.put(cv_image)

    def process_images(self):
        while self.running and rclpy.ok():
            try:
                image = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # Decode QR codes
            decoded_objs = pyzbar.decode(image)
            for obj in decoded_objs:
                points = obj.polygon
                if len(points) > 4:
                    hull = cv2.convexHull(
                        np.array([point for point in points], dtype=np.float32))
                    points = hull.reshape(-1, 2)
                for j in range(len(points)):
                    pt1 = tuple(points[j])
                    pt2 = tuple(points[(j+1) % len(points)])
                    cv2.line(image, pt1, pt2, (0, 255, 0), 3)
                x, y = obj.rect.left, obj.rect.top
                data = obj.data.decode('utf-8')
                cv2.putText(image, data, (x, y-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(f'Detected QR code: {data}')
            
            _imshow_fit('QR Code Detection', image)
            key = cv2.waitKey(1)
            if key in {ord('q'), 27}:  # Exit on 'q' or ESC
                self.running = False
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    # Run executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    # Process images in the main thread
    try:
        node.process_images()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()
