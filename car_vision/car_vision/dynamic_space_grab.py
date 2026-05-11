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
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import message_filters
from message_filters import ApproximateTimeSynchronizer,Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera
import car_vision.pid as pid

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



color_range = ([11,119,59],[172,178,102])

class ColorFinder:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(0.05, 0.0, 0.02) # 0.25,0,05,0,02
        self.pid_pitch = pid.PID(0.03, 0.0, 0.02) #0.25, 0.05, 0.02
        self.yaw_limit = [-1.000, 1.000]
        self.pitch_limit = [0.800, 1.600]
        self.yaw = 0
        self.pitch = 1.200
        

    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        
        img = cv2.resize(source_image, (w//2, h//2))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB)
        mask = cv2.inRange(img_lab, np.array(color_ranges[0]), np.array(color_ranges[1]))

        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c)) < 300:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c)
            if min_c is None or center_x < min_c[1]:
                min_c = (c, center_x)

        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0])
            cv2.circle(result_image, (int(center_x*2), int(center_y*2)), int(radius*2), (0x55, 0x55, 0x55), 2)

            center_x = center_x * 2
            center_x_1 = center_x / w
            if abs(center_x_1 - 0.5) > 0.02:
                self.pid_yaw.SetPoint = 0.5 
                self.pid_yaw.update(center_x_1)
                self.yaw = max(min(self.yaw - self.pid_yaw.output, self.yaw_limit[1]), self.yaw_limit[0])
            else:
                self.pid_yaw.clear()

            center_y = center_y * 2
            center_y_1 = center_y / h
            if abs(center_y_1 - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y_1)
                self.pitch = max(min(self.pitch - self.pid_pitch.output, self.pitch_limit[1]), self.pitch_limit[0])
            else:
                self.pid_pitch.clear()

            return (result_image, (self.pitch, self.yaw), (center_x, center_y), radius * 2)
        else:
            return (result_image, None, None, 0)

class DynamicSpaceGrab(Node):
    def __init__(self):
        super().__init__("dynamic_space_grab")
        self.moving = False
        self.last_pitch_yaw = (0, 0)
        self.first_time = time.time()
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.tracker = ColorFinder(color_range)
        self.bridge = CvBridge()
        self.endpoint = None

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(3)
        self.arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        # self.client = self.create_client(SetBool, '/camera/set_ldp')
        # self.timer = self.create_timer(1.0, self.check_service)

        cb_group = ReentrantCallbackGroup()
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth/camera_info', callback_group=cb_group)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], 
            queue_size=3, 
            slop=0.03
        )
        self.ts.registerCallback(self.multi_callback)
        self.queue = queue.Queue(maxsize=1)

        self.timer = self.create_timer(0.1, self.image_proc)

    def multi_callback(self, rgb_msg, depth_msg, info_msg):
        if self.queue.empty():
            self.queue.put_nowait((rgb_msg, depth_msg, info_msg))

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
        
    def pick(self, position):
        self.moving = True
        x, y, z = position
        t, euler = mat_to_xyz_euler(self.endpoint)
        
        self.arm_controller.move_arm([x, y, z], pitch=euler[1], duration=2000)
        time.sleep(1.5)
        self.arm_controller.move_arm([x,y,z],pitch=euler[1],hand=-0.30,duration=1000)
        time.sleep(2)
        self.arm_controller.move_arm([-.03,-0.17,0.08-0.093],pitch=80,roll=-0.0,hand=-0.30,duration=2000)
        time.sleep(0.5)
        self.arm_controller.move_arm([-0.03,-0.17,0.02-0.093],pitch=80,roll=-0.0,hand=-0.30,duration=600)
        time.sleep(1.0)
        self.arm_controller.move_arm([-0.03,-0.17,0.02-0.093],pitch=80,roll=-0.0,hand=0.800,duration=400)
        time.sleep(0.5)
        self.arm_controller.move_arm([-0.03,-0.17,0.08-0.093],pitch=80,roll=-0.0,hand=0.800,duration=1000)
        time.sleep(0.5)
        self.arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(2)

        self.tracker.yaw = 0
        self.tracker.pitch = 1.200
        self.tracker.pid_yaw.clear()
        self.tracker.pid_pitch.clear()
        self.stamp = time.time()
        self.first_time = time.time() + 3
        self.moving = False

    def image_proc(self):
        try:
            rgb_msg, depth_msg, info_msg = self.queue.get_nowait()
        except queue.Empty:
            return
        if self.endpoint is None:
            self.endpoint = self.get_endpoint()    
        rgb_image = np.ndarray(shape=(rgb_msg.height, rgb_msg.width, 3), dtype=np.uint8, buffer=rgb_msg.data)
        depth_image = np.ndarray(shape=(depth_msg.height, depth_msg.width), dtype=np.uint16, buffer=depth_msg.data)
        rh, rb = rgb_image.shape[:2]
        ih, iw = depth_image.shape[:2]
        rgb_image=rgb_image[(rh-ih)//2:rh-(rh-ih)//2,]

        result_image = np.copy(rgb_image)

        depth = np.copy(depth_image).reshape((-1, ))
        depth[depth<=0] = 55555
        
        sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
        sim_depth_image = sim_depth_image / 2000.0 * 255.0
        depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
        
        if self.tracker and self.moving == False:
            result_image, p_y, center, roll = self.tracker.proc(rgb_image, result_image, color_range)
            
            if p_y is not None:
                if abs(self.last_pitch_yaw[0] - p_y[0]) > 0.01 or abs(self.last_pitch_yaw[1] - p_y[1]) > 0.01:
                    self.arm_controller.set_steer([p_y[1],-0.930,1.6,p_y[0],0,0.801],0) #0,-0.930,1.178,1.650,0,0.801) # 0和3号舵机运动，即左右和上下，其他舵机保持不变
                print(p_y)
                self.detect_times=0
                o_center_x, o_center_y = center
                center_x = int(((o_center_x + 10)-(iw//2)) / 1.26 + (iw//2))
                center_y = int((o_center_y-(ih//2)) / 1.26+(ih//2))
                if center_x > 639:
                    center_x = 639
                if center_y > 479:
                    center_y = 479
                print("center_x:", center_x)
                if abs(self.last_pitch_yaw[0] - p_y[0]) < 0.01 and abs(self.last_pitch_yaw[1] - p_y[1]) < 0.01 and depth_image[int(center_y),int(center_x)] > 150:
                    if time.time() - self.stamp > 2.0:
                        self.stamp = time.time()
                        dist = depth_image[int(center_y),int(center_x)]/1000.0
                        dist += 0.015 # 物体半径补偿
                        K = info_msg.k

                        position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                        print("position",position)
                        position[0] -= 0.007  # rgb相机和深度相机tf有1cm偏移
                        temp=position[0]
                        position[0]=position[1]
                        position[1]=-temp
                        self.endpoint = self.get_endpoint()
                        pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标 即机械臂抓手到像素点的相对坐标
                        world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                        pose_t, pose_R = mat_to_xyz_euler(world_pose)
    
                        pose_t[0] += -0.01
                        pose_t[1] += 0.006
                        pose_t[1] = pose_t[1] * 1.01
                        pose_t[2] += 0.015
                        self.stamp = time.time()

                        print(pose_t)
                        threading.Thread(target=self.pick, args=(pose_t,)).start()
                else:
                    print("center point dist wrong")
                    self.stamp = time.time()
                dist = depth_image[int(center_y),int(center_x)]
                print("dist:",dist)
                if dist < 100 or dist > 320:
                    txt = "TOO CLOSE !!!"
                else:
                    txt = "Dist: {}mm".format(dist)
                cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                self.last_pitch_yaw = p_y
            else:
                self.stamp = time.time()
                print("not find")
        ee_image=cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        ee_image=cv2.resize(ee_image,(320,240))
        depth_color_map=cv2.resize(depth_color_map,(320,240))
        result_image = np.concatenate([ee_image, depth_color_map, ], axis=1)
        _imshow_fit("depth", result_image)
        if cv2.waitKey(1) & 0XFF == 27:  # 退出键,  27=ESC
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DynamicSpaceGrab()
    try:
        while rclpy.ok():
            node.image_proc()
            rclpy.spin_once(node, timeout_sec=0.04)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
