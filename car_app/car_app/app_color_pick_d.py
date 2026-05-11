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
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import car_vision.Kinematics as Kinematics
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool
import message_filters
from message_filters import ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

# 颜色范围定义
color_ranges = {
    "RED": [[9, 142, 141], [185, 183, 189]],
    "GREEN": [[20, 60, 128], [100, 120, 172]],
    "BLUE": [[10, 100, 85], [100, 180, 110]]
}
color_in_order=[color_ranges['RED'],color_ranges['GREEN'],color_ranges['BLUE']]

class ColorFinder:
    def proc (self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color_ranges[0]), tuple(color_ranges[1])) # 二值化

        # 滤波，去掉噪点，平滑边缘
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 找出最大轮廓
        contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # [-2]
        #max_contour_area = get_area_max_contour(contours, 10)
        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c)) < 500:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c)
            if min_c is None:
                min_c = (c, center_x)
            elif center_x < min_c[1]:
                if center_x < min_c[1]:
                    min_c = (c, center_x)

        # 如果有符合要求的轮廓
        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0]) # 最小外接圆
            rect=cv2.minAreaRect(min_c[0])

            circle_color = (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)

            center_x = center_x * 2
            center_x_1 = center_x / w
            center_y = center_y * 2
            center_y_1 = center_y / h
            return (result_image, (0, 0), (center_x, center_y), radius * 2,rect[2])
        else:
            return (result_image, None, None, 0,0)
        
class ColorRectPickD(Node):
    def __init__(self):
        super().__init__('color_pick_d')
        self.debug = False
        self.moving = False
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.color_order = 0
        self.endpoint = None
        self.detect_times = 0
        self.bridge = CvBridge()
        
        self.get_logger().info(f"本次共检测 {len(color_in_order)} 个颜色")
        self.tracker = ColorFinder()


        self.Kinematics = Kinematics.DoF5_ARM_Kinematics()
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = Pose()

        self.arm_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.arm_joint_states

        self.Arm_controller = arm_ik_sdk.ArmControl()
        # self.client = self.create_client(SetBool, '/camera/set_ldp')
        # self.timer = self.create_timer(1.0, self.check_service)

        self.rgb_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.result_publisher = self.create_publisher(RosImage, 'color_pick/image_result', 1)
        self.create_service(Trigger, 'color_pick/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, 'color_pick/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.ts = None
        self.queue = queue.Queue(maxsize=1)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter")
        time.sleep(3)
        self.Arm_controller.move_arm([0.14,0.0,0.16],70.0,duration=1300)
        time.sleep(2.0)
        self.Arm_controller.move_arm([0.14,0.0,0.13],70.0,duration=1300)
        time.sleep(2.0)
        cb_group = ReentrantCallbackGroup()
        if self.rgb_sub is None:
             self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        if self.depth_sub is None:
             self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        if self.info_sub is None:
             self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth/camera_info', callback_group=cb_group)
        if self.ts is None:
            self.ts = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.info_sub], 
                queue_size=3, 
                slop=0.03
            )
            self.ts.registerCallback(self.multi_callback)

        self.timer = self.create_timer(0.1, self.image_proc)
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit")
        try:
            if self.rgb_sub is not None:
                self.destroy_subscription(self.rgb_sub)
                self.rgb_sub = None
            if self.depth_sub is not None:
                self.destroy_subscription(self.depth_sub)
                self.depth_sub = None
            if self.info_sub is not None:
                self.destroy_subscription(self.info_sub)
                self.info_sub = None
            if self.ts is not None:
                self.ts = None
            self.timer.cancel()
        except Exception as e:
            self.get_logger().error(str(e))
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        time.sleep(1)
        response.success = True
        response.message = "exit"
        return response
    
    def check_service(self):
        if self.client.service_is_ready():
            self.client.call_async(SetBool.Request(data=True))
            self.timer.cancel()  # 服务可用后取消定时器
        else:
            self.get_logger().info('service not available, waiting again...')

    def multi_callback(self, rgb_msg, depth_msg, info_msg):
        if self.queue.empty():
            self.queue.put_nowait((rgb_msg, depth_msg, info_msg))

    def joint_states_callback(self, states):
            self.joints[0] = states.position[4]
            self.joints[1] = states.position[5]
            self.joints[2] = states.position[6]
            self.joints[3] = states.position[7]
            self.joints[4] = states.position[8]

    def get_current_pose(self):
        position, orientation = self.Kinematics.get_forwardKinematics(self.joints, radians=True)
        self.get_logger().info(f"self.joints: {self.joints}")
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]

        self.pose.orientation.w = orientation[0]
        self.pose.orientation.x = orientation[1]
        self.pose.orientation.y = orientation[2]
        self.pose.orientation.z = orientation[3]
        return self.pose

    def get_endpoint(self):
        endpoint = self.get_current_pose()
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

    def pick(self, position ,rolll):
        self.moving = True
        x,y,z=position

        yaw = math.degrees(math.atan2(y, x))
        print("now yaw",yaw)
        if rolll==90.0:
            rolll=0
        if y <= 0:
            rolll=-rolll-yaw
        else:
            rolll=rolll+yaw
            rolll=90-rolll
            
        print("new rolll",rolll)

        self.Arm_controller.move_arm([x,y,z+0.01],pitch=70,roll=rolll,duration=1500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,duration=400)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,hand=-0.035,duration=500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z+0.05],pitch=70,hand=-0.035,duration=400)
        time.sleep(0.5)
        set_x=0.03
        set_y=-0.205
        set_z=0.027
        set_roll=-10
        if color_in_order[self.color_order] == color_ranges['RED']:
            set_x=0.09 # 0.105
            set_y=-0.19
            set_roll=-21
            set_z=0.03
        elif color_in_order[self.color_order] == color_ranges['GREEN']:
            set_x=0.03
            set_y=-0.19
            set_roll=-13
            set_z=0.03
        elif color_in_order[self.color_order] == color_ranges['BLUE']:
            set_x=-0.032 # -0.02
            set_y=-0.20
            set_roll=11
            set_z=0.03

        self.Arm_controller.move_arm([set_x,set_y,0.08],pitch=70,roll=set_roll,hand=-0.035,duration=1500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.035,duration=700)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=0.800,duration=600)
        time.sleep(1)
        self.Arm_controller.move_arm([set_x,set_y,0.05],pitch=70,roll=-10,hand=0.800,duration=400)
        time.sleep(0.5)
        self.Arm_controller.move_arm([0.14,0.0,0.16],70.0,duration=1300)
        time.sleep(2.0)
        self.Arm_controller.move_arm([0.14,0.0,0.13],70.0,duration=1300)
        time.sleep(2.0)

        self.color_order+=1
        if (self.color_order < len(color_in_order)):
            self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
        else:
            self.color_order=0
            self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
        self.moving = False

    def image_proc(self):
        try:
            rgb_msg, depth_msg, info_msg = self.queue.get_nowait()
        except queue.Empty:
            return

        if self.endpoint is None:
            self.endpoint = self.get_endpoint()

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            depth_image = depth_image [40:-40, 40:-40] # 裁切为 620x460
            depth_image = cv2.resize (depth_image, (640, 480)) # 放大回 640x480
            result_image = np.copy(rgb_image)
            h, w = depth_image.shape[:2]
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555

            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            if self.tracker is not None and self.moving == False:
                result_image, p_y, center, r ,roll = self.tracker.proc(rgb_image, result_image, color_in_order[self.color_order])
                
                if p_y is not None:
                    print(p_y)
                    self.detect_times=0
                    center_x, center_y = center
                    if center_x > 639:
                        center_x = 639
                    if center_y > 479:
                        center_y = 479
                    print("center_x:", center_x)
                    if abs(self.last_pitch_yaw[0] - p_y[0]) < 0.08 and abs(self.last_pitch_yaw[1] - p_y[1]) < 0.08 and depth_image[int(center_y),int(center_x)] > 150:
                        if time.time() - self.stamp > 1.5:
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

                            pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标 即机械臂抓手到像素点的相对坐标
                            world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                            pose_t, pose_R = mat_to_xyz_euler(world_pose)
     
                            # pose_t[0] += 0.01
                            pose_t[1] = pose_t[1] * 0.9
                            self.stamp = time.time()

                            print(pose_t)
                            threading.Thread(target=self.pick, args=(pose_t,roll)).start()
                    else:
                        print("center point dist wrong")
                        self.stamp = time.time()
                    dist = depth_image[int(center_y),int(center_x)]
                    print("dist:",dist)
                    if dist < 100:
                        txt = "TOO CLOSE !!!"
                    else:
                        txt = "Dist: {}mm".format(dist)
                    cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                    cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                    cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                    cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                    self.last_pitch_yaw = p_y
                else:
                    self.detect_times+=1
                    if self.detect_times > 30:
                        if self.color_order < len(color_in_order)-1:
                            self.color_order+=1
                        else:
                            self.get_logger().info("找不到任何颜色物块，重新开始检索" + str(color_in_order[self.color_order]))
                            self.color_order=0
                        self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
                        self.detect_times=0
    
                    self.stamp = time.time()
                    # print("not find")

            ee_image=cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            ee_image=cv2.resize(ee_image,(320,240))
            depth_color_map=cv2.resize(depth_color_map,(320,240))
            result_image = np.concatenate([ee_image, depth_color_map, ], axis=1)
            if self.debug:
                cv2.imshow("depth", result_image)
                cv2.waitKey(1)
            else:
                self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    color_pick_d = ColorRectPickD()
    rclpy.spin(color_pick_d)
    color_pick_d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
