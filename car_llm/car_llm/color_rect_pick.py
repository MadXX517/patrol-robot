#!/usr/bin/env python3
# coding=utf-8

import cv2,rclpy,math,time,queue,message_filters,threading
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from message_filters import ApproximateTimeSynchronizer

import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world
from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor

# 颜色范围定义
color_ranges = {
    "RED": [[9, 142, 141], [185, 220, 189]],
    "GREEN": [[20, 60, 128], [100, 120, 172]],
    "BLUE": [[10,100,40], [180,180,110]]
}

# color_in_order=[color_ranges['RED'],color_ranges['GREEN'],color_ranges['BLUE']]

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
            center_y = center_y * 2
            return (result_image, (0, 0), (center_x, center_y), radius * 2,rect[2])
        else:
            return (result_image, None, None, 0,0)
        
class ColorRectPick(Node):
    def __init__(self):
        super().__init__('color_rect_pick')

        self.moving = False
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.color_order = 0
        self.endpoint = None
        self.detect_times = 0
        self.info_msg = None
        self.color_in_order=[[[9, 142, 141], [185, 183, 189]]]
        self.bridge = CvBridge()
        
        self.tracker = ColorFinder()

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2.0)
        self.Arm_controller.move_arm([0.14,0.0,0.08],70.0)
        time.sleep(2.0)

        if self.endpoint is None:
            self.endpoint = self.get_endpoint()

        cb_group = ReentrantCallbackGroup()
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        # self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth/camera_info', callback_group=cb_group)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_callback, qos_profile_sensor_data)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=3, 
            slop=0.2
        )
        self.ts.registerCallback(self.multi_callback)
        self.queue = queue.Queue(maxsize=1)

        self.get_logger().info("初始化完成！")

        self.pick_sub = self.create_subscription(String, '/chat_model/whichone', self.pickinfo_callback, qos_profile_sensor_data)

    def multi_callback(self, rgb_msg, depth_msg):
        try:
            # 检查队列状态
            if self.queue.full():
                self.queue.get_nowait()  # 移除旧消息
            self.queue.put((rgb_msg, depth_msg))  # 使用阻塞操作
        except Exception as e:
            self.get_logger().error(f"处理图像消息时出错: {str(e)}")


    def caminfo_callback(self, info_msg):
        if self.info_msg is None:
            self.info_msg=info_msg

    def pickinfo_callback(self,msg):
        try:
            color_names = msg.data.split(",")  # e.g., ["RED", "GREEN", "BLUE"]
            self.color_in_order = [color_ranges[name.strip()] for name in color_names]
            print("color_in_order =", self.color_in_order,",len",len(self.color_in_order))

        except KeyError as e:
            self.get_logger().error(f"未知颜色名称：{e}")
        except Exception as e:
            self.get_logger().error(f"处理 pickinfo 消息出错: {e}")

        self.color_order=0

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

    def move_AtoB(self,A):
        self.moving = True
        x=A.position.x
        y=A.position.y
        z=A.position.z
        rolll=A.roll

        self.Arm_controller.move_arm([x,y,z+0.03],pitch=70,roll=rolll,duration=1500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,duration=800)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,hand=-0.5,duration=500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([x,y,z+0.07],pitch=70,hand=-0.5,duration=800)
        time.sleep(0.5)
        set_x=0.03
        set_y=-0.205
        set_z=0.027
        set_roll=-10
        if self.color_in_order[self.color_order] == color_ranges['RED']:
            set_x=0.088 # 0.08
            set_y=-0.20
            set_roll=-21
            set_z=0.03
        elif self.color_in_order[self.color_order] == color_ranges['GREEN']:
            set_x=0.02
            set_y=-0.19
            set_roll=-13
            set_z=0.03
        elif self.color_in_order[self.color_order] == color_ranges['BLUE']:
            set_x=-0.042 # -0.02
            set_y=-0.20
            set_roll=11
            set_z=0.03

        self.Arm_controller.move_arm([set_x,set_y,0.10],pitch=70,roll=set_roll,hand=-0.5,duration=1500)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.5,duration=700)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=0.800,duration=600)
        time.sleep(1.0)
        self.Arm_controller.move_arm([set_x,set_y,0.10],pitch=70,roll=-10,hand=0.800,duration=800)
        time.sleep(0.5)
        self.Arm_controller.move_arm([0.12,0.0,0.17],pitch=70.0,duration=1500)
        time.sleep(3.0)

        self.color_order+=1
        
        self.moving = False

    def image_proc(self):
        try:
            ros_rgb, ros_depth = self.queue.get_nowait()
        except queue.Empty:
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb, 'rgb8')
            depth_image = self.bridge.imgmsg_to_cv2(ros_depth, '16UC1')

            rh, rb = rgb_image.shape[:2]
            ih, iw = depth_image.shape[:2]
            rgb_image=rgb_image[(rh-ih)//2:rh-(rh-ih)//2,]

            result_image = np.copy(rgb_image)

            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555

            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            if self.moving == False:
                for ncolor in ["RED","GREEN","BLUE"]:
                    result_image, p_y, center, r ,roll = self.tracker.proc(rgb_image, result_image, color_ranges[ncolor])
                    
                    if p_y is not None:
                        if len(self.color_in_order)-1 >= self.color_order and self.color_in_order[self.color_order] == color_ranges[ncolor]:
                            depth_center_x = int(((center[0] + 10)-(iw//2)) / 1.26 + (iw//2))
                            depth_center_y = int((center[1]-(ih//2)) / 1.26+(ih//2))
                            cv2.circle(depth_color_map, (int(depth_center_x), int(depth_center_y)), 5, (255,255,255), 2)
                            PoseRC = self.count_position("Rect", [depth_center_x,depth_center_y], depth_image, self.info_msg.k, roll)
                            p_y=None

                            # 处理抓取
                            threading.Thread(target=self.move_AtoB, args=(PoseRC,)).start()
                            break

            ee_image=cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            result_image = np.concatenate([ee_image, depth_color_map, ], axis=1)
            cv2.imshow("depth", result_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def count_position(self,Name,center,depth_image,K,roll):
        PWRAC=PoseWithRollAndColor()

        center_x, center_y = center

        center_x = min(639, int(center_x))
        center_y = min(479, int(center_y))

        dist = depth_image[int(center_y),int(center_x)]/1000.0
        dist += 0.015 # 物体半径补偿

        position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))

        position[0] -= 0.007  # rgb相机和深度相机tf有1cm偏移
        temp=position[0]
        position[0]=position[1]
        position[1]=-temp

        pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标 即机械臂抓手到像素点的相对坐标
        world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
        pose_t, pose_R = mat_to_xyz_euler(world_pose)

        pose_t[0] = pose_t[0] + 0.008 # * 0.9
        pose_t[1] = pose_t[1] - 0.015 # * 0.9

        yaw = math.degrees(math.atan2(pose_t[1], pose_t[0]))
        if roll==90.0:
            roll=0
        if pose_t[1] <= 0:
            roll=-roll-yaw
        else:
            roll=roll+yaw
            roll=90-roll

        PWRAC.name = Name
        PWRAC.position.x = pose_t[0]
        PWRAC.position.y = pose_t[1]
        PWRAC.position.z = pose_t[2]
        PWRAC.roll = roll
        return PWRAC  # 返回位置和roll角度

def main(args=None):
    rclpy.init(args=args)
    color_rect_pick = ColorRectPick()

    executor = MultiThreadedExecutor()
    executor.add_node(color_rect_pick)
    try:
        while rclpy.ok():
            # 手动处理所有回调（包括订阅、服务等）
            executor.spin_once(timeout_sec=0.1)

            # 手动调用图像处理（代替定时器）
            color_rect_pick.image_proc()

            time.sleep(0.1)  # 控制频率，避免占满CPU
    finally:
        color_rect_pick.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
