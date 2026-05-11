#!/usr/bin/env python3
# coding=utf8

import cv2
import sys
import rclpy
import time
import numpy as np
import threading
import queue
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import message_filters
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
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



class ObjectShape:
    triangle = 0
    square = 1
    sphere = 2
    cylinder = 3
    unknow = 4
    shapes_name = ["triangle", "square", "sphere", "cylinder", "unknow"]

class ShapePick(Node):
    def __init__(self):
        # ROS2节点初始化（继承Node类）
        super().__init__('shape_recognition')
        self.last_shape = "none"
        self.moving = False
        self.count = 0
        self.object_nums = [0, 0, 0, 0, 0]

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

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(3)
        self.Arm_controller.move_arm([0.12,0.0,0.16],70.0)
        time.sleep(1.0)

        # ROS2消息同步订阅（适配message_filters）
        cb_group = ReentrantCallbackGroup()
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        # 时间同步（参数与ROS1一致：队列大小2，时间误差0.03s）
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 2, 0.03)
        self.sync.registerCallback(self.multi_callback)  # ROS2用小写register_callback

        self.queue = queue.Queue(maxsize=1)
        self.timer = self.create_timer(0.1, self.image_proc)

    def multi_callback(self, rgb_msg, depth_msg):
        if self.queue.empty():
            self.queue.put_nowait((rgb_msg, depth_msg))

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
    def move(self, shape):
        # 机械臂动作逻辑保持不变（仅替换rospy.sleep为time.sleep）
        self.moving = True
        if shape == "square" or shape == "sphere":
            self.Arm_controller.move_arm([0.17, 0, 0.03], pitch=80)
            time.sleep(1)
            self.Arm_controller.move_arm([0.17, 0, 0.03], pitch=80, hand=-0.020, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.17, 0, 0.08], pitch=80, hand=-0.020, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.155, 0.08], pitch=80, roll=-10, hand=-0.020, duration=2000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.155, 0.02], pitch=80, roll=-10, hand=-0.020, duration=600)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.155, 0.02], pitch=80, roll=-10, hand=0.800, duration=400)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.155, 0.08], pitch=80, roll=-10, hand=0.800, duration=1000)
        elif shape == "cylinder":
            self.Arm_controller.move_arm([0.17, 0, 0.03], pitch=60)
            time.sleep(1)
            self.Arm_controller.move_arm([0.17, 0, 0.03], pitch=60, hand=-0.000, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.17, 0, 0.08], pitch=60, hand=-0.000, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.15, 0.08], pitch=60, roll=-10, hand=-0.000, duration=2000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.15, 0.03], pitch=60, roll=-10, hand=-0.000, duration=600)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.15, 0.03], pitch=60, roll=-10, hand=0.800, duration=400)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.03, -0.15, 0.08], pitch=80, roll=-10, hand=0.800, duration=1000)

        time.sleep(1.0)
        self.Arm_controller.move_arm([0.13, 0.0, 0.14], 70.0)
        time.sleep(1.0)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        # 回调逻辑不变，保持队列缓存
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image))

    def image_proc(self):
        try:
            ros_rgb_image, ros_depth_image = self.queue.get(block=True)

            # RGB图像转换（ROS2 Image消息格式与ROS1一致，直接复用）
            rgb_image = np.ndarray(
                shape=(ros_rgb_image.height, ros_rgb_image.width, 3),
                dtype=np.uint8,
                buffer=ros_rgb_image.data
            )
            # 深度图像转换
            depth_image = np.ndarray(
                shape=(ros_depth_image.height, ros_depth_image.width),
                dtype=np.uint16,
                buffer=ros_depth_image.data
            )

            ih, iw = depth_image.shape[:2]

            # 深度图像预处理（逻辑不变）
            depth = np.copy(depth_image).reshape((-1,))
            depth[depth <= 100] = 55555
            min_index = np.argmin(depth)
            min_y = min_index // iw
            min_x = min_index - min_y * iw

            min_dist = depth_image[min_y, min_x]
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
            depth_image = np.where(depth_image > min_dist + 12, 0, depth_image)
            sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
            depth_gray = sim_depth_image_sort.astype(np.uint8)
            depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            depth_bit = cv2.erode(depth_bit, np.ones((3, 3), np.uint8))
            depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            # 轮廓检测与形状识别（逻辑不变）
            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            shape = 'none'

            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 2000 or area > 60000 or self.moving:
                    continue
                cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 4)
                perimeter = cv2.arcLength(obj, True)
                approx = cv2.approxPolyDP(obj, 0.03 * perimeter, True)
                cv2.drawContours(depth_color_map, approx, -1, (255, 0, 0), 4)
                corner_num = len(approx)

                x, y, w, h = cv2.boundingRect(approx)

                # 形状判断逻辑不变
                if corner_num == 3:
                    obj_type = ObjectShape.triangle
                elif corner_num == 4:
                    obj_type = ObjectShape.square
                elif corner_num > 5:
                    if abs(min_x - (x + w / 2)) < w / 5 and abs(min_y - (y + h / 2)) < h / 5:
                        obj_type = ObjectShape.sphere
                    else:
                        obj_type = ObjectShape.cylinder
                else:
                    obj_type = ObjectShape.unknow

                shape = obj_type
                self.object_nums[obj_type] += 1
                self.get_logger().info(f"识别到形状: {ObjectShape.shapes_name[obj_type]}")
                cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)

            if shape != 'none':
                self.count += 1

            # 形状确认后触发机械臂动作（逻辑不变）
            if self.count > 10:
                max_shape_idx = np.argmax(self.object_nums)
                self.get_logger().info(f"最终确认形状: {ObjectShape.shapes_name[max_shape_idx]}")
                if self.object_nums[ObjectShape.unknow] < 10:
                    # 启动线程执行机械臂动作（避免阻塞图像处理）
                    threading.Thread(target=self.move, args=(ObjectShape.shapes_name[max_shape_idx],)).start()
                else:
                    self.get_logger().warn("未知形状过多，重启识别！")
                self.object_nums = [0, 0, 0, 0, 0]
                self.count = 0

            self.last_shape = shape

            # 图像标注（逻辑不变）
            txt = f'Dist: {depth_image[min_y, min_x]}mm'
            cv2.putText(depth_color_map, txt, (11, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
            cv2.putText(depth_color_map, txt, (10, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
            cv2.putText(bgr_image, txt, (10, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

            result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
            _imshow_fit("RGB-D Shape Recognition", result_image)

            # 退出逻辑（ROS2适配）
            if cv2.waitKey(40) & 0XFF == 27:
                self.get_logger().info("用户按下ESC，退出节点")
                rclpy.shutdown()
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f"图像处理回调错误: {str(e)}")

def main(args=None):
    # ROS2初始化
    rclpy.init(args=args)
    shape_pick = ShapePick()
    rclpy.spin(shape_pick)
    shape_pick.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()