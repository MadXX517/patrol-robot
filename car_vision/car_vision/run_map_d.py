#!/usr/bin/env python3
# encoding: utf-8

import time
import sys
import cv2
import math
import rclpy
import queue
import threading
import message_filters
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav_msgs.msg import Odometry
import transforms3d as tfs
from car_vision.map_detect import line_detect, cross_rois_area, color_rect_detect, calculate_square_mean
import car_vision.pid as pid
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)




# 颜色识别参数
detect_color_range = ([0, 104, 111], [43, 144, 151])
target_lab = [33, 124, 131]
rois_line = ((0.78, 0.80, 0, 1, 0.7), (0.72, 0.74, 0, 1, 0.2), (0.66, 0.68, 0, 1, 0.1))
rois_cross = {
    'mid_up': (0.71, 0.75, 0.3, 0.7),
    'mid_down': (0.77, 0.81, 0.3, 0.7),
}
color_ranges = {"RED": [[9, 142, 130], [185, 183, 189]], "GREEN": [[0, 0, 130], [200, 120, 200]], "BLUE": [[10, 100, 50], [180, 180, 115]]}
color_in_order = [color_ranges['RED'], color_ranges['GREEN'], color_ranges['BLUE']]

class Map2Node(Node):
    def __init__(self):
        super().__init__('map2_node')  # ROS2节点初始化
        self.get_logger().info("Map2 node started!")

        # PID参数
        self.pid = pid.PID(0.01, 0.0, 0.05)
        self.pid_z = 0

        # 状态标志位
        self.cross_first_detect = False
        self.gyro_yaw = 0.0
        self.start_turn_yaw = 0.0
        self.cross_first_detect_time = 0.0
        self.cross_flag = False
        self.cross_pass_time = 0.0
        self.cross_num = 0
        self.cross_keep_going_time = 0.0
        self.turn_90_flag = False
        self.turn_180_flag = False
        self.turn_where = ''
        self.car_status = 'stop'
        self.pick_rect_on_road_time = 0.0
        self.pick_rect_on_road = False
        self.pick_rect_ready = False
        self.put_rect_ready = False
        self.put_rect_on_road = False
        self.before_put_time = 0.0
        self.before_put_time_used = 5.0
        self.back_home_ready = False
        self.back_home_time = 0.0
        self.moving = False
        self.stamp = 0.0
        self.detect_times = 0
        self.color_order = 0
        self.last_pitch_yaw = (0, 0)
        self.endpoint = None

        # 运动参数
        self.catch_stage = 0
        self.set_speed = 0.09
        self.set_angular = 0.45
        self.cross_keep_choose = 0
        self.cross_keep_times = [2.0, 2.0]

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2)
        self.Arm_controller.set_steer([0.0, 0.0, 1.6, 1.200, 0, 0.00])
        time.sleep(1)

        self.image_height = None
        self.image_width = None

        # ROS2话题订阅
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.info_sub = self.create_subscription( CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.get_odom, 1)

        # 时间同步器
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 2, 0.05
        )
        self.sync.registerCallback(self.multi_callback)
        self.queue = queue.Queue(maxsize=2)

        # ROS2话题发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # ROS2定时器
        self.timer = self.create_timer(0.033, self.image_process)
    
    def camera_info_callback(self, camera_info: CameraInfo):
        """摄像头内参回调"""
        self.camera_info = camera_info
    def get_odom(self, odometry: Odometry):
        """ROS2 Odometry回调"""
        self.gyro_yaw = odometry.pose.pose.position.z

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
        
    def multi_callback(self, ros_rgb_image: Image, ros_depth_image: Image):
        """多消息同步回调（ROS2消息类型适配）"""
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image))
        else:
            self.queue.get()

    def pick(self, position, rolll):
        """机械臂抓取/放置逻辑"""
        self.moving = True
        x, y, z = position
        rolll = 0

        if self.catch_stage == 0:
            z = -0.06
            self.Arm_controller.move_arm([x, y, z+0.08], pitch=80, roll=rolll, duration=1500)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x, y, z+0.015], pitch=80, roll=rolll, duration=1200)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x, y, z+0.015], pitch=80, roll=rolll, hand=-0.200, duration=500)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x, y, z+0.08], pitch=80, hand=-0.200, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.set_steer([0.0, 0.0, 1.6, 1.200, 0, -0.250], duration=1500)
            time.sleep(2.0)

            self.pick_rect_ready = False
            self.start_turn_yaw = self.gyro_yaw
            self.turn_where = 'left'
            self.turn_90_flag = False
            self.turn_180_flag = True
            self.car_status = 'turnning'

            self.moving = False
            self.catch_stage = 1
        elif self.catch_stage == 1:
            z = -0.08
            self.Arm_controller.move_arm([x, y, z+0.12], pitch=85, hand=-0.200, duration=1500)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x, y, z+0.04], pitch=85, hand=-0.200, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x, y, z+0.04], pitch=85, duration=500)
            time.sleep(1.0)
            self.Arm_controller.move_arm([x, y, z+0.12], pitch=85, duration=1000)
            time.sleep(0.5)
            self.Arm_controller.set_steer([0.0, 0.0, 1.6, 1.200, 0, 0.00], duration=1500)
            time.sleep(2.0)

            self.put_rect_ready = False

            if self.cross_num == 6:
                self.start_turn_yaw = self.gyro_yaw
                self.turn_where = 'left'
                self.turn_90_flag = False
                self.turn_180_flag = True
                self.car_status = 'turnning'
            else:
                self.car_status = 'straight'

            self.cross_keep_choose = 1
            self.catch_stage = 0
            self.moving = False

    def turn_arm(self, where):
        """机械臂转向逻辑"""
        time.sleep(1.0)
        if self.cross_num > 5:
            where = 'right'
        if where == 'left':
            self.Arm_controller.move_arm([0.0, 0.17, 0.10], pitch=85.0, hand=-0.500, duration=1500)
        elif where == 'right':
            self.Arm_controller.move_arm([0.0, -0.17, 0.10], pitch=85.0, hand=-0.500, duration=1500)
        time.sleep(2.0)
        self.put_rect_ready = True

    def image_process(self):
        """核心图像处理+车辆控制逻辑"""
        if self.queue.empty():
            return

        now_time = time.time()
        ros_rgb_image, ros_depth_image = self.queue.get(block=True)

        # 图像格式转换
        rgb_image = np.ndarray(
            shape=(ros_rgb_image.height, ros_rgb_image.width, 3),
            dtype=np.uint8, buffer=ros_rgb_image.data
        )
        depth_image = np.ndarray(
            shape=(ros_depth_image.height, ros_depth_image.width),
            dtype=np.uint16, buffer=ros_depth_image.data
        )
        depth = np.copy(depth_image).reshape((-1,))

        self.image_height, self.image_width = rgb_image.shape[:2]
        result_image = np.copy(rgb_image)
        twist = Twist()

        try:
            self.endpoint = self.get_endpoint()
            # 车道线检测+十字路口检测
            result_image, deflection_angle = line_detect(rgb_image, result_image, rois_line, detect_color_range)
            result_image, rois_areas = cross_rois_area(rgb_image, result_image, rois_cross, detect_color_range)

            # 车辆状态机逻辑
            if self.car_status != 'stop':
                if self.car_status == 'straight' and self.cross_flag == False:
                    if rois_areas['mid_up'] > 2000 and self.cross_first_detect == False:
                        self.cross_first_detect = True
                        self.cross_first_detect_time = time.time()
                    elif rois_areas['mid_down'] > 2000 and self.cross_first_detect == True:
                        self.cross_first_detect = False
                        self.cross_flag = True
                        self.cross_pass_time = time.time()
                        self.start_turn_yaw = self.gyro_yaw

                        # 十字路口决策逻辑
                        if self.cross_num == 0: self.car_status = 'straight'
                        elif self.cross_num == 1:
                            self.car_status = 'waitting'
                            self.pick_rect_on_road = True
                            self.pick_rect_on_road_time = time.time()
                        elif self.cross_num == 2:
                            self.car_status = 'turnning'
                            self.turn_where = 'right'
                            self.turn_90_flag = True
                            self.put_rect_on_road = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 0
                        elif self.cross_num == 3:
                            self.car_status = 'turnning'
                            self.turn_where = 'right'
                            self.turn_90_flag = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 1
                        elif self.cross_num == 4:
                            self.car_status = 'waitting'
                            self.pick_rect_on_road = True
                            self.pick_rect_on_road_time = time.time()
                        elif self.cross_num == 5:
                            self.car_status = 'turnning'
                            self.turn_where = 'left'
                            self.turn_90_flag = True
                            self.put_rect_on_road = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 0
                        elif self.cross_num == 6:
                            self.car_status = 'turnning'
                            self.turn_where = 'left'
                            self.turn_90_flag = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 1
                        elif self.cross_num == 7:
                            self.car_status = 'waitting'
                            self.pick_rect_on_road = True
                            self.pick_rect_on_road_time = time.time()
                        elif self.cross_num == 8:
                            self.car_status = 'turnning'
                            self.turn_where = 'right'
                            self.turn_90_flag = True
                            self.put_rect_on_road = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 0
                        elif self.cross_num == 9:
                            self.car_status = 'turnning'
                            self.turn_where = 'right'
                            self.turn_90_flag = True
                            self.cross_keep_going_time = time.time()
                            self.cross_keep_choose = 1
                        elif self.cross_num == 10:
                            self.car_status = 'waitting'
                            self.back_home_ready = True
                            self.back_home_time = time.time()

                        self.cross_num += 1
                    elif self.cross_first_detect and (time.time() - self.cross_first_detect_time) > 2.0:
                        self.cross_first_detect = False

                if self.cross_flag and (time.time() - self.cross_pass_time) > 2.0:
                    self.cross_flag = False

                # 转向控制逻辑
                if self.car_status == 'turnning' and (time.time() - self.cross_keep_going_time) > self.cross_keep_times[self.cross_keep_choose]:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.35 if self.turn_where == 'left' else -0.35

                    should_turn_angle = 1.43 if self.turn_90_flag else 3.00
                    if abs(self.gyro_yaw - self.start_turn_yaw) > should_turn_angle:
                        self.car_status = 'straight'
                        if self.put_rect_on_road:
                            self.car_status = 'waitting'
                            # 放置物块时间参数
                            if self.cross_num == 3:
                                self.before_put_time_used = 3.00 if color_in_order[self.color_order] == color_ranges['BLUE'] else 4.80 if color_in_order[self.color_order] == color_ranges['RED'] else 3.70
                            elif self.cross_num in [6, 9]:
                                self.before_put_time_used = 4.80 if color_in_order[self.color_order] == color_ranges['BLUE'] else 3.00 if color_in_order[self.color_order] == color_ranges['RED'] else 3.70
                            self.before_put_time = time.time()
                        self.cross_flag = False
                        self.turn_90_flag = False
                        self.turn_180_flag = False
                        self.cross_first_detect = False

                # 抓取/放置事件处理
                if self.car_status == 'waitting':
                    if self.pick_rect_ready or self.put_rect_ready:
                        if not self.moving:
                            result_image, p_y, center, r, roll = color_rect_detect(rgb_image, result_image, color_in_order[self.color_order])
                            if p_y is not None:
                                self.detect_times = 0
                                center_x, center_y = center
                                center_x += 3
                                center_y += (r - 30) if self.catch_stage == 0 else (r - 105)
                                center_x = min(639, max(0, center_x))
                                center_y = min(399, max(0, center_y))

                                dist = depth_image[int(center_y), int(center_x)]
                                if abs(self.last_pitch_yaw[0] - p_y[0]) < 5 and abs(self.last_pitch_yaw[1] - p_y[1]) < 5 and dist > 150:
                                    if (time.time() - self.stamp) > 0.0:
                                        self.stamp = time.time()
                                        dist = dist / 1000.0 + 0.015
                                        K = self.camera_info.k
                                        position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                                        position[0] -= 0.01
                                        temp = position[0]
                                        position[0] = position[1]
                                        position[1] = -temp

                                        # 坐标变换
                                        pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))
                                        world_pose = np.matmul(self.endpoint, pose_end)
                                        pose_t, pose_R = mat_to_xyz_euler(world_pose)
                                        pose_t[1] += -0.005
                                        pose_t[1] *= 1.005

                                        # 启动抓取线程
                                        threading.Thread(target=self.pick, args=(pose_t, roll)).start()
                            else:
                                if self.catch_stage == 0:
                                    self.detect_times += 1
                                    if self.detect_times > 10:
                                        self.color_order = (self.color_order + 1) % len(color_in_order)
                                        self.get_logger().info(f"Color range updated to: {color_in_order[self.color_order]}")
                                        self.detect_times = 0
                                    self.stamp = time.time()

                    elif self.put_rect_on_road and (time.time() - self.before_put_time) > self.before_put_time_used:
                        self.put_rect_on_road = False
                        threading.Thread(target=self.turn_arm, args=('left',)).start()
                    elif self.pick_rect_on_road and (time.time() - self.pick_rect_on_road_time) > 1.0:
                        self.pick_rect_ready = True
                        self.pick_rect_on_road = False
                    elif self.back_home_ready and (time.time() - self.back_home_time) > 3.7:
                        self.back_home_ready = False

            # 车辆速度控制
            turnning_forward = self.car_status == 'turnning' and (time.time() - self.cross_keep_going_time) <= self.cross_keep_times[self.cross_keep_choose]
            waitting_put = self.car_status == 'waitting' and self.put_rect_on_road
            if self.car_status == 'straight' or turnning_forward or waitting_put:
                if deflection_angle is not None:
                    if abs(deflection_angle) > 0.010:
                        self.pid.SetPoint = 0
                        self.pid.update(deflection_angle)
                        self.pid_z += self.pid.output
                        self.pid_z = max(-0.20, min(0.20, self.pid_z))
                    else:
                        self.pid.clear()
                        self.pid_z = 0.0
                    twist.angular.z = self.pid_z
                    twist.linear.x = self.set_speed - 0.03 if twist.angular.z > 0.18 else self.set_speed
                elif self.car_status == 'turnning' and (time.time() - self.cross_keep_going_time) <= self.cross_keep_times[self.cross_keep_choose]:
                    twist.angular.z = 0.0
                    twist.linear.x = self.set_speed
            elif self.car_status == 'waitting' and self.back_home_ready:
                twist.angular.z = 0.0
                twist.linear.x = self.set_speed + 0.01
            elif self.car_status == 'waitting':
                self.pid.clear()
                self.pid_z = 0.0
                twist.angular.z = 0.0
                twist.linear.x = 0.0

            # 发布速度指令（ROS2发布逻辑）
            if self.car_status != 'stop':
                self.cmd_vel_pub.publish(twist)
            else:
                self.pid.clear()
                self.pid_z = 0.0
                self.cmd_vel_pub.publish(Twist())

            # 图像显示与键盘控制
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.putText(result_image, f"chos:{self.cross_keep_choose}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"gap:{round(abs(self.gyro_yaw-self.start_turn_yaw),2)}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"yaw:{round(self.gyro_yaw,2)}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"sta:{self.car_status}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"z:{round(twist.angular.z,2)}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"time:{round(time.time()-now_time,4)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, f"cn:{self.cross_num}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            result_image = cv2.resize(result_image, (320, 240))
            _imshow_fit("image", result_image)
            key = cv2.waitKey(1)

            # 键盘控制（ROS2节点关闭逻辑）
            if key == ord('q'):
                self.cmd_vel_pub.publish(Twist())
                self.destroy_node()
                rclpy.shutdown()
            elif key == ord('s'):
                self.get_logger().info("Stop!")
                self.cmd_vel_pub.publish(Twist())
                self.car_status = 'stop'
            elif key == ord('w'):
                self.get_logger().info("Go!")
                self.car_status = 'straight'

        except Exception as e:
            self.get_logger().error(f"Error in image_process: {str(e)}")

def main(args=None):
    """ROS2节点主函数"""
    rclpy.init(args=args)
    node = Map2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 资源释放
        cv2.destroyAllWindows()
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()