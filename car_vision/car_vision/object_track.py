#!/usr/bin/python3
# coding=utf8
import os
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # 导入Twist消息类型
from cv_bridge import CvBridge
import numpy as np
import car_vision.pid as pid
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world
from pathlib import Path

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



class ObjectTracker:
    # 保持原有跟踪逻辑不变
    def __init__(self, node, use_mouse=False, automatic=False): 
        self.node = node
        self.desired_size = 200  # 目标期望尺寸（像素，基于宽高最大值）
        self.start_track = False
        self.automatic = automatic
        self.use_mouse = use_mouse
        if self.use_mouse:
            self.name = 'image'
            cv2.namedWindow(self.name, 1)
            cv2.setMouseCallback(self.name, self.onmouse)
        
        # 初始化跟踪器
        self.params = cv2.TrackerNano_Params()
        model_path = str(Path(__file__).parent)
        self.node.get_logger().info(f'Model path: {model_path}')
        self.params.backbone = os.path.join(model_path, 'weights/nanotrack_backbone_sim.onnx')
        self.params.neckhead = os.path.join(model_path, 'weights/nanotrack_head_sim.onnx')
        self.tracker = cv2.TrackerNano_create(self.params)
        
        # 鼠标交互相关变量
        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False

        # 速度控制变量
        self.linear_speed = 0
        self.linear_base_speed = 0.007
        self.angular_speed = 0
        self.angular_base_speed = 0.03
        
        # PID初始化（线性控制基于目标尺寸， angular控制基于水平位置）
        self.linear_pid = pid.PID(0.005, 0.0, 0.0)
        self.angular_pid  = pid.PID(0.003, 0.0, 0.0)

    def set_init_param(self, linear_pid, angular_pid): 
        self.linear_pid = linear_pid
        self.angular_pid = angular_pid

    def update_pid(self, p1, p2):
        self.linear_pid = pid.PID(p1[0], p1[1], p1[2])
        self.angular_pid = pid.PID(p2[0], p2[1], p2[2])

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # 记录鼠标拖动位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:  # 右键重置跟踪
            self.mouse_click = False
            self.selection = None
            self.track_window = None
            self.drag_start = None
            self.start_circle = True
            self.start_click = False
            self.tracker = cv2.TrackerNano_create(self.params)

    def set_track_target(self, target, image):
        self.start_circle = False
        self.start_track = True
        self.tracker.init(image, target)

    def stop(self):
        self.start_circle = False
        self.tracker = cv2.TrackerNano_create(self.params)

    def get_target(self, image):
        if self.start_circle and self.use_mouse and not self.automatic:
            # 鼠标拖拽框选目标
            h, w = image.shape[:2]
            if self.track_window:  # 显示已选定的跟踪框
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # 显示拖拽中的框
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 255, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click and not self.mouse_click:
                self.start_circle = False
            if not self.start_circle:
                self.node.get_logger().info('Start tracking')
                # 初始化跟踪器（转换为(x, y, w, h)格式）
                bbox = (self.track_window[0], self.track_window[1], 
                        self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
                self.start_track = True
        else:
            if not self.start_circle:
                ok, box = self.tracker.update(image)
                if ok and min(box) > 0:
                    return image, box
                else:
                    # 跟踪失败提示
                    cv2.putText(image, "Tracking failure detected !", (10, 460), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        return image, None

    def track(self, image):
        image, box = self.get_target(image)
        if box is not None:
            img_h, img_w = image.shape[:2]
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

            # 绘制跟踪框和中心点
            cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            center_x = (p1[0] + p2[0]) / 2
            center_y = (p1[1] + p2[1]) / 2
            cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)

            # 初始化时设置期望尺寸（首次跟踪时）
            if self.start_track:
                self.start_track = False
                # 根据初始目标大小动态调整期望尺寸
                init_size = max(box[2], box[3])
                self.desired_size = min(init_size * 1.5, img_w * 0.6)  # 不超过图像宽度60%

            # 线性速度控制（基于目标尺寸与期望尺寸的偏差）
            current_size = max(box[2], box[3])  # 当前目标的最大尺寸
            self.linear_pid.SetPoint = self.desired_size
            if abs(current_size - self.desired_size) < 10:  # 接近目标尺寸时视为到达
                current_size = self.desired_size
            self.linear_pid.update(current_size)
            
            # 计算线性速度
            tmp = self.linear_pid.output
            self.linear_speed = tmp
            # 速度限制
            self.linear_speed = np.clip(self.linear_speed, -0.0, 0.0)
            if abs(self.linear_speed) <= 0.0075:  # 微小速度视为停止
                self.linear_speed = 0

            # 角速度控制（基于目标中心与图像中心的水平偏差）
            if abs(center_x - img_w/2.0) < 25:  # 接近中心时视为对准
                center_x = img_w / 2.0
            self.angular_pid.SetPoint = img_w / 2.0
            self.angular_pid.update(center_x)
            
            tmp = self.angular_pid.output
            self.angular_speed = tmp
            # 角速度限制
            self.angular_speed = np.clip(self.angular_speed, -1.2, 1.2)
            if abs(self.angular_speed) <= 0.038:  # 微小角度视为对准
                self.angular_speed = 0

            return float(self.linear_speed), float(self.angular_speed), image
        else:
            return 0.0, 0.0, image


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        self.bridge = CvBridge()
        self.Arm_controller=arm_ik_sdk.ArmControl(self) # 初始化控制SDK，本包括坐标系变换、臂和抓手控制、获取节点状态  详情看arm_ik_sdk.py
        time.sleep(2)
        self.Arm_controller.set_steer([0,-0.930,1.6,1.200,0,0.801])
        time.sleep(1)
        self.tracker = ObjectTracker(self, use_mouse=True)
        # 订阅图像话题
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        # 创建cmd_vel发布者（控制底盘运动）
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',  # 标准底盘控制话题
            10)
        
        self.get_logger().info('目标跟踪节点已启动，按q键退出')
        self.exit_flag = False  # 退出标志位

    def image_callback(self, msg):
        if self.exit_flag:
            return
        
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {str(e)}')
            return

        # 进行目标跟踪，获取速度指令
        linear_speed, angular_speed, frame = self.tracker.track(cv_image)
        
        # 构建Twist消息（底盘控制命令）
        twist = Twist()
        # 线性速度：仅x方向有效（前进/后退）
        twist.linear.x = linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        # 角速度：仅z方向有效（旋转）
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed
        
        # 发布控制命令
        self.cmd_vel_pub.publish(twist)
        
        # 在图像上显示退出提示
        cv2.putText(frame, "Press 'q' to exit", (10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 显示处理后的图像并检测按键
        _imshow_fit('image', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('检测到q键，退出节点')
            self.exit_flag = True
            # 发布零速度指令，确保底盘停止
            twist_zero = Twist()
            self.cmd_vel_pub.publish(twist_zero)
            # 关闭窗口并终止节点
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，退出节点')
    finally:
        # 停止时发布零速度，确保底盘停止
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()