#!/usr/bin/env python3
# coding: utf8

import sys
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist
import car_vision.pid as pid
import queue

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)




class KCFTrackerNode(Node):
    def __init__(self):
        super().__init__('kcf_tracker_node')
        
        # 初始化PID控制器
        self.pid_yaw = pid.PID(0.15, 0.0, 0.0)
        self.pid_pitch = pid.PID(0.15, 0.0, 0.0)
        self.yaw = 0.0
        self.pitch = 0.0

        # 初始化跟踪器和标志
        self.tracker = None
        self.enable_select = False

        # 配置QoS策略
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布器和订阅器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.image_sub = self.create_subscription(
            RosImage,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile=qos_profile
        )

        # 初始化图像处理队列和定时器
        self.queue = queue.Queue(maxsize=1)
        self.timer = self.create_timer(0.001, self.image_proc)

        self.get_logger().info("按下键盘's'键选择追踪目标")

    def image_callback(self, msg):
        if self.queue.empty():
            self.queue.put_nowait(msg)

    def image_proc(self):
        if self.queue.empty():
            return

        try:
            ros_image = self.queue.get_nowait()
            # 转换ROS图像消息为OpenCV格式
            rgb_image = np.ndarray(
                shape=(ros_image.height, ros_image.width, 3),
                dtype=np.uint8,
                buffer=ros_image.data
            )
            result_image = np.copy(rgb_image)
            factor = 4
            small_image = cv2.resize(rgb_image, 
                (ros_image.width//factor, ros_image.height//factor))

            twist = Twist()
            
            if self.tracker is None:
                if self.enable_select:
                    # 选择ROI区域
                    roi = cv2.selectROI("Tracking", 
                        cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), False)
                    if roi != (0,0,0,0):
                        scaled_roi = tuple(int(x/factor) for x in roi)
                        # 初始化KCF跟踪器
                        params = cv2.TrackerKCF.Params()
                        params.detect_thresh = 0.15
                        self.tracker = cv2.TrackerKCF_create(params)
                        self.tracker.init(small_image, scaled_roi)
                        self.enable_select = False
                        # 等待跟踪器稳定
                        time.sleep(2)
            else:
                # 更新跟踪器
                success, box = self.tracker.update(small_image)
                if success:
                    # 计算目标中心坐标
                    p1 = (int(box[0]*factor), int(box[1]*factor))
                    p2 = (int((box[0]+box[2])*factor), 
                         int((box[1]+box[3])*factor))
                    cv2.rectangle(result_image, p1, p2, (255,255,0), 2)
                    
                    # 计算归一化坐标
                    center_x = (p1[0] + p2[0]) / 2 / result_image.shape[1]
                    center_y = (p1[1] + p2[1]) / 2 / result_image.shape[0]

                    # Yaw轴控制
                    if abs(center_x - 0.5) > 0.02:
                        self.pid_yaw.SetPoint = 0.5
                        self.pid_yaw.update(center_x)
                        self.yaw += self.pid_yaw.output
                        self.yaw = np.clip(self.yaw, -0.5, 0.5)
                    else:
                        self.pid_yaw.clear()
                        self.yaw = 0.0

                    # Pitch轴控制
                    if abs(center_y - 0.5) > 0.02:
                        self.pid_pitch.SetPoint = 0.5
                        self.pid_pitch.update(center_y)
                        self.pitch += self.pid_pitch.output
                        self.pitch = np.clip(self.pitch, -0.08, 0.08)
                    else:
                        self.pid_pitch.clear()
                        self.pitch = 0.0

                    # 发布控制指令
                    twist.linear.x = self.pitch
                    twist.angular.z = self.yaw
                    self.cmd_vel_pub.publish(twist)
                else:
                    self.cmd_vel_pub.publish(Twist())

            # 显示处理结果
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            _imshow_fit("Tracking", result_image)
            key = cv2.waitKey(1)

            # 处理键盘输入
            if key == ord('s'):
                self.tracker = None
                self.enable_select = True
            elif key == ord('q'):
                self.cmd_vel_pub.publish(Twist())
                raise KeyboardInterrupt

        except Exception as e:
            self.get_logger().error(f"处理错误: {str(e)}")
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = KCFTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
