#!/usr/bin/env python3

# 图片收集 (ROS 2 版本)

import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # ROS 2 中仍然需要 cv_bridge 进行格式转换

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



class PictureNode(Node):
    def __init__(self):
        # 1. 初始化节点，使用 Node 基类
        super().__init__("picture_node")

        # 创建 CvBridge 实例 (ROS 2 中用法不变)
        self.bridge = CvBridge()

        # 保存图片的文件夹
        self.save_folder = 'garbage'
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
            self.get_logger().info(f"创建文件夹: {self.save_folder}")

        # 初始化图片编号
        self.image_counter = 1183

        # 2. 创建订阅者 (ROS 2 中使用 create_subscription)
        # 参数分别为：消息类型、话题名、回调函数、QoS 配置
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 话题名保持不变
            self.image_callback,
            10  # QoS profile, 10 是队列大小
        )
        self.image_sub  # 防止未使用变量的警告

        # 视频录制相关的标志
        self.video_start_flag = False
        self.video_file_created = False
        self.out = None  # VideoWriter 实例

        # 创建一个定时器来定期显示图像，避免在回调中阻塞
        # ROS 2 中推荐使用定时器来处理 GUI 相关操作
        self.timer = self.create_timer(0.03, self.show_image)  # 约 30fps

        # 用于存储最新的图像
        self.latest_image = None

    def image_callback(self, ros_image):
        """ROS 图像消息回调函数"""
        try:
            # 3. 将 ROS Image 消息转换为 OpenCV 格式
            # ROS 2 中 cv_bridge 的用法基本不变
            # 注意：需要根据实际图像格式调整转换方式
            # 如果是 RGB 格式：
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')
            # 转换为 BGR 格式用于 OpenCV 显示和保存
            self.latest_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # 视频录制逻辑
            if self.video_start_flag:
                if not self.video_file_created:
                    # 创建 VideoWriter 实例
                    # 注意：需要根据实际图像尺寸调整
                    height, width = self.latest_image.shape[:2]
                    self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    self.out = cv2.VideoWriter('output2.avi', self.fourcc, 20.0, (width, height))
                    self.video_file_created = True
                    self.get_logger().info("创建视频文件: output2.avi")
                
                # 写入帧
                self.out.write(self.latest_image)

        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def show_image(self):
        """显示图像并处理键盘事件"""
        if self.latest_image is None:
            return

        # 调整图像大小以适应显示窗口
        height, width = self.latest_image.shape[:2]
        show_image = cv2.resize(self.latest_image, (int(width/2), int(height/2)))
        
        # 显示图像
        _imshow_fit('image', show_image)
        
        # 处理键盘事件
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            # 保存图片
            image_path = os.path.join(self.save_folder, f'garbage_{self.image_counter:04d}.png')
            cv2.imwrite(image_path, self.latest_image)
            self.get_logger().info(f"图片已保存为：{image_path} 序号：{self.image_counter}")
            self.image_counter += 1
        elif key == ord('g'):
            # 开始录制
            self.video_start_flag = True
            self.get_logger().info("开始录制视频")
        elif key == ord('p'):
            # 暂停录制
            self.video_start_flag = False
            self.get_logger().info("暂停录制视频")
        elif key == ord('q'):
            # 退出程序
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        """节点销毁时的清理工作"""
        # 释放视频写入资源
        if self.out is not None and self.out.isOpened():
            self.out.release()
            self.get_logger().info("视频文件已保存并释放资源")
        
        # 关闭 OpenCV 窗口
        cv2.destroyAllWindows()
        self.get_logger().info("关闭所有窗口")
        
        # 调用父类的销毁方法
        super().destroy_node()

def main(args=None):
    # 4. 初始化 rclpy
    rclpy.init(args=args)

    # 创建节点实例
    picture_node = PictureNode()

    # 5.  spin 节点，开始处理回调
    rclpy.spin(picture_node)

    # 6. 关闭 rclpy
    rclpy.shutdown()

if __name__ == "__main__":
    main()