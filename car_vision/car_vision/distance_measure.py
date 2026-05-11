#!/usr/bin/env python3
# coding=utf8

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import queue
from rclpy.executors import SingleThreadedExecutor
import threading

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)




class RgbDepthImageNode(Node):
    def __init__(self):
        super().__init__('g1et_rgb_and_depth_image')
        
        # 初始化服务客户端并调用服务
        # self.cli = self.create_client(SetBool, '/camera/set_ldp')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('等待服务可用...')
        # req = SetBool.Request()
        # req.data = False
        # future = self.cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info('关闭LDP成功' if future.result().success else '关闭LDP失败')
        
        # 配置QoS策略
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建订阅者并同步
        rgb_sub = Subscriber(self, RosImage, '/camera/color/image_raw', qos_profile=qos_policy)
        depth_sub = Subscriber(self, RosImage, '/camera/depth/image_raw', qos_profile=qos_policy)
        
        self.ts = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=2, slop=0.03)
        self.ts.registerCallback(self.multi_callback)
        
        # 初始化图像处理相关
        self.queue = queue.Queue(maxsize=1)
        self.target_point = None
        self.last_event = 0
        cv2.namedWindow("depth")
        cv2.setMouseCallback('depth', self.click_callback)

    def click_callback(self, event, x, y, flags, params):
        if event in (cv2.EVENT_RBUTTONDOWN, cv2.EVENT_MBUTTONDOWN, cv2.EVENT_LBUTTONDBLCLK):
            self.target_point = None
        if event == cv2.EVENT_LBUTTONDOWN and self.last_event != cv2.EVENT_LBUTTONDBLCLK:
            if x >= 640:
                self.target_point = (x - 640, y)
            else:
                self.target_point = (x, y)
        self.last_event = event

    def multi_callback(self, ros_rgb, ros_depth):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb, ros_depth))
    
    def process_images(self):
        try:
            ros_rgb, ros_depth = self.queue.get_nowait()
        except queue.Empty:
            return
        
        # 转换图像消息为numpy数组
        rgb_image = np.frombuffer(ros_rgb.data, dtype=np.uint8).reshape(
            ros_rgb.height, ros_rgb.width, 3)
        depth_image = np.frombuffer(ros_depth.data, dtype=np.uint16).reshape(
            ros_depth.height, ros_depth.width)
        
        h, w = depth_image.shape[:2]
        
        depth = np.copy(depth_image).reshape((-1, ))
        depth[depth<=0] = 55555
        min_index = np.argmin(depth)
        min_y = min_index // w
        min_x = min_index - min_y * w
        # print(h,w,min_index,min_x,min_y)
        print(depth_image.shape)
        print(min_x,min_y)
        if self.target_point is not None:
            min_x, min_y = self.target_point
        min_x = int(((min_x + 10)-(w//2)) / 1.26 + (w//2))
        min_y = int((min_y-(h//2)) / 1.26+(h//2))
        # 生成深度伪彩色图
        sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
        depth_color = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
        
        # 绘制目标点和文字
        txt = f'Dist: {depth_image[min_y, min_x]}mm'
        cv2.circle(depth_color, (min_x, min_y), 8, (32,32,32), -1)
        cv2.circle(depth_color, (min_x, min_y), 6, (255,255,255), -1)
        cv2.putText(depth_color, txt, (11, 200), cv2.FONT_HERSHEY_PLAIN, 2.0, (32,32,32), 6)
        cv2.putText(depth_color, txt, (10, 200), cv2.FONT_HERSHEY_PLAIN, 2.0, (240,240,240), 2)
        
        # 处理RGB图像
        bgr_image = cv2.cvtColor(rgb_image[40:440, :], cv2.COLOR_RGB2BGR)
        cv2.circle(bgr_image, (min_x, min_y), 8, (32,32,32), -1)
        cv2.circle(bgr_image, (min_x, min_y), 6, (255,255,255), -1)
        cv2.putText(bgr_image, txt, (11, h-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32,32,32), 6)
        cv2.putText(bgr_image, txt, (10, h-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240,240,240), 2)
        
        # 拼接并显示图像
        if bgr_image.shape[0] != depth_color.shape[0]:
            height = min(bgr_image.shape[0], depth_color.shape[0])
            bgr_image = bgr_image[:height, :, :]
            depth_color = depth_color[:height, :, :]
        combined = np.concatenate([bgr_image, depth_color], axis=1)
        _imshow_fit("depth", combined)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RgbDepthImageNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # 使用线程处理ROS2的spin
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            node.process_images()
            if cv2.waitKey(40) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
