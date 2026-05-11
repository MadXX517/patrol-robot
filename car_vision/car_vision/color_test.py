import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from threading import Thread
import queue

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



class WebcamThresholdApp:
    def __init__(self):
        self.mode = "HSV"  # 默认模式为HSV
        self.is_stopped = False
        self.locked_frame = None
        self.bridge = CvBridge()
        # 仅保留HSV和LAB的阈值配置
        self.threshold_values = {
            "HSV": [100, 255, 100, 255, 100, 255], 
            "LAB": [100, 255, 100, 255, 100, 255]
        }
        self.queue = queue.Queue(maxsize=3)
        
        # 创建单个合并窗口
        cv2.namedWindow('Color Threshold App', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Color Threshold App', 1400, 850)  # 适配单行文字高度
        cv2.moveWindow('Color Threshold App', 0, 0)
        
        # 创建控制条（滑块添加模式前缀，分组更清晰）
        self.create_trackbars()
        # 初始化激活默认模式滑块
        self.update_trackbar_visibility(self.mode)
        
        # ROS 2节点初始化
        rclpy.init()
        self.node = Node("color_node")
        self.node.create_subscription(RosImage, '/camera/color/image_raw', self.camera_callback, qos_profile_sensor_data)
        self.node.create_subscription(RosImage, '/usb_cam/image_raw', self.camera_callback, qos_profile_sensor_data)
        
        # 启动ROS线程
        self.ros_thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def create_trackbars(self):
        """创建控制条（滑块添加模式前缀，区分HSV/LAB组）"""
        # 模式选择控制条（0:HSV, 1:LAB）
        cv2.createTrackbar('Mode (0:HSV, 1:LAB)', 'Color Threshold App', 0, 1, self.on_mode_changed)
        
        # HSV模式滑块（添加HSV前缀，明确分组）
        cv2.createTrackbar('HSV - Min H', 'Color Threshold App', self.threshold_values["HSV"][0], 179, self.on_threshold_changed)
        cv2.createTrackbar('HSV - Max H', 'Color Threshold App', self.threshold_values["HSV"][1], 179, self.on_threshold_changed)
        cv2.createTrackbar('HSV - Min S', 'Color Threshold App', self.threshold_values["HSV"][2], 255, self.on_threshold_changed)
        cv2.createTrackbar('HSV - Max S', 'Color Threshold App', self.threshold_values["HSV"][3], 255, self.on_threshold_changed)
        cv2.createTrackbar('HSV - Min V', 'Color Threshold App', self.threshold_values["HSV"][4], 255, self.on_threshold_changed)
        cv2.createTrackbar('HSV - Max V', 'Color Threshold App', self.threshold_values["HSV"][5], 255, self.on_threshold_changed)
        
        # LAB模式滑块（添加LAB前缀，明确分组）
        cv2.createTrackbar('LAB - Min L', 'Color Threshold App', self.threshold_values["LAB"][0], 255, self.on_threshold_changed)
        cv2.createTrackbar('LAB - Max L', 'Color Threshold App', self.threshold_values["LAB"][1], 255, self.on_threshold_changed)
        cv2.createTrackbar('LAB - Min A', 'Color Threshold App', self.threshold_values["LAB"][2], 255, self.on_threshold_changed)
        cv2.createTrackbar('LAB - Max A', 'Color Threshold App', self.threshold_values["LAB"][3], 255, self.on_threshold_changed)
        cv2.createTrackbar('LAB - Min B', 'Color Threshold App', self.threshold_values["LAB"][4], 255, self.on_threshold_changed)
        cv2.createTrackbar('LAB - Max B', 'Color Threshold App', self.threshold_values["LAB"][5], 255, self.on_threshold_changed)

    def update_trackbar_visibility(self, active_mode):
        """更新控制条可见性（仅处理HSV和LAB）"""
        # 所有滑块默认设为不可用
        # HSV滑块（对应带前缀的名称）
        cv2.setTrackbarMin('HSV - Min H', 'Color Threshold App', 0)
        cv2.setTrackbarMax('HSV - Min H', 'Color Threshold App', 0)
        cv2.setTrackbarMin('HSV - Max H', 'Color Threshold App', 0)
        cv2.setTrackbarMax('HSV - Max H', 'Color Threshold App', 0)
        for trackbar in ['HSV - Min S', 'HSV - Max S', 'HSV - Min V', 'HSV - Max V']:
            cv2.setTrackbarMin(trackbar, 'Color Threshold App', 0)
            cv2.setTrackbarMax(trackbar, 'Color Threshold App', 0)
        
        # LAB滑块（对应带前缀的名称）
        for trackbar in ['LAB - Min L', 'LAB - Max L', 'LAB - Min A', 'LAB - Max A', 'LAB - Min B', 'LAB - Max B']:
            cv2.setTrackbarMin(trackbar, 'Color Threshold App', 0)
            cv2.setTrackbarMax(trackbar, 'Color Threshold App', 0)
        
        # 激活当前模式的滑块
        if active_mode == "HSV":
            cv2.setTrackbarMin('HSV - Min H', 'Color Threshold App', 0)
            cv2.setTrackbarMax('HSV - Min H', 'Color Threshold App', 179)
            cv2.setTrackbarMin('HSV - Max H', 'Color Threshold App', 0)
            cv2.setTrackbarMax('HSV - Max H', 'Color Threshold App', 179)
            for trackbar in ['HSV - Min S', 'HSV - Max S', 'HSV - Min V', 'HSV - Max V']:
                cv2.setTrackbarMin(trackbar, 'Color Threshold App', 0)
                cv2.setTrackbarMax(trackbar, 'Color Threshold App', 255)
        elif active_mode == "LAB":
            for trackbar in ['LAB - Min L', 'LAB - Max L', 'LAB - Min A', 'LAB - Max A', 'LAB - Min B', 'LAB - Max B']:
                cv2.setTrackbarMin(trackbar, 'Color Threshold App', 0)
                cv2.setTrackbarMax(trackbar, 'Color Threshold App', 255)
        
        # 重新加载当前模式阈值
        self.load_threshold_values()

    def on_threshold_changed(self, value):
        """阈值改变回调（仅处理HSV和LAB）"""
        self.update_threshold_values()

    def on_mode_changed(self, value):
        """模式改变回调（仅HSV和LAB切换）"""
        modes = ["HSV", "LAB"]
        self.mode = modes[value]
        self.update_trackbar_visibility(self.mode)
        self.load_threshold_values()
        print(f"Mode changed to: {self.mode}")

    def camera_callback(self, ros_rgb_image):
        """ROS图像回调（保持不变）"""
        if self.queue.empty():
            self.queue.put_nowait(ros_rgb_image)

    def update_threshold_values(self):
        """更新阈值（适配带前缀的滑块名称）"""
        try:
            if self.mode == "HSV":
                self.threshold_values[self.mode] = [
                    cv2.getTrackbarPos('HSV - Min H', 'Color Threshold App'),
                    cv2.getTrackbarPos('HSV - Max H', 'Color Threshold App'),
                    cv2.getTrackbarPos('HSV - Min S', 'Color Threshold App'),
                    cv2.getTrackbarPos('HSV - Max S', 'Color Threshold App'),
                    cv2.getTrackbarPos('HSV - Min V', 'Color Threshold App'),
                    cv2.getTrackbarPos('HSV - Max V', 'Color Threshold App')
                ]
            elif self.mode == "LAB":
                self.threshold_values[self.mode] = [
                    cv2.getTrackbarPos('LAB - Min L', 'Color Threshold App'),
                    cv2.getTrackbarPos('LAB - Max L', 'Color Threshold App'),
                    cv2.getTrackbarPos('LAB - Min A', 'Color Threshold App'),
                    cv2.getTrackbarPos('LAB - Max A', 'Color Threshold App'),
                    cv2.getTrackbarPos('LAB - Min B', 'Color Threshold App'),
                    cv2.getTrackbarPos('LAB - Max B', 'Color Threshold App')
                ]
        except cv2.error:
            # Trackbar state may be transient while switching modes; keep previous values.
            return

    def load_threshold_values(self):
        """加载阈值（适配带前缀的滑块名称）"""
        if self.mode == "HSV":
            cv2.setTrackbarPos('HSV - Min H', 'Color Threshold App', self.threshold_values[self.mode][0])
            cv2.setTrackbarPos('HSV - Max H', 'Color Threshold App', self.threshold_values[self.mode][1])
            cv2.setTrackbarPos('HSV - Min S', 'Color Threshold App', self.threshold_values[self.mode][2])
            cv2.setTrackbarPos('HSV - Max S', 'Color Threshold App', self.threshold_values[self.mode][3])
            cv2.setTrackbarPos('HSV - Min V', 'Color Threshold App', self.threshold_values[self.mode][4])
            cv2.setTrackbarPos('HSV - Max V', 'Color Threshold App', self.threshold_values[self.mode][5])
        elif self.mode == "LAB":
            cv2.setTrackbarPos('LAB - Min L', 'Color Threshold App', self.threshold_values[self.mode][0])
            cv2.setTrackbarPos('LAB - Max L', 'Color Threshold App', self.threshold_values[self.mode][1])
            cv2.setTrackbarPos('LAB - Min A', 'Color Threshold App', self.threshold_values[self.mode][2])
            cv2.setTrackbarPos('LAB - Max A', 'Color Threshold App', self.threshold_values[self.mode][3])
            cv2.setTrackbarPos('LAB - Min B', 'Color Threshold App', self.threshold_values[self.mode][4])
            cv2.setTrackbarPos('LAB - Max B', 'Color Threshold App', self.threshold_values[self.mode][5])

    def save_threshold_values(self):
        """保存阈值（适配仅有的两种模式）"""
        self.update_threshold_values()
        print(f"Threshold values saved for {self.mode} mode: {self.threshold_values[self.mode]}")

    def create_combined_display(self, original_frame, thresholded_frame):
        """创建合并显示（阈值文字合并为一行）"""
        display_width = 600
        display_height = 400
        
        original_resized = cv2.resize(original_frame, (display_width, display_height))
        thresholded_resized = cv2.resize(thresholded_frame, (display_width, display_height))
        
        if len(thresholded_resized.shape) == 2:
            thresholded_resized = cv2.cvtColor(thresholded_resized, cv2.COLOR_GRAY2BGR)
        
        # 调整合并图像高度，适配单行阈值文字
        combined_height = 450
        combined_width = display_width * 2
        combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)
        
        # 放置原始图像和阈值图像（顶部对齐）
        combined_image[:display_height, :display_width] = original_resized
        combined_image[:display_height, display_width:] = thresholded_resized
        
        # 添加基础标题
        cv2.putText(combined_image, 'Original Image', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined_image, 'Thresholded Image', (display_width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 单行显示当前模式和阈值参数（缩小字体确保完整显示）
        threshold_text = self.get_current_threshold_text()
        cv2.putText(combined_image, threshold_text, (10, combined_height - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 1)
        
        return combined_image

    def get_current_threshold_text(self):
        """获取当前模式的阈值文本（合并为一行）"""
        if self.mode == "HSV":
            h_min, h_max, s_min, s_max, v_min, v_max = self.threshold_values["HSV"]
            return f'Current Mode: HSV | H: [{h_min:3d}-{h_max:3d}] | S: [{s_min:3d}-{s_max:3d}] | V: [{v_min:3d}-{v_max:3d}]'
        elif self.mode == "LAB":
            l_min, l_max, a_min, a_max, b_min, b_max = self.threshold_values["LAB"]
            return f'Current Mode: LAB | L: [{l_min:3d}-{l_max:3d}] | A: [{a_min:3d}-{a_max:3d}] | B: [{b_min:3d}-{b_max:3d}]'

    def update_display(self):
        """更新显示（仅保留HSV和LAB阈值处理）"""
        if self.is_stopped:
            if self.locked_frame is not None:
                frame = self.locked_frame.copy()
            else:
                return
        else:
            try:
                ros_rgb_image = self.queue.get(block=False)
                frame = self.bridge.imgmsg_to_cv2(ros_rgb_image, 'rgb8')
                frame = np.copy(frame)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.locked_frame = frame.copy()
            except queue.Empty:
                return

        # 仅处理HSV和LAB模式的阈值计算
        if self.mode == "HSV":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            low = np.array([cv2.getTrackbarPos('HSV - Min H', 'Color Threshold App'), 
                           cv2.getTrackbarPos('HSV - Min S', 'Color Threshold App'), 
                           cv2.getTrackbarPos('HSV - Min V', 'Color Threshold App')])
            up = np.array([cv2.getTrackbarPos('HSV - Max H', 'Color Threshold App'), 
                          cv2.getTrackbarPos('HSV - Max S', 'Color Threshold App'), 
                          cv2.getTrackbarPos('HSV - Max V', 'Color Threshold App')])
            thresholded = cv2.inRange(hsv, low, up)
        elif self.mode == "LAB":
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            low = np.array([cv2.getTrackbarPos('LAB - Min L', 'Color Threshold App'), 
                           cv2.getTrackbarPos('LAB - Min A', 'Color Threshold App'), 
                           cv2.getTrackbarPos('LAB - Min B', 'Color Threshold App')])
            up = np.array([cv2.getTrackbarPos('LAB - Max L', 'Color Threshold App'), 
                          cv2.getTrackbarPos('LAB - Max A', 'Color Threshold App'), 
                          cv2.getTrackbarPos('LAB - Max B', 'Color Threshold App')])
            thresholded = cv2.inRange(lab, low, up)

        combined_image = self.create_combined_display(frame, thresholded)
        _imshow_fit('Color Threshold App', combined_image)

    def run(self):
        """运行主循环（更新提示信息）"""
        print("Starting color threshold application...")
        print("Controls:")
        print("- Use the 'Mode' trackbar to switch between HSV (0) and LAB (1)")
        print("- Adjust threshold values using the prefixed trackbars (only active mode's sliders work)")
        print("- Threshold values are displayed in one line on the image interface")
        print("- Press 'q' to quit, 's' to save values, 'r' to reset values")
        
        while True:
            self.update_display()
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_threshold_values()
            elif key == ord('r'):
                self.load_threshold_values()
        
        # 清理资源
        cv2.destroyAllWindows()
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    app = WebcamThresholdApp()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nApplication interrupted by user")
    finally:
        cv2.destroyAllWindows()
        if hasattr(app, 'node'):
            app.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()