import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QSlider, QPushButton, QGridLayout)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import QSizePolicy

class ImageProcessor(Node):
    """ROS2节点，负责处理图像和阈值计算"""
    def __init__(self):
        super().__init__('color_thresholder')
        
        # 订阅原始图像话题
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        self.image_sub2 = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        
        # 发布阈值化后的图像
        self.publisher = self.create_publisher(Image, 'thresholded_image', 10)
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 初始阈值设置
        self.color_space = 'HSV'  # 默认使用HSV
        self.h_low, self.h_high = 0, 179
        self.s_low, self.s_high = 0, 255
        self.v_low, self.v_high = 0, 255
        self.l_low, self.l_high = 0, 255
        self.a_low, self.a_high = 0, 255
        self.b_low, self.b_high = 0, 255
        
        # 当前图像
        self.current_image = None
        
    def set_color_space(self, space):
        """设置颜色空间"""
        self.color_space = space
        
    def set_hsv_thresholds(self, h_low, h_high, s_low, s_high, v_low, v_high):
        """设置HSV阈值"""
        self.h_low, self.h_high = h_low, h_high
        self.s_low, self.s_high = s_low, s_high
        self.v_low, self.v_high = v_low, v_high
        
    def set_lab_thresholds(self, l_low, l_high, a_low, a_high, b_low, b_high):
        """设置LAB阈值"""
        self.l_low, self.l_high = l_low, l_high
        self.a_low, self.a_high = a_low, a_high
        self.b_low, self.b_high = b_low, b_high
        
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # 处理图像并获取阈值化结果
            thresholded_image = self.process_image(cv_image)
            
            # 发布阈值化后的图像
            try:
                img_msg = self.bridge.cv2_to_imgmsg(thresholded_image, "bgr8")
                img_msg.header = msg.header
                self.publisher.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"转换为ROS图像消息失败: {e}")
                
        except CvBridgeError as e:
            self.get_logger().error(f"从ROS图像消息转换失败: {e}")
    
    def process_image(self, cv_image):
        """处理图像，应用阈值"""
        if self.color_space == 'HSV':
            # 转换为HSV颜色空间
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 应用HSV阈值
            lower = np.array([self.h_low, self.s_low, self.v_low])
            upper = np.array([self.h_high, self.s_high, self.v_high])
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # 将掩码应用到原始图像
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            return result
            
        elif self.color_space == 'LAB':
            # 转换为LAB颜色空间
            lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
            
            # 应用LAB阈值
            lower = np.array([self.l_low, self.a_low, self.b_low])
            upper = np.array([self.l_high, self.a_high, self.b_high])
            mask = cv2.inRange(lab_image, lower, upper)
            
            # 将掩码应用到原始图像
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            return result
            
        return cv_image

class ROS2Thread(QThread):
    """ROS2运行线程，避免阻塞QT界面"""
    image_updated = pyqtSignal(np.ndarray, np.ndarray)  # 原始图像和处理后图像信号
    
    def __init__(self, processor):
        super().__init__()
        self.processor = processor
        self.running = True
        
    def run(self):
        """运行ROS2节点"""
        while self.running and rclpy.ok():
            rclpy.spin_once(self.processor, timeout_sec=0.1)
            
            # 如果有新图像，处理并发送信号
            if self.processor.current_image is not None:
                original = self.processor.current_image
                processed = self.processor.process_image(original)
                self.image_updated.emit(original, processed)
    
    def stop(self):
        """停止线程"""
        self.running = False
        self.wait()

class ThresholdAdjuster(QMainWindow):
    """QT界面，用于调整颜色阈值"""
    def __init__(self, processor):
        super().__init__()
        self.processor = processor
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('颜色阈值调节器')
        self.setGeometry(100, 100, 1000, 800)
        
        # 主布局
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)
        self.setCentralWidget(main_widget)
        
        # 图像显示区域
        image_layout = QHBoxLayout()
        
        # 原始图像
        self.original_label = QLabel('原始图像')
        self.original_label.setAlignment(Qt.AlignCenter)
        self.original_label.setMinimumSize(320, 240)
        image_layout.addWidget(self.original_label)
        
        # 处理后图像
        self.processed_label = QLabel('阈值化图像')
        self.processed_label.setAlignment(Qt.AlignCenter)
        self.processed_label.setMinimumSize(320, 240)
        image_layout.addWidget(self.processed_label)
        
        main_layout.addLayout(image_layout)
        
        # 颜色空间切换按钮
        space_layout = QHBoxLayout()
        self.hsv_btn = QPushButton('HSV')
        self.lab_btn = QPushButton('LAB')
        self.hsv_btn.setCheckable(True)
        self.lab_btn.setCheckable(True)
        self.hsv_btn.setChecked(True)  # 默认选中HSV
        
        self.hsv_btn.clicked.connect(lambda: self.set_color_space('HSV'))
        self.lab_btn.clicked.connect(lambda: self.set_color_space('LAB'))
        
        space_layout.addWidget(self.hsv_btn)
        space_layout.addWidget(self.lab_btn)
        main_layout.addLayout(space_layout)
        
        # 滑动条布局
        self.sliders = {}
        slider_layout = QGridLayout()
        
        # HSV滑动条
        self.add_slider(slider_layout, 'H_low', 0, 179, 0, 0, 0)
        self.add_slider(slider_layout, 'H_high', 0, 179, 179, 0, 1)
        self.add_slider(slider_layout, 'S_low', 0, 255, 0, 1, 0)
        self.add_slider(slider_layout, 'S_high', 0, 255, 255, 1, 1)
        self.add_slider(slider_layout, 'V_low', 0, 255, 0, 2, 0)
        self.add_slider(slider_layout, 'V_high', 0, 255, 255, 2, 1)
        
        # LAB滑动条 (初始隐藏)
        self.add_slider(slider_layout, 'L_low', 0, 255, 0, 3, 0)
        self.add_slider(slider_layout, 'L_high', 0, 255, 255, 3, 1)
        self.add_slider(slider_layout, 'A_low', 0, 255, 0, 4, 0)
        self.add_slider(slider_layout, 'A_high', 0, 255, 255, 4, 1)
        self.add_slider(slider_layout, 'B_low', 0, 255, 0, 5, 0)
        self.add_slider(slider_layout, 'B_high', 0, 255, 255, 5, 1)
        
        # 隐藏LAB滑动条
        self.set_slider_visible('L_low', False)
        self.set_slider_visible('L_high', False)
        self.set_slider_visible('A_low', False)
        self.set_slider_visible('A_high', False)
        self.set_slider_visible('B_low', False)
        self.set_slider_visible('B_high', False)
        
        main_layout.addLayout(slider_layout)
        
    def add_slider(self, layout, name, min_val, max_val, initial_val, row, col):
        """添加滑动条到布局"""
        widget = QWidget()
        slider_layout = QVBoxLayout(widget)
        
        label = QLabel(f'{name}: {initial_val}')
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(initial_val)
        
        # 连接滑动条事件
        slider.valueChanged.connect(lambda val, n=name, l=label: self.slider_changed(n, val, l))
        
        slider_layout.addWidget(label)
        slider_layout.addWidget(slider)
        
        layout.addWidget(widget, row, col)
        
        # 存储滑动条引用
        self.sliders[name] = {
            'slider': slider,
            'label': label,
            'widget': widget
        }
    
    def set_slider_visible(self, name, visible):
        """设置滑动条可见性"""
        if name in self.sliders:
            self.sliders[name]['widget'].setVisible(visible)
    
    def slider_changed(self, name, value, label):
        """处理滑动条值变化"""
        label.setText(f'{name}: {value}')
        
        # 更新对应的阈值
        if self.processor.color_space == 'HSV':
            if name == 'H_low':
                self.processor.h_low = value
            elif name == 'H_high':
                self.processor.h_high = value
            elif name == 'S_low':
                self.processor.s_low = value
            elif name == 'S_high':
                self.processor.s_high = value
            elif name == 'V_low':
                self.processor.v_low = value
            elif name == 'V_high':
                self.processor.v_high = value
        elif self.processor.color_space == 'LAB':
            if name == 'L_low':
                self.processor.l_low = value
            elif name == 'L_high':
                self.processor.l_high = value
            elif name == 'A_low':
                self.processor.a_low = value
            elif name == 'A_high':
                self.processor.a_high = value
            elif name == 'B_low':
                self.processor.b_low = value
            elif name == 'B_high':
                self.processor.b_high = value
    
    def set_color_space(self, space):
        """切换颜色空间"""
        self.processor.set_color_space(space)
        
        # 更新按钮状态
        self.hsv_btn.setChecked(space == 'HSV')
        self.lab_btn.setChecked(space == 'LAB')
        
        # 显示/隐藏相应的滑动条
        if space == 'HSV':
            # 显示HSV滑动条，隐藏LAB滑动条
            self.set_slider_visible('H_low', True)
            self.set_slider_visible('H_high', True)
            self.set_slider_visible('S_low', True)
            self.set_slider_visible('S_high', True)
            self.set_slider_visible('V_low', True)
            self.set_slider_visible('V_high', True)
            
            self.set_slider_visible('L_low', False)
            self.set_slider_visible('L_high', False)
            self.set_slider_visible('A_low', False)
            self.set_slider_visible('A_high', False)
            self.set_slider_visible('B_low', False)
            self.set_slider_visible('B_high', False)
        else:  # LAB
            # 显示LAB滑动条，隐藏HSV滑动条
            self.set_slider_visible('H_low', False)
            self.set_slider_visible('H_high', False)
            self.set_slider_visible('S_low', False)
            self.set_slider_visible('S_high', False)
            self.set_slider_visible('V_low', False)
            self.set_slider_visible('V_high', False)
            
            self.set_slider_visible('L_low', True)
            self.set_slider_visible('L_high', True)
            self.set_slider_visible('A_low', True)
            self.set_slider_visible('A_high', True)
            self.set_slider_visible('B_low', True)
            self.set_slider_visible('B_high', True)
    
    def update_images(self, original, processed):
        """更新显示的图像"""
        # 显示原始图像
        self.display_image(original, self.original_label)
        # 显示处理后的图像
        self.display_image(processed, self.processed_label)
    
    def display_image(self, cv_img, label):
        """在QT标签中显示OpenCV图像"""
        # 转换颜色空间 (OpenCV使用BGR，QT使用RGB)
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        
        # 获取图像尺寸
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        
        # 转换为QImage
        q_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        # 缩放图像以适应标签
        scaled_img = q_img.scaled(label.width(), label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # 显示图像
        label.setPixmap(QPixmap.fromImage(scaled_img))

def main(args=None):
    """主函数"""
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建图像处理器节点
    processor = ImageProcessor()
    
    # 创建QT应用
    app = QApplication(sys.argv)
    
    # 创建界面
    window = ThresholdAdjuster(processor)
    
    # 创建ROS2线程
    ros_thread = ROS2Thread(processor)
    ros_thread.image_updated.connect(window.update_images)
    ros_thread.start()
    
    # 显示窗口
    window.show()
    
    # 运行QT应用
    ret = app.exec_()
    
    # 停止ROS2线程
    ros_thread.stop()
    
    # 销毁ROS2节点
    processor.destroy_node()
    rclpy.shutdown()
    
    sys.exit(ret)

if __name__ == '__main__':
    main()
