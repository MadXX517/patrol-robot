#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO  # Ultralytics库支持ONNX
from std_msgs.msg import String
import car_vision.fps as fps
from pathlib import Path

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



class YoloV8ONNXDetector(Node):
    def __init__(self):
        super().__init__('yolov8_onnx_detector')
        
        # 关键修改：加载ONNX模型（替换为你的.onnx模型路径）
        self.model = YOLO(str(Path(__file__).parent / "weights" / "traffic_signs.onnx"))  # 绝对路径或相对路径
        # 验证模型类型（可选）
        self.get_logger().info(f"加载的模型类型: {self.model.model.__class__.__name__}")
        
        self.bridge = CvBridge()
        self.fps = fps.FPS()  # fps计算器(FPS calculator)

        # 订阅USB摄像头图像
        self.subscription = self.create_subscription(
            Image,
            '/usb_cam/image_raw',
            self.image_callback,
            10)
        
        # 发布检测结果图像和信息
        self.detected_img_pub = self.create_publisher(Image, 'yolov8/detected_image', 10)
        self.detections_pub = self.create_publisher(String, 'yolov8/detections', 10)
        
        self.get_logger().info('YOLOv8 (ONNX)检测器已启动，等待图像...')

    def image_callback(self, msg):
        try:
            # ROS图像转OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {str(e)}')
            return
        
        # ONNX模型推理（接口与.pt模型完全一致）
        results = self.model(cv_image)
        
        # 处理检测结果
        detected_objects = []
        for result in results:
            annotated_image = result.plot()  # 绘制检测框
            
            # 提取目标信息
            for box in result.boxes:
                class_name = result.names[int(box.cls)]
                confidence = float(box.conf)
                detected_objects.append(f"{class_name} ({confidence:.2f})")
        
        self.fps.update()
        annotated_image = self.fps.show_fps(annotated_image)
        # 显示检测结果
        _imshow_fit("YOLOv8 (ONNX) Detection", annotated_image)
        cv2.waitKey(1)
        
        # 发布带检测框的图像
        try:
            detected_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            detected_msg.header = msg.header
            self.detected_img_pub.publish(detected_msg)
        except Exception as e:
            self.get_logger().error(f'图像发布失败: {str(e)}')
        
        # 发布检测到的目标信息
        if detected_objects:
            detection_str = ", ".join(detected_objects)
            self.detections_pub.publish(String(data=detection_str))
            self.get_logger().info(f'检测到: {detection_str}')

def main(args=None):
    rclpy.init(args=args)
    detector = YoloV8ONNXDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('用户中断，退出程序')
    finally:
        cv2.destroyAllWindows()
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()