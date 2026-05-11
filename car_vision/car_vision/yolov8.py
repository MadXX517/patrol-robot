#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2  # 用于imshow显示图像
from ultralytics import YOLO
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



class YoloV8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        
        # 加载YOLOv8模型（可替换为你的模型路径）
        self.model = YOLO(str(Path(__file__).parent / 'weights' / 'garbage.pt'))  # 或指定路径：'/home/user/models/yolov8s.pt'
        
        self.bridge = CvBridge()
        self.fps = fps.FPS()  # fps计算器(FPS calculator)

        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        # 发布检测结果图像
        self.publisher_ = self.create_publisher(Image, 'yolov8/detected_image', 10)
        # 发布检测信息
        self.detections_pub = self.create_publisher(String, 'yolov8/detections', 10)
        

    def image_callback(self, msg):
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {str(e)}')
            return
        
        # 目标检测
        results = self.model(cv_image)
        
        # 处理结果并绘制检测框
        detected_objects = []
        for result in results:
            annotated_image = result.plot()  # 绘制检测框、类别、置信度
            
            # 提取检测信息
            for box in result.boxes:
                class_id = result.names[int(box.cls)]
                confidence = float(box.conf)
                detected_objects.append(f"{class_id} ({confidence:.2f})")
                
        self.fps.update()
        annotated_image = self.fps.show_fps(annotated_image)
        # 关键：用imshow显示检测后的图像
        _imshow_fit("YOLOv8 Detection Result", annotated_image)
        # 必须调用waitKey，否则窗口会卡住（1ms延迟，不影响实时性）
        cv2.waitKey(1)
        
        # 发布带检测结果的图像
        try:
            detected_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            detected_msg.header = msg.header
            self.publisher_.publish(detected_msg)
        except Exception as e:
            self.get_logger().error(f'图像发布错误: {str(e)}')
        
        # 发布检测信息
        if detected_objects:
            detection_str = ", ".join(detected_objects)
            self.detections_pub.publish(String(data=detection_str))
            self.get_logger().info(f'检测到: {detection_str}')

def main(args=None):
    rclpy.init(args=args)
    yolov8_detector = YoloV8Detector()
    try:
        rclpy.spin(yolov8_detector)
    except KeyboardInterrupt:
        yolov8_detector.get_logger().info('用户中断，退出程序')
    finally:
        # 退出时销毁OpenCV窗口，释放资源
        cv2.destroyAllWindows()
        yolov8_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()