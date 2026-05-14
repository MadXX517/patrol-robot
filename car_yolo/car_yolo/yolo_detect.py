import rclpy
import car_yolo.fps as fps
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import os
from car_yolo.rknn_yolov5 import RKNNYoloV5, load_class_names
from interfaces.msg import ObjectInfo, ObjectsInfo

from std_srvs.srv import Trigger

# Get the ROS distribution version and set the shared directory for YoloV5 configuration files.
ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('car_yolo')

# Create a ROS 2 Node class YoloV5Ros2.
class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('car_yolo')
        self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")
        self.fps = fps.FPS()
        self.start = True

        self.declare_parameter("backend", "rknn", ParameterDescriptor(
            name="backend", description="Inference backend: rknn or torch"))

        self.declare_parameter("device", "cpu", ParameterDescriptor(
            name="device", description="Torch compute device when backend=torch"))

        self.declare_parameter("model", "yolov5s", ParameterDescriptor(
            name="model", description="Model base name in car_yolo/config"))

        self.declare_parameter("rknn_model", "", ParameterDescriptor(
            name="rknn_model", description="RKNN model file name or absolute path; empty uses <model>.rknn"))

        self.declare_parameter("class_names", "", ParameterDescriptor(
            name="class_names", description="Comma-separated class names for RKNN output"))

        self.declare_parameter("img_size", 640, ParameterDescriptor(
            name="img_size", description="YOLO RKNN input size"))

        self.declare_parameter("conf_thres", 0.25, ParameterDescriptor(
            name="conf_thres", description="YOLO confidence threshold"))

        self.declare_parameter("iou_thres", 0.45, ParameterDescriptor(
            name="iou_thres", description="YOLO NMS IoU threshold"))

        self.declare_parameter("npu_core", "auto", ParameterDescriptor(
            name="npu_core", description="RKNN NPU core mask: auto, 0, 1, 2, 0_1, 0_1_2"))

        self.declare_parameter("image_topic", "/camera/color/image_raw", ParameterDescriptor(
            name="image_topic", description="Image topic, default: /camera/color/image_raw"))

        self.declare_parameter("show_result", False, ParameterDescriptor(
            name="show_result", description="Whether to display detection results, default: False"))

        self.declare_parameter("pub_result_img", False, ParameterDescriptor(
            name="pub_result_img", description="Whether to publish detection result images, default: False"))

        self.declare_parameter("display_width", 1024, ParameterDescriptor(
            name="display_width", description="Width of cv2.imshow window when show_result=True, default: 1024"))

        self.declare_parameter("display_height", 600, ParameterDescriptor(
            name="display_height", description="Height of cv2.imshow window when show_result=True, default: 600"))

        self.create_service(Trigger, '/yolov5/start', self.start_srv_callback)
        self.create_service(Trigger, '/yolov5/stop', self.stop_srv_callback) 
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # Load the model.
        self.yolov5 = self._load_detector()

        # Create publishers.
        self.yolo_result_pub = self.create_publisher(Detection2DArray, "yolo_result", 10)
        self.result_msg = Detection2DArray()
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_img_pub = self.create_publisher(Image, "result_img", 10)

        # Create an image subscriber with the updated topic.
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # Image format conversion (using cv_bridge).
        self.bridge = CvBridge()

        self.show_result = self.get_parameter('show_result').value
        self.pub_result_img = self.get_parameter('pub_result_img').value
        self.display_width = int(self.get_parameter('display_width').value)
        self.display_height = int(self.get_parameter('display_height').value)

    def _load_detector(self):
        backend = str(self.get_parameter('backend').value).lower()
        model_name = str(self.get_parameter('model').value)

        if backend == 'rknn':
            rknn_model = str(self.get_parameter('rknn_model').value or '')
            model_path = self._resolve_model_path(rknn_model or (model_name + '.rknn'))
            names = load_class_names(
                package_share_directory,
                model_name,
                str(self.get_parameter('class_names').value or ''),
            )
            detector = RKNNYoloV5(
                model_path=model_path,
                class_names=names,
                img_size=int(self.get_parameter('img_size').value),
                conf_thres=float(self.get_parameter('conf_thres').value),
                iou_thres=float(self.get_parameter('iou_thres').value),
                npu_core=str(self.get_parameter('npu_core').value),
            )
            self.get_logger().info('YOLO backend=rknn model=%s' % model_path)
            return detector

        if backend == 'torch':
            from yolov5 import YOLOv5

            model_path = self._resolve_model_path(model_name + '.pt')
            device = self.get_parameter('device').value
            self.get_logger().info('YOLO backend=torch model=%s device=%s' % (model_path, device))
            return YOLOv5(model_path=model_path, device=device)

        raise ValueError('Unsupported YOLO backend: %s' % backend)

    @staticmethod
    def _resolve_model_path(model_file):
        if os.path.isabs(model_file):
            return model_file
        return os.path.join(package_share_directory, 'config', model_file)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolov5 detect")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolov5 detect")
        self.start = False
        response.success = True
        response.message = "stop"
        return response

    def image_callback(self, msg: Image):
        if not self.start:
            return

        # 5. Detect and publish results.
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        detect_result = self.yolov5.predict(image)

        self.result_msg.detections.clear()
        self.result_msg.header.frame_id = "camera"
        self.result_msg.header.stamp = self.get_clock().now().to_msg()

        # Parse the results.
        predictions = detect_result.pred[0]
        boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        objects_info = []
        h, w = image.shape[:2]

        for index in range(len(categories)):
            category = int(categories[index])
            if category < len(detect_result.names):
                name = detect_result.names[category]
            else:
                name = 'class_%d' % category
            detection2d = Detection2D()
            detection2d.id = name
            x1, y1, x2, y2 = boxes[index]
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0

            if ros_distribution == 'galactic':
                detection2d.bbox.center.x = center_x
                detection2d.bbox.center.y = center_y
            else:
                detection2d.bbox.center.position.x = center_x
                detection2d.bbox.center.position.y = center_y

            detection2d.bbox.size_x = float(x2 - x1)
            detection2d.bbox.size_y = float(y2 - y1)

            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = float(scores[index])

            detection2d.results.append(obj_pose)
            self.result_msg.detections.append(detection2d)

            # Draw results.
            if self.show_result or self.pub_result_img:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, f"{name}:{obj_pose.hypothesis.score:.2f}", (x1, y1),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.waitKey(1)

            object_info = ObjectInfo()
            object_info.class_name = name
            object_info.box = [int(coord) for coord in [x1, y1, x2, y2]]
            object_info.score = round(float(scores[index]), 2)
            object_info.width = w
            object_info.height = h
            objects_info.append(object_info)

        if objects_info:
            object_msg = ObjectsInfo()
            object_msg.objects = objects_info
            self.object_pub.publish(object_msg)

        # Display results if needed.
        if self.show_result:
            self.fps.update()
            image = self.fps.show_fps(image)
            display_img = cv2.resize(image, (self.display_width, self.display_height),
                                     interpolation=cv2.INTER_AREA)
            cv2.imshow('result', cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

        if self.pub_result_img:
            result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
            result_img_msg.header = msg.header
            self.result_img_pub.publish(result_img_msg)
        if len(categories) > 0:
            self.yolo_result_pub.publish(self.result_msg)
  

def main():
    rclpy.init()
    node = YoloV5Ros2()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node.yolov5, 'release'):
            node.yolov5.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


