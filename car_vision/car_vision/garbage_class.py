#!/usr/bin/env python3
# coding=utf-8

import cv2
import rclpy
from rclpy.node import Node
import sys
import numpy as np
import threading
import math
import time
import queue
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, box_center, distance, pixels_to_world,extristric_plane_shift
from pathlib import Path

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)




TRT_NUM_CLASSES = 16
TRT_CLASS_NAMES = ['Hazardous_Waste', 'Recyclables', 'Other_Waste','Food_Waste','Old_Toy','Shrimp_Shell','Old_Bathtub','Paint_Bucket','ECmedicine','Fishbone','Old_Schoolbag','Lighter','Cup','Watermelon_rind','Basketball','Waste_Battery']
TRT_CLASS_garbage_class={
    'Hazardous_Waste':('Hazardous_Waste','Paint_Bucket','ECmedicine','Waste_Battery'),
    'Recyclables':('Recyclables','Old_Toy','Old_Schoolbag','Basketball'),
    'Other_Waste':('Other_Waste','Old_Bathtub','Lighter','Cup'),
    'Food_Waste':('Food_Waste','Shrimp_Shell','Fishbone','Watermelon_rind')
}

class GarbageClass(Node):
    def __init__(self):
        super().__init__('garbage_class')
        self.K = None
        self.D = None
        self.moving=False

        self.time=time.time()
        self.count = 0
        self.endpoint = None
        self.garbage_stack=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.garbage_pick_order=[]
        self.garbage_pick_start=False
        self.box_pos_last=[-99,-99,-99,-99]
        
        self.imgpts = None
        self.center_imgpts = None
        self.roi = None

        self.min_area = 500
        self.max_area = 7000

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2)
        self.Arm_controller.move_arm([0.15,0.0,0.13], 90.0)
        time.sleep(1)
        self.rgb_sub = self.create_subscription(
            Image,
            '/usb_cam/image_raw',
            self.camera_callback,
            10)
        self.rgb_sub

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/usb_cam/camera_info',
            self.camera_info_callback,
            10)
        self.info_sub  

        weights = str(Path(__file__).parent / 'weights' / 'garbage.pt')
        self.yolo = YOLO(weights)

        self.queue = queue.Queue(maxsize=1)
        # 初始化相机外参和识别区域
        self.white_area_center = np.array([[1, 0, 0, 0.199],
                                          [0, 1, 0,   0],
                                          [0, 0, 1,   0],
                                          [0, 0, 0,   1]], dtype=np.float64)
        
        self.white_area_cam = np.array([[1, 0, 0,     0],
                                       [0, 1, 0,     0],
                                       [0, 0, 1, 0.2140],
                                       [0, 0, 0,     1]], dtype=np.float64)

        # 添加定时器（例如每0.03秒调用一次 image_proc）
        self.create_timer(0.03, self.image_proc)

    def camera_callback(self, msg):
        if self.queue.empty():
            self.queue.put_nowait(msg)

    def camera_info_callback(self, msg):
        if self.K is None:
            try:
                K = np.array(msg.k).reshape(3, 3)
                D = np.array(msg.d)
                new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
                self.K, self.D = np.matrix(new_K), np.zeros((5, 1))
                self.camera_info_received = True
                self.initialize_after_camera_info()
                self.get_logger().info("Received K & D successfully")
            except Exception as e:
                self.get_logger().error(f"Error processing camera info: {e}")

    def initialize_after_camera_info(self):
        white_area_height = 0.118
        white_area_width = 0.210

        # 计算识别区域角点
        white_area_lt = np.matmul(self.white_area_center, xyz_euler_to_mat((white_area_height/2, white_area_width/2, 0.0), (0, 0, 0)))
        white_area_lb = np.matmul(self.white_area_center, xyz_euler_to_mat((-white_area_height/2, white_area_width/2, 0.0), (0, 0, 0)))
        white_area_rb = np.matmul(self.white_area_center, xyz_euler_to_mat((-white_area_height/2, -white_area_width/2, 0.0), (0, 0, 0)))
        white_area_rt = np.matmul(self.white_area_center, xyz_euler_to_mat((white_area_height/2, -white_area_width/2, 0.0), (0, 0, 0)))

        self.endpoint = self.get_endpoint()

        corners_cam = np.matmul(np.linalg.inv(np.matmul(self.endpoint, matrix_hand_to_cam)), 
                               [white_area_lt, white_area_lb, white_area_rb, white_area_rt, self.white_area_center])
        corners_cam = np.matmul(np.linalg.inv(self.white_area_cam), corners_cam)
        corners_cam = corners_cam[:, :3, 3:].reshape((-1, 3))

        tvec = np.array([-0.025, 0.003, 0.2140]).reshape((3, 1))
        rmat = np.array([[0, -1.0, 0],
                        [1.0, 0, 0],
                        [0, 0, 1.0]], dtype=np.float64)
        # 计算图像坐标
        center_imgpts, jac = cv2.projectPoints(corners_cam[-1:], rmat, tvec, self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)

        tvec, rmat = extristric_plane_shift(tvec, rmat, 0.015)
        self.extristric = (tvec, rmat)
        imgpts, jac = cv2.projectPoints(corners_cam[:-1], rmat, tvec, self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

        # 计算ROI区域
        x_min = max(0, min(p[0] for p in self.imgpts))
        x_max = min(640, max(p[0] for p in self.imgpts))
        y_min = max(0, min(p[1] for p in self.imgpts))
        y_max = min(480, max(p[1] for p in self.imgpts))
        self.roi = np.array([y_min, y_max, x_min, x_max])
        self.get_logger().info(f"ROI: {self.roi}")

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

    def point_remapped(self, point, now, new, data_type=float):
        x, y = point
        now_w, now_h = now
        new_w, new_h = new
        new_x = x * new_w / now_w
        new_y = y * new_h / now_h
        return data_type(new_x), data_type(new_y)

    def adaptive_threshold(self, gray_image):
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)
        _imshow_fit("BIN", binary)
        return binary

    def canny_proc(self, bgr_image):
        mask = cv2.Canny(bgr_image, 23, 51, 23, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))
        return mask

    def get_top_surface(self, rgb_image):
        image_scale = cv2.convertScaleAbs(rgb_image, alpha=2.5, beta=0)
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)
        image_gs = cv2.GaussianBlur(image_mb, (5, 5), 5)
        binary = self.adaptive_threshold(image_gs)
        mask = self.canny_proc(image_gs)
        mask1 = cv2.bitwise_and(binary, mask)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)
        return roi_image_mask

    def pick(self, position, rolll, which_class):
        self.moving = True
        x,y,z=position

        yaw = math.degrees(math.atan2(y, x))
        rolll -= yaw
        
        self.Arm_controller.move_arm([x,y,z+0.03],pitch=80,roll=rolll,duration=1500)
        time.sleep(0.7)
        self.Arm_controller.move_arm([x,y,z-0.003],pitch=80,roll=rolll,duration=400)
        time.sleep(0.2)
        self.Arm_controller.move_arm([x,y,z-0.003],pitch=80,roll=rolll,hand=-0.35,duration=500)
        time.sleep(0.2)
        self.Arm_controller.move_arm([x,y,0.13],pitch=70,hand=-0.35,duration=1300)
        time.sleep(0.6)
        set_x=-0.035
        set_y=0.150
        set_z=0.040
        set_roll=-13
        if which_class == 'Other_Waste':
            set_x=-0.139
            set_y=0.160
            set_roll=-9
        elif which_class == 'Recyclables':
            set_x=-0.067
            set_y=0.160
            set_roll=5.5
        elif which_class == 'Food_Waste':
            set_x=0.017
            set_y=0.158
            set_roll=16
        elif which_class == 'Hazardous_Waste':
            set_x=0.10
            set_y=0.155
            set_z=0.042
            set_roll=28

        self.Arm_controller.move_arm([set_x,set_y,0.13],pitch=70,roll=set_roll,hand=-0.35,duration=1300)
        time.sleep(0.6)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.35,duration=1000)
        time.sleep(0.5)
        self.Arm_controller.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=0.150,duration=600)
        time.sleep(0.3)
        self.Arm_controller.move_arm([set_x,set_y,0.13],pitch=70,roll=set_roll,hand=0.150,duration=400)
        time.sleep(0.2)
        self.Arm_controller.move_arm([0.15,0.0,0.13],pitch=90.0,duration=1300)
        time.sleep(2.0)

        self.garbage_pick_start = False
        self.moving = False


    def image_proc(self):
        if self.queue.empty():
            return

        ros_image = self.queue.get(block=True)
        try:
            if ros_image.encoding == 'rgb8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 3).astype(np.uint8)
            elif ros_image.encoding == 'mono8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width).astype(np.uint8)
                # 如果需要RGB图像，可以将单通道图像复制到三个通道
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB)
            elif ros_image.encoding == 'yuv422_yuy2':
                yuv_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 2).astype(np.uint8)
                rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_YUY2)
            else:
                self.get_logger().error(f"Unsupported image encoding: {ros_image.encoding}")
                return

            result_image = np.copy(rgb_image)

            if self.endpoint is None:
                self.endpoint=self.get_endpoint()

            # 绘制识别区域
            if self.imgpts is not None:
                cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA)
                for p in self.imgpts:
                    cv2.circle(result_image, tuple(p), 8, (255, 0, 0), -1)
            
            if self.center_imgpts is not None:
                cv2.line(result_image, (self.center_imgpts[0]-10, self.center_imgpts[1]), 
                        (self.center_imgpts[0]+10, self.center_imgpts[1]), (255, 255, 0), 2)
                cv2.line(result_image, (self.center_imgpts[0], self.center_imgpts[1]-10), 
                        (self.center_imgpts[0], self.center_imgpts[1]+10), (255, 255, 0), 2)

            box_pos=None
            garbage_name=None
            if self.roi is not None and self.K is not None and self.D is not None and self.moving == False:
                roi_area_mask = np.zeros(shape=(ros_image.height, ros_image.width, 1), dtype=np.uint8)
                roi_area_mask = cv2.drawContours(roi_area_mask, [self.imgpts], -1, 255, cv2.FILLED)
                rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=roi_area_mask)  # 和原图做遮罩，保留需要识别的区域

                # YOLO预测（调整参数适配）
                detect_result = self.yolo.predict(
                    cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR),
                    conf=0.75,  # 置信度阈值（与原代码cls_conf≥0.75一致）
                    iou=0.45,   # NMS的IOU阈值
                    verbose=False  # 关闭详细输出
                )
                
                # 处理YOLO预测结果
                boxes = []
                confs = []
                classes = []
                if len(detect_result) > 0 and hasattr(detect_result[0], 'boxes'):
                    det_boxes = detect_result[0].boxes
                    if det_boxes is not None and len(det_boxes) > 0:
                        boxes = det_boxes.xyxy.numpy()  # x1y1x2y2格式
                        confs = det_boxes.conf.numpy()  # 置信度
                        classes = det_boxes.cls.numpy().astype(int)  # 类别ID

                for box, cls_conf, cls_id in zip(boxes, confs, classes):
                    x1, y1, x2, y2 = map(int, box)
                    object_name = TRT_CLASS_NAMES[cls_id]

                    if self.garbage_pick_start == True and self.garbage_pick_order[0] == object_name:
                        box_pos=[int(x1),int(y1),int(x2),int(y2)]
                        garbage_name = object_name

                    result_image = cv2.putText(result_image, object_name + " " + str(float(cls_conf))[:4], (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, [255,0,0], 2)
                    result_image = cv2.rectangle(result_image, (int(x1), int(y1)), (int(x2), int(y2)), [255,0,0], 3)

                    if self.garbage_pick_start == False: self.garbage_stack[cls_id]+=1

                if self.count == 10 and self.garbage_pick_start == False:
                    self.garbage_pick_order.clear()
                    for i in range(0,16):
                        if self.garbage_stack[i] > 8:
                            self.garbage_pick_order.append(TRT_CLASS_NAMES[i])
                    print("10帧中检测到垃圾累计堆:",self.garbage_stack)
                    print("抓取顺序：",self.garbage_pick_order)
                    self.count=0
                    self.garbage_pick_start=True
                    self.time=time.time()
                elif self.garbage_pick_start == False and len(boxes) > 0:
                    self.count += 1
                else: self.count=0
                    
                if self.garbage_pick_start == True:
                    if time.time() - self.time < 4.0:
                        if box_pos is not None:
                            a=box_center(box_pos)
                            b=box_center(self.box_pos_last)
                            if distance(a,b) < 50:
                                projection_matrix = np.row_stack((np.column_stack((self.extristric[1], self.extristric[0])), np.array([[0, 0, 0, 1]])))
                                position = pixels_to_world([b, ], self.K, projection_matrix)[0]  # 像素坐标相对于识别区域中心的相对坐标
                                position[0] = -position[0]
                                position[2] = -0.063
                                world_pose = np.matmul(self.white_area_center, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换到相机相对坐标
                                world_pose[2] = -0.063
                                pose_t, pose_R = mat_to_xyz_euler(world_pose)
                                print(pose_t)

                                pose_t[0]+=0.018
                                pose_t[1]=pose_t[1]+0.01 # 抓手偏移修正
                                pose_t[1]=pose_t[1]*0.93

                                which_class=None
                                for category, items in TRT_CLASS_garbage_class.items():
                                    if garbage_name in items:
                                        which_class=category
                                        break
                                print("garbage_name:",garbage_name,"class:",which_class)

                                threading.Thread(target=self.pick, args=(pose_t, 0, which_class)).start()
                                self.garbage_stack=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                            self.box_pos_last=box_pos
                    else:
                        print("finder out of time")
                        self.garbage_pick_start=False
                        self.garbage_stack=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    

            _imshow_fit("color", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    color_rect_pick = GarbageClass()
    rclpy.spin(color_rect_pick)
    color_rect_pick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
