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
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
import message_filters
from message_filters import ApproximateTimeSynchronizer
from rclpy.callback_groups import ReentrantCallbackGroup
from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, box_center, distance, pixels_to_world
from pathlib import Path


TRT_NUM_CLASSES = 16
TRT_CLASS_NAMES = ['Hazardous_Waste', 'Recyclables', 'Other_Waste','Food_Waste','Old_Toy','Shrimp_Shell','Old_Bathtub','Paint_Bucket','ECmedicine','Fishbone','Old_Schoolbag','Lighter','Cup','Watermelon_rind','Basketball','Waste_Battery']
TRT_CLASS_garbage_class={
    'Hazardous_Waste':('Hazardous_Waste','Paint_Bucket','ECmedicine','Waste_Battery'),
    'Recyclables':('Recyclables','Old_Toy','Old_Schoolbag','Basketball'),
    'Other_Waste':('Other_Waste','Old_Bathtub','Lighter','Cup'),
    'Food_Waste':('Food_Waste','Shrimp_Shell','Fishbone','Watermelon_rind')
}
color_ranges = {
    "RED": [[9, 142, 100], [255, 183, 189]],
    "GREEN": [[20, 60, 100], [255, 120, 172]],
    "BLUE": [[10,100,40], [180,180,120]],
    "gray": [[0, 0, 0], [120, 130, 140]]
}
color_in_order=[color_ranges['RED'],color_ranges['GREEN'],color_ranges['BLUE'],color_ranges['gray']]

class ColorFinder:
    def proc (self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        
        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color_ranges[0]), tuple(color_ranges[1])) # 二值化

        # 滤波，去掉噪点，平滑边缘
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 找出最大轮廓
        contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # [-2]
        #max_contour_area = get_area_max_contour(contours, 10)
        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c)) < 500:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c)
            if min_c is None:
                min_c = (c, center_x)
            elif center_x < min_c[1]:
                if center_x < min_c[1]:
                    min_c = (c, center_x)

        # 如果有符合要求的轮廓
        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0]) # 最小外接圆
            rect=cv2.minAreaRect(min_c[0])

            circle_color = (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)

            center_x = center_x * 2
            center_x_1 = center_x / w
            center_y = center_y * 2
            center_y_1 = center_y / h
            return (result_image, (0, 0), (center_x, center_y), radius * 2,rect[2])
        else:
            return (result_image, None, None, 0,0)

class GarbageClass(Node):
    def __init__(self):
        super().__init__('garbage_class_d')
        self.K = None
        self.D = None
        self.moving=False
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.color_order = 0
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
        self.bridge = CvBridge()
        self.tracker = ColorFinder()
        self.catch_stage=0
        self.min_area = 500
        self.max_area = 7000

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2)
        self.Arm_controller.move_arm([0.14,0.0,0.08],70.0)
        time.sleep(1)
        # self.client = self.create_client(SetBool, '/camera/set_ldp')
        # self.timer = self.create_timer(1.0, self.check_service)

        cb_group = ReentrantCallbackGroup()
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth/camera_info', callback_group=cb_group)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], 
            queue_size=3, 
            slop=0.03
        )
        self.ts.registerCallback(self.multi_callback)

        weights = str(Path(__file__).parent / 'weights' / 'best.pt')
        self.yolo = YOLO(weights)

        self.queue = queue.Queue(maxsize=1)

        self.timer = self.create_timer(0.1, self.image_proc)
    
    def check_service(self):
        if self.client.service_is_ready():
            self.client.call_async(SetBool.Request(data=True))
            self.timer.cancel()  # 服务可用后取消定时器
        else:
            self.get_logger().info('service not available, waiting again...')

    def multi_callback(self, rgb_msg, depth_msg, info_msg):
        if self.queue.empty():
            self.queue.put_nowait((rgb_msg, depth_msg, info_msg))

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
        
    def pick(self, A, which_class):
        self.moving = True
        x=A.position.x
        y=A.position.y
        z=A.position.z
        rolll=A.roll

        if self.catch_stage == 0:
            self.Arm_controller.move_arm([x,y,z+0.03],pitch=70,roll=rolll,duration=1500)
            time.sleep(0.6)
            self.Arm_controller.move_arm([x,y,z-0.003],pitch=70,roll=rolll,duration=400)
            time.sleep(0.2)
            self.Arm_controller.move_arm([x,y,z-0.003],pitch=70,roll=rolll,hand=-0.35,duration=500)
            time.sleep(0.2)
            self.Arm_controller.move_arm([0.14,0.0,0.08],pitch=70,hand=-0.35,duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([0.0,0.14,0.08],pitch=70,hand=-0.35,duration=1000)
            time.sleep(0.5)
            if which_class == 'Other_Waste':
                self.color_order = 3
            elif which_class == 'Recyclables':
                self.color_order = 2
            elif which_class == 'Food_Waste':
                self.color_order = 1
            elif which_class == 'Hazardous_Waste':
                self.color_order = 0

            self.moving = False
            self.catch_stage=1
        elif self.catch_stage == 1:
            z=-0.01
            self.Arm_controller.move_arm([x,y,z+0.06],pitch=70,roll=rolll,hand=-0.35,duration=1000)
            time.sleep(0.5)
            self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,hand=-0.35,duration=400)
            time.sleep(0.2)
            self.Arm_controller.move_arm([x,y,z-0.005],pitch=70,roll=rolll,hand=0.3,duration=400)
            time.sleep(0.2)
            self.Arm_controller.move_arm([x,y,z+0.06],pitch=70,roll=rolll,hand=0.3,duration=400)
            time.sleep(0.2)
            self.Arm_controller.move_arm([0.0,0.14,0.08],pitch=70,hand=0.3,duration=600)
            time.sleep(0.3)
            self.Arm_controller.move_arm([0.14,0.0,0.08],pitch=70,duration=1000)
            time.sleep(0.5)

            self.garbage_pick_start = False
            self.moving = False
            self.catch_stage=0

            


    def image_proc(self):
        try:
            ros_rgb, ros_depth, info_msg = self.queue.get_nowait()
        except queue.Empty:
            return
        if self.K is None:
            self.K = info_msg.k
            print(self.K)
        if self.endpoint is None:
            self.endpoint = self.get_endpoint()
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb, 'rgb8')
            depth_image = self.bridge.imgmsg_to_cv2(ros_depth, '16UC1')

            rh, rb = rgb_image.shape[:2]
            ih, iw = depth_image.shape[:2]
            rgb_image=rgb_image[(rh-ih)//2:rh-(rh-ih)//2,]

            result_image = np.copy(rgb_image)

            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET) # 转换成伪彩色图

            box_pos=None
            garbage_name=None
            if self.catch_stage == 0:
                if self.moving == False:
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

                        if self.garbage_pick_start == False:self.garbage_stack[cls_id]+=1

                    if self.count == 5 and self.garbage_pick_start == False:
                        self.garbage_pick_order.clear()
                        for i in range(0,16):
                            if self.garbage_stack[i] > 3:
                                self.garbage_pick_order.append(TRT_CLASS_NAMES[i])
                        
                        print("10帧中检测到数字累计堆:",self.garbage_stack)
                        print("抓取顺序：",self.garbage_pick_order)
                        self.garbage_pick_start=True
                        self.time=time.time()
                    elif self.garbage_pick_start == False and len(boxes) > 0:
                        self.count += 1       
                    else: self.count=0   

                    if self.garbage_pick_start == True:
                        if time.time() - self.time < 6.0:
                            if box_pos is not None:
                                a=box_center(box_pos)
                                b=box_center(self.box_pos_last)

                                if distance(a,b) < 50:
                                    center_x, center_y = a
                                    center_x = int(((center_x + 10)-(iw//2)) / 1.26 + (iw//2))
                                    center_y = int((center_y-(ih//2)) / 1.26+(ih//2))
                                    depth = np.copy(depth_image).reshape((-1, ))
                                    depth[depth<=130] = 55555

                                    # mean=round(calculate_filtered_mean(depth_image,(box_pos[0],box_pos[1],box_pos[2],box_pos[3]),lower_threshold=160,upper_threshold=250))
                                    # print("mean:",mean)
                                    dist = depth_image[int(center_y),int(center_x)]/1000.0
                                    if dist < 100 or dist > 320:
                                        txt = "TOO CLOSE !!!"
                                    else:
                                        txt = "Dist: {}mm".format(dist)
                                    cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                                    cv2.circle(depth_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                                    cv2.putText(depth_image, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                                    cv2.putText(depth_image, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                                    sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                                    depth_image = np.where(depth_image > dist*1000+20, 0, depth_image) # 找出最近的点，筛选出距离大于此点的像素作为新图
                                    sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255 # 裁掉小于0大于2m的像素
                                    depth_gray = sim_depth_image_sort.astype(np.uint8) # 转换成uint8类型方便二值化
                                    depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0) # 5*5内核的高斯模糊
                                    _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY) # 二值化
                                    depth_bit = cv2.erode(depth_bit, np.ones((5, 5), np.uint8)) # 腐蚀
                                    depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8)) # 膨胀

                                    contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # 找角点 # opencv3.2有三个输出，opencv4只有contours, hierarchy两个输出，要匹配opencv版本

                                    for obj in contours:
                                        area = cv2.contourArea(obj)
                                        if area < 500 or area > 10000 or self.moving is True:
                                            continue
                                        cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 4) # 绘制轮廓线
                                        center, radius = cv2.minEnclosingCircle(obj) # 计算包裹轮廓最大圆

                                        if distance(center,[center_x,center_y]) > 100:
                                            continue

                                        rect = cv2.minAreaRect(obj)
                                        self.endpoint = self.get_endpoint()
                                        PoseRC = self.count_position("Rect", center, depth_image, self.K, rect[2])
                                        which_class=None
                                        for category, items in TRT_CLASS_garbage_class.items():
                                            if garbage_name in items:
                                                which_class=category
                                                break

                                        print("garbage_name:",garbage_name,"class:", which_class)

                                        threading.Thread(target=self.pick, args=(PoseRC,which_class)).start()
                                        self.garbage_stack=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                        break
                                self.box_pos_last=box_pos
                        else:      
                            print("finder out of time")
                            self.garbage_pick_start=False
                            self.garbage_stack=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            else:
                if self.tracker is not None and self.moving == False:                
                    result_image, p_y, center, r ,roll = self.tracker.proc(rgb_image, result_image, color_in_order[self.color_order])
                    if p_y is not None:
                        self.detect_times=0
                        center_x, center_y = center
                        center_x = int(((center_x + 10)-(iw//2)) / 1.26 + (iw//2))
                        center_y = int((center_y-(ih//2)) / 1.26+(ih//2))
                        if center_x > 639:
                            center_x = 639
                        if center_y > 479:
                            center_y = 479
                        if abs(self.last_pitch_yaw[0] - p_y[0]) < 0.08 and abs(self.last_pitch_yaw[1] - p_y[1]) < 0.08:
                            if time.time() - self.stamp > 1.5:
                                which_class=None
                                self.stamp = time.time()
                                self.endpoint = self.get_endpoint()
                                PoseRC = self.count_position("Rect", center, depth_image, self.K, roll)
                                self.stamp = time.time()
                                threading.Thread(target=self.pick, args=(PoseRC,which_class)).start()
                        else:
                            print("center point dist wrong")
                            self.stamp = time.time()
                        dist = depth_image[int(center_y),int(center_x)]
                        print("dist:",dist)
                        if dist < 100:
                            txt = "TOO CLOSE !!!"
                        else:
                            txt = "Dist: {}mm".format(dist)
                        cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                        self.last_pitch_yaw = p_y
                    else:
                        self.detect_times+=1
                        if self.detect_times > 30:
                            if self.color_order < len(color_in_order)-1:
                                self.color_order+=1
                            else:
                                self.get_logger().info("找不到任何颜色物块，重新开始检索" + str(color_in_order[self.color_order]))
                                self.color_order=0
                            self.get_logger().info("color range:" + str(color_in_order[self.color_order]))
                            self.detect_times=0
        
                        self.stamp = time.time()
                        # print("not find")

            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            depth_color_map = cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_JET) # 转换成伪彩色图
            result_image = np.concatenate([result_image, depth_color_map, ], axis=1)
            cv2.imshow("depth", result_image)
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def whichclass_belong(self,name):
        for category, items in TRT_CLASS_garbage_class.items():
            if name in items:
                return category

    def count_position(self,Name,center,depth_image,K,roll):
        PWRAC=PoseWithRollAndColor()

        center_x, center_y = center

        center_x = min(639, int(center_x))
        center_y = min(359, int(center_y))

        if self.catch_stage == 0:
            dist = depth_image[int(center_y),int(center_x)]/1000.0
            dist += 0.015 # 物体半径补偿
        else:
            dist = 270/1000.0 # 固定高度
            dist += 0.015 # 物体半径补偿

        position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))

        position[0] -= 0.007  # rgb相机和深度相机tf有1cm偏移
        temp=position[0]
        position[0]=position[1]
        position[1]=-temp

        pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标 即机械臂抓手到像素点的相对坐标
        world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
        pose_t, pose_R = mat_to_xyz_euler(world_pose)

        if self.catch_stage == 0:
            pose_t[0] = pose_t[0] + 0.002 # - 0.002
            pose_t[1] = pose_t[1] # * 1.1
        else:
            pose_t[0] = pose_t[0]*0.6
            pose_t[1] = pose_t[1]*0.85

        yaw = math.degrees(math.atan2(pose_t[1], pose_t[0]))
        if roll==90.0:
            roll=0
        if pose_t[1] <= 0:
            roll=-roll-yaw
        else:
            roll=roll+yaw
            roll=90-roll

        if roll>45.0:
            roll=roll-90.0
        elif roll<-45.0:
            roll=roll+90

        PWRAC.name = Name
        PWRAC.position.x = pose_t[0]
        PWRAC.position.y = pose_t[1]
        PWRAC.position.z = pose_t[2]
        PWRAC.roll = roll
        return PWRAC  # 返回位置和roll角度


def main(args=None):
    rclpy.init(args=args)
    garbage_class_d = GarbageClass()
    rclpy.spin(garbage_class_d)
    garbage_class_d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
