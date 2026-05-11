#!/usr/bin/env python3
# coding=utf-8
import sys
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
from rclpy.node import Node
# from app.common import Heart:w
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from car_vision.line_detect import get_saidaobianyuan, get_saidao
import car_vision.pid as pid
import car_vision.arm_ik_sdk as arm_ik_sdk
from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



TRT_NUM_CLASSES = 7
TRT_CLASS_NAMES = ['Keep_Straight_Sign', 'Turn_Right_Sign', 'Parking_Sign', 'Sidewalk_Sign', 'Crossing', 'Green_Light', 'Turn_R']

picture_set_shape = [640, 460]
lower_yellow = np.array([0, 65, 30])
upper_yellow = np.array([179, 255, 255])

lower_gray = np.array([0, 0, 0])
upper_gray = np.array([179, 70, 130])
new_uppger_gray=np.array([179, 90, 100]) 

lower_white = np.array([0, 0, 180])
upper_white = np.array([40, 50, 240])

normal_speed = 0.08
turn_right_z = -0.45

debug_mode = True

class CamControlNode(rclpy.node.Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.pid_z = pid.PID(0.3, 0.0, 0.0)
        self.z_dis = 0.0
        self.pid_z_grey = pid.PID(0.0005, 0.0, 0.0)
        self.z_grey_dis = 0.0
        self.set_speed = 0.0

        self.signs_name = None
        self.cross_time_stamp = 0
        self.stop = True
        self.set_z = -999

        self.start_park = False
        self.park_sign_pre=False

        self.keep_straight_sign_pre = False
        self.count_keep_straight_sign = 0

        self.sidewalk_sign_pre = False
        self.count_sidewalk_sign = 0

        self.turn_right = False
        self.count_right = 0

        self.turn_right_sign = False
        self.turn_right_sign_pre = False
        self.count_turn_right_sign = 0
        self.count_right_miss = 0
        self.start_turn = False
        
        self.traffic_light = False

        self.Turn_R_box = None
        self.Crossing_box = None
        self.rois = ((310, 340, 0, int(picture_set_shape[0] / 2), 0.4), (260, 290, 0, int(picture_set_shape[0] / 2), 0.5), (210, 240, 0, int(picture_set_shape[0] / 2), 0.3))
        self.weight_sum = 1.0
        self.last_error = 0
        self.count_park_sign = 0
        self.passed_turn_right_signs = False
        self.sign_detect = True
        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2)
        self.Arm_controller.set_steer([0.0, 0.0, 1.6, 1.200, 0, 0.00])
        time.sleep(1)
        self.declare_parameter('machine_type', 'Mec', ParameterDescriptor(name='machine_type', description='Mec,Ack'))
        self.machine_type = self.get_parameter('machine_type').value   #麦轮：Mec,阿克曼：Ack
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.image_sub = self.create_subscription(RosImage, '/camera/color/image_raw', self.camera_callback, 1)
        self.object_sub = self.create_subscription(ObjectsInfo, '/car_yolo/object_detect', self.get_object_callback, 1)      
        self.queue = queue.Queue(maxsize=2)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_image = None
    
    def timer_callback(self):
        if self.latest_image is not None:
            self.process_image(self.latest_image)

    def camera_callback(self, ros_rgb_image):
        # 淇濆瓨鏈€鏂扮殑鍥惧儚
        self.latest_image = ros_rgb_image

    def get_banmaxian(self,frame):
        data_gap=3
        row_len=picture_set_shape[0]
        # std_deviation=np.std(row_data)
        good_line=0
        for k in range(int(picture_set_shape[1]*0.7),picture_set_shape[1],10):
            row_index = k
            row_data = frame[row_index, :]
            good_sum=0

            good=0
            bad=0
            for i in range(int(row_len/2),row_len,data_gap):
                if row_data[i] != row_data[i-data_gap]:
                    j=i+data_gap
                    if j>=row_len:break
                    while row_data[j] == row_data[j-data_gap]:
                        j=j+data_gap
                        if j>=row_len:break
                    if j-i>50 or j-i<30:
                        bad+=1
                    else:
                        good+=1
            good_sum+=good

            good=0
            bad=0
            for i in range(int(row_len/2),0,-data_gap):
                if row_data[i] != row_data[i+data_gap]:
                    j=i-data_gap
                    if j<=0:break
                    while row_data[j] == row_data[j+data_gap]:
                        j=j-data_gap
                        if j<=0:break
                    if i-j>55 or i-j<25:
                        bad+=1
                    else:
                        good+=1
            good_sum+=good

            # print("good:",good)
            if good_sum >= 4:
                good_line+=1

        # if debug_mode==True: print("good_line",good_line)

        if good_line >= 4:
            self.cross_time_stamp = time.time()
            self.sign_detect=True

    def pid_control(self, error, aim):
        twist = Twist()
        if self.stop: self.set_speed = 0.0
        if abs(error - aim) > 0.01:
            self.pid_z.SetPoint = aim
            self.pid_z.update(error)
            self.z_dis += self.pid_z.output
            if self.z_dis > 0.8: self.z_dis = 0.8
            elif self.z_dis < -0.8: self.z_dis = -0.8
        else:
            self.pid_z.clear()
            self.z_dis = 0.0
        twist.linear.x = self.set_speed
        if twist.linear.x == 0.0:
            twist.angular.z = 0.0
            self.pid_z.clear()
            self.z_dis = 0.0
        else:
            if self.set_z != -999: twist.angular.z = self.set_z
            else: twist.angular.z = self.z_dis
        self.cmd_vel_pub.publish(twist)
        return twist.angular.z

    def pid_control_grey(self, angle, aim):
        twist = Twist()
        if self.stop: self.set_speed = 0.0
        if abs(angle - aim) > 1:
            self.pid_z_grey.SetPoint = aim
            self.pid_z_grey.update(angle)
            self.z_grey_dis += self.pid_z_grey.output
            if self.z_grey_dis > 0.8: self.z_grey_dis = 0.8
            elif self.z_grey_dis < -0.8: self.z_grey_dis = -0.8
        else:
            self.pid_z_grey.clear()
            self.z_grey_dis = 0.0
        twist.linear.x = self.set_speed
        if twist.linear.x == 0.0:
            twist.angular.z = 0.0
            self.pid_z_grey.clear()
            self.z_grey_dis = 0.0
        else:
            if self.set_z != -999: twist.angular.z = self.set_z
            else: twist.angular.z = -self.z_grey_dis
        self.cmd_vel_pub.publish(twist)
        return twist.angular.z

    def park_action(self):
        twist = Twist()
        if self.machine_type == 'Mec': 
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5 / 0.1)
            twist.linear.x = 0.0
            twist.linear.y = -0.2
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("start_park")
            time.sleep(0.28 / 0.2)
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("end_park")
        else:
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.3/0.1) #0.36
            print("start_park")
            twist.linear.x= 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(1)
            twist.linear.x = -0.08
            twist.angular.z = 0.150
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.65/0.08)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.7)
            self.cmd_vel_pub.publish(Twist())
            print("end_park")
            
    def process_image(self, ros_rgb_image):
        self.signs_name=None
        now_time=time.time()
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)[:picture_set_shape[1],]
        result_image = np.copy(rgb_image)
        
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        kernel = np.ones((3, 3), np.uint8)

        mask_yellow = cv2.erode(mask_yellow, kernel, iterations=2)
        mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=3)

        roi = mask_yellow[0:picture_set_shape[1], 0:int(picture_set_shape[0] / 2)]
        max_y = None
        for y in range(picture_set_shape[1]):
            row = roi[y, :]
            if 255 in row:
                max_y = y
                break

        roi = [(0, max_y), (640, max_y), (640, 0), (0, 0)]
        cropped_image = np.copy(hsv)
        if self.Turn_R_box is not None:
            cv2.fillPoly(cropped_image, [np.array(self.Turn_R_box, dtype=np.int32)], [0, 0, 0])
        if max_y is not None:
            cv2.fillPoly(cropped_image, [np.array(roi, dtype=np.int32)], [255, 255, 255])

        if self.passed_turn_right_signs:
            mask_gray = cv2.inRange(cropped_image, lower_gray, upper_gray)
        else:
            mask_gray = cv2.inRange(cropped_image, lower_gray, upper_gray)
        mask_gray = cv2.erode(mask_gray, kernel, iterations=2)
        mask_gray = cv2.dilate(mask_gray, kernel, iterations=3)

        mask_white = cv2.inRange(cropped_image, lower_white, upper_white)
        mask_white = cv2.erode(mask_white, kernel, iterations=2)
        mask_white = cv2.dilate(mask_white, kernel, iterations=3)

        not_mask_gray = cv2.bitwise_not(mask_gray)
        if self.sign_detect:
            if time.time() - self.cross_time_stamp > 0.2 / normal_speed:
                self.set_speed = 0.0
        if not self.sign_detect:
            self.set_speed = normal_speed
            if time.time() - self.cross_time_stamp > 1.0 / normal_speed:
                self.get_banmaxian(mask_gray)

        if self.Crossing_box is not None:
            for crx in self.Crossing_box:
                cv2.fillPoly(cropped_image, [np.array(crx)], [0, 0, 0])

        if self.park_sign_pre:
            if not self.start_park :  
                self.start_park = True
                self.stop = True
                threading.Thread(target=self.park_action).start()

        if not self.start_park:
            if self.turn_right_sign_pre and self.sign_detect:
                self.cross_time_stamp = time.time()
                self.turn_right_sign = True
                self.set_speed = normal_speed

            if self.keep_straight_sign_pre :
                self.cross_time_stamp = time.time()
                self.set_speed = normal_speed
                self.sign_detect = False
                self.keep_straight_sign_pre = False

            if self.sidewalk_sign_pre :
                if time.time() - self.cross_time_stamp > 4:
                    self.cross_time_stamp = time.time()
                    self.set_speed = normal_speed
                    self.sign_detect = False
                    self.sidewalk_sign_pre = False
                self.set_speed=0.0
            
            if self.traffic_light :
                self.cross_time_stamp = time.time()
                self.set_speed = normal_speed
                self.sign_detect = False
                self.traffic_light = False

            if self.turn_right:
                self.count_right_miss += 1
                if self.machine_type == 'Mec':
                    if self.count_right_miss >= 61:
                        self.count_right_miss = 0
                        self.turn_right = False
                        self.set_speed = normal_speed
                    if self.count_right_miss <= 28:
                        twist = Twist()
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                    else:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = turn_right_z
                        self.cmd_vel_pub.publish(twist)
                else:
                    if self.count_right_miss >= 62:#49 #51 #47 #47 # 48 # 50
                        self.count_right_miss = 0
                        self.turn_right = False
                        self.set_speed=normal_speed
                    if self.count_right_miss <= 15: #8 #9 #10 #11 # 12 # 14
                        twist = Twist()
                        twist.linear.x = 0.10
                        twist.angular.z= 0.0
                        self.cmd_vel_pub.publish(twist)
                    else:
                        twist = Twist()
                        twist.linear.x = 0.10
                        twist.angular.z= turn_right_z
                        self.cmd_vel_pub.publish(twist)
            if self.turn_right_sign:
                self.count_right_miss += 1
                if self.machine_type == 'Mec':
                    if self.count_right_miss >= 88:
                        self.count_right_miss = 0
                        self.turn_right_sign = False
                        self.turn_right_sign_pre = False
                        self.passed_turn_right_signs = True
                        self.sign_detect = False
                    if self.count_right_miss <= 55:
                        angle = get_saidao(mask_gray, result_image, self.rois)
                        self.pid_control_grey(angle, 28)
                        cv2.putText(result_image, "angle:" + str(int(angle)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                    else:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = turn_right_z
                        self.cmd_vel_pub.publish(twist)
                else:
                    if self.count_right_miss >= 71: #67 #67 
                        self.count_right_miss = 0
                        self.turn_right_sign = False
                        self.turn_right_sign_pre=False
                        self.passed_turn_right_signs=True
                        self.set_speed=normal_speed
                    if self.count_right_miss <= 35: #24 #25
                        angle=get_saidao(mask_gray,result_image,self.rois)
                        self.pid_control_grey(angle,28)
                        cv2.putText(result_image, "angle:"+str(int(angle)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                    else:
                        twist = Twist()
                        twist.linear.x = 0.10
                        twist.angular.z= turn_right_z
                        self.cmd_vel_pub.publish(twist)

            if not self.start_turn and not self.turn_right and not self.turn_right_sign:
                if self.passed_turn_right_signs:
                    errors = get_saidaobianyuan(not_mask_gray, result_image)
                    center_z_ratio = (errors + ros_rgb_image.width // 2) / ros_rgb_image.width
                    self.pid_control(center_z_ratio, 0.5)
                    cv2.putText(result_image, "errors:" + str(int(errors)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                else:
                    angle = get_saidao(mask_gray, result_image, self.rois)
                    self.pid_control_grey(angle, 28)
                    cv2.putText(result_image, "angle:" + str(int(angle)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            cv2.putText(result_image, "turn_r:" + str(self.turn_right), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, "pid_z" + str(round(self.z_dis, 4)), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, "yolo:" + str(self.sign_detect), (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, "turn_r_s:" + str(self.turn_right_sign_pre), (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.putText(result_image, "stop:" + str(self.stop), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        result_image = cv2.resize(result_image, (320, 240))
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        _imshow_fit("result", result_image)
        mask_gray = cv2.resize(mask_gray, (320, 240))
        _imshow_fit("mask_gray", mask_gray)

        key = cv2.waitKey(1)
        if key == ord('q'):
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            rclpy.shutdown()
        elif key == ord('w'):
            self.stop = False
            self.traffic_light=False
            self.sign_detect = False
        elif key == ord('s'):
            self.stop = True
            self.sign_detect = True
        elif key == ord('r'):
            self.start_park = False
            self.get_logger().info("restart_park")

    def get_object_callback(self, msg):
        self.Turn_R_box=None
        self.Crossing_box=None
        self.set_z = -999
        self.objects_info = msg.objects
        if self.objects_info == []:    
            self.traffic_signs_status = None
        else:
            for i in self.objects_info:
                x1 = int(i.box[0])
                y1 = int(i.box[1])
                x2 = int(i.box[2])
                y2 = int(i.box[3])
                object_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                object_area=abs(x1-x2)*abs(y1-y2)
                if object_name == 'Crossing':
                    if self.Crossing_box is None:
                        self.Crossing_box=[]
                    self.Crossing_box.append([(x1,y1),(x2,y1),(x2,y2),(x1,y2)])
                elif object_name == 'Turn_R' and not self.turn_right_sign_pre:
                    self.Turn_R_box=[(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
                    if self.machine_type == 'Mec':
                        # if center[1] >= 400:
                        if y1 > 400 or y2 > 400:
                            self.count_right += 1
                            self.count_right_miss = 0
                            if self.count_right >= 2: 
                                self.turn_right = True
                                self.count_right = 0
                    else:
                        if y1 > 330 or y2 > 330:
                            self.count_right += 1
                            self.count_right_miss = 0
                            if self.count_right >= 2:
                                self.turn_right = True
                                self.count_right = 0
                elif object_name == 'Turn_Right_Sign' and not self.turn_right_sign_pre and self.sign_detect and object_area > 600:
                    self.count_turn_right_sign+=1
                    if self.count_turn_right_sign >= 2:
                        self.turn_right_sign_pre=True
                        self.count_turn_right_sign=0
                elif object_name == 'Keep_Straight_Sign' and not self.keep_straight_sign_pre and self.sign_detect and object_area > 400:
                    self.count_keep_straight_sign+=1
                    if self.count_keep_straight_sign >= 2:
                        self.keep_straight_sign_pre=True
                        self.count_keep_straight_sign=0
                elif object_name == 'Sidewalk_Sign' and not self.sidewalk_sign_pre and self.sign_detect and object_area > 600:
                    self.count_sidewalk_sign+=1
                    if self.count_sidewalk_sign >= 2:
                        self.sidewalk_sign_pre=True
                        self.count_sidewalk_sign=0
                elif object_name == 'Parking_Sign' and not self.park_sign_pre and object_area > 2200:
                        self.count_park_sign+=1
                        if self.count_park_sign >= 2:
                            self.park_sign_pre=True
                            self.count_park_sign=0
                elif object_name == 'Green_Light' and not self.traffic_light and self.sign_detect:
                    self.traffic_light = True
                
            # self.get_logger().info('\033[1;32m%s\033[0m' % object_name)

def main(args=None):
    rclpy.init(args=args)
    node = CamControlNode("cam_control_node")
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()