#!/usr/bin/env python3
# coding: utf8

import cv2, sys, math, time, queue, threading, signal, rclpy, message_filters
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image as RosImage, CameraInfo
from std_srvs.srv import SetBool
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer

from car_yolo.yolov5_trt import YoLov5TRT
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, box_center, distance,  pixels_to_world
from car_vision.cv2_common import calculate_filtered_mean

from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
from car_msg.srv import MoveArm, SetSteer, GetCurrentPose
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor

TRT_NUM_CLASSES = 16
TRT_CLASS_NAMES = ['Hazardous_Waste', 'Recyclables', 'Other_Waste','Food_Waste','Old_Toy','Shrimp_Shell','Old_Bathtub','Paint_Bucket','ECmedicine','Fishbone','Old_Schoolbag','Lighter','Cup','Watermelon_rind','Basketball','Waste_Battery']
TRT_CLASS_garbage_class = {
    'Hazardous_Waste': ('Hazardous_Waste','Paint_Bucket','ECmedicine','Waste_Battery'),
    'Recyclables': ('Recyclables','Old_Toy','Old_Schoolbag','Basketball'),
    'Other_Waste': ('Other_Waste','Old_Bathtub','Lighter','Cup'),
    'Food_Waste': ('Food_Waste','Shrimp_Shell','Fishbone','Watermelon_rind')
}

class GarbageClassNode(Node):
    def __init__(self):
        super().__init__('garbage_class_node')

        signal.signal(signal.SIGINT, self.clean_up)
        signal.signal(signal.SIGTERM, self.clean_up)

        self.bridge = CvBridge()
        self.moving = False
        self.count = 0
        self.endpoint = None
        self.info_msg = None
        self.garbage_stack = [0]*16
        self.garbage_pick_order = []
        self.garbage_order=0
        self.garbage_pick_start = True
        self.box_pos_last = [-99]*4

        self.cli_move_arm = self.create_client(MoveArm, 'move_arm')
        self.cli_get_current_pose = self.create_client(GetCurrentPose, 'get_current_pose')
        self.cli_set_steer = self.create_client(SetSteer, 'set_steer')
        while not self.cli_move_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_arm service...')

        self.move_arm([0.12,0.0,0.17],70.0)
        time.sleep(1.0)

        if self.endpoint is None:
            self.endpoint = self.get_endpoint()

        weights = '/home/nvidia/weights/garbage_classification/garbage_class.engine'
        lib = '/home/nvidia/weights/garbage_classification/libmyplugins.so'
        self.yolov5 = YoLov5TRT(weights, lib, TRT_CLASS_NAMES, 0.75)
        time.sleep(1.0)

        # TODO: Port service call to rclpy

        cb_group = ReentrantCallbackGroup()
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        # self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth/camera_info', callback_group=cb_group)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_callback, qos_profile_sensor_data)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=3, 
            slop=0.2
        )
        self.ts.registerCallback(self.multi_callback)

        self.queue = queue.Queue(maxsize=1)
        self.get_logger().info('初始化完成！')

        self.pick_sub = self.create_subscription(String, '/chat_model/whichone', self.pickinfo_callback, qos_profile_sensor_data)

    def move_arm(self, pos, pitch=90.0, roll=0.0, hand=0.80, duration=1000):
        req = MoveArm.Request()
        req.pos = [float(x) for x in pos]
        req.pitch = float(pitch)
        req.roll = float(roll)
        req.hand = float(hand)
        req.duration = duration

        future = self.cli_move_arm.call_async(req)
        
        # 用 threading.Event + 超时机制等待结果
        done = threading.Event()

        def done_callback(fut):
            done.set()

        future.add_done_callback(done_callback)
        done.wait(timeout=5.0)  # 等待最多5秒

        if future.done():
            return future.result()
        else:
            self.get_logger().error("move_arm 超时未响应")
            return None

    def set_steer(self,joints,duration):
        req = SetSteer.Request()
        req.angles = joints
        req.duration = duration

        future = self.cli_set_steer.call_async(req)
        done = threading.Event()

        def done_callback(fut):
            done.set()

        future.add_done_callback(done_callback)
        if done.wait(timeout=5.0):
            return future.result()
        else:
            self.get_logger().error("set_steer 超时未响应")
            return None

    def get_endpoint(self):
        req = GetCurrentPose.Request()
        future = self.cli_get_current_pose.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        endpoint=future.result().pose
        print("endpoint:",endpoint)
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

    def multi_callback(self, rgb_msg, depth_msg):
        try:
            # 检查队列状态
            if self.queue.full():
                self.queue.get_nowait()  # 移除旧消息
            self.queue.put((rgb_msg, depth_msg))  # 使用阻塞操作
        except Exception as e:
            self.get_logger().error(f"处理图像消息时出错: {str(e)}")

    def caminfo_callback(self, info_msg):
        if self.info_msg is None:
            self.info_msg=info_msg

    def pickinfo_callback(self,msg):
        try:
            garbage_names = msg.data.split(",")  # e.g., ["Hazardous_Waste", "Recyclables", "Other_Waste", "Food_Waste"]
            self.garbage_pick_order = garbage_names
            print("garbage_pick_order =", self.garbage_pick_order,",len",len(self.garbage_pick_order))
        except KeyError as e:
            self.get_logger().error(f"未知颜色名称：{e}")
        except Exception as e:
            self.get_logger().error(f"处理 pickinfo 消息出错: {e}")

        self.garbage_order=0

    def clean_up(self, *args):
        self.destroy_node()
        cv2.destroyAllWindows()
        print("节点退出")

    def shut_down_callback(self):
        self.yolov5.destroy()
        cv2.destroyAllWindows()

    def pick(self, A, which_class):
        self.moving = True
        x=A.position.x
        y=A.position.y
        z=A.position.z
        rolll=A.roll
        
        self.move_arm([x,y,z+0.03],pitch=70,roll=rolll,duration=1500)
        time.sleep(0.5)
        self.move_arm([x,y,z-0.003],pitch=70,roll=rolll,duration=800)
        time.sleep(0.5)
        self.move_arm([x,y,z-0.003],pitch=70,roll=rolll,hand=-0.5,duration=500)
        time.sleep(0.5)
        self.move_arm([x,y,z+0.07],pitch=70,hand=-0.5,duration=800)
        time.sleep(0.5)
        set_x=-0.035
        set_y=0.230
        set_z=0.020
        set_roll=-10
        if which_class == 'Other_Waste':
            set_x=-0.039
            set_y=0.240
            set_roll=-9
        elif which_class == 'Recyclables':
            set_x=0.007
            set_y=0.240
            set_roll=5.5
        elif which_class == 'Food_Waste':
            set_x=0.057
            set_y=0.238
            set_roll=16
        elif which_class == 'Hazardous_Waste':
            set_x=0.10
            set_y=0.235
            set_z=0.022
            set_roll=28

        self.move_arm([set_x,set_y,0.10],pitch=70,roll=set_roll,hand=-0.5,duration=1300)
        time.sleep(0.5)
        self.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.5,duration=1000)
        time.sleep(0.5)
        self.move_arm([set_x,set_y,set_z],pitch=70,roll=set_roll,hand=-0.2,duration=400)
        time.sleep(0.5)
        self.move_arm([set_x,set_y,0.10],pitch=70,roll=set_roll,hand=-0.2,duration=800)
        time.sleep(1.0)
        self.move_arm([0.12,0.0,0.17],pitch=70.0,duration=1500)
        time.sleep(3.0)

        self.garbage_order+=1
        self.moving = False

    def image_proc(self):
        try:
            ros_rgb, ros_depth = self.queue.get_nowait()
        except queue.Empty:
            return

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
            
            if self.moving == False and self.info_msg:
                boxes, confs, classes = self.yolov5.infer(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

                for box, cls_conf, cls_id in zip(boxes, confs, classes):
                    x1 = box[0]
                    y1 = box[1]
                    x2 = box[2]
                    y2 = box[3]
                    object_name = TRT_CLASS_NAMES[cls_id]

                    if self.garbage_pick_start == True and len(self.garbage_pick_order) > self.garbage_order and self.garbage_pick_order[self.garbage_order] == self.whichclass_belong(object_name):
                        box_pos=[int(x1),int(y1),int(x2),int(y2)]
                        garbage_name=object_name

                    result_image = cv2.putText(result_image, object_name + " " + str(float(cls_conf))[:4], (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, [255,0,0], 2)
                    result_image = cv2.rectangle(result_image, (int(x1), int(y1)), (int(x2), int(y2)), [255,0,0], 3)
                    depth_color_map = cv2.rectangle(depth_color_map, (int(x1), int(y1)), (int(x2), int(y2)), [255,255,0], 3)
                    self.garbage_stack[cls_id]+=1

                if self.garbage_pick_start == True:
                    if box_pos is not None:
                        a=box_center(box_pos)
                        b=box_center(self.box_pos_last)

                        if distance(a,b) < 50:
                            center_x, center_y = a
                            depth = np.copy(depth_image).reshape((-1, ))
                            depth[depth<=130] = 55555

                            mean=round(calculate_filtered_mean(depth_image,(box_pos[0],box_pos[1],box_pos[2],box_pos[3]),lower_threshold=160,upper_threshold=250))
                            print("mean:",mean)
                            dist = mean/1000.0 # depth_image[int(center_y),int(center_x)]/1000.0

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

                                PoseRC = self.count_position("Rect", center, depth_image, self.info_msg.k, rect[2])

                                print("garbage_name:",garbage_name,"class:", self.whichclass_belong(garbage_name))

                                threading.Thread(target=self.pick, args=(PoseRC,self.whichclass_belong(garbage_name))).start()
                                break
                        self.box_pos_last=box_pos

            result_image=cv2.resize(result_image,(320,200))
            result_image=cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            depth_color_map=cv2.resize(depth_color_map,(320,200))
            con_image = np.concatenate([result_image, depth_color_map], axis=1)
            cv2.imshow("con", con_image)

            key=cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def whichclass_belong(self,name):
        for category, items in TRT_CLASS_garbage_class.items():
            if name in items:
                return category

    def count_position(self,Name,center,depth_image,K,roll):
        PWRAC=PoseWithRollAndColor()

        center_x, center_y = center

        center_x = min(639, int(center_x))
        center_y = min(359, int(center_y))

        dist = depth_image[int(center_y),int(center_x)]/1000.0
        dist += 0.015 # 物体半径补偿

        position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))

        position[0] -= 0.007  # rgb相机和深度相机tf有1cm偏移
        temp=position[0]
        position[0]=position[1]
        position[1]=-temp

        pose_end = np.matmul(matrix_hand_to_cam, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标 即机械臂抓手到像素点的相对坐标
        world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
        pose_t, pose_R = mat_to_xyz_euler(world_pose)

        pose_t[0] = pose_t[0] + 0.008 # * 0.9
        pose_t[1] = pose_t[1] - 0.015 # * 0.9

        yaw = math.degrees(math.atan2(pose_t[1], pose_t[0]))
        if roll==90.0:
            roll=0
        if pose_t[1] <= 0:
            roll=-roll-yaw
        else:
            roll=roll+yaw
            roll=90-roll

        PWRAC.name = Name
        PWRAC.position.x = pose_t[0]
        PWRAC.position.y = pose_t[1]
        PWRAC.position.z = pose_t[2]
        PWRAC.roll = roll
        return PWRAC  # 返回位置和roll角度

def main(args=None):
    rclpy.init(args=args)
    garbage_class_node = GarbageClassNode()

    executor = MultiThreadedExecutor()
    executor.add_node(garbage_class_node)
    try:
        while rclpy.ok():
            # 手动处理所有回调（包括订阅、服务等）
            executor.spin_once(timeout_sec=0.1)

            # 手动调用图像处理（代替定时器）
            garbage_class_node.image_proc()

            time.sleep(0.01)  # 控制频率，避免占满CPU
    finally:
        garbage_class_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
