#!/usr/bin/env python3
# coding=utf8

import cv2,rclpy,threading,math,time,queue,message_filters
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

from car_msg.msg import PoseWithRollAndColor,PoseWithRollAndColorArray
from std_msgs.msg import String
from car_msg.msg import Beep
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


# 颜色库（保持原配置）
color_ranges={"RED":[[9,142,130],[185,183,189]],"GREEN":[[0,0,130],[200,120,200]],"BLUE":[[10,100,50],[180,180,115]]}

close_cv_then_pub=True

class MultiNav(Node):
    def __init__(self):
        super().__init__("nav2_goals_node")  # 节点名称可自定义
        self.rect_aim_targets_order = 0
        self.nav_status="wait"
        self.nav_order=0                           
        self.targets = [[0.40, 0.86, 0.7017, 0.7071],[2.06, 0.81, 0.92, 0.38]]

        self.navigator = BasicNavigator()
        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)

    def beep_open(self):
        msg = Beep()
        msg.times = 1
        msg.on_time = 0.3
        msg.off_time = 0.1
        self.beep_pub.publish(msg)

    def start_navigage_goal(self,goal_xyzw):
        if goal_xyzw is not None:
            goal=PoseStamped()
            goal.header.frame_id='map'
            goal.pose.position.x=goal_xyzw[0]
            goal.pose.position.y=goal_xyzw[1]
            goal.pose.orientation.w=goal_xyzw[2]
            goal.pose.orientation.z=goal_xyzw[3]
            self.get_logger().info(f'发送目标: {goal.pose.position.x}, {goal.pose.position.y}')

            self.navigator.goToPose(goal)
        else:
            self.get_logger().error('目标为空！')

    def check_navigate_status(self):
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('导航成功！')
                self.nav_status = 'success'
                self.beep_open()
            else:
                self.get_logger().info('导航失败！')
                self.nav_status = 'wait'
        else:
            feedback = self.navigator.getFeedback()

class ColorFinder:
    """颜色识别类（保持原逻辑，适配ROS2图像格式）"""
    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB)
        mask = cv2.inRange(img_lab, tuple(color_ranges[0]), tuple(color_ranges[1]))

        # 底部/左侧区域掩码（过滤干扰）
        # mask_height = mask.shape[0]
        # crop_bottom = 30
        # mask[mask_height - crop_bottom:mask_height, :] = 0
        # mask[:, 0:30] = 0

        # 形态学滤波
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 查找最大轮廓
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        min_c = None
        max_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue
            if area > max_area:
                max_area = area
                min_c = c

        if min_c is not None:
            rect = cv2.minAreaRect(min_c)
            center_x, center_y = rect[0]
            box = cv2.boxPoints(rect)
            box[:, 0] *= 2
            box[:, 1] *= 2
            box = np.intp(box)
            cv2.drawContours(result_image, [box], 0, (0, 0, 255), 2)

            # 绘制中心点和圆
            circle_color = (0x55, 0x55, 0x55)
            radius = cv2.minEnclosingCircle(min_c)[1]
            cv2.circle(result_image, (int(center_x*2), int(center_y*2)), int(radius*2), circle_color, 2)

            # 坐标缩放回原图尺寸
            center_x *= 2
            center_y *= 2
            return (result_image, (0, 0), (center_x, center_y), radius*2, rect[2])
        else:
            return (result_image, None, None, 0, 0)

class ColorRectPickNode(Node):
    """ROS2核心节点（继承Node类）"""
    def __init__(self):
        super().__init__("color_rect_pick_node")  # ROS2节点初始化

        # 核心变量初始化（保持原逻辑）
        self.running=True
        self.moving = False
        self.last_pitch_yaw = (0, 0)
        self.stamp = time.time()
        self.color_in_order=[color_ranges['RED']]  # 抓取顺序
        self.put_box_order = [color_ranges['GREEN']]
        self.color_order = 0
        self.endpoint = None
        self.info_msg=None
        self.detect_times = 0

        self.bridge = CvBridge()
        self.get_logger().info(f"本次需检测 {len(self.color_in_order)} 种颜色")
        
        self.tracker = ColorFinder()
        self.catch_stage = 0

        self.need_arm_turn_origin = True
        self.turn_origin_with_close = False

        # 初始化导航模块（传入当前节点实例）
        self.multi_nav = MultiNav()

        self.Arm_controller = arm_ik_sdk.ArmControl(self)
        time.sleep(2.0)
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        time.sleep(2.0)

        cb_group = ReentrantCallbackGroup()
        self.result_img_pub = self.create_publisher(RosImage, '/vision/result', 1)  # 结果发布
        self.rgb_sub = message_filters.Subscriber(self, RosImage, '/camera/color/image_raw', callback_group=cb_group)
        self.depth_sub = message_filters.Subscriber(self, RosImage, '/camera/depth/image_raw', callback_group=cb_group)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_callback, qos_profile_sensor_data)

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=3, slop=0.2)
        self.ts.registerCallback(self.multi_callback)
        self.image_queue = queue.Queue(maxsize=2)

        self.image_thread = threading.Thread(target=self.image_proc,daemon=True)
        self.image_thread.start()


    def caminfo_callback(self, info_msg):
        if self.info_msg is None:
            self.info_msg=info_msg

    def multi_callback(self,ros_image,depth_image):
        if self.image_queue.full():
            self.image_queue.get()  # Discard the oldest frame
        self.image_queue.put([ros_image, depth_image])

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

    def pick(self,PoseRC):
        """机械臂抓取/放置逻辑（保持原动作序列，适配ROS2睡眠）"""
        self.moving = True
        x=PoseRC.position.x
        y=PoseRC.position.y
        z=PoseRC.position.z
        rolll=PoseRC.roll
        
        # 抓取阶段（catch_stage=0）
        if self.catch_stage == 0:
            # self.Arm_controller.move_arm([x, y, z+0.04], pitch=80, roll=rolll, duration=800)
            # self.create_rate(2.0).sleep()
            self.Arm_controller.move_arm([x, y, z], pitch=80, roll=rolll, duration=1400)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([x, y, z], pitch=80, roll=rolll, hand=-0.05, duration=500)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([x, y, z+0.04], pitch=80, hand=-0.05, duration=600)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([0.14, 0.0, 0.12], pitch=70, hand=-0.05,duration=1000)
            self.create_rate(2).sleep()
            self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,-0.05])
            self.create_rate(2).sleep()

            self.multi_nav.nav_status="wait"
            self.moving = False
            self.multi_nav.nav_order+=1
            self.need_arm_turn_origin=True
            self.turn_origin_with_close=True
            self.catch_stage = 1

        # 放置阶段（catch_stage=1）
        elif self.catch_stage == 1:
            self.Arm_controller.move_arm([x, y, z+0.08], pitch=85,hand=-0.05, duration=1000)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([x, y, z+0.05], pitch=85, hand=-0.05, duration=300)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([x, y, z+0.05], pitch=85, duration=300)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([x, y, z+0.08], pitch=85, duration=300)
            self.create_rate(2).sleep()
            self.Arm_controller.move_arm([0.14, 0.0, 0.09], pitch=70,duration=1200)
            self.create_rate(2).sleep()
            self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
            self.create_rate(2).sleep()

            self.moving = False
            # self.multi_nav.nav_status="wait"
            self.need_arm_turn_origin=True
            # self.multi_nav.nav_order+=1
            # self.color_order+=1
            self.catch_stage = 0

    def image_proc(self):
        """图像处理主逻辑（保持原识别+抓取逻辑）"""
        while self.running:
            time.sleep(0.1)
            try:
                self.multi_nav.nav_status="success"

                if self.moving is False and self.multi_nav.nav_status == "wait":
                    self.multi_nav.start_navigage_goal(self.multi_nav.targets[self.multi_nav.nav_order])
                    self.multi_nav.nav_status = "sent"

                if self.moving is False and self.multi_nav.nav_status == "sent":
                    self.multi_nav.check_navigate_status()
                    if self.multi_nav.nav_status == "success":
                        time.sleep(2.0)

                if self.moving is False and self.multi_nav.nav_status == "success":
                    # 从队列获取同步数据
                    ros_rgb_image, ros_depth_image = self.image_queue.get(block=True, timeout=1.0)
                    if self.endpoint is None:
                        self.endpoint = self.get_endpoint()

                    rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, 'rgb8')
                    depth_image = self.bridge.imgmsg_to_cv2(ros_depth_image, '16UC1')

                    rh, rb = rgb_image.shape[:2]
                    ih, iw = depth_image.shape[:2]
                    rgb_image=rgb_image[(rh-ih)//2:rh-(rh-ih)//2,]

                    result_image = np.copy(rgb_image)

                    # 深度图预处理（保持原逻辑）
                    depth = np.copy(depth_image).reshape((-1,))
                    depth[depth <= 0] = 55555

                    sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
                    sim_depth_image = sim_depth_image / 2000.0 * 255.0
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                    # 颜色识别与抓取判断（保持原逻辑）
                    if self.moving is False:
                        if self.catch_stage == 0:
                            result_image, p_y, center, r, roll = self.tracker.proc(rgb_image, result_image, self.color_in_order[self.color_order])
                        elif self.catch_stage == 1:
                            result_image, p_y, center, r, roll = self.tracker.proc(rgb_image, result_image, self.put_box_order[self.color_order])

                        if self.need_arm_turn_origin==True:
                            if self.turn_origin_with_close==True:
                                self.Arm_controller.move_arm([0.14, 0.0, 0.09], pitch=70.0, hand=-0.05, duration=500)
                            else:
                                self.Arm_controller.move_arm([0.14, 0.0, 0.09], pitch=70.0, duration=500)

                            self.create_rate(1.0).sleep()
                            self.endpoint = self.get_endpoint()
                            self.need_arm_turn_origin = False
                            self.turn_origin_with_close=False
                            continue

                        # 识别到目标颜色
                        if center is not None:
                            if abs(self.last_pitch_yaw[0]-center[0]) < 60 and abs(self.last_pitch_yaw[1]-center[1]) < 60:
                                self.detect_times+=1

                                if self.detect_times > 10:
                                    self.detect_times=0

                                    depth_center_pixel=[0,0]
                                    depth_center_pixel[0] = int(((center[0] + 10)-(iw//2)) / 1.26 + (iw//2))
                                    depth_center_pixel[1] = int((center[1]-(ih//2)) / 1.26+(ih//2))                            
                                    PoseRC = self.count_position("Rect", depth_center_pixel, depth_image, self.info_msg.k, roll)
                                    print("PoseRC",PoseRC)
                                    
                                    # 启动抓取线程
                                    if PoseRC is not None:
                                        threading.Thread(target=self.pick, args=(PoseRC,), daemon=True).start()

                                    # 绘制深度信息
                                    # txt = "TOO CLOSE!!!" if dist < 100 else f"Dist: {dist}mm"
                                    cv2.circle(result_image, (int(center[0]), int(center[1])), 5, (255, 255, 255), -1)
                                    cv2.circle(depth_color_map, (int(depth_center_pixel[0]), int(depth_center_pixel[1])), 5, (255, 255, 255), -1)
                            else:
                                if self.detect_times > 0:
                                    self.detect_times-=1

                            self.last_pitch_yaw = center
                        
                    # 图像显示（保持原逻辑）
                    ee_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
                    ee_image = cv2.resize(ee_image, (320, 240))
                    depth_color_map = cv2.resize(depth_color_map, (320, 240))
                    result_image = np.concatenate([ee_image, depth_color_map], axis=1)
                    
                    if not close_cv_then_pub:
                        cv2.imshow('image', result_image)
                    else:
                        self.result_img_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

                    # 退出按键处理
                    key = cv2.waitKey(1)
                    if key in [27, ord('q')]:  # ESC或q退出
                        rclpy.shutdown()
                        cv2.destroyAllWindows()

            except Exception as e:
                self.get_logger().error(f"图像处理异常：{str(e)}")

    def count_position(self,Name,center,depth_image,K,roll):
        PWRAC=PoseWithRollAndColor()

        center_x, center_y = center

        center_x = min(639, int(center_x))
        center_y = min(479, int(center_y))

        dist = depth_image[int(center_y),int(center_x)]/1000.0
        if dist == 0:
            return None
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
            ### [0.14, 0.0, 0.09]

            # x轴系数主要跟机械臂前后倾倒角度有关
            pose_t[0] = pose_t[0] * 1.02 # 0.93
            # y轴系数主要与机械臂左右夹取位置有关
            pose_t[1] = pose_t[1] + 0.01
            pose_t[1] = pose_t[1] * 1.08
            # z轴系数主要与夹取物体高度有关
            pose_t[2] = pose_t[2] + 0.0

            ### [0.14, 0.0, 0.08]

            # pose_t[0] = (pose_t[0]-0.178) * 0.88 + 0.178
            # pose_t[0] = pose_t[0] + -0.01

            # pose_t[1] = pose_t[1] + 0.0
            # pose_t[1] = pose_t[1] * 1.06
        else:
            pose_t[0] = pose_t[0]*0.85
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

def main():
    try:
        rclpy.init()
        node = ColorRectPickNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断，节点关闭...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()