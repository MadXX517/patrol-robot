import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math
#################################################调节参数############################################################

img_size = (640,480)

#调整中点，用于调整机械臂抓取物块中点
#如果机械臂抓取偏左，mid_block_cx减小，反之增加
mid_block_cx = img_size[0]/2
mid_block_cy = img_size[1]/2

#机械臂在抓取的时候够不着需要调节 arm_skewing 的值
#如果机械臂抓取偏前， arm_skewing 减小，反之增加
arm_skewing = 55

#机械臂抓取物品高度调节
#如果机械爪太高 grasp_height 减小，反之增加
grasp_height = 55

#机械臂识别高度
arm_up = 170

#机械臂初始角度和执行时间
joints = [0,-0.93,2.07,1.3,0,0.8,1500]

time_cnt = 0#时间计数

#机械臂移动位置
move_x = 0
move_y = 175
pro_status = 0
move_status = 0#机械臂移动的方式
spin_calw = 0#机械爪角度
color_read_succed =0
cap_color_status = 0#抓取物块颜色标志，用来判断物块抓取
is_pick_flag = 0
start_flag = 0 #开机标志位，用于开机初始化姿态

###################################################################################################################

class Nav2Pick(Node):
    def __init__(self):
        super().__init__('nav2_pick')
        # Initialize navigator
        self.navigator = BasicNavigator()
        # Image processing
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub_arm = self.create_publisher(JointState, '/ik_states', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscription  # Prevent unused variable warning

        # 初始化状态
        self.state = 'navigate_to_first_goal'
        self.start_flag = 0

        # 定义目标点
        self.first_goal = PoseStamped()
        self.first_goal.header.frame_id = 'map'
        self.first_goal.pose.position.x = 1.0
        self.first_goal.pose.position.y = 1.0
        self.first_goal.pose.orientation.w = 1.0
        
        self.second_goal = PoseStamped()
        self.second_goal.header.frame_id = 'map'
        self.second_goal.pose.position.x = 1.0
        self.second_goal.pose.position.y = -2.0
        self.second_goal.pose.orientation.w = 1.0
        
        self.origin = PoseStamped()
        self.origin.header.frame_id = 'map'
        self.origin.pose.position.x = 0.0
        self.origin.pose.position.y = 0.0
        self.origin.pose.orientation.w = 1.0
        
    def navigate_to_goal(self, goal):
        if goal is not None:
            self.get_logger().info(f'发送目标: {goal.pose.position.x}, {goal.pose.position.y}')
            self.navigator.goToPose(goal)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # self.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f} 米')
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('导航成功！')
                self.car_move(0,0,0)
                if self.state == 'navigate_to_first_goal':
                    self.state = 'pick_object'
                elif self.state == 'navigate_to_second_goal':
                    self.state = 'place_object'
                elif self.state == 'navigate_to_origin':
                    self.state = 'done'
            else:
                self.get_logger().info('导航失败！')
        else:
            self.get_logger().error('目标为空！')
    
    def image_callback(self, msg):
        if self.start_flag == 0:
            self.robot_init()
            self.start_flag = 1 
        if self.state == 'navigate_to_first_goal':
            self.navigate_to_goal(self.first_goal)
        elif self.state == 'pick_object':
            self.pick_task(msg)
        elif self.state == 'navigate_to_second_goal':
            self.navigate_to_goal(self.second_goal)
        elif self.state == 'place_object':
            self.place_task(msg)
        elif self.state == 'navigate_to_origin':
            self.navigate_to_goal(self.origin)

    def pick_task(self,msg):
        global move_x,move_y,move_status,cap_color_status
        global time_cnt,spin_calw,color_read_succed,block_cx,block_cy
        # Integrate the grasping logic here
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = mask1 + mask2
        contours = self.find_contours_with_min_area(mask[0:300, 0:640],1000)
        max_area = 0
        max_area_coord = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    block_cx = int(M["m10"] / M["m00"])
                    block_cy = int(M["m01"] / M["m00"])
                    max_area_coord = (block_cx, block_cy)

        if max_area_coord:
            block_cx, block_cy = max_area_coord
            color_read_succed=1
            coordinates = f"Max area detected at x: {block_cx}, y: {block_cy}"
            # self.get_logger().info(coordinates)
        if move_status==0:
            if color_read_succed==1:
                self.car_move(0,0,0) 
                move_status=1
            elif color_read_succed==0:
                self.car_move(0.08,0,0)
        elif move_status==1: 
            if(abs(block_cx-mid_block_cx)>20):
                if block_cx > mid_block_cx:
                    self.car_move(0,0,-0.06)
                else:
                    self.car_move(0,0,0.06)
            if(abs(block_cy-200)>20):
                if block_cy > 200:
                    self.car_move(-0.06,0,0)
                else:
                    self.car_move(0.06,0,0)
            if abs(block_cy-200)<=20 and abs(block_cx-mid_block_cx)<=20:
                time_cnt += 1
                if time_cnt>20: #计数100次对准物块，防止误差
                    time_cnt=0
                    move_status=2
                self.car_move(0,0,0)
        elif move_status==2: 
            self.car_move(0,0,0)
            if(abs(block_cx-mid_block_cx)>5):
                if block_cx > mid_block_cx:
                    move_x-=0.3
                else:
                    move_x+=0.3
            if(abs(block_cy-mid_block_cy)>5):
                if block_cy > mid_block_cy and move_y>1:
                    move_y-=0.3
                else:
                    move_y+=0.3
            if abs(block_cy-mid_block_cy)<=5 and abs(block_cx-mid_block_cx)<=5: #寻找到物块，机械臂进入第二阶段
                time_cnt += 1
                if time_cnt>50: #计数50次对准物块，防止误差
                    time_cnt=0
                    move_status=3
                    spin_calw=0
                    #三角函数计算色块与机械爪距离
                    l=math.sqrt(move_x*move_x+move_y*move_y)
                    sin=move_y/l
                    cos=move_x/l
                    move_x=(l+arm_skewing)*cos#(l+x)--x调整的距离
                    move_y=(l+arm_skewing)*sin
            else:
                time_cnt=0
                self.kinematics_move(move_x,move_y,arm_up,0)
        elif move_status==3 : #第1阶段：机械臂抓取物块
            time_cnt += 1
            if time_cnt<2: #旋转机械臂与色块平齐
                # 轮廓近似  
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)  
                # 检查轮廓是否有四个顶点（可能是正方形）  
                if len(approx) == 4:
                    # 计算最小面积外接矩形  
                    rect = cv2.minAreaRect(approx)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # # 绘制外接矩形  
                    # cv2.drawContours(img, [box], 0, (0, 0, 0), 2)  
                    # 计算旋转角度（以水平方向为基准）  
                    angle = rect[-1]
                    if angle < -45:  
                        angle += 90  
                    elif angle > 45:  
                        angle -= 90  
                    spin_calw = -angle * (3.1415926 / 180.0)
                    if abs(angle)>30: #偏转角度机械臂需要前移
                        #三角函数计算色块与机械爪距离
                        move_y+=0.5
                        # print(f"Square angle: {angle} degrees {spin_calw}")
                self.claw_move(spin_calw,0.8,1000)
            elif time_cnt>=2 and time_cnt<35: #移动机械臂到物块上方
                self.kinematics_move(move_x,move_y,arm_up,1000)
            elif time_cnt>=35 and time_cnt<70: #移动机械臂下移到物块
                self.kinematics_move(move_x,move_y,grasp_height,1000)
            elif time_cnt>=105 and time_cnt<140: #机械爪抓取物块
                self.claw_move(spin_calw,-0.3,1000)
            elif time_cnt>=175 and time_cnt<210: #移动机械臂抬起
                self.kinematics_move(move_x,move_y,arm_up,1000)
            elif time_cnt>=245 and time_cnt<280: ##机械臂归位
                move_x=0
                move_y=150
                spin_calw=0
                self.kinematics_move(move_x,move_y,arm_up,1000)
                self.claw_move(spin_calw,-0.3,1000)   
            elif time_cnt>=305 and time_cnt<340:
                move_status=4
                self.state='navigate_to_second_goal'
                color_read_succed=0

    def place_task(self,msg):
        global move_x,move_y,move_status,cap_color_status
        global time_cnt,spin_calw,color_read_succed,block_cx,block_cy
        # Integrate the grasping logic here
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = mask1 + mask2
        contours = self.find_contours_with_min_area(mask[0:300, 0:640],1000)
        max_area = 0
        max_area_coord = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    block_cx = int(M["m10"] / M["m00"])
                    block_cy = int(M["m01"] / M["m00"])
                    max_area_coord = (block_cx, block_cy)
        if max_area_coord:
            block_cx, block_cy = max_area_coord
            color_read_succed=1
            coordinates = f"Max area detected at x: {block_cx}, y: {block_cy}"
            # self.get_logger().info(coordinates)
        if move_status==4:
            if color_read_succed==1:
                self.car_move(0,0,0) 
                move_status=5
            elif color_read_succed==0:
                self.car_move(0.1,0,0)
        elif move_status==5: 
            if(abs(block_cx-mid_block_cx)>20):
                if block_cx > mid_block_cx:
                    self.car_move(0,0,-0.06)
                else:
                    self.car_move(0,0,0.06)
            if(abs(block_cy-mid_block_cy)>20):
                if block_cy > mid_block_cy:
                    self.car_move(-0.06,0,0)
                else:
                    self.car_move(0.06,0,0)
            if abs(block_cy-mid_block_cy)<=20 and abs(block_cx-mid_block_cx)<=20:
                time_cnt += 1
                if time_cnt>20: #计数100次对准物块，防止误差
                    time_cnt=0
                    move_status=6
                self.car_move(0,0,0)
        # elif move_status==6: 
        #     self.car_move(0,0,0)
        #     if(abs(block_cx-mid_block_cx)>5):
        #         if block_cx > mid_block_cx:
        #             move_x-=0.3
        #         else:
        #             move_x+=0.3
        #     if(abs(block_cy-mid_block_cy)>5):
        #         if block_cy > mid_block_cy and move_y>1:
        #             move_y-=0.3
        #         else:
        #             move_y+=0.3
        #     if abs(block_cy-mid_block_cy)<=10 and abs(block_cx-mid_block_cx)<=10: #寻找到物块，机械臂进入第二阶段
        #         time_cnt += 1
        #         if time_cnt>20: #计数50次对准物块，防止误差
        #             time_cnt=0
        #             move_status=7
        #             spin_calw=0
        #             #三角函数计算色块与机械爪距离
        #             l=math.sqrt(move_x*move_x+move_y*move_y)
        #             sin=move_y/l
        #             cos=move_x/l
        #             move_x=(l+arm_skewing)*cos #(l+x)--x调整的距离
        #             move_y=(l+arm_skewing)*sin
        #     else:
        #         time_cnt=0
        #         self.kinematics_move(move_x,move_y,arm_up,0)
        elif move_status==6 : #第1阶段：机械臂抓取物块
            time_cnt += 1
            if time_cnt<2: 
                self.claw_move(spin_calw,-0.3,1000)
            elif time_cnt>=2 and time_cnt<35: #移动机械臂到物块上方
                move_x=0
                move_y=180
                spin_calw=0
                self.kinematics_move(move_x,move_y,arm_up,1000)
            elif time_cnt>=35 and time_cnt<70: #移动机械臂下移到物块
                self.kinematics_move(move_x,move_y,100,1000)
            elif time_cnt>=105 and time_cnt<140: #机械爪抓取物块
                self.claw_move(spin_calw,0.8,1000)
            elif time_cnt>=175 and time_cnt<210: #移动机械臂抬起
                self.kinematics_move(move_x,move_y,arm_up,1000)
            elif time_cnt>=245 and time_cnt<280: ##机械臂归位
                move_x=0
                move_y=150
                spin_calw=0
                self.kinematics_move(move_x,move_y,arm_up,1000)
                self.claw_move(spin_calw,0.8,1000)   
            elif time_cnt>=305 and time_cnt<340:
                move_status=7
                self.state='navigate_to_origin'

    # 寻找轮廓并筛选面积  
    def find_contours_with_min_area(self,mask, min_area):  
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
        filtered_contours = []  
        for contour in contours:  
            area = cv2.contourArea(contour)  
            # print(f"Contour area: {area}")  # 打印轮廓面积  
            if area >= min_area:  
                filtered_contours.append(contour)  
        return filtered_contours

    def claw_move(self,spin_calw,hand,time):
        # 控制机械爪
        joints[4]=spin_calw
        joints[5]=hand
        joints[6]=time
        self.arm_pub()

    # 寻找最佳角度
    def kinematics_move(self,x:float,y:float,z:float,time:int)->int:

        if y < 0:
            return
        # 寻找最佳角度
        flag = 0
        cnt = 0
        for i in range(0, -136, -1):
            # print(isinstance(kinematics_analysis(x, y, z, i),str),i,kinematics_analysis(x, y, z, i))
            if  self.kinematics_analysis(x, y, z, i)==0:
                if i < cnt:
                    cnt = i
                flag = 1

        # 用3号舵机与水平最大的夹角作为最佳值
        if flag:
            self.kinematics_analysis(x, y, z, cnt)
            joints[6] = time
            self.arm_pub()
            return 0

    # 求逆运动学解
    def kinematics_analysis(self,x:float, y:float, z:float, Alpha:float) -> int: 
        '''
            x,y 为映射到平面的坐标
            z为距离地面的距离
            Alpha 为爪子和平面的夹角 -25~-65范围比较好
        '''
        pi=3.1415926

        #放大10倍
        x = x*10
        y = y*10
        z = z*10
        #请注意版本
        l0 = 2100     
        l1 = 1250     
        l2 = 1200
        l3 = 1550

        if x == 0:
            theta6 = 0.0
        else:
            theta6 = math.atan(x/y)*270.0/pi


        y = math.sqrt(x*x + y*y)
        y = y-l3 * math.cos(Alpha*pi/180.0)
        z = z-l0-l3*math.sin(Alpha*pi/180.0)
        if z < -l0:
            return 1
        if math.sqrt(y*y + z*z) > (l1+l2):
            return 2

        ccc = math.acos(y / math.sqrt(y * y + z * z))
        bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*math.sqrt(y*y+z*z))
        if bbb > 1 or bbb < -1:
            return 5
        if z < 0:
            zf_flag = -1
        else:
            zf_flag = 1

        theta5 = ccc * zf_flag + math.acos(bbb)
        theta5 = theta5 * 180.0 / pi
        if theta5 > 180.0 or theta5 < 0.0:
            return 6

        aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2)
        if aaa > 1 or aaa < -1:
            return 3

        theta4 = math.acos(aaa)
        theta4 = 180.0 - theta4 * 180.0 / pi
        if theta4 > 135.0 or theta4 < -135.0:
            return 4

        theta3 = Alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return 7

        joints[0] = -theta6*pi/180.0
        joints[1] = -(theta5-90.0)*pi/180.0
        joints[2] = theta4*pi/180.0
        joints[3] = -theta3*pi/180.0
        return 0
                            
    def arm_pub(self):
        flaot_joints = [float(x) for x in joints]
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "time"]
        joint_state.position = flaot_joints
        self.pub_arm.publish(joint_state)

    def car_move(self, x, y, w):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(w)
        self.pub_vel.publish(twist)

    def robot_init(self):
        self.car_move(0,0,0)
        self.kinematics_move(0,175,arm_up,1500)
        self.claw_move(0, 0.8, 1000)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Pick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
