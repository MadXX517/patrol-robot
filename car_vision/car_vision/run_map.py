import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)



################################################调节参数############################################################
img_size=(640,480)
#调整中点，用于调整机械臂抓取物块中点
#如果机械臂抓取偏左，mid_block_cx减小，反之增加
mid_block_cx=img_size[0]/2
mid_block_cy=img_size[1]/2

#机械臂在抓取的时候够不着需要调节 arm_skewing 的值
#如果机械臂抓取偏前， arm_skewing 减小，反之增加
arm_skewing=65

#机械臂抓取物品高度调节
#如果机械爪太高 grasp_height 减小，反之增加
grasp_height=55
arm_up=170
err_x=-0 #机械臂安装误差，偏左为负，偏右为正

##########转弯90度延时
#巡线地图的时候旋转90度延时调整
#如果没有旋转90度，需要加大延时，反之超过90度就需要减小
first_turn_delay = 50
second_turn_delay = 50
third_turn_delay = 60
fourth_turn_delay = 50
fifth_turn_delay = 60
sixth_turn_delay = 50
return_delay=100

# 色彩矩阵，当前测试出的几种色彩
red_color_arry = [[0, 100, 100],[10, 255, 255]]
green_color_arry=[[40, 30, 0],[90, 255, 255]]
blue_color_arry=[[100, 100, 70],[130, 255, 255]]
black__color_arry=[[0,0,0],[179,255,160]]

# 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
# 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
# 较小的权重分配给下一个roi，以此类推。
ROIS = [ # [ROI, weight]
        (0, 260, 640, 20, 0.25,1), # 你需要为你的应用程序调整权重
        (0, 130, 640, 20, 0.2,2), # 取决于你的机器人是如何设置的。
        (0, 000, 640, 20, 0.1,3)
    ]
#roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
#weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
#三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
#如上图的最下方的矩形，即(0, 100, 200, 20, 0.7,1)（最后一个值1是用来记录的）

#机械臂初始角度和执行时间
joints = [0.0,-0.93,2.07,1.3,0.0,0.8,1500]

"""底盘控制初始化"""
twist = Twist() #创建ROS速度话题变量
x      = 0   #前进后退方向
th     = 0   #转向/横向移动方向
count  = 0   #键值不再范围计数
target_speed = 0 #前进后退目标速度
target_turn  = 0 #转向目标速度
target_HorizonMove = 0 #横向移动目标速度
control_speed = 0 #前进后退实际控制速度
control_turn  = 0 #转向实际控制速度
control_HorizonMove = 0 #横向移动实际控制速度

time_cnt=0#时间计数

#机械臂移动位置
move_x=0
move_y=160

move_status=0#机械臂移动的方式
spin_calw=0#机械爪角度

is_line_flag=1#是否可以巡线标志
cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
crossing_flag=0#标记路口情况计数，判断是否经过一个路口
crossing_record_cnt=0#用来记录经过的路口数量
mid_adjust_position=0#小车到中间横线时需要调整身位后在寻找分拣区，变量为标志位
over_flag=0#用来标记小车180度翻转
mid_over_flag=0#记录小车翻转到一半
mid_over_cnt=0#记录小车翻转到一半计数
car_back_flag=0#小车后退还是前进标志
start_flag=0#开机标志位，用于开机初始化姿态
###################################################################################################################
class RunMap(Node):
    def __init__(self):
        super().__init__('run_map')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/usb_cam/image_raw',self.image_callback,10)
        self.result_pub = self.create_publisher(Image, '/result_image', 10)
        self.mask_pub = self.create_publisher(Image, '/mask_image', 10)
        self.pub_arm = self.create_publisher(JointState, '/ik_states', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_init()

    def image_callback(self, msg):
        global start_flag
        if start_flag == 0:
            time.sleep(2)
            start_flag = 1
        # convert ROS topic to CV image formart
        # 将将ROS主题转换为CV图像格式
        raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        raw_image = cv2.resize(raw_image, img_size, interpolation=cv2.INTER_AREA)#提高帧率
        # 将图像从 RGB 转为 HSV    
        hsv_image = cv2.cvtColor(raw_image,cv2.COLOR_BGR2HSV)
        # close operation to fit some little hole
        # 创建一个5行5列的数组
        kernel = np.ones((5,5),np.uint8)
        # 对图片进行膨胀腐蚀操作
        hsvimage_erode = cv2.erode(hsv_image,kernel,iterations=1)
        hsvimag_dilate = cv2.dilate(hsvimage_erode,kernel,iterations=1)
        # 得到处理后的二值化图像
        lower_black = np.array(black__color_arry[0])
        upper_black = np.array(black__color_arry[1])
        mask_image = cv2.inRange(hsvimag_dilate,lower_black,upper_black)
        img_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding="passthrough")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(img_msg)
        self.line_walk(raw_image,hsvimag_dilate)


    def line_walk(self,img,hsvimag_dilate):#巡线功能,
        global move_x,move_y,err_x,move_status,is_line_flag,crossing_flag,crossing_record_cnt,cap_color_status
        global time_cnt,mid_adjust_position,spin_calw,over_flag,mid_over_flag,mid_over_cnt
        global car_back_flag
        #物块中心点
        block_cx=mid_block_cx
        block_cy=mid_block_cy
        
        color_read_succed=0#是否识别到颜色
        color_status=0
        contour = 0# 色块轮廓

        #***************首先巡线找路口，计算路口位置后在识别夹取色块********************
        if is_line_flag==1: #寻线
            weight_sum = 0 #权值和初始化
            centroid_sum = 0
            centers = []
            
            #记录寻找到的线
            roi1_area = 0 #记录最大的块
            roi2_area = 0
            roi3_area = 0

            #利用颜色识别分别寻找三个矩形区域内的线段
            # 遍历每个ROI  
            for roi in ROIS:  
                x, y, w, h ,weight,roi_id= roi  
                # # 绘制轮廓外接矩形  
                # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 2)  
                # # 添加文本标签  
                # cv2.putText(img, 'red', (x - 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)    
                
                # 裁剪出ROI区域  
                roi_image = img[y:y+h, x:x+w]
                lower_black = np.array(black__color_arry[0])
                upper_black = np.array(black__color_arry[1])
                # 查找最大颜色块并获取其中心点  
                center_x, center_y ,roi_area= self.find_largest_color_blob(roi_image, lower_black, upper_black)  
                if center_x is not None and center_y is not None:  
                    # 你可以在这里发布中心点的消息或者做其他处理  
                    # print(f"Largest color blob center in ROI: ({center_x + x}, {center_y + y})")  
                    # print("id=%d  area=%d"%(roi_id,roi_area))
                    # 在原图上绘制检测到的最大颜色块的中心点  
                    cv2.circle(img, (center_x + x, center_y + y), 5, (255, 0, 0), -1)  
                    centers.append((center_x + x, center_y + y)) 

                    if roi_id==1:
                        roi1_area = roi_area
                    elif roi_id==2:
                        roi2_area = roi_area
                    elif roi_id==3:
                        roi3_area = roi_area

                    if roi_area<4000:
                        centroid_sum += center_x * weight # r[4] is the roi weight.
                        weight_sum += weight
            
            # 绘制中心点之间的连线  
            self.draw_lines_between_centers(img, centers)

            if over_flag==1: #原地翻转
                if roi3_area==0 and roi1_area==0 and mid_over_flag==0: #摄像头已经识别不到线，说明翻转到一半
                    mid_over_cnt+=1
                    if mid_over_cnt>5:
                        mid_over_flag=1
                        mid_over_cnt=0
                elif roi1_area!=0 and roi3_area!=0 and mid_over_flag==1: #重新识别到三条范围内的线，取消翻转，重新开始巡线
                    mid_over_cnt+=1
                    if mid_over_cnt>20:
                        over_flag=0
                        mid_over_flag=0
                        mid_over_cnt=0
                self.car_move(0,0,-0.8)
                return

            #**********************************判断路口情况，检测两个自定义的范围都检测到路口就说明经过一个路口***********************************************
            if mid_adjust_position!=1 and over_flag!=1: #调整身位不计数
                if crossing_flag==0 and roi2_area>4000:
                    crossing_flag=1
                elif crossing_flag==1 and roi1_area>4000: #1号ROIS检测到路口
                    crossing_flag=2
                    crossing_record_cnt+=1
                    time_cnt=0
            
            #**********************************判断路口情况***********************************************
            if crossing_flag==2:#找到路口
                if crossing_record_cnt==2 or crossing_record_cnt==5 or crossing_record_cnt==9: #经过的路口数量在物品区，小车停止，开始颜色识别
                    is_line_flag=0
                    crossing_flag=0
                    self.car_move(0,0,0)
                    return
                elif crossing_record_cnt==3: #第3个路口小车需要右转,改为颜色识别，旋转机械臂到左边
                    mid_adjust_position=1
                    self.car_move(0.1,0,-0.8)
                    time_cnt+=1
                    if time_cnt>first_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==4: #第4个路口小车需要右转
                    self.car_move(0.1,0,-0.8)
                    time_cnt+=1
                    if time_cnt>second_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==6: #第6个路口小车需要左转,改为颜色识别，旋转机械臂到右边
                    mid_adjust_position=1
                    self.car_move(0.1,0,0.8)
                    time_cnt+=1
                    if time_cnt>third_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==7: #第7个路口小车需要旋转180度
                    self.car_move(0,0,0.8)
                    time_cnt+=1
                    if time_cnt>return_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==8: #第8个路口小车需要左转
                    self.car_move(0.1,0,0.8)
                    time_cnt+=1
                    if time_cnt>fourth_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==10: #第10个路口小车需要右转,改为颜色识别，旋转机械臂到右边
                    mid_adjust_position=1
                    self.car_move(0.1,0,-0.8)
                    time_cnt+=1
                    if time_cnt>fifth_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==11: #第11个路口小车需要右转
                    self.car_move(0.1,0,-0.8)
                    time_cnt+=1
                    if time_cnt>sixth_turn_delay:
                        crossing_flag=0
                        time_cnt=0
                    return
                elif crossing_record_cnt==12: #第12个路口小车回到原点，需要原地旋转180度后倒车
                    self.car_move(0,0,0)
                    crossing_flag=1
                    car_back_flag=1
                    over_flag=1
                    return
                elif crossing_record_cnt==13: #第13个路口小车回到原点:
                    self.car_move(0,0,0)
                    is_line_flag=-1
                    return
                else:
                    crossing_flag=0
                    

            if weight_sum>0:#开始巡线

                center_pos = (centroid_sum / weight_sum) # Determine center of line.

                # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
                # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
                deflection_angle = 0
                #机器人应该转的角度

                # 640/2是X的一半，480/2是Y的一半。
                # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
                # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。

                deflection_angle = -math.atan((center_pos-640/2)/(480/2))
                #注意计算得到的是弧度值

                deflection_angle = math.degrees(deflection_angle)
                #将计算结果的弧度值转化为角度值

                # 现在你有一个角度来告诉你该如何转动机器人。
                # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。

                # print("Turn Angle: %f" % deflection_angle)
                if mid_adjust_position==1: #调整身位
                    # print(deflection_angle)
                    if abs(deflection_angle)<3: #身位调整完毕
                        time_cnt+=1
                        self.car_move(0,0,0)
                        if time_cnt>10: #计时判断,调整身位成功
                            move_x=-120 #旋转机械臂寻找颜色框
                            if crossing_record_cnt==3: #第三个路口的机械臂旋转的方向不同
                                move_x=120
                            move_y= 80
                            time_cnt=0
                            is_line_flag=0
                            car_back_flag=0
                            mid_adjust_position=0  
                        return
                    elif roi3_area>4000: #小车身位太靠前，需后退
                        time_cnt+=1
                        if time_cnt>5:
                            car_back_flag=1
                    elif roi1_area>4000: #小车倒退出线,需前进
                        car_back_flag=0
                    time_cnt=0
                car_x=0.3-abs(deflection_angle*0.01)
                car_w=deflection_angle*0.03
                car_y=0
                if car_back_flag==1: #小车需倒退
                    car_x = -car_x
                    car_w = car_w/5
                    car_y = 0

                self.car_move(car_x,car_y,car_w)
            else:
                self.car_move(0,0,0)
        
        #识别夹取色块*
        elif is_line_flag==0:
            # 定义颜色范围  
            # 注意：这些值可能需要根据你的图像进行调整  
            lower_red = np.array(red_color_arry[0])
            upper_red = np.array(red_color_arry[1])
            lower_green = np.array(green_color_arry[0])
            upper_green = np.array(green_color_arry[1])
            lower_blue = np.array(blue_color_arry[0])
            upper_blue = np.array(blue_color_arry[1])

            # 创建颜色掩膜  
            mask_red = cv2.inRange(hsvimag_dilate, lower_red, upper_red)  
            mask_green = cv2.inRange(hsvimag_dilate, lower_green, upper_green)  
            mask_blue = cv2.inRange(hsvimag_dilate, lower_blue, upper_blue)  

            # 寻找轮廓  [0:380, 0:640]这个是去掉摄像头看到的机械爪部分
            contours_red = self.find_contours_with_min_area(mask_red[0:300, 0:640], 1000)  
            contours_green = self.find_contours_with_min_area(mask_green[0:300, 0:640], 1000)  
            contours_blue = self.find_contours_with_min_area(mask_blue[0:300, 0:640], 1000)  
            
            #***************首先进行色块检测，如果没有检测到色块，那就寻线********************
            if contours_red and (cap_color_status==0 or cap_color_status=='R'):  
                # 根据轮廓面积排序  
                contours = sorted(contours_red, key=cv2.contourArea, reverse=True)  
                largest_contour = contours[0]  
                
                # 绘制轮廓外接矩形  
                x, y, w, h = cv2.boundingRect(largest_contour)  
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)  
                # 添加文本标签  
                cv2.putText(img, 'red', (x - 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)    
                # 计算轮廓中心点（质心）  
                M = cv2.moments(largest_contour)  
                if M["m00"] != 0:  
                    block_cx = int(M["m10"] / M["m00"])  
                    block_cy = int(M["m01"] / M["m00"])  
                    color_read_succed=1
                    contour=largest_contour
                    color_status='R'
                    cv2.circle(img, (block_cx, block_cy), 5, (0, 0, 255), -1)

            if contours_green and (cap_color_status==0 or cap_color_status=='G'):
                # 根据轮廓面积排序  
                contours = sorted(contours_green, key=cv2.contourArea, reverse=True)  
                largest_contour = contours[0]  
                
                # 绘制轮廓外接矩形  
                x, y, w, h = cv2.boundingRect(largest_contour)  
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  
                # 添加文本标签  
                cv2.putText(img, 'green', (x - 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)  
                # 计算轮廓中心点（质心）  
                M = cv2.moments(largest_contour)  
                if M["m00"] != 0:  
                    block_cx = int(M["m10"] / M["m00"])  
                    block_cy = int(M["m01"] / M["m00"])  
                    color_read_succed=1
                    contour=largest_contour
                    color_status='G'
                    cv2.circle(img, (block_cx, block_cy), 5, (0, 255, 0), -1)  

            if contours_blue and (cap_color_status==0 or cap_color_status=='B'):
                # 根据轮廓面积排序  
                contours = sorted(contours_blue, key=cv2.contourArea, reverse=True)  
                largest_contour = contours[0]  

                # 绘制轮廓外接矩形  
                x, y, w, h = cv2.boundingRect(largest_contour)  
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  
                # 添加文本标签  
                cv2.putText(img, 'blue', (x - 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)  
                # 计算轮廓中心点（质心）  
                M = cv2.moments(largest_contour)  
                if M["m00"] != 0:  
                    block_cx = int(M["m10"] / M["m00"])  
                    block_cy = int(M["m01"] / M["m00"])  
                    color_read_succed=1
                    contour=largest_contour
                    color_status='B'
                    cv2.circle(img, (block_cx, block_cy), 5, (255, 0, 0), -1) 

            #************************************************ 运动机械臂*************************************************************************************
            if move_status==0 and color_read_succed==1: #第0阶段：识别到颜色块机械臂寻找物块位置
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
                    if time_cnt>50: #计数100次对准物块，防止误差
                        time_cnt=0
                        move_status=1
                        cap_color_status=color_status
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
            elif move_status==1 : #第1阶段：机械臂抓取物块
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
                    move_status=2
                    move_x=err_x
                    move_y=160
                    spin_calw=0
                    self.kinematics_move(move_x,move_y,arm_up,1000)
                    self.claw_move(spin_calw,-0.3,1000)
                    is_line_flag=1  #机器人开始巡线
                    over_flag=1  #原地翻转标志
                    time_cnt=0
            elif move_status==2:
                time_cnt+=1
                if time_cnt<35:
                    self.kinematics_move(move_x,move_y,arm_up,800)
                if time_cnt>=35:    
                    move_status=21
                    time_cnt=0
            elif move_status==21:
                time_cnt+=1
                if color_read_succed==0:
                    if time_cnt<50:
                        self.car_move(0.1,0,0)
                    else:
                        self.car_move(-0.1,0,0)
                elif color_read_succed==1:
                    self.car_move(0,0,0)
                    move_status=3
                    time_cnt=0
            elif move_status==3 and color_read_succed==1: #第2阶段：调整身位
                if block_cx-mid_block_cx>50:
                    if crossing_record_cnt==3: #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        self.car_move(0.1,0,0) #前进
                    else:
                        self.car_move(-0.1,0,0) #后退
                    
                elif block_cx-mid_block_cx<-50:
                    if crossing_record_cnt==3: #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        self.car_move(-0.1,0,0) #后退
                    else:
                        self.car_move(0.1,0,0) #前进
                else: #调整完毕，停止
                    time_cnt+=1
                    if time_cnt>40:
                        self.car_move(0,0,0)
                        move_status=4
                        time_cnt=0

            elif move_status==4 and color_read_succed==1: #第3阶段：机械臂寻找放下物块的框框
                if(abs(block_cx-mid_block_cx)>10):
                    if block_cx > mid_block_cx and move_y>1:
                        if crossing_record_cnt==3: #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                            move_y+=0.3
                        else:
                            move_y-=0.3
                    else:
                        if crossing_record_cnt==3:
                            move_y-=0.3
                        else:
                            move_y+=0.3
                if(abs(block_cy-mid_block_cy)>10):
                    if block_cy > mid_block_cy:
                        if crossing_record_cnt==3:
                            move_x-=0.3
                        else:
                            move_x+=0.3
                    else:
                        if crossing_record_cnt==3:
                            move_x+=0.3
                        else:
                            move_x-=0.3
                if abs(block_cy-mid_block_cy)<=10 and abs(block_cx-mid_block_cx)<=10: #寻找到物块，机械臂进入第二阶段
                    time_cnt += 1
                    if time_cnt>10: #计数10次对准物块，防止误差
                        time_cnt=0
                        move_status=5
                        l=math.sqrt(move_x*move_x+move_y*move_y)
                        sin=move_y/l
                        cos=move_x/l
                        move_x=(l+arm_skewing)*cos #(l+x)--x调整的距离
                        move_y=(l+arm_skewing)*sin
                else:
                    time_cnt=0
                    self.kinematics_move(move_x,move_y,arm_up,0)
                
            elif move_status==5:#第4阶段：机械臂放下物块
                time_cnt += 1
                if time_cnt<35:#移动机械臂到物块上方
                    self.kinematics_move(move_x,move_y,arm_up,1000)
                elif time_cnt>=35 and time_cnt<70: #移动机械臂下移到物块
                    self.kinematics_move(move_x,move_y,grasp_height,1000)
                elif time_cnt>=70 and time_cnt<100: #机械爪放下物块
                    self.claw_move(0.0,0.8,1000)
                elif time_cnt>=135 and time_cnt<170: #移动机械臂抬起
                    self.kinematics_move(move_x,move_y,arm_up,1000)
                elif time_cnt>=200 and time_cnt<235: #机械臂归位
                    move_x=err_x #机械臂归位
                    move_y=160
                    self.kinematics_move(move_x,move_y,arm_up,1000)
                elif time_cnt>=270 and time_cnt<300: #机器人开始巡线
                    is_line_flag=1 #机器人开始巡线
                    cap_color_status=0
                    crossing_flag=1
                    move_status=0
                    time_cnt=0
        # 将实际图像和二值化图像通过话题发出
        _imshow_fit('result_image', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.result_pub.publish(img_msg)

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
    def draw_lines_between_centers(self,image, centers):  
        # 遍历中心点列表，并绘制连线  
        for i in range(len(centers) - 1):  
            center1 = centers[i]  
            center2 = centers[i + 1]  
            cv2.line(image, center1, center2, (0, 255, 0), 2)

    # 计算最大连通区域
    def find_largest_color_blob(self,roi_image, lower_color, upper_color):  
        # 创建颜色掩码  
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)  
        # close operation to fit some little hole
        # 创建一个5行5列的数组
        kernel = np.ones((5,5),np.uint8)
        # 对图片进行膨胀腐蚀操作
        hsvimage_erode = cv2.erode(hsv,kernel,iterations=1)
        hsvimag_dilate = cv2.dilate(hsvimage_erode,kernel,iterations=1)
        # 得到处理后的二值化图像
        mask = cv2.inRange(hsvimag_dilate,lower_color,upper_color)
        
        # 找到连通区域  
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
        
        # 计算每个连通区域的面积，并找到面积最大的  
        max_area = 0  
        largest_contour = None  
        for contour in contours:  
            area = cv2.contourArea(contour)  
            if area > max_area:  
                max_area = area  
                largest_contour = contour  
        
        # 计算最大连通区域的中心点  
        if largest_contour is not None:  
            M = cv2.moments(largest_contour) 
            # print(M["m00"])
            if M["m00"] > 400:  #太小说明检测到虚线之类的,略过
                cX = int(M["m10"] / M["m00"])  
                cY = int(M["m01"] / M["m00"])
                return cX, cY ,M["m00"]
        
        return None, None,None
    
    # 控制机械臂末端执行器姿态
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
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(w)
        self.pub_vel.publish(twist)

    def robot_init(self):
        self.car_move(0,0,0)
        self.kinematics_move(err_x,160,arm_up,1500)
        self.claw_move(0, 0.8, 1000)

    def shutdown_hook(self):
        self.get_logger().info("Shutting down...")
        self.robot_init()

def main(args=None):
    rclpy.init(args=args)
    node = RunMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
