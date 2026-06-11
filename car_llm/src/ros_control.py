import rclpy,json,time,math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

import car_vision.arm_ik_sdk as arm_ik_sdk
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# 功能：解释器+执行器，解释器+消息下发
# 2025/11/19 整合car arm c2控制功能

line_speed_x=0.08
line_speed_y=0.10
ang_speed_z=0.60

class ros_control(Node):
    def __init__(self,name='ros_control_node',ros_control_queue=None,tts_queue=None):
        super().__init__(name,allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.action_queue = ros_control_queue
        self.tts_queue = tts_queue
        
        self.poseRC = []
        self.hand_close = -0.6
        self.current_joints = [0.0] * 6

        self.Arm_controller = arm_ik_sdk.ArmControl(self)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_arm = self.create_publisher(JointState, '/ik_states', 1)
        self.pickinfo_publisher = self.create_publisher(String, '/chat_model/whichone', qos_profile_sensor_data)

        self.tts_sub = self.create_subscription(String, '/tts_node/tts_text', self.tts_callback, 1)
        
        print("ROS控制器初始化完毕！")

    def tts_callback(self,msg):
        print("收到ros tts消息：", msg.data)
        if len(msg.data.strip())>0:
            self.tts_queue.put(msg.data.strip())

    def ros2_sleep(self, seconds: float):
        """精确延迟指定秒数"""
        start_time = self.get_clock().now()
        duration = Duration(seconds=seconds)
        
        # 循环等待，直到达到指定的持续时间
        while (self.get_clock().now() - start_time) < duration:
            # 允许其他回调执行
            rclpy.spin_once(self, timeout_sec=0.01)

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

    def analyze_communication(self, data):
        # 解析单个 step 命令
        try:
            # 检查是否需要JSON解析
            if isinstance(data, str):
                step_data = json.loads(data)
            else:
                step_data = data

            step = step_data.get("step", {})

            func = step.get("function", "")
            obj = step.get("object", "")
            params = step.get("parameters", {})

            print("正在执行:",step,"end")

            # c2执行动作
            if func == "导航搬运颜色物块":
                print("导航搬运颜色物块")
            elif func == "形状物块抓取并放置":
                user_command = params.get("ss", [])
                print("user_command:", user_command[0])
                self.publish_string(user_command[0])
                time.sleep(2.0)
            elif func == "导航到某位置后场景理解":
                user_command = params.get("nav_scene_track", [])
                self.publish_pickinfo(user_command)
                time.sleep(5.0)
            elif func == "物体追踪":
                user_command = params.get("ot", [])[0]
                self.publish_string(user_command)
            elif func == "小车前后平移运动":
                if "car_move" in params:
                    dx, dy = params["car_move"]
                    self.move_car(float(dx), float(dy), 0.0)
            elif func == "小车旋转运动":
                if "car_turn" in params:
                    avz = params["car_turn"][0]
                    avz = math.radians(avz)
                    self.move_car(0.0, 0.0, float(avz))
            elif func == "单独设置关节（舵机）角度":
                steer = params.get("steer_set", [])
                if len(steer) == 2:
                    servo_id, angle = steer
                    self.set_servo(servo_id,angle)
            elif func == "从A位置搬运到B位置":
                move_path = params.get("AtoB", [])
                if len(move_path) == 2:
                    # pass
                    self.move_AtoB(move_path[0], move_path[1])
            elif func == "固定动作":
                action = params.get("Routine", [])[0]

                if action == "夹爪开":
                    self.set_graper(0.6)
                elif action == "夹爪关":
                    self.set_graper(-0.3)
                elif action == "恢复初始状态":
                    self.reset_arm()
                elif action == "比个耶":
                    self.action_yeah()
                elif action == "摇摇头":
                    self.action_wave()
                elif action == "点点头":
                    self.action_nod()
            elif func == "依次归类颜色":
                classino = params.get("Color_in_seq", [])
                self.publish_pickinfo(classino)
            elif func == "依次归类垃圾":
                classino = params.get("Garbage_in_seq", [])
                self.publish_pickinfo(classino)
            else:
                self.get_logger().warn(f"⚠️ 未知指令：{func} - {obj}")

        except Exception as e:
            print("[analyze_communication] JSON解析或执行出错：", str(e))
            # pass

    def move_car(self, dx, dy, avz):
        twist = Twist()
        print(dx, dy, avz)
        # 计算持续时间
        duration = 0.0
        if abs(dx) != 0.0:
            twist.linear.x = line_speed_x if dx > 0 else -line_speed_x
            duration = abs(dx / line_speed_x)
        elif abs(dy) != 0.0:
            twist.linear.y = line_speed_y if dy > 0 else -line_speed_y
            duration = abs(dy / line_speed_y)
        elif abs(avz) != 0.0:
            twist.angular.z = ang_speed_z if avz > 0 else -ang_speed_z
            duration = abs(avz / ang_speed_z)

        # 发布速度
        if duration > 0:
            self.get_logger().info(f"🚗 执行动作，持续 {duration:.2f} 秒")
            start = time.time()
            self.pub_vel.publish(twist)
            while time.time() - start < duration:
                time.sleep(0.1)
            self.pub_vel.publish(Twist())  # 停止
            self.get_logger().info("✅ 动作完成")

    def set_servo(self, id, angle):
        angle=math.radians(angle)
        # 确保id在有效范围内 (1-6)
        if id < 0 or id > 5:
            self.get_logger().error(f"无效的关节编号: {id}，必须在0-5之间")
            return
        
        # 获取当前关节状态
        try:
            self.current_joints = self.Arm_controller.get_joints()
        except Exception as e:
            self.get_logger().error(f"获取关节状态失败: {str(e)}")
            return
        
        # 创建新的关节状态消息
        joint = JointState()
        joint.header.stamp = self.get_clock().now().to_msg()
        joint.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
        
        # 使用当前关节状态，仅修改指定id的关节角度
        joint.position = [float(pos) for pos in self.current_joints]
        joint.position[id] = float(angle)

        # 发布关节状态
        self.pub_arm.publish(joint)
        self.get_logger().info(f"🌀 设置关节 {id} 角度为 {angle}°")
        
        # 等待执行完成
        self.ros2_sleep(2.0)

    def set_graper(self, angle):
        # 获取当前关节状态
        try:
            self.current_joints = self.Arm_controller.get_joints()
        except Exception as e:
            self.get_logger().error(f"获取关节状态失败: {str(e)}")
            return

        joint = JointState()
        joint.header.stamp = self.get_clock().now().to_msg()
        joint.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
        joint.position = [self.current_joints[0],self.current_joints[1],self.current_joints[2],self.current_joints[3],self.current_joints[4],angle]
        self.pub_arm.publish(joint)
        self.get_logger().info(f"🌀 设置夹爪角度为 {angle}°")
        self.ros2_sleep(2.0)

    def reset_arm(self):
        self.Arm_controller.move_arm([0.12, 0.0, 0.16],pitch=70.0, duration=1500)
        self.ros2_sleep(3.0)

    def action_yeah(self):
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)
        self.ros2_sleep(1.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,0.3,0.0,0.0],duration=1500)
        self.ros2_sleep(1.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.0],duration=1500)
        self.ros2_sleep(1.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,-0.4],duration=700)
        self.ros2_sleep(0.7)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.4],duration=700)
        self.ros2_sleep(0.7)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,-0.4],duration=700)
        self.ros2_sleep(0.7)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.4],duration=1000)
        self.ros2_sleep(2.0)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)
        self.ros2_sleep(2.0)

    def action_wave(self):
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)
        self.ros2_sleep(0.5)
        for i in range(2):
            self.Arm_controller.set_steer([-0.4,-0.3,0.6,1.2,0.0,0.0],duration=500)
            self.ros2_sleep(0.5)
            self.Arm_controller.set_steer([0.4,-0.3,0.6,1.2,0.0,0.0],duration=500)
            self.ros2_sleep(0.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)

    def action_nod(self):
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)
        self.ros2_sleep(1.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.8,0.0,0.0],duration=500)
        self.ros2_sleep(0.5)
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)  
        self.ros2_sleep(0.5) 
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.8,0.0,0.0],duration=500)
        self.ros2_sleep(0.5) 
        self.Arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)
        self.ros2_sleep(2.0)

    def publish_string(self,name):
        msg=String()
        msg.data = name 
        self.pickinfo_publisher.publish(msg)

    def publish_pickinfo(self,name):
        msg=String()
        msg.data = ",".join(name) # name
        self.pickinfo_publisher.publish(msg)

    def loop(self):
        if self.action_queue.qsize()>0:
            action = self.action_queue.get()
            self.analyze_communication(action)
        self.ros2_sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ros_control()
    try:
        while rclpy.ok():
            node.loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
