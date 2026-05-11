# coding=utf8
# auther=danche 2024/10/11
# 
# ROS2无服务版 2025/11/18，调用方法：
# import car_vision.arm_ik_sdk as arm_ik_sdk # 导入模块
# 
# self.Arm_controller = arm_ik_sdk.ArmControl(self) # 输入self
# time.sleep(2.0)
# self.Arm_controller.move_arm([0.14,0.0,0.08],70.0) # 跟ROS1版一样调用
# time.sleep(2.0)

import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import car_vision.Kinematics as Kinematics
from rclpy.qos import qos_profile_sensor_data

bias_correction = 0  # 1.7 # 修正舵机物理偏移，由于舵机安装不一定正，造成的摄像头画面倾斜可以修改此处
bias_correction_4 = 0 # -0.05
matrix_hand_to_cam = [
    [1.0, 0.0, 0.0, -0.055],  # x=-0.101  -0.00224 相机镜头 前后
    [0.0, 1.0, 0.0, 0.011],   # y= 0.011    0.011          左右               固定值
    [0.0, 0.0, 1.0, -0.00224],  # z= 0.045    0.055       高度
    [0.0, 0.0, 0.0, 1.0]
]

class ArmControl:
    def __init__(self,parent_node):
        self.parent = parent_node
        self.get_logger = parent_node.get_logger

        self.arm_states_pub = parent_node.create_publisher(JointState, '/ik_states', 2)
        self.last_time = time.time()

        self.arm_joint_states = parent_node.create_subscription(JointState,'/joint_states', self.joint_states_callback,qos_profile_sensor_data)

        self.Kinematics = Kinematics.DoF5_ARM_Kinematics()
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = Pose()
        self.last_pos = [0.0, 0.0, 0.0]
        self.last_ans = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_pitch = 0.0

    def joint_states_callback(self, states):
        if time.time() - self.last_time > 1.0:
            # print("states",states)
            self.joints[0] = states.position[4] - math.radians(bias_correction)
            self.joints[1] = states.position[5]
            self.joints[2] = states.position[6]
            self.joints[3] = states.position[7]
            self.joints[4] = states.position[8]
            # print("self.joints",self.joints)
            self.last_time = time.time()

    def get_current_pose(self):
        print("self.joints",self.joints)
        position, orientation = self.Kinematics.get_forwardKinematics(self.joints, radians=True)
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]

        self.pose.orientation.w = orientation[0]
        self.pose.orientation.x = orientation[1]
        self.pose.orientation.y = orientation[2]
        self.pose.orientation.z = orientation[3]
        return self.pose

    def get_joints(self,joints):
        return joints

    def move_arm(self, pos, pitch=90.0, roll=0.0, hand=0.80, duration=1000, service_mode=False):
        self.get_logger().info("[Arm control]arm Planning...")
        
        if np.array_equal(self.last_pos, pos) and self.last_pitch == pitch:
            self.get_logger().info("[Arm control]ik repeat or hand grab")
            self.last_ans[4] = math.radians(roll) + bias_correction_4
            self.last_ans[5] = hand
            self.set_steer(self.last_ans, duration)
            return 0
        else:
            pos[2] = pos[2] - 0.1034  # 以第一个舵机转盘下方的底座为原点
            ans, finally_pitch = self.Kinematics.get_inverseKinematics(pos, pitch)
            if ans is not None:
                ans[0] = ans[0] + math.radians(bias_correction)
                ans[4] = math.radians(roll) + bias_correction_4
                ans[5] = hand
                self.get_logger().info(f"[Arm control]ik get: {ans}")
                self.set_steer(ans, duration)

                time.sleep(duration / 1000.0)
                self.last_pos = pos
                self.last_ans = ans
                self.last_pitch = pitch
                return 0
            else:
                self.get_logger().error("[Arm control]arm found no plan")
                return 1

    def set_steer(self, angles, duration=1000):
        joint_states = JointState()
        joint_states.header.stamp = self.parent.get_clock().now().to_msg()
        joint_states.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
        joint_states.position = [
            float(angles[0]),
            float(angles[1]),
            float(angles[2]),
            float(angles[3]),
            float(angles[4]),
            float(angles[5]),
            float(duration)
        ]
        self.arm_states_pub.publish(joint_states)

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmControl()
    
    try:
        # rclpy.spin_once(arm_controller)
        # arm_controller.get_logger().info("Testing current pose...")
        # time.sleep(3)
        # ans = arm_controller.get_current_pose()
        # arm_controller.get_logger().info(str(ans))
        
        # 示例移动指令
        # arm_controller.move_arm([0.12, 0.0, 0.16], 70.0)
        """
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)
        time.sleep(0.5)
        for i in range(2):
            arm_controller.set_steer([-0.3,-0.3,0.6,1.2,0.0,0.0],duration=500)
            time.sleep(0.5)
            arm_controller.set_steer([0.3,-0.3,0.6,1.2,0.0,0.0],duration=500)
            time.sleep(0.5)
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)
        """

        """
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)
        time.sleep(1.5)
        arm_controller.set_steer([0.0,-0.3,0.6,0.3,0.0,0.0],duration=1500)
        time.sleep(1.5)
        arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.0],duration=1500)
        time.sleep(1.5)
        arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,-0.4],duration=700)
        time.sleep(0.7)
        arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.4],duration=700)
        time.sleep(0.7)
        arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,-0.4],duration=700)
        time.sleep(0.7)
        arm_controller.set_steer([0.0,-0.3,0.6,-0.3,0.0,0.4],duration=1000)
        time.sleep(2.0)
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)      
        """
        """
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=1500)
        time.sleep(1.5)
        arm_controller.set_steer([0.0,-0.3,0.6,1.8,0.0,0.0],duration=500)
        time.sleep(0.5)
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)  
        time.sleep(0.5) 
        arm_controller.set_steer([0.0,-0.3,0.6,1.8,0.0,0.0],duration=500)
        time.sleep(0.5) 
        arm_controller.set_steer([0.0,-0.3,0.6,1.2,0.0,0.0],duration=500)        
        """



        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info("Shutting down...")
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
