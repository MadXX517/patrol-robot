#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from threading import Lock
import time

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Moving arm:
   1    2    3   4   5   6
   q    w    e   r   t   y

a/z : increase/decrease max speeds by 10%
s/x : increase/decrease only linear speed by 10%
d/c : increase/decrease precision by 0.05
space key : reset
k : force stop
f : special position
anything else : stop
b : switch to OmniMode/CommonMode
precision is not less than or equal to zero
CTRL-C to quit
"""
Omni = 0  # 全向移动模式
precision = 0.03  # 默认精度(rad)

# 各关节的角度限制
joint_limits = {
    '1': (-2.33, 2.33),
    'q': (-2.33, 2.33),
    '2': (-1.57, 1.57),
    'w': (-1.57, 1.57),
    '3': (-2.33, 2.33),
    'e': (-2.33, 2.33),
    '4': (-1.57, 1.57),
    'r': (-1.57, 1.57),
    '5': (-2.33, 2.33),
    't': (-2.33, 2.33),
    '6': (-0.3, 1.1),
    'y': (-0.3, 1.1),
}

# 键值对应转动方向
rotateBindings = {
    '1': (1, 1),
    'q': (1, -1),
    '2': (2, 1),
    'w': (2, -1),
    '3': (3, 1),
    'e': (3, -1),
    '4': (4, 1),
    'r': (4, -1),
    '5': (5, 1),
    't': (5, -1),
    '6': (6, 1),
    'y': (6, -1)
}

# 键值对应精度增量
precisionBindings = {
    'd': 0.01,
    'c': -0.01
}

# 键值对应移动/转向方向
moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1)
}

# 键值对应速度增量
speedBindings = {
    'a': (1.1, 1),
    'z': (0.9, 1),
    's': (1, 1.1),
    'x': (1, 0.9)
}

current_joints = None
joints_target = None
current_joints_lock = Lock()
joints_target_lock = Lock()

def joint_states_callback(msg):
    global current_joints, joints_target
    names = msg.name
    positions = msg.position
    new_joints = [0.0] * 6
    for i in range(len(names)):
        name = names[i]
        if name == 'arm_0_joint':
            new_joints[0] = positions[i]
        elif name == 'arm_1_joint':
            new_joints[1] = positions[i]
        elif name == 'arm_2_joint':
            new_joints[2] = positions[i]
        elif name == 'arm_3_joint':
            new_joints[3] = positions[i]
        elif name == 'arm_4_joint':
            new_joints[4] = positions[i]
        elif name == 'arm_5_1_joint':
            new_joints[5] = positions[i]
    with current_joints_lock:
        current_joints = new_joints

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def prec(speed, turn, precision):
    return f"currently:\tspeed {speed}\tturn {turn}\tprecision {precision} "

def main(args=None):
    rclpy.init(args=args)
    node = Node('arm_teleop')
    
    # 创建发布者
    pub_arm = node.create_publisher(JointState, '/arm_states', 5)
    pub_vel = node.create_publisher(Twist, '/cmd_vel', 5)
    
    # 创建订阅者
    sub_joint = node.create_subscription(
        JointState,
        '/joint_states',
        joint_states_callback,
        10
    )
    
    global current_joints, joints_target
    
    # 等待直到接收到初始关节状态
    node.get_logger().info("Waiting for initial joint states...")
    while current_joints is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # 初始化目标关节状态为当前关节状态
    with current_joints_lock:
        joints_target = list(current_joints)
    
    jointState = JointState()
    jointState.header.stamp = node.get_clock().now().to_msg()
    jointState.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    with joints_target_lock:
        jointState.position = joints_target
    pub_arm.publish(jointState)
    
    precision = 0.03
    speed = 0.2
    turn = 1
    x = 0
    th = 0
    target_speed = 0
    target_turn = 0
    target_HorizonMove = 0
    control_speed = 0
    control_turn = 0
    control_HorizonMove = 0
    Omni = 0
    twist = Twist()
    
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        print(msg)
        print(prec(speed, turn, precision))
        while rclpy.ok():
            key = getKey(settings)
            
            if key == 'b':
                Omni = ~Omni
                if Omni:
                    print("Switch to OmniMode")
                    moveBindings['.'] = [-1, -1]
                    moveBindings['m'] = [-1, 1]
                else:
                    print("Switch to CommonMode")
                    moveBindings['.'] = [-1, 1]
                    moveBindings['m'] = [-1, -1]
            
            # 处理移动键 - 立即设置移动参数
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            # 处理速度调整键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(prec(speed, turn, precision))
            # 处理停止键 - 立即停止
            elif key == 'k':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                control_HorizonMove = 0
            # 处理关节控制键
            elif key in rotateBindings.keys():
                joint_index = rotateBindings[key][0] - 1
                with joints_target_lock:
                    joint_value = joints_target[joint_index] + precision * rotateBindings[key][1]
                    key_name = key
                    lower_limit, upper_limit = joint_limits[key_name]
                    if joint_value > upper_limit:
                        joint_value = upper_limit
                    elif joint_value < lower_limit:
                        joint_value = lower_limit
                    joints_target[joint_index] = joint_value
            # 处理精度调整键
            elif key in precisionBindings.keys():
                new_precision = precision + precisionBindings[key]
                if 0 < new_precision <= 0.1:
                    precision = new_precision
                print(prec(speed, turn, precision))
            # 退出程序
            elif key == '\x03':
                break
            # 重置关节位置
            elif key == ' ':
                with current_joints_lock, joints_target_lock:
                    if current_joints is not None:
                        joints_target[:] = current_joints[:]
            
            # 计算目标速度（无平滑过渡）
            target_speed = speed * x
            target_turn = turn * th
            target_HorizonMove = speed * th
            
            # 直接使用目标速度，删除平滑过渡逻辑
            control_speed = target_speed
            control_turn = target_turn
            control_HorizonMove = target_HorizonMove
            
            # 设置运动指令
            if Omni == 0:
                twist.linear.x = float(control_speed)
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = float(control_turn)
            else:
                twist.linear.x = float(control_speed)
                twist.linear.y = float(control_HorizonMove)
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
            pub_vel.publish(twist)
            
            # 发布关节状态
            jointState.header.stamp = node.get_clock().now().to_msg()
            with joints_target_lock:
                jointState.position = joints_target
            pub_arm.publish(jointState)
            
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(e)
    
    finally:
        # 停止所有运动
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub_vel.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("Keyboard control off")

if __name__ == '__main__':
    main()