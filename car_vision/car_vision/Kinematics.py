# coding=utf8
# 6dof(5dof)机械臂运动学解算程序
# auther=单车 2024/10/21
'''
2024/11/8 优化逆运动学多次找解所需时间
2025/1/6  修复多次正运动学解算结果跳动问题
2025/6/6  修复x=0时，无法解算的问题
'''
import math,time
import numpy as np
import transforms3d as tfs

ARM_LINK2=1220 # 0.1220*10000 放大10000倍，提高运算精度
ARM_LINK3=1220.5
ARM_LINK4=1550
stero_limit=[-2.335,2.335] # 限位

# DH参数和关节角度
dh_params = {
    'alpha': [0, -90, 0, 0, 90, 0], # 绕x(i)的角度
    'a': [0, 0, 0.122, 0.12205, 0, 0],  # X(i)轴上的长度
    'd': [0.0475, 0.0559, 0, 0, 0, 0.0685], # Z(i-1)轴上的长度
    'theta': [0, 0, -90, 0, 90, 0]  # 绕z(i-1)的角度 示例角度，单位为度
}
dh_params_new = {
    'alpha': [0, -90, 0, 0, 90, 0], # 绕x(i)的角度
    'a': [0, 0, 0.122, 0.12205, 0, 0],  # X(i)轴上的长度
    'd': [0.0475, 0.0559, 0, 0, 0, 0.0685], # Z(i-1)轴上的长度
    'theta': [0, 0, -90, 0, 90, 0]  # 绕z(i-1)的角度 示例角度，单位为度
}

class DoF5_ARM_Kinematics:
    '''
    正运动学和逆运动学解算
    '''
    # 定义DH矩阵函数
    def dh_matrix(self,a, alpha, d, theta):
        alpha = np.radians(alpha)
        theta = np.radians(theta)
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    # 计算齐次变换矩阵
    def forward_kinematics(self,dh_params):
        """
        代数法基于DH参数求解正运动学
        """
        T = np.eye(4)  # 初始化为单位矩阵
        for i in range(1, len(dh_params['theta']) + 1):
            T_i = self.dh_matrix(dh_params['a'][i-1], dh_params['alpha'][i-1], dh_params['d'][i-1], dh_params['theta'][i-1])
            T = np.dot(T, T_i)  # 更新总变换矩阵
        return T

    def get_forwardKinematics(self,thetas,radians=False):
        if radians==True:
            dh_params_new['theta'][1]=dh_params['theta'][1]-math.degrees(thetas[0])
            dh_params_new['theta'][2]=dh_params['theta'][2]+math.degrees(thetas[1])
            dh_params_new['theta'][3]=dh_params['theta'][3]+math.degrees(thetas[2])
            dh_params_new['theta'][4]=dh_params['theta'][4]+math.degrees(thetas[3])
            dh_params_new['theta'][5]=dh_params['theta'][5]+math.degrees(thetas[4])
        elif radians==False:
            dh_params_new['theta'][1]=dh_params['theta'][1]-thetas[0]
            dh_params_new['theta'][2]=dh_params['theta'][2]+thetas[1]
            dh_params_new['theta'][3]=dh_params['theta'][3]+thetas[2]
            dh_params_new['theta'][4]=dh_params['theta'][4]+thetas[3]
            dh_params_new['theta'][5]=dh_params['theta'][5]+thetas[4]       
        # 计算正向运动学
        T = self.forward_kinematics(dh_params_new)
        # 提取位置和姿态
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        orientation=tfs.quaternions.mat2quat(rotation_matrix)
        return position, orientation

    def inverseKinematics(self, toolPosi, pitch, thetas):
        """
        几何法求解逆运动学
        """
        # 关节弧度
        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        theta4 = 0.0
        wristPosi = {}  # 腕关节坐标

        # 判断腕关节的原点是否在机械臂坐标系的Z轴上
        if toolPosi[0] == 0 and toolPosi[1] == 0:
            print("机械臂在z轴上，无须解算")
        else:
            # 求解theta1
            theta1 = math.atan2(toolPosi[1], toolPosi[0])
            thetas['theta1'] = math.degrees(theta1)
            # 判断theta1是否超过限位
            if theta1 < stero_limit[0] or theta1 > stero_limit[1]:
                return 1

        # 俯仰角， 角度转弧度
        pitch_rad = math.radians(pitch)
        # 计算腕关节的位置
        wristPosi['x'] = toolPosi[0] - ARM_LINK4 * math.cos(pitch_rad) * math.cos(theta1)
        wristPosi['y'] = toolPosi[1] - ARM_LINK4 * math.cos(pitch_rad) * math.sin(theta1)
        wristPosi['z'] = toolPosi[2] + ARM_LINK4 * math.sin(pitch_rad)
        # 计算theta3
        b = 0
        if abs(math.cos(theta1)) >= 0.00001:
            b = wristPosi['x'] / math.cos(theta1)
        else:
            b = wristPosi['y'] / math.sin(theta1)
        cos_theta3 = (pow(wristPosi['z'], 2) + pow(b, 2) - pow(ARM_LINK2, 2) - pow(ARM_LINK3, 2)) / (2 * ARM_LINK2 * ARM_LINK3)
        if cos_theta3 < -1 or cos_theta3 > 1:
            # print("arm cant reach",cos_theta3)
            return 2

        sin_theta3 = math.sqrt(1 - pow(cos_theta3, 2))
        theta3 = math.atan2(sin_theta3, cos_theta3)
        thetas['theta3'] = math.degrees(theta3)
        if theta3 < stero_limit[0] or theta3 > stero_limit[1]:
            return 1

        # 计算theta2
        k1 = ARM_LINK2 + ARM_LINK3 * math.cos(theta3)
        k2 = ARM_LINK3 * math.sin(theta3)
        r = math.sqrt(pow(k1, 2) + pow(k2, 2))
        theta2 = math.atan2(-wristPosi['z'] / r, b / r) - math.atan2(k2 / r, k1 / r)
        thetas['theta2'] = math.degrees(theta2)
        if theta2 < stero_limit[0] or theta2 > stero_limit[1]:
            return 1

        # 计算theta4
        theta4 = pitch_rad - (theta2 + theta3)
        thetas['theta4'] = math.degrees(theta4)
        if theta4 < stero_limit[0] or theta4 > stero_limit[1]:
            return 1

        # 成功完成求解
        theta2=theta2+1.570795
        thetas['theta2']=math.degrees(theta2)
        thetas['theta1']=theta1
        thetas['theta2']=theta2
        thetas['theta3']=theta3
        thetas['theta4']=theta4
        return 0
    
    def get_inverseKinematics(self, toolPosi, pitch):
        toolPosi[0] = toolPosi[0] * 10000
        toolPosi[1] = -toolPosi[1] * 10000
        toolPosi[2] = toolPosi[2] * 10000
        # pitch范围检查
        if pitch < -90 or pitch > 90:
            print("input Pitch out of range,please set in [-90,90]")
            return None,None
        # 根据抓手距离机械臂基坐标系的直线距离，检查是否超出机械臂最长范围
        disO2Tool = math.sqrt(pow(toolPosi[0], 2) + pow(toolPosi[1], 2) + pow(toolPosi[2], 2))
        if disO2Tool > (ARM_LINK2 + ARM_LINK3 + ARM_LINK4 + 1000000):
            print("input xyz out of range")
            return None,None
        
        pitch_down_lookup=0
        pitch_down_flag=False
        pitch_up_lookup=0
        pitch_up_flag=False

        thetas={'theta1':0,'theta2':0,'theta3':0,'theta4':0}
        stero_thetas={}
        pitch=int(pitch)
        finally_pitch=pitch

        for i_up in range(pitch+1,90,1):
            if self.inverseKinematics(toolPosi,i_up,thetas) == 0:
                pitch_up_lookup=i_up
                pitch_up_flag=True
                thetas_up=thetas
                break

        for i_down in range(pitch,-90,-1):
            if self.inverseKinematics(toolPosi,i_down,thetas) == 0:
                pitch_down_lookup=i_down
                pitch_down_flag=True
                thetas_down=thetas
                break

        if pitch_down_flag == True and pitch_up_flag == True:
            if abs(i_down - pitch) >= abs(i_up - pitch):
                thetas=thetas_up
                finally_pitch=pitch_up_lookup
            else:
                thetas=thetas_down
                finally_pitch=pitch_down_lookup
        elif pitch_down_flag == False and pitch_up_flag == False:
            return None,None
        elif pitch_down_flag == True:
            thetas=thetas_down
            finally_pitch=pitch_down_lookup
        elif pitch_up_flag == True:
            thetas=thetas_up
            finally_pitch=pitch_up_lookup
        print("finally_pitch",finally_pitch)
        stero_thetas=[thetas['theta1'],thetas['theta2'],thetas['theta3'],thetas['theta4'],0.0,0.80]
        return stero_thetas,finally_pitch

mtoolPosi = [0.24,0.04,0.05]
mpitch=30
mthetas = {}

if __name__ == '__main__':
    Kinematics=DoF5_ARM_Kinematics()
    nowtime=time.time()
    ans=Kinematics.get_inverseKinematics(mtoolPosi,mpitch)
    print("gap:",time.time()-nowtime)
    if ans != None:
        print(ans)

    anw=Kinematics.get_forwardKinematics([25,-2,79,48,0])
    print(anw)

