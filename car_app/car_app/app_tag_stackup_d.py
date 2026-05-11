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
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from dt_apriltags import Detector
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import car_vision.Kinematics as Kinematics
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import car_vision.arm_ik_sdk as arm_ik_sdk
import transforms3d.euler as t3de
from car_vision.arm_ik_sdk import matrix_hand_to_cam
from car_vision.transform import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler, depth_pixel_to_camera, extristric_plane_shift, pixels_to_world

def draw_tags(image, tags, corners_color=(0, 125, 255), center_color=(0, 255, 0)):
    for tag in tags:
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        cv2.putText(image, f"{tag.tag_id}", (int(center[0] - (7 * len(f"{tag.tag_id}"))), int(center[1]-10)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if corners_color:
            for p in corners:
                cv2.circle(image, tuple(p.tolist()), 5, corners_color, -1)
        if center_color:
            cv2.circle(image, tuple(center.tolist()), 8, center_color, -1)
    return image
def get_rect_angle(corners):
    corners_copy = corners.copy()
    sorted_array = np.lexsort((corners_copy[:, 0], -corners_copy[:, 1]))
    bottom_pos = corners_copy[sorted_array[0]]
    
    if corners_copy[sorted_array[1]][0] < corners_copy[sorted_array[2]][0]:
        left_pos = corners_copy[sorted_array[1]]
        right_pos = corners_copy[sorted_array[2]]
    else:
        left_pos = corners_copy[sorted_array[2]]
        right_pos = corners_copy[sorted_array[1]]
        
    left_angle = math.degrees(math.atan2(abs(bottom_pos[1]-left_pos[1]), abs(bottom_pos[0]-left_pos[0])))
    right_angle = math.degrees(math.atan2(abs(bottom_pos[1]-right_pos[1]), abs(bottom_pos[0]-right_pos[0])))
    
    return left_angle, right_angle

class TagStackup(Node):
    def __init__(self, node_name='tag_stackup_d', log_level=rclpy.logging.LoggingSeverity.INFO):
        super().__init__(node_name)
        rclpy.logging.set_logger_level(node_name, log_level)
        self.debug = False
        self.tag_size = 0.0260
        self.K = None
        self.D = None
        self.moving = False
        self.count = 0
        self.endpoint = None
        self.tags_id_stack = [0, 0, 0]
        self.aim_tags_order = []
        self.start_grab = False
        self.grab_order = 0
        self.at_detector = Detector(
            searchpath=['apriltags'],
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.bridge = CvBridge()
        self.Kinematics = Kinematics.DoF5_ARM_Kinematics()
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = Pose()

        self.arm_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.arm_joint_states

        self.Arm_controller = arm_ik_sdk.ArmControl()
        # ROS 2 订阅器
        self.image_sub = None
        self.camera_info_sub = None
        self.result_publisher = self.create_publisher(Image, 'tag_stackup/image_result', 1)
        self.create_service(Trigger, 'tag_stackup/enter', self.enter_srv_callback)
        self.create_service(Trigger, 'tag_stackup/exit', self.exit_srv_callback)
        
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter")
        self.grab_order = 0
        time.sleep(3)
        self.Arm_controller.move_arm([0.15, 0.0, 0.13], 90.0)
        time.sleep(1)
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1) # 摄像头订阅(subscribe camera)
        if self.camera_info_sub is None:
            self.camera_info_sub = self.create_subscription( CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1) # 摄像头参数订阅(subscribe camerainfo)
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit")
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            if self.camera_info_sub is not None:
                self.destroy_subscription(self.camera_info_sub)
                self.camera_info_sub = None
        except Exception as e:
            self.get_logger().error(str(e))
        self.Arm_controller.set_steer([0,-0.93,2.07,1.3,0,0.8])
        time.sleep(1)
        response.success = True
        response.message = "exit"
        return response

    def joint_states_callback(self, states):
            self.joints[0] = states.position[4]
            self.joints[1] = states.position[5]
            self.joints[2] = states.position[6]
            self.joints[3] = states.position[7]
            self.joints[4] = states.position[8]
    
    def get_current_pose(self):
        position, orientation = self.Kinematics.get_forwardKinematics(self.joints, radians=True)
        self.get_logger().info(f"self.joints: {self.joints}")
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]

        self.pose.orientation.w = orientation[0]
        self.pose.orientation.x = orientation[1]
        self.pose.orientation.y = orientation[2]
        self.pose.orientation.z = orientation[3]
        return self.pose
    
    def get_endpoint(self):
        endpoint = self.get_current_pose()
        self.get_logger().info(f"Endpoint: {endpoint}")
        return xyz_quat_to_mat(
            [endpoint.position.x, endpoint.position.y, endpoint.position.z],
            [endpoint.orientation.w, endpoint.orientation.x,
             endpoint.orientation.y, endpoint.orientation.z]
        )
    def camera_info_callback(self, msg):
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
        self.K, self.D = new_K, np.zeros((5, 1))
    def pick(self, position, rect_angle):
        self.moving = True
        x, y, z = position
        z = 0.030
        yaw = math.degrees(math.atan2(y, x))
        
        if y <= 0:
            rolll = rect_angle[1]+abs(yaw) if rect_angle[0] > rect_angle[1] else -(rect_angle[0]-abs(yaw))
        else:
            rolll = -(rect_angle[0]+abs(yaw)) if rect_angle[0] <= rect_angle[1] else rect_angle[1]-abs(yaw)
        set_x, set_y = 0.03, -0.13
        
        # 执行动作序列
        actions = [
            ([x,y,z+0.01], {'pitch':70, 'roll':rolll}, 1400),
            ([x,y,z-0.005], {'pitch':70, 'roll':rolll}, 400),
            ([x,y,z-0.005], {'pitch':70, 'roll':rolll, 'hand':-0.04}, 500),
            ([x,y,z+0.07], {'pitch':70, 'hand':-0.04}, 800),
            ([set_x, set_y, 0.05+self.grab_order*0.032], {'pitch':70, 'roll':-12, 'hand':-0.04}, 1400),
            ([set_x, set_y, 0.015+self.grab_order*0.032], {'pitch':70, 'roll':-12, 'hand':-0.03}, 1000),
            ([set_x, set_y, 0.015+self.grab_order*0.032], {'pitch':70, 'roll':-12, 'hand':0.8}, 600),
            ([set_x, set_y, 0.05+self.grab_order*0.032], {'pitch':70, 'roll':-12, 'hand':0.8}, 400),
            ([0.15,0.0,0.13], {'pitch':90.0}, 1300)
        ]
        
        for pos, params, duration in actions:
            self.Arm_controller.move_arm(pos, duration=duration, **params)
            time.sleep(0.5 if duration < 1000 else 2.0)
        self.grab_order += 1
        if self.grab_order < len(self.aim_tags_order):
            self.moving = False
    def image_callback(self, ros_image: Image):
        if self.endpoint is None:
            self.endpoint = self.get_endpoint()
            
        try:
            if ros_image.encoding == 'rgb8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 3).astype(np.uint8)
            elif ros_image.encoding == 'mono8':
                rgb_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width).astype(np.uint8)
                # 如果需要RGB图像，可以将单通道图像复制到三个通道
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB)
            elif ros_image.encoding == 'yuv422_yuy2':
                yuv_image = np.array(ros_image.data).reshape(ros_image.height, ros_image.width, 2).astype(np.uint8)
                rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_YUY2)
            else:
                self.get_logger().error(f"Unsupported image encoding: {ros_image.encoding}")
                return

            result_image = np.copy(rgb_image)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            return
        if self.K is not None and not self.moving:
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            tags = self.at_detector.detect(gray_image, True, 
                                        (self.K[0][0], self.K[1][1], self.K[0][2], self.K[1][2]), 
                                        self.tag_size)
            
            if tags and not self.start_grab:
                for tag in tags:
                    if 1 <= tag.tag_id <= 3:
                        self.tags_id_stack[tag.tag_id-1] += 1
                self.count += 1
                
                if self.count == 20:
                    self.aim_tags_order = [i+1 for i, count in enumerate(self.tags_id_stack) if count >= 3]
                    self.get_logger().info(f"Tag grab order: {self.aim_tags_order}")
                    self.tags_id_stack = [0, 0, 0]
                    self.count = 0
                    self.start_grab = bool(self.aim_tags_order)
            if self.start_grab and self.grab_order < len(self.aim_tags_order):
                target_tag = next((t for t in tags if t.tag_id == self.aim_tags_order[self.grab_order]), None)
                
                if target_tag:
                    result_image = draw_tags(result_image, [target_tag], (0,0,255), (0,255,0))
                    position = [target_tag.pose_t[1], -target_tag.pose_t[0], target_tag.pose_t[2]]
                    
                    pose_end = matrix_hand_to_cam @ xyz_euler_to_mat(position, [0,0,0])
                    pose_world = self.endpoint @ pose_end
                    pose_world_T, _ = mat_to_xyz_euler(pose_world, True)
                    
                    pose_world_T[0] += 0.035
                    pose_world_T[1] = (pose_world_T[1] - 0.003) * 1.05
                    
                    threading.Thread(target=self.pick, args=(pose_world_T, get_rect_angle(target_tag.corners))).start()
                else:
                    self.get_logger().warning(f"Target tag {self.aim_tags_order[self.grab_order]} not found")
        if self.debug:
            cv2.imshow("Camera View", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        else:
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
            
def main(args=None):
    rclpy.init(args=args)
    node = TagStackup()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
