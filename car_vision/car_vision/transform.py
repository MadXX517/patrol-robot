# coding=utf8

import math
import numpy as np
import transforms3d as tfs

def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上
    :param point: 点的坐标
    :param now: 现在图片的尺寸
    :param new: 新的图片尺寸
    :return: 新的点坐标
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)

# 以相机的坐标为参考，求出目标像素的坐标
def depth_pixel_to_camera(pixel_coords, depth, intrinsics): 
    fx, fy, cx, cy = intrinsics # 深度相机内参矩阵
    px, py = pixel_coords
    x = (px - cx) * depth / fx # 矩阵运算
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

# 坐标+欧拉角 转换成 齐次矩阵
def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat
# 坐标+四元数 转换成 齐次矩阵
def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

# 齐次矩阵 转换成 坐标+欧拉角
def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler

def xyz_rot_to_mat(xyz, rot):
    return np.row_stack((np.column_stack((rot, xyz)), np.array([[0, 0, 0, 1]])))

def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况
    :param contours: 轮廓列表
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None

def box_center(box):
    """
    计算四边形box的中心
    :param box: box （x1, y1, x2, y2)形式
    :return:  中心坐标（x, y)
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi
    :param v1: 第一个向量
    :param v2: 第二个向量
    :return: 角度
    """
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (norm_v1_v2)
    sin = np.cross(v1, v2) / (norm_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle

def extristric_plane_shift(tvec, rmat, delta_z):
    delta_t = np.array([[0], [0], [delta_z]])
    tvec_new = tvec + np.dot(rmat, delta_t)
    return tvec_new, rmat

def pixels_to_world(pixels, K, T):
    """
    像素坐标转世界坐标
    pixels 像素坐标列表
    K 相机内参 np 矩阵
    T 相机外参 np 矩阵
    """
    invK = K.I
    t, r, _, _ =  tfs.affines.decompose(np.matrix(T))
    invR = np.matrix(r).I
    R_inv_T = np.dot(invR, np.matrix(t).T)
    world_points = []
    for p in pixels:
        coords = np.float64([p[0], p[1], 1.0]).reshape(3, 1)
        cam_point = np.dot(invK, coords)
        world_point = np.dot(invR, cam_point)
        scale = R_inv_T[2][0] / world_point[2][0]
        scale_world = np.multiply(scale, world_point)
        world_point = np.array((np.asmatrix(scale_world) - np.asmatrix(R_inv_T))).reshape(-1,)
        world_points.append(world_point)
    return world_points

def distance(point_1, point_2 ):
    """
    计算两个点间欧氏距离
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离
    """
    if len(point_1) != len(point_2):
        raise ValueError("两点的维度不一致")
    return math.sqrt(sum([(point_2[i] - point_1[i]) ** 2 for i in range(len(point_1))]))

# 一些常用的角度转换

def angle_transform(angle, param):
    # param = [min_old, max_old, center_old, min_new, max_new, center_new]
    new_angle = ((angle - param[2])/(param[1] - param[0])) * (param[4] - param[3]) + param[5]

    return new_angle

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6  

def rot2rpy(R):
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    
    if not singular:
        r = math.atan2(R[2, 1], R[2, 2])
        p = math.atan2(-R[2, 0], sy)
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        p = math.atan2(-R[2, 0], sy)
        y = 0
    
    return [math.degrees(r), math.degrees(p), math.degrees(y)]

def rpy2rot(r, p, y):
    r = math.radians(r)
    p = math.radians(p)
    y = math.radians(y)
    
    cr = math.cos(r)
    sr = math.sin(r)
    cp = math.cos(p)
    sp = math.sin(p)
    cy = math.cos(y)
    sy = math.sin(y)

    R = np.array([[cp*cy, -cr*sy + cy*sp*sr, cr*cy*sp + sr*sy],
        [cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr],
        [-sp, cp*sr, cp*cr]])

    return R

def qua2rot(x, y, z, w):
    rot_matrix = np.array(
        [[1.0 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (w * y + x * z)],
        [2 * (x * y + w * z), 1.0 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1.0 - 2 * (x * x + y * y)]])

    return rot_matrix

def rot2qua(M):
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if q[0] < 0:
        q *= -1
    return [q[1], q[2], q[3], q[0]]

def rpy2qua(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    x = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) + math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    y = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) + math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    z = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) - math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    w = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) - math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
   
    return [x, y, z, w]

def qua2rpy(x, y, z, w):
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(2 * (w * y - x * z)))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)))
    
    return roll, pitch, yaw