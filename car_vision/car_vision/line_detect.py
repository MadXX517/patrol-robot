#!/usr/bin/env python3
# encoding: utf-8

import sys,cv2,math
from scipy.ndimage import gaussian_filter
import numpy as np
from car_vision import common

picture_set_shape=[640,460]
debug_mode=True
error_line=275
last_error=0
last_angle=0
def get_area_max_contour(contours, threshold=100):
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None

def get_error(data,mid,range):
    start_index = max(0, mid - range)  # 防止下标为负
    end_index = min(len(data), mid + range)  # 防止下标超出数组长度
    new_data = data[start_index:end_index]
    error=np.mean(new_data)-picture_set_shape[0]/2-1
    return error


def get_saidaobianyuan(frame,img):
    global last_error
    data_gap=5
    msum_num=0
    msum=0
    row_len=picture_set_shape[0]
    line_mid=np.zeros(picture_set_shape[1])
    line_mid_flag=np.zeros(picture_set_shape[1],dtype=bool)
    line_num=0
    for k in range(420,picture_set_shape[1],5):
        row_index = k
        row_data = frame[row_index, :]
        left_pos=row_len-1
        left_exist=False
        right_pos=0
        right_exist=False
        for i in range(int(row_len/2),row_len,data_gap):
            if row_data[i] != row_data[i-data_gap]:
                j=i+data_gap
                if j>=row_len:break
                while row_data[j] == row_data[j-data_gap]:
                    j=j+data_gap
                    if j>=row_len:break
                if j-i>data_gap:
                    left_pos=i
                    left_exist=True
                    break

        for i in range(int(row_len/2),0,-data_gap):
            if row_data[i] != row_data[i+data_gap]:
                j=i-data_gap
                if j<=0:break
                while row_data[j] == row_data[j+data_gap]:
                    j=j-data_gap
                    if j<=0:break
                if i-j>data_gap:
                    right_pos=i
                    right_exist=True
                    break
        line_num+=1
        line_mid[row_index]=int((left_pos+right_pos)//2)
        if debug_mode==True:
            cv2.circle(img, (left_pos,row_index), 2, (255, 255, 125), -1)
            cv2.circle(img, (right_pos,row_index), 2, (125, 255, 255), -1)
            cv2.circle(img, (int(line_mid[row_index]),row_index), 1, (0, 0, 120), -1)
    
    line_mid_average=int(sum(line_mid)/line_num) # 这个数值根据间隔改变
    # if debug_mode==True: print("line_mid_average",line_mid_average)

    line_last=line_mid_average
    for k in range(419,210-1,-5):
        row_index = k
        row_data = frame[row_index, :]
        line_now=line_last

        left_pos=row_len-1
        left_exist=False
        right_pos=0
        right_exist=False
        for i in range(line_now,row_len,data_gap):
            if row_data[i] != row_data[i-data_gap]:
                j=i+data_gap
                if j>=row_len:break
                while row_data[j] == row_data[j-data_gap]:
                    j=j+data_gap
                    if j>=row_len:break
                if j-i>data_gap:
                    left_pos=i
                    left_exist=True
                    i=j
                    break

        for i in range(line_now,0,-data_gap):
            if row_data[i] != row_data[i+data_gap]:
                j=i-data_gap
                if j<=0:break
                while row_data[j] == row_data[j+data_gap]:
                    j=j-data_gap
                    if j<=0:break
                if i-j>data_gap:
                    right_pos=i
                    right_exist=True
                    i=j
                    break

        if left_exist==True:
            if debug_mode==True: cv2.circle(img, (left_pos,row_index), 2, (255, 255, 125), -1)
            line_mid_flag[row_index]=True
        if right_exist==True:
            if debug_mode==True: cv2.circle(img, (right_pos,row_index), 2, (125, 255, 255), -1)
            line_mid_flag[row_index]=True

        # if left_exist==True and right_exist==True:
        #     line_mid_flag[row_index]=True

        temp=int((left_pos+right_pos)//2)
        if right_exist or left_exist:
            if abs(line_last-temp) > 10000:
                line_last=line_last
            else:
                line_last=temp
        else:
            line_last=line_last
            
        line_mid[row_index]=line_last
        if debug_mode==True: cv2.circle(img, (int(temp),row_index), 1, (0, 0, 120), -1)
        # line_mid[row_index]=int((left_pos+right_pos)//2)

    std_line_mid=[]
    for p in range(len(line_mid)):
        if line_mid_flag[p]==True:
            msum+=line_mid[p]
            msum_num+=1
            std_line_mid.append(line_mid[p])
    
    if msum_num>5:
        mean = msum/msum_num #np.mean(line_mid)
        cv2.putText(img, "mean:"+str(int(mean)), (10,90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        std = np.std(std_line_mid)
        threshold = 2
        outliers = np.abs(line_mid - mean) > threshold * std
        line_mid[outliers] = mean
        # window_size = 5
        # moving_average = np.convolve(line_mid, np.ones(window_size)/window_size, mode='valid')
        gaussian_filtered = gaussian_filter(line_mid, sigma=25)
        for k in range(picture_set_shape[1]-1,0,-1):
            cv2.circle(img, (int(gaussian_filtered[k]),k), 2, (0, 0, 255), -1)

        if debug_mode==True:
            cv2.line(img, (0,error_line), (picture_set_shape[0]-1,error_line),(255, 122, 0), thickness=2)
            cv2.line(img, (int(gaussian_filtered[error_line]),error_line-20), (int(gaussian_filtered[error_line]),error_line+20),(255, 122, 0), thickness=2)
            cv2.circle(img, (int(gaussian_filtered[error_line]),error_line), 10, (255, 122, 0), -1)

        error=get_error(gaussian_filtered,error_line,60)
        if error > 40:
            error=40
        elif error < -40:
            error=-40
        last_error=error

    else: # error还是原来的error
        error=last_error
    return error

def get_saidao(mask_gray,result_image,rois):
    global last_angle
    centroid_sum = 0
    max_center_x = -999
    center_x = []
    for roi in rois:
        blob = mask_gray[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi
        contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓
        max_contour_area = get_area_max_contour(contours, 50)  # 获取最大面积对应轮廓
        if max_contour_area is not None:
            rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形
            box = np.intp(cv2.boxPoints(rect))  # 四个角
            for j in range(4):
                box[j, 1] = box[j, 1] + roi[0]
            cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # 画出四个点组成的矩形
            # 获取矩形对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            # 线的中心点
            # line_center_x, line_center_y = pt1_x , pt1_y
            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
            cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # 画出中心点
            center_x.append(line_center_x)
        else:
            center_x.append(-999)

    weight_sum=0
    for i in range(len(center_x)):
        if center_x[i] != -999:
            if center_x[i] > max_center_x:
                max_center_x = center_x[i]
            centroid_sum += center_x[i] * rois[i][-1]
            weight_sum+=rois[i][-1]

    if weight_sum!=0:
        center_pos = centroid_sum / weight_sum  # 按比重计算中心点
        angle = math.degrees(-math.atan((center_pos - (picture_set_shape[0]/2.0)) / (picture_set_shape[1]/2.0)))
        last_angle=angle
    else:
        angle=last_angle

    return angle

if __name__ == '__main__':
    print("1")
