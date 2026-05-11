import sys,cv2,math
from scipy.ndimage import gaussian_filter
import numpy as np
from car_vision import common

def _imshow_fit(title, img, max_w=1024, max_h=600):
    import cv2 as _cv2
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale < 1.0:
        img = _cv2.resize(img, (int(w * scale), int(h * scale)))
    _cv2.imshow(title, img)




def get_area_max_contour(contours, threshold=100):
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None

def line_detect(image, result_image, rois, color_range):
    centroid_sum = 0
    h, w = image.shape[:2]

    for roi in rois:
        blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi(intercept roi)
        cv2.rectangle(result_image,(int(roi[2]*w), int(roi[0]*h)),(int(roi[3]*w), int(roi[1]*h)),(120, 255, 0), 2)# 在result_image上绘制ROI的矩形框

        img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
        mask = cv2.inRange(img_blur, tuple(color_range[0]), tuple(color_range[1]))  # 二值化(image binarization)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
        # _imshow_fit('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
        max_contour_area = get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)

        if max_contour_area is not None:
            rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
            box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
            for j in range(4):
                box[j, 1] = box[j, 1] + int(roi[0]*h)
            cv2.drawContours(result_image, [box], -1, (0, 0, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

            # 获取矩形对角点(acquire the diagonal points of the rectangle)
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            # 线的中心点(center point of the line)
            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

            cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (255, 0, 0), -1)   # 画出中心点(draw the center point)
            centroid_sum += line_center_x * roi[-1]
        else:
            cv2.rectangle(result_image,(int(0.45*w), int(roi[0]*h)),(int(0.55*w), int(roi[1]*h)),(255, 255, 255), 2)

    if centroid_sum == 0:
        return result_image, None
    center_pos = centroid_sum / 1.0  # 按比重计算中心点(calculate the center point according to the ratio)
    deflection_angle = math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
    return result_image, deflection_angle



def cross_rois_area(image,result_image,rois,color_range):
    '''
    rgb_image:输入图像
    rois:四块区域
    color_range:颜色范围
    '''
    rois_area = {
        'mid_up':   0, 
        'mid_down': 0,
        'left':     0,
        'right':    0
        }

    h, w = image.shape[:2]

    for roi in rois:
        blob = image[int(rois[roi][0]*h):int(rois[roi][1]*h), int(rois[roi][2]*w):int(rois[roi][3]*w)]  # 截取roi(intercept roi)
        cv2.rectangle(result_image,(int(rois[roi][2]*w), int(rois[roi][0]*h)),(int(rois[roi][3]*w), int(rois[roi][1]*h)),(120, 0, 255), 2)# 在result_image上绘制ROI的矩形框
        img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
        mask = cv2.inRange(img_lab, tuple(color_range[0]), tuple(color_range[1]))  # 二值化(image binarization)
        area = cv2.countNonZero(mask)
        rois_area[roi]=area

    return result_image,rois_area

def color_rect_detect(source_image, result_image, color_ranges):
    h, w = source_image.shape[:2]
    
    img = cv2.resize(source_image, (int(w/2), int(h/2)))
    img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
    img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
    mask = cv2.inRange(img_lab, tuple(color_ranges[0]), tuple(color_ranges[1])) # 二值化

    # 滤波，去掉噪点，平滑边缘
    eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    # 找出最大轮廓
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # [-2]
    #max_contour_area = get_area_max_contour(contours, 10)
    min_c = None
    max_area = 0
    for c in contours:
        if math.fabs(cv2.contourArea(c)) < 500:
            continue
        (center_x, center_y), radius = cv2.minEnclosingCircle(c)
        area = cv2.contourArea(c)
        if min_c is None:
            min_c = (c, center_x)
        elif max_area < area:
            min_c = (c, center_x)

    # 如果有符合要求的轮廓
    if min_c is not None:
        (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0]) # 最小外接圆
        rect=cv2.minAreaRect(min_c[0])

        circle_color = (0x55, 0x55, 0x55)
        cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)

        center_x = center_x * 2
        center_x_1 = center_x / w
        center_y = center_y * 2
        center_y_1 = center_y / h
        return (result_image, (0, 0), (center_x, center_y), radius * 2,rect[2])
    else:
        return (result_image, None, None, 0,0)

def calculate_square_mean(image, center_x, center_y, side_length, lower_threshold=130, upper_threshold=255):
    """
    计算正方形范围内，排除过大过小值后的像素平均值
    :param image: 输入的图像
    :param center_x: 正方形中心的 x 坐标
    :param center_y: 正方形中心的 y 坐标
    :param side_length: 正方形的边长
    :param lower_threshold: 像素值的下限
    :param upper_threshold: 像素值的上限
    :return: 符合条件的像素平均值
    """
    # 计算正方形左上角的坐标
    half_side = side_length // 2
    x = center_x - half_side
    y = center_y - half_side
    # 提取正方形区域
    square = image[y:y + side_length, x:x + side_length]
    # 过滤掉过大和过小的值
    valid_pixels = square[(square >= lower_threshold) & (square <= upper_threshold)]
    if valid_pixels.size == 0:
        return 0
    # 计算平均值
    mean_value = np.mean(valid_pixels)
    return mean_value

if __name__ == '__main__':
    print("1")
