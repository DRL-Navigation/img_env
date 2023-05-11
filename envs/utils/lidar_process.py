import torch.nn as nn
import torch
import cv2
import math
import numpy as np


def _trans_lidar_log_map(lasers):
    # print(max(lasers))
    view_image1 = np.array(lidar_log_map(down_sample(lasers, 20), 48))
    view_image = map_generate(view_image1)
    view_image = view_image.astype('float16')
    # imageio.imsave("{}.png".format(self.index), view_image)
    # print(view_image.shape, flush=True)
    # self.index += 1
    return view_image


def down_sample(lidar, sample_num, is_circle=0):
    # print("length: ", len(lidar))
    if is_circle:
        lidar = lidar[288:-288]
    down_lidar = []
    n = len(lidar) // sample_num
    for i in range(n):
        min_ld = min(lidar[i * sample_num:(i + 1) * sample_num])
        if min_ld > 6.0:
            min_ld = 6.0
        down_lidar.append(min_ld)
    # print(len(down_lidar))
    return down_lidar


def normal_sample(lidar, sample_num):
    rq_lidar = []
    n = len(lidar) // sample_num
    for i in range(n):
        rq_lidar.append(lidar[i * sample_num])
    return rq_lidar


def lidar_to_map(lidar, length):
    ld_map = []
    gap = 6.0 / length
    for ld in lidar:
        temp = []
        left = 0
        right = left + gap
        for i in range(length):
            if right <= ld:
                temp.append(1.)
            elif left <= ld < right:
                temp.append(0.)
            elif left > ld:
                temp.append(0.5)
            left = right
            right += gap
        ld_map.append(temp)
    return ld_map  # size len(lidar) * length


def math_fun(x):
    y = math.e ** x - 1
    return y


def lidar_log_map(lidar, length):
    max_dis = max(6.0, max(lidar))
    ld_map = []
    gap = math.log(max_dis+1) / length
    for ld in lidar:
        temp = []
        left = 0
        right = left + gap
        for i in range(length):
            e_left = math_fun(left)
            e_right = min(math_fun(right), 6)
            if e_right <= ld:
                temp.append(0.)
            elif e_left <= ld < e_right:
                temp.append(1.)
            elif e_left > ld:
                temp.append(0.5)
            else:
                temp.append(0.5)
            left = right
            right += gap
        ld_map.append(temp)
    return ld_map  # size len(lidar) * length


def lidar_e_map(lidar, length):
    max_dis = 6.0
    e_map = []
    gap = (math.e ** (max_dis) - 1) / length
    for ld in lidar:
        temp = []
        left = 0
        right = left + gap
        for i in range(length):
            e_left = math.log(left+1)
            e_right = min(math.log(right+1), 6)
            if e_right <= ld:
                temp.append(0.)
            elif e_left <= ld < e_right:
                temp.append(1.)
            elif e_left > ld:
                temp.append(0.5)
            left = right
            right += gap
        e_map.append(temp)
    return e_map  # size len(lidar) * length

def map_generate(img):
    # cv2.imshow("img", img)
    # cv2.waitKey(1000)
    h, w = img.shape[:2]
    maxRadius = math.hypot(w / 2, h / 2)
    m = w / math.log(maxRadius)
    # log_polar = cv2.logPolar(img, (w / 2, h / 2), m, cv2.WARP_FILL_OUTLIERS + cv2.INTER_LINEAR)
    # linear_polar = cv2.linearPolar(img, (w / 2, h / 2), maxRadius, cv2.WARP_FILL_OUTLIERS + cv2.INTER_LINEAR)
    inverse_log = cv2.logPolar(img, (w / 2, h / 2), m * 0.99, cv2.WARP_INVERSE_MAP)
    inverse_linear = cv2.linearPolar(img, (w / 2, h / 2), maxRadius*1.1, cv2.WARP_INVERSE_MAP)
    return inverse_linear


if __name__ == '__main__':
    lidar = [1, 2, 3, 4, 5]
    length = 20
    ans = lidar_to_map(lidar, length)
