#!/usr/bin/env python3
# coding: utf-8
from cv2 import aruco, imread, imwrite
import numpy as np

# 引数nのIDのマーカーを作成する
def make_marker(n):
    # Size and offset value
    size = 150
    offset = 10
    x_offset = y_offset = int(offset) // 2

    # get dictionary and generate image
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    ar_img = aruco.generateImageMarker(dictionary, n, size)

    # make white image
    img = np.zeros((size + offset, size + offset), dtype=np.uint8)
    img += 255

    # overlap image
    img[y_offset:y_offset + ar_img.shape[0], x_offset:x_offset + ar_img.shape[1]] = ar_img
    imwrite(f"/root/practice_ws/images/marker_{n}.png", img)

def d_marker(img, n: int):
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # detect and draw marker's information
    corners, ids, _ = aruco.detectMarkers(img, dictionary, parameters=parameters)
    if n in np.ravel(ids) :
        index = np.where(ids == n)[0][0] #num_id が格納されているindexを抽出
        cornerUL = corners[index][0][0]
        cornerUR = corners[index][0][1]
        cornerBR = corners[index][0][2]
        cornerBL = corners[index][0][3]
        center = [ (cornerUL[0]+cornerBR[0])/2 , (cornerUL[1]+cornerBR[1])/2 ]
    return tuple(center)