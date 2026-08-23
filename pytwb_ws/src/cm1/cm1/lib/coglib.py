#!/usr/bin/env python3
# coding: utf-8

import cv2
import numpy as np
from cv2 import aruco

def make_marker(n):
    # Size and offset value
    size = 150
    offset = 10
    x_offset = y_offset = int(offset) // 2

    # get dictionary and generate image
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    ar_img = aruco.generateImageMarker(dictionary, n, size)

    # make white image
    img = np.zeros((size + offset, size + offset), dtype=np.uint8)
    img += 255

    # overlap image
    img[y_offset:y_offset + ar_img.shape[0], x_offset:x_offset + ar_img.shape[1]] = ar_img

    cv2.imwrite(f"marker_{n}.png", img)


def read_marker(input_img):
    # get dicionary and get parameters
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    try:
        corners, ids, rejectedCandidates = aruco.detectMarkers(input_img, dictionary, parameters=parameters)
        print(ids)
        return ids
    except:
        return None



def read_multiple_marker(infile:str, outfile: str):
    # get dicionary and get parameters
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    input_img = cv2.imread(infile)

    # detect and draw marker's information
    corners, ids, rejectedCandidates = aruco.detectMarkers(input_img, dictionary, parameters=parameters)
    print(ids)
    ar_image = aruco.drawDetectedMarkers(input_img, corners, ids)

    cv2.imwrite(outfile, ar_image)

