#!/usr/bin/env python

import time
import matplotlib.pyplot as plt

import charuco_util
from camera import Camera

import numpy as np

camera = Camera()
time.sleep(1) # Give camera some time to load data


while True:
    color_img, depth_img = camera.get_data()

    tf = charuco_util.get_charuco_tf(color_img, 0, camera.intrinsics, np.zeros(4))

    color_img_axified = draw_axis(color_img, tf, camera.intrinsics, np.zeros(4), marker_size=0.075)

    plt.subplot(211)
    plt.imshow(color_img_axified)
    plt.subplot(212)
    plt.imshow(depth_img)
    plt.show()
