#!/usr/bin/python
# -*- coding: utf-8 -*-
# @File    : test.py
# @Time    : 18-7-20 下午4:48
# @Author  : Jee
# @Email   : jee_shao@163.com

import cv2
import numpy as np

img = cv2.imread('./calibration/imgs/40.jpg')
board = img[100:416, 210:511]
cv2.imshow("board", board)
cv2.waitKey(0)