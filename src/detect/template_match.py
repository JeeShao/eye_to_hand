#!/usr/bin/python
# -*- coding: utf-8 -*-
# @File    : template_match.py
# @Time    : 18-7-18 上午11:43
# @Author  : Jee
# @Email   : jee_shao@163.com

import cv2
import numpy as np
from matplotlib import pyplot as plt
img_rgb = cv2.imread('imgs/21.jpg')
img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
template = cv2.imread('imgs/tmp.png',0)

w, h = template.shape[::-1]
res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

threshold = 0.75

loc = np.where( res >= threshold)
for pt in zip(*loc[::-1]):
    cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
cv2.imshow("tmp",img_rgb)
cv2.waitKey(0)
# cv2.imwrite('res.png',img_rgb)