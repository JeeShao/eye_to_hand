#!/usr/bin/python
# -*- coding: utf-8 -*-
# @File    : detect.py
# @Time    : 18-7-17 上午10:01
# @Author  : Jee
# @Email   : jee_shao@163.com

import numpy as np
import cv2

img = cv2.imread("imgs/1.jpg",0)
img2 = cv2.imread("imgs/6.jpg",0)
img1 = img.copy()
img1 = cv2.GaussianBlur(img1,(3,3),0)  #高斯平滑
# img1 = cv2.medianBlur(img1,5)  #中通滤波
img1_canny = cv2.Canny(img1,200,300,apertureSize=3)  #边缘检测
image,contours,hierarchy = cv2.findContours(img1_canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  #轮廓检测
color = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

for i in range(0,len(contours)):
    #矩形
    x, y, w, h = cv2.boundingRect(contours[i])
    cv2.rectangle(color, (x,y), (x+w,y+h), (153,153,0), 3)
    #最小外接矩形
    rect = cv2.minAreaRect(contours[i])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    print(box)

    cv2.drawContours(color, [box], 0, (0, 0, 255), 2)

# img3 = cv2.drawContours(color,contours,-1,(0,0,255),2)
cv2.imshow("canny",color)
cv2.imshow("canny1",img1_canny)
cv2.waitKey(0)
orb = cv2.ORB_create()

kp1, des1 = orb.detectAndCompute(img1, None)

kp2, des2 = orb.detectAndCompute(img2, None)
# print(des1)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

matches = bf.match(des1, des2)

matches = sorted(matches, key=lambda x: x.distance)
print(matches)

img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:50], None, flags=2)
cv2.imshow("match",img3)
cv2.waitKey(0)