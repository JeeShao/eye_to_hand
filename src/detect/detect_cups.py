#!/usr/bin/python
# -*- coding: utf-8 -*-
# @File    : detect_cups.py
# @Time    : 18-7-18 上午9:53
# @Author  : Jee
# @Email   : jee_shao@163.com

import numpy as np
import cv2
import os

points = []

img = cv2.imread("./imgs/13.jpg",0)

ret,th1 = cv2.threshold(img,180,255,cv2.THRESH_BINARY_INV)
# th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,5,2)
cv2.imshow("th1",th1)
# OpenCV定义的结构元素
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
# # 腐蚀图像
# eroded = cv2.erode(th1, kernel)
# # 显示腐蚀后的图像
# cv2.imshow("Eroded Image", eroded);
#
# # 膨胀图像
# dilated = cv2.dilate(th1, kernel)
# # 显示膨胀后的图像
# cv2.imshow("Dilated Image", dilated);
# # 开运算
# opened = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel)
# # 显示腐蚀后的图像
# cv2.imshow("Open", opened);

# cv2.waitKey(0)

# 闭运算
#闭运算
closed = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
cv2.imshow("Close", closed);

img1 = closed.copy()
img1 = cv2.GaussianBlur(img1,(3,3),0)  #高斯平滑
# img1 = cv2.medianBlur(img1,5)  #中通滤波
img1_canny = cv2.Canny(img1,200,300,apertureSize=3)  #边缘检测
cv2.imshow("guass",img1_canny)
image,contours,hierarchy = cv2.findContours(img1_canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  #轮廓检测
color = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

for i in range(0,len(contours)):
    #矩形
    x, y, w, h = cv2.boundingRect(contours[i])
    if w>100 and h>100:
        points.append([x+w/2,y+h/2])
        cv2.rectangle(color, (x,y), (x+w,y+h), (255,0,0), 3)
    #最小外接矩形
    rect = cv2.minAreaRect(contours[i])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    # cv2.drawContours(color, [box], 0, (0, 0, 255), 2)
cv2.imshow("canny",color)
print(points)
cv2.waitKey(0)

#ORB特征匹配
orb = cv2.ORB_create()

kp1, des1 = orb.detectAndCompute(img1, None)

feature_name = '1.feature'
try:
    np.savetxt(feature_name, des1, fmt='%.5e') # 保存特征向量
    feature = np.loadtxt('1.feature') # 加载特征向量
    # cv2.imwrite(drawpath, img) # 保存绘制了SURF特征的图片
    feature = np.int0(feature)
except:
    print("ERROR")

kp2, des2 = orb.detectAndCompute(img2, None)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# print(feature.shape,des2.shape)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)
# print(matches)

#框处特征区域
img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags=2)
matches = matches[:10]
for  i in matches:
    points.append([list(kp1[i.queryIdx].pt)])
print(points)
a = np.array(points)

a = a.astype(int)
x, y, w, h = cv2.boundingRect(a)
print(x, y, w, h)
cv2.rectangle(color, (x,y), (x+w,y+h), (255,0,0), 2)

cv2.imshow("match",img3)
cv2.imshow("match2",color)
cv2.waitKey(0)


#获取特征点坐标
# vector<Dmatch>matches_surf
#
# KeyPoint_1[matches_surf[i].queryIdx].pt   =>  queryIdx为kp1的index
#
# KeyPoint_2[matches_surf[i].trainIdx].pt   =>  trainIdx为kp2的index