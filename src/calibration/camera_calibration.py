# -*- coding: utf-8 -*-
# @File    : camera_calibration.py
# @Time    : 18-7-9 下午10:29
# @Author  : Jee
# @Email   : jee_shao@163.com
import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*5,3), np.float32)   #35*3
objp[:,:2] = 30*np.mgrid[0:5,0:7].T.reshape(-1,2)   # 35*3  [:,:2] => 所有行的0-1列
# print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob("./calibration/imgs/*.jpg")

# print(images)
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (5,7),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)  #3D

        corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
        imgpoints.append(corners2)  #2D

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (5,7), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(1000)


###########################################
###计算获取内参，畸变系数，外参（旋转向量+平移矩阵）
###########################################
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print("ret:",ret,"\n","mtx:",mtx,'\n',"dist:",dist,'\n',"rves:",rvecs,'\n',"tves:",tvecs)
# print(rvecs[0])

##旋转向量rvecs转换到旋转矩阵（罗德里格斯变换）
print(cv2.Rodrigues(rvecs[0])[0])
#################################
###通过已知内参和畸变系数获取新图像的外参
#################################
img = cv2.imread("./calibration/imgs/4.jpg")
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

#######################
#Undistortion
#######################
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
 # crop the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]
cv2.imwrite('calibresult.png', dst)

# undistort
# mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
# dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
# 5  # crop the image
# x, y, w, h = roi
# dst = dst[y:y + h, x:x + w]
# cv2.imwrite('calibresult.png', dst)
# cv2.destroyAllWindows()

# 反投影误差
total_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)  #投影3D点到2D点
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error
print "total error: ", total_error/len(objpoints)