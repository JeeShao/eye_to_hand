# -*- coding: utf-8 -*-
# @File    : detector.py
# @Time    : 18-7-11 下午9:00
# @Author  : Jee
# @Email   : jee_shao@163.com
# @url     : https://github.com/PacktPublishing/ROS-Robotics-By-Example/blob/master/Chapter_9_code/crazyflie_autonomous/scripts/detect_target.py
'''
检测与定位
'''
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,PointCloud2
from eye_to_hand.srv import GetPosition
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
import time, os, sys

class Detector():
    def __init__(self):
        rospy.init_node('detector', anonymous=True)
        rospy.Service('detector', GetPosition, self.start)
        self.points = [[12,33],[36,55]]  #像素坐标点集
        self.target_x = 0  #像素坐标
        self.target_y = 0
        self.martix = np.random.rand(4,4) #np.zeros((4,4))  #4×4转换矩阵(相机=>工件)
        self.camera_position = np.array([0,0,0,1],dtype=np.float32) #1×4相机坐标系下坐标(m)
        self.workpiece_position = np.array([0,0,0,1],dtype=np.float32)  #1×4工件坐标系下坐标(mm)
        self.final_position = np.zeros((1,3),dtype=np.float32) #最终坐标 (mm)
        self.res = []  #最终三维坐标点集
        self.target_found = False # 1=>检测到目标

        self.bridge = CvBridge()
        self.rate = rospy.Rate(1.0) #发布频率

    '''
    目标检测
    :param data: 2D Image(ros格式)
    :return: [x,y] => 目标像素点位置
    '''
    def detect(self,image):
        if self.target_found:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            # image = cv2.imread('13.jpg')
            # cv2.imshow("image",image)
            # cv2.waitKey(1000)
            cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)
        print(11111111)
        ###############

        self.points = []
        img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        ret, th1 = cv2.threshold(img, 180, 255, cv2.THRESH_BINARY_INV)
        # th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
        th2 = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 2)
        # cv2.imshow("th1", th1)
        # cv2.imwrite('th1.jpg',th1)
        # OpenCV定义的结构元素
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        closed = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow("Close", closed);

        img1 = closed.copy()
        img1 = cv2.GaussianBlur(img1, (3, 3), 0)  # 高斯平滑
        # img1 = cv2.medianBlur(img1,5)  #中通滤波
        img1_canny = cv2.Canny(img1, 200, 300, apertureSize=3)  # 边缘检测
        # cv2.imwrite("guass.jpg", img1_canny)

        # cv2.imshow("guass", img1_canny)
        image, contours, hierarchy = cv2.findContours(img1_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 轮廓检测
        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        for i in range(0, len(contours)):
            # 矩形
            x, y, w, h = cv2.boundingRect(contours[i])
            if w > 100 and h > 100:
                self.points.append([x + w / 2, y + h / 2])
                cv2.rectangle(color, (x, y), (x + w, y + h), (255, 0, 0), 3)
            # 最小外接矩形
            rect = cv2.minAreaRect(contours[i])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # cv2.drawContours(color, [box], 0, (0, 0, 255), 2)
        cv2.imwrite("res.jpg", color)
        print(self.points)
        ################

        #若检测到目标
        if len(self.points):
            self.target_found = True
    '''
    坐标变换 （像素坐标(x,y) => 相机坐标系下3D坐标(X,Y,Z)）
    ####1.获取到点云图像后立马传给detect(),在detect()中处坐标变换
    ####2.获取到点云图像后等待detect()传目标像素点(x,y),直到拿到(x,y)后再进行变换
    :param data: PointCloud2 
    :return: [X,Y,Z]
    '''
    def transform(self,point_cloud):
        print(22222222)
        self.res = []
        if self.target_found:#已检测到目标
            # rospy.loginfo('Data acquiring')
            # pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
            for i in self.points:
                # pc = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True,uvs=[[self.target_x, self.target_y]])
                pc = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True,uvs=[i])
                int_data = list(pc) #[(x,y,z)]
                # while not int_data:
                #     i = [j+1 for j in i]
                #     pc = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=[i])
                #     int_data = list(pc)  # [(x,y,z)]
                if int_data:
                    print("Camera_Position:", int_data)
                    self.camera_position[:3] = list(int_data[0])
                    self.workpiece_position = 1000 * self.camera_position.dot(self.martix.T)  #(x,y,z)*A = (X,Y,Z)  (注意单位换算)
                    self.final_position = self.workpiece_position[0:3]
                    print("****Final****:",self.final_position)
                    self.res.append(self.final_position)
                else:
                    print("坐标转换失败！")
            self.target_found = False
            print("###########",self.res)
    def start(self,req):
        if req:
            a = ''
            rospy.wait_for_message('/kinect2/qhd/image_color_rect', Image)  # 等待可用相机信息

            rospy.Subscriber('/kinect2/qhd/image_color_rect', Image, self.detect)
            rospy.Subscriber('/kinect2/qhd/points', PointCloud2, self.transform)
            self.rate.sleep()  # 暂停直到下一个循环
            while not (self.res and not self.target_found):
                pass
            for l in self.res:
                b = [str(int(i)) for i in l]
                # print('-'.join(b))
                a = a + "|" + '-'.join(b)
            return a

if __name__ == '__main__':
   try:
      detector = Detector()
      rospy.spin()
   except rospy.ROSInterruptException:
      rospy.loginfo("Detector node terminated.")
