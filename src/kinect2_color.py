# coding=utf-8
# !/usr/bin/env python
'''
 订阅Kinect2的RGB图形发布节点 显示图像
'''

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    # print(data)
    global count, bridge
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    print(cv_img.shape)
    cv2.imshow("frame", cv_img)
    cv2.waitKey(3)
    return cv_img

def kinect2_display():
    rospy.init_node("kinect2_color", anonymous=True)
    # make a video_object and init the video object
    global count, bridge
    bridge = CvBridge()
    rospy.Subscriber('/kinect2/hd/image_color', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    kinect2_display()
