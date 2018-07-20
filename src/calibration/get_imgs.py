# -*- coding: utf-8 -*-
# @File    : get_imgs.py
# @Time    : 18-7-9 下午10:33
# @Author  : Jee
# @Email   : jee_shao@163.com
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def get_imgs(data):
    # show a frame
    global i, bridge
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    # print(cv_img.shape)
    cv2.imshow("frame", cv_img)
    if cv2.waitKey(1) & 0xFF == 0x20:
        cv2.imwrite("imgs/%d.jpg"%i, cv_img)
        print(i)
        i += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        exit()

def kinect2_display():
    global i, bridge
    i = 40
    rospy.init_node("kinect2_color", anonymous=True)
    # make a video_object and init the video object
    bridge = CvBridge()

    rospy.Subscriber('/kinect2/qhd/image_color', Image, get_imgs)
    rospy.spin()


if __name__ == '__main__':
    kinect2_display()