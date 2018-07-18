#!/usr/bin/python
# -*- coding: utf-8 -*-
# @File    : client.py
# @Time    : 18-7-18 下午3:40
# @Author  : Jee
# @Email   : jee_shao@163.com

import sys
import rospy
from eye_to_hand.srv import *

def get(x):
    rospy.wait_for_service('detector')
    try:
        add_two_ints = rospy.ServiceProxy('detector', GetPosition)
        resp1 = add_two_ints(x)
        # return resp1
        print(resp1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    get(1)