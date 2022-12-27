#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

from converter import f32multi2numpy


def callback(msg):
    arr = f32multi2numpy(msg)
    rospy.loginfo(arr)
    rospy.loginfo(type(arr))
    rospy.loginfo(arr.shape)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()