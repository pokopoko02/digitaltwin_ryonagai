#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

from converter import numpy2f32multi


def talker():
    pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        arr = np.random.rand(2, 2, 2).astype(np.float32)
        arr_msg = numpy2f32multi(arr)
#        print(arr)
        print(type(arr_msg))
        pub.publish(arr_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
