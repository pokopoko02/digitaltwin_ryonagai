#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import math

class input_publisher():
    def __init__(self):
        a=0

    def callback_path(self,data):
        odom_x = data.pose.pose.position.x
        odom_y = data.pose.pose.position.y
        odom_a = data.pose.pose.orientation.z * (math.pi/2)

        goal_point = self.goal_point#ゴールポイントが入る rviz
        odom_point = np.array([odom_x, odom_y])#現在情報
        relative = goal_point - odom_point

        t_theta = np.angle(complex(relative[0], relative[1]))#距離までの角度情報
        if t_theta < 0:
            t_theta = math.pi + (math.pi+t_theta)
        theta = t_theta
        if odom_a < math.pi:
            if t_theta > odom_a+math.pi:
                theta=t_theta-2*math.pi
        if odom_a > math.pi:
            if 0 < t_theta < odom_a-math.pi:
                theta=t_theta+2*math.pi
        goal_angle = theta-odom_a#-piからpiへ-1から1

        self.input_data[-2] = np.linalg.norm(relative)/self.max_goal_dist#goal 距離0~1 20以上　
        self.input_data[-1] = goal_angle/math.pi#goal angle -1~1


    def input_publish(self):
        rospy.init_node('publisher_test')

        pose_pub = rospy.Publisher('/initialpose', PoseStamped, queue_size = 10)

        pose_test = PoseStamped()
        pose_test.header.frame_id = 'map'
        pose_test.pose.position.x = 10.0
        pose_test.pose.position.y = 20.0
        pose_test.pose.orientation.z = np.pi

        while not rospy.is_shutdown():
             pose_pub.publish(pose_test)
             rospy.Subscriber("/rtabmap/odom",Odometry , self.callback_odom)
             rospy.sleep(0.2)


if __name__ == '__main__':
    publisher=input_publisher()
    publisher.input_publish()
