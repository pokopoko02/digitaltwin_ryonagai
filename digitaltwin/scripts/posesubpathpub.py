#!/usr/bin/env python
# coding: UTF-8

import queue
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import pandas as pd
import math

len_n=50

def pose_callback(msg_pose):
    global subscribed_pose
    subscribed_pose = msg_pose

if __name__ == '__main__':
    print("PUBするよ")
    rospy.init_node('path_change')
    #print("line.22")

    posearray=[]
    
    pose_sub = rospy.Subscriber('/PoseForPath', PoseStamped, pose_callback)
    #print("line.27")

    #publisher = rospy.Publisher('/move_base/TrajectoryPlannerROS/local_plan', Path, pose_callback)
    publisher = rospy.Publisher('/local_plan', Path, queue_size=10)

    #print("line.32")


    #pose_sub = rospy.Subscriber('/initialpose', PoseStamped, pose_callback)
    subscribed_pose = PoseStamped()

    path=Path()#初期化しない場合
    r = rospy.Rate(10) # 10hz
    #print("確認中だよ line.41")
    data={'Position x':[],
            'Position y':[],
            'Position z':[],
            'Orientation x':[],
            'Orientation y':[],
            'Orientation z':[],
            'Orientation w':[],
            'Frame ID':[],}
    df =pd.DataFrame(data)
    print(df)

    while not rospy.is_shutdown():
#        print(pose_sub)
#        for num in range(pose_sub):
 #           print(num)

        odom_x = subscribed_pose.pose.position.x
        odom_y = subscribed_pose.pose.position.y
        odom_z = subscribed_pose.pose.position.z

        #path=Path()#初期化する場合
        path.poses.append(subscribed_pose)
        #print(len(path.poses))
        if(len(path.poses)>=2):
            latest_odom=path.poses[-1].pose.position
            latest_orientation=path.poses[-1].pose.orientation
            pre_latest_odom=path.poses[-2].pose.position
            odom_sum=latest_odom.x+latest_odom.y+latest_odom.z
            pre_odom_sum=pre_latest_odom.x+pre_latest_odom.y+pre_latest_odom.z
            if(odom_sum!=pre_odom_sum):
                df=df.append({'Position x':latest_odom.x,
                'Position y':latest_odom.y,
                'Position z':latest_odom.z,
                'Orientation x':float(latest_orientation.x),
                'Orientation y':float(latest_orientation.y),
                'Orientation z':float(latest_orientation.z),
                'Orientation w':float(latest_orientation.w),
                'Frame ID':'map'},ignore_index=True)
            
#            print(df['Position x'])
            if(np.abs(odom_sum-pre_odom_sum)>10 or len(path.poses)>=len_n):
                rospy.loginfo(path)
                #df.to_csv('/home/moriokalab/my_workspace/src/digitaltwin/config/change_path.csv',index=False)#csv形式で出力
                df =pd.DataFrame(data)
                print("published CSV!!!")
                path.poses.clear()
 #       rospy.loginfo(path)
        publisher.publish(path)#
#        print("published!")
#        print(subscribed_pose.pose.position.x)
#        rospy.loginfo(subscribed_pose)
#        rospy.sleep(0.2)
        r.sleep()