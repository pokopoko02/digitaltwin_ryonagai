#!/usr/bin/env python

import math
import sys
import threading

import rospy
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
#from carla_msgs.msg import CarlaWorldInfo
#from carla_waypoint_types.srv import GetWaypointResponse, GetWaypoint
#from carla_waypoint_types.srv import GetActorWaypointResponse, GetActorWaypoint

#import rospy
from std_msgs.msg import Float64

def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses))) 
#from agents.navigation.global_route_planner import GlobalRoutePlanner
#from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


rospy.init_node('time_pub')
#pub = rospy.Publisher('UnixTime', Float64 , queue_size=1)
pub = rospy.Publisher('test', PoseStamped , queue_size=2)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
#    now = rospy.get_time()
 #   pub.publish(now)
    publish_waypoints()
    rate.sleep()

