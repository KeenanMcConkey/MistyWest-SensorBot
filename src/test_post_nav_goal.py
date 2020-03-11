#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import time
import tf
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import copy
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes

'''
Convert Euler rotation to Quaternion
'''
def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

if __name__=="__main__":

    rospy.init_node('test_post_nav_goal', anonymous=True)

    goal_simple_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)

    l = """{header: 
seq: 0
stamp: 
    secs: 1583892632
    nsecs: 339109681
frame_id: "map"
pose: 
position: 
    x: -0.00680875778198
    y: -0.061126768589
    z: 0.0
orientation: 
    x: 0.0
    y: 0.0
    z: 0.983146331622
    w: 0.182820378018}"""


    # Set at dropoff for now
    dropoff_pose = PoseStamped()
    dropoff_pose.header.seq = 0
    dropoff_pose.header.stamp = rospy.Time.now()
    dropoff_pose.header.frame_id = "map"
    dropoff_pose.pose.position.x = 0.0
    dropoff_pose.pose.position.y = 0.0
    dropoff_pose.pose.position.z = 0.0
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)
    dropoff_pose.pose.orientation.x = qx
    dropoff_pose.pose.orientation.y = qy
    dropoff_pose.pose.orientation.z = qz
    dropoff_pose.pose.orientation.w = qw

    goal_simple_pub.publish(dropoff_pose)
    print("Published Goal, ID = ", dropoff_pose.header.frame_id)


