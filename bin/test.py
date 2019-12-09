#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist,PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes

"""
Navigator class for trash bot
"""
class TrashBot:
   

    def __init__(self):
        # Initialize this node
        rospy.init_node('Navigator', anonymous=True)

        self.tracker_sub = rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,self.callback)

    def callback(self,data):
        print("xpos = {}".format(data.pose.position.x))
        print("zpos = {}".format(data.pose.position.z))
        print("xpos/zpos = {}".format(min(max(-1.0,data.pose.position.x*2/data.pose.position.z),1.0)))

        

   

if __name__ == '__main__':
    try:
        bot = TrashBot()
        while(True):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
