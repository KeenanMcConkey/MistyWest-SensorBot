#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import time
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist

"""
Navigator class for trash bot
"""
class motor_intermediate:
    VEL_PUBLISH_RATE = 7.0
    MINIUMUM_TURN = 1.0
    QUEUE_SIZE = 10
    
    def __init__(self):
        # Velocity message, sent to /cmd_vel at VEL_PUBLISH_RATE
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.zero_vel = Twist()
        self.zero_vel.linear.x = 0.0
        self.zero_vel.linear.y = 0.0
        self.zero_vel.linear.z = 0.0
        self.zero_vel.angular.x = 0.0
        self.zero_vel.angular.y = 0.0
        self.zero_vel.angular.z = 0.0
        self.minvel = Twist()
        self.minvel.linear.x = 0.0
        self.minvel.linear.y = 0.0
        self.minvel.linear.z = 0.0
        self.minvel.angular.x = 0.0
        self.minvel.angular.y = 0.0
        self.minvel.angular.z = 0.0
        

        #  Create this ROSPy node
        rospy.init_node('motor_intermediate', anonymous=True)

        # Published topics and publish rates
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.vel_sub = rospy.Subscriber("/intermediate_vel", Twist, self.set_vel)
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        
    
    def write_vel(self):
        while not rospy.is_shutdown():
            if self.vel.angular.z == 0.0 or abs(self.vel.angular.z) >= self.MINIUMUM_TURN: 
                self.vel_pub.publish(self.vel)
                self.vel_rate.sleep()
            else:
                self.minvel.angular.z = cmp(self.vel.angular.z,0)*self.MINIUMUM_TURN
                self.vel_pub.publish(self.minvel)
                time.sleep(abs(self.vel.angular.z)/(self.MINIUMUM_TURN)/self.VEL_PUBLISH_RATE)
                self.vel_pub.publish(self.zero_vel)
                time.sleep((1.0-abs(self.vel.angular.z)/(self.MINIUMUM_TURN))/self.VEL_PUBLISH_RATE)
                self.vel_rate.sleep()
    '''
    Callback function to set turn velocity and forward velocity (i.e. Z Gyro and X Velocity in Twist msg)
    '''
    def set_vel(self, data):
        self.vel.angular.z = data.angular.z
        self.vel.linear.x = data.linear.x

if __name__ == '__main__':
    try:
        controller = motor_intermediate()
        
        while not rospy.is_shutdown():
            controller.write_vel()

    except rospy.ROSInterruptException:
        controller.set_vel(0.0, 0.0)
        controller.vel_pub.publish(self.zero_vel)
