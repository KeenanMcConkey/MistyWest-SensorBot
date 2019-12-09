#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import numpy as np
import time
import statistics
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
import message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Transform
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes


"""
Navigator class for trash bot
"""
class TrashBot:
    # Physical constants
    CLOSE_TO_BOTTLE = 0.0
    # Optical constants
    BOTH_FOV = 1.211259 #degrees
    DEPTH_WIDTH = 720
    DEPTH_FOV = 1.211259
    RGB_WIDTH = 640
    RGB_FOV = 1.211259
    DEPTH_TO_RGB_SCALE = 1.0

    # Class constants
    QUEUE_SIZE = 10
    
    
    # Different robot states
    STATE_STOP = 0
    STATE_FIND_BOTTLE = 1
    STATE_NAV_BOTTLE = 2
    STATE_PICKUP_BOTTLE = 3
    STATE_FIND_QR = 4
    STATE_NAV_QR = 5
    STATE_DROPOFF_BOTTLE = 6


    def __init__(self):
        #  Create this ROSPy node
        rospy.init_node('Navigator', anonymous=True)

        # Published topics and publish rates
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        self.nav_goal_pub = rospy.Publisher("/test_goal",PoseStamped,queue_size= self.QUEUE_SIZE)

        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_BOTTLE
        self.state_pub.publish(self.robot_state)

        

        print("="*50)
        print("= Initialize TrashBot")
        print("="*50)
    
    def euler_to_quaternion(self,yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return qx, qy, qz, qw

    def quaternion_to_euler(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw, pitch, roll

    '''
    Navigate randomly until a bottle is found.
    '''
    def find_bottle(self):
        self.box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes)
        self.laser_scan_sub = message_filters.Subscriber('/mybot/laser/scan',
                                        LaserScan)
        self.odom_sub = message_filters.Subscriber('/odom',
                                        Odometry)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.box_sub, self.laser_scan_sub,self.odom_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.find_bottle_callback)
        
        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            rospy.spin()


        self.box_sub.unregister()


    '''
    Callback function for finding bottle whenever a new bouding box is published
    '''
    def find_bottle_callback(self, bounding_boxes, laser_scan, odom):
        # o = odom.pose.pose.orientation
        # radians = self.quaternion_to_euler(o.x,o.y,o.z,o.w)[0]
        # degrees = (radians*180)/math.pi
        
        boxes = bounding_boxes.bounding_boxes
        box = next(iter(list(filter(lambda x : x.Class == "bottle" ,boxes))), None)
        if box != None:
            ls_min = int(math.ceil(self.DEPTH_WIDTH*(float(box.xmin)/self.RGB_WIDTH)))
            ls_max = int(math.floor(self.DEPTH_WIDTH*(float(box.xmax)/self.RGB_WIDTH)))
            ranges = list(laser_scan.ranges)
            ranges.reverse()
            min_val = ranges[ls_min]
            min_idx = ls_min
            for i in range(ls_min+1,ls_max):
                if min_val > ranges[i]:
                    min_val = ranges[i]
                    min_idx = i

            robot_pose = odom.pose.pose
            r_pos = robot_pose.position
            r_x = r_pos.x
            r_y = r_pos.y
            ro = robot_pose.orientation
            yaw, pitch, roll = self.quaternion_to_euler(ro.x,ro.y,ro.z,ro.w)
            r_angle = yaw
            distance = min_val
            l_angle = i*(float(self.BOTH_FOV)/float(self.DEPTH_WIDTH)) - self.BOTH_FOV/2.0 # laser angle
    
            goal_angle = r_angle - l_angle

            gq_x,gq_y,gq_z,gq_w = self.euler_to_quaternion(l_angle,pitch,roll)
            g_x = r_x - distance*math.cos(math.pi - goal_angle)
            g_y = r_y - distance*math.sin(math.pi - goal_angle)
            
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = g_x
            goal.pose.position.y = g_y
            goal.pose.position.z = r_pos.z

            goal.pose.orientation.x = gq_x
            goal.pose.orientation.y = gq_y
            goal.pose.orientation.z = gq_z
            goal.pose.orientation.w = gq_w

            self.nav_goal_pub.publish(goal)

            
            

    # '''
    # Navigate to the first classified bottle in view until it's close enough to
    # be picked up
    # '''
    # def navigate_bottle(self):
    #     self.box_sub = rospy.Subscriber('/object_tracker/bounding_box',
    #                                     BoundingBox, self.navigate_bottle_callback)

    #     while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
    #         self.vel_pub.publish(self.vel)
    #         self.vel_rate.sleep()

    #     self.box_sub.unregister()

    # '''
    # Callback function for navigating to a bottle whenever a new bounding box is published
    # '''
    # def navigate_bottle_callback(self, data):
    #     box = data

    #     # Determine size of bottle
    #     size = box.xmax - box.xmin
    #     print("Bottle Size = {}".format(size))
    #     if size > self.GRAB_SIZE_THRESHOLD:
    #         self.robot_state = self.STATE_PICKUP_BOTTLE

    #     # Determine bottle position relative to 0, in range [-1, 1]
    #     xpos = ((box.xmax + box.xmin) / 2.0 - self.IMAGE_WIDTH / 2.0) / (self.IMAGE_WIDTH / 2.0)
    #     print("Bottle X Position = {}".format(xpos))

    #     # Go forward at constant speed
    #     if abs(xpos) < self.FORWARD_THRESHOLD:
    #         print("Robot Forward Speed = {}".format(self.FORWARD_SPEED))
    #         self.set_vel(0.0, self.FORWARD_SPEED)
    #     # Rotate in place
    #     else:
    #         turn = -xpos * self.PROPORTIONAL
    #         turn = turn if abs(turn) > self.MINIUMUM_TURN else math.copysign(self.MINIUMUM_TURN, turn)
    #         print("Robot Turn Speed = {}".format(turn))
    #         self.set_vel(turn, 0.0)

    # '''
    # Pickup a bottle
    # '''
    # def pickup_bottle(self):
    #     self.set_vel(0.0, 0.0)
    #     self.vel_pub.publish(self.vel)

    #     self.servo1_pub.publish(20)
    #     self.servo2_pub.publish(90)

    #     time.sleep(self.STARTUP_QR_DELAY)

    # '''
    # Rotate in place until a QR code is seen
    # '''
    # def find_qr(self):
    #     self.qr_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
    #                                     PoseStamped, self.find_qr_callback)
    #     self.set_vel(self.MINIUMUM_TURN, 0.0)

    #     while self.robot_state is self.STATE_FIND_QR and not rospy.is_shutdown():
    #         self.vel_pub.publish(self.vel)
    #         time.sleep(TURN_DELAY)
    #         self.vel_pub.publish(self.zero_vel)
    #         time.sleep(TURN_DELAY)
    #         self.vel_rate.sleep()

    #     self.qr_sub.unregister()
    #     self.set_vel(0.0, 0.0)
    #     self.vel_pub.publish(self.vel)

    # def find_qr_callback(self, data):
    #     if data.pose.position.z > 0:
    #         xpos = min(max(-1.0,data.pose.position.x*2/data.pose.position.z),1.0)
    #         print("Bottle X Position = {}".format(xpos))

    #         # Exit state when the bottle is centered
    #         if abs(xpos) < self.FORWARD_THRESHOLD:
    #             self.robot_state = self.STATE_NAV_QR

    # #def navigate_qr(self):
    # #def navigate_qr_callback(self, data):
    # #def dropoff_bottle(self):

    # '''
    # Set turn velocity and forward velocity (i.e. Z Gyro and X Velocity in Twist msg)
    # '''
    # def set_vel(self, turn, forward):
    #     self.vel.angular.z = turn
    #     self.vel.linear.x = forward

if __name__ == '__main__':
    try:
        bot = TrashBot()

        while not rospy.is_shutdown():
            bot.state_pub.publish(bot.robot_state)

            if bot.robot_state == bot.STATE_FIND_BOTTLE:
               bot.find_bottle()
            # elif bot.robot_state == bot.STATE_NAV_BOTTLE:
            #    bot.navigate_bottle()
            # elif bot.robot_state == bot.STATE_PICKUP_BOTTLE:
            #     bot.pickup_bottle()
            # elif bot.robot_state == bot.STATE_FIND_QR:
            #     bot.find_qr()
            # elif bot.robot_state == bot.STATE_NAV_QR:
            #     bot.navigate_qr()
            # elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
            #     bot.dropoff_bottle()
            # else:
            #     bot.stop()

    except rospy.ROSInterruptException:
        bot.stop()

    bot.stop()
