#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import time
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    FORWARD_THRESHOLD = 60.0
    FORWARD_SPEED = 1.7
    GRAB_SIZE_THRESHOLD = 265.0
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.8
    FIND_TURN_DELAY = 0.05
    STARTUP_TRACKER_DELAY = 2.0
    VEL_PUBLISH_RATE = 7.0
    SERVO_PUBLISH_RATE = 1.0
    QUEUE_SIZE = 10

    # State types
    STATE_STOP = 0
    STATE_FIND_BOTTLE = 1
    STATE_NAV_BOTTLE = 2
    STATE_PICKUP_BOTTLE = 3
    STATE_FIND_QR = 4
    STATE_NAV_QR = 5
    STATE_DROPOFF_BOTTLE = 6

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

#  Create this ROSPy node
        rospy.init_node('Navigator', anonymous=True)

        # Published topics and publish rates
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.servo1_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.servo2_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        self.tracker_flag = rospy.Publisher("/tracker_flag", Bool, queue_size = self.QUEUE_SIZE)
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_BOTTLE
        self.state_pub.publish(self.robot_state)

        print("="*50)
        print("= Initialize TrashBot")
        print("="*50)

    '''
    Stop the robot's movement
    '''
    def stop(self):
        self.set_vel(0.0, 0.0)

        while self.robot_state is self.STATE_STOP and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

    '''
    Rotate in place until a bottle is seen, then centre it in the robots camera
    '''
    def find_bottle(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.find_bottle_callback)
        self.set_vel(self.MINIUMUM_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            time.sleep(self.FIND_TURN_DELAY)
            self.vel_pub.publish(self.zero_vel)
            time.sleep(self.FIND_TURN_DELAY)
            self.vel_rate.sleep()

        self.box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        time.sleep(self.STARTUP_TRACKER_DELAY)
        bool_msg = Bool()
        bool_msg.data = True
        self.tracker_flag.publish(bool_msg)
        time.sleep(self.STARTUP_TRACKER_DELAY)
    '''
    Callback function for finding bottle whenever a new bouding box is published
    '''
    def find_bottle_callback(self, data):
        boxes = data.bounding_boxes
        box = next(iter(list(filter(lambda x : x.Class == "bottle" ,boxes))),None)
        if box != None:
            xpos = (box.xmax + box.xmin)/2 - self.IMAGE_WIDTH/2

            # Exit state when the bottle is centered
            if abs(xpos) < self.FORWARD_THRESHOLD:
                self.robot_state = self.STATE_NAV_BOTTLE

    '''
    Navigate to the first classified bottle in view until it's close enough to
    be picked up
    '''
    def navigate_bottle(self):
        self.box_sub = rospy.Subscriber('/object_tracker/bounding_box', BoundingBox, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()

    '''
    Callback function for navigating to a bottle whenever a new bounding box is published
    '''
    def navigate_bottle_callback(self, data):
        box = data

        # Determine size of bottle
        size = box.xmax - box.xmin
        print("Bottle Size = {}".format(size))
        if size > self.GRAB_SIZE_THRESHOLD:
            self.robot_state = self.STATE_PICKUP_BOTTLE

        # Determine bottle position relative to 0
        xpos = (box.xmax + box.xmin) / 2.0 - self.IMAGE_WIDTH / 2.0
        print("Bottle X Position = {}".format(xpos))

        # Go forward at constant speed
        if abs(xpos) < self.FORWARD_THRESHOLD:
            print("Robot Forward Speed = {}".format(self.FORWARD_SPEED))
            self.set_vel(0.0, self.FORWARD_SPEED)
        # Rotate in place
        else:
            turn = -xpos / (self.IMAGE_WIDTH / 2.0) * self.PROPORTIONAL
            turn = turn if abs(turn) > self.MINIUMUM_TURN else math.copysign(self.MINIUMUM_TURN, turn)
            print("Robot Turn Speed = {}".format(turn))
            self.set_vel(turn, 0.0)

    '''
    Pickup a bottle
    '''
    def pickup_bottle(self):
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.servo1_pub.publish(90)
        self.servo2_pub.publish(90)

    '''Functions to be written'''
    #def find_qr(self):
    #def find_qr_callback(self, data):
    #def navigate_qr(self):
    #def navigate_qr_callback(self, data):
    #def dropoff_bottle(self):

    '''
    Set turn velocity and forward velocity (i.e. Z Gyro and X Velocity in Twist msg)
    '''
    def set_vel(self, turn, forward):
        self.vel.angular.z = turn
        self.vel.linear.x = forward

if __name__ == '__main__':
    try:
        bot = TrashBot()

        while not rospy.is_shutdown():
            bot.state_pub.publish(bot.robot_state)

            if bot.robot_state == bot.STATE_STOP:
               bot.stop()
            elif bot.robot_state == bot.STATE_FIND_BOTTLE:
               bot.find_bottle()
            elif bot.robot_state == bot.STATE_NAV_BOTTLE:
               bot.navigate_bottle()
            elif bot.robot_state == bot.STATE_PICKUP_BOTTLE:
                bot.pickup_bottle()
            elif bot.robot_state == bot.STATE_FIND_QR:
                bot.find_qr()
            elif bot.robot_state == bot.STATE_NAV_QR:
                bot.navigate_qr()
            elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
                bot.dropoff_bottle()
            else:
                bot.stop()
        bot.stop()

    except rospy.ROSInterruptException:
        bot.stop()
