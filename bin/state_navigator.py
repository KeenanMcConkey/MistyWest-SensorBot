#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    FORWARD_THRESHOLD = 75
    IMAGE_HEIGHT = 480
    IMAGE_WIDTH = 640
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.5
    VEL_PUBLISH_RATE = 7
    SERVO_PUBLISH_RATE = 1
    QUEUE_SIZE = 10

    STATE_STOP = 0
    STATE_FIND_BOTTLE = 1
    STATE_NAV_BOTTLE = 2
    STATE_PICKUP_BOTTLE = 3
    STATE_FIND_QR = 4
    STATE_NAV_QR = 5
    STATE_DROPOFF_BOTTLE = 6

    def __init__(self):
        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_BOTTLE

        # Velocity message, sent to /cmd_vel at VEL_PUBLISH_RATE
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0

        print("="*50)
        print("= Initialize TrashBot")
        print("="*50)

        # Initialize this node
        rospy.init_node('Navigator', anonymous=True)

        # Published topics
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.servo1_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.servo2_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Subscribed topics
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.find_bottle_callback)

    def stop(self):
        self.set_vel(0.0, 0.0)

        while self.robot_state is self.STATE_STOP and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

    def find_bottle(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.find_bottle_callback)
        self.set_vel(0.0, self.MINIUMUM_TURN)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

    def find_bottle_callback(self, data):
        boxes =  list(filter(lambda x: x.Class == "bottle", data.bounding_boxes))
        if boxes:
            self.robot_state = self.STATE_NAV_BOTTLE

    def navigate_bottle(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()

    def navigate_bottle_callback(self, data):
        boxes =  list(filter(lambda x: x.Class == "bottle", data.bounding_boxes))
        if boxes:
            # Navigate to first bottle seen
            box = boxes[0]

            # Determine size of bottle
            size = box.xmax - box.xmin
            print("size {}".format(size))
            if size > 200:
                self.robot_state = self.STATE_PICKUP_BOTTLE

            # Determine bottle position relative to 0
            xpos = (box.xmax + box.xmin)/2 - self.IMAGE_WIDTH/2
            #print("See bottle in x range = {}-{}".format(box.xmin, box.xmax))
            print("xpos {}".format(xpos))

            # Go forward
            if abs(xpos) < self.FORWARD_THRESHOLD:
                self.set_vel(0.0, 2.0)
            # Rotate
            else:
                turn = -xpos / (self.IMAGE_WIDTH/2) * self.PROPORTIONAL
                turn = turn if abs(turn) > self.MINIUMUM_TURN else self.MINIUMUM_TURN
                print("turn {}".format(turn))
                self.set_vel(turn, 0.0)
        else:
            self.set_vel(0.0, 0.0)

    def pickup_bottle(self):
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.servo1_pub.publish(90)
        self.servo2_pub.publish(90)

    ### Functions to be written ###
    #def find_qr(self):
    #def find_qr_callback(self, data):
    #def navigate_qr(self):
    #def navigate_qr_callback(self, data):
    #def dropoff_bottle(self):

    # Set turn velocity and forward velocity i.e. Z Gyro and X Velocity
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

    except rospy.ROSInterruptException:
        pass
