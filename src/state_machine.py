#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import time
import numpy as np
import copy
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,Pose
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    FORWARD_THRESHOLD = 0.18
    FORWARD_SPEED = 1.3
    GRAB_SIZE_THRESHOLD = 300.0
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    IMAGE_HALF_WIDTH = 320.0
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.5
    FIND_TURN = 0.75
    VEL_PUBLISH_RATE = 7.0
    SERVO_PUBLISH_RATE = 1.0
    QUEUE_SIZE = 10
    TURN_DELAY = 0.08
    STARTUP_TRACKER_DELAY = 2.0
    STARTUP_QR_DELAY = 1.0
    ARM_DOWN_ANGLE = 40.0
    ARM_UP_ANGLE = 20.0
    CLAW_CLOSED_ANGLE = 0.0
    CLAW_OPEN_ANGLE = 0.0

    # Different robot states
    STATE_STOP = 0
    STATE_SET_DROPOFF = 1
    STATE_FIND_BOTTLE = 2
    STATE_NAV_BOTTLE = 3
    STATE_PICKUP_BOTTLE = 4
    STATE_NAV_DROPOFF = 5
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

        # Zero velocity message
        self.zero_vel = copy.deepcopy(self.vel)

        #  Create this ROS Py node
        rospy.init_node('Navigator', anonymous=True)

        # Published topics and publish rates
        self.vel_pub = rospy.Publisher("/intermediate_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.servo1_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.servo2_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        self.tracker_flag = rospy.Publisher("/tracker_flag", Bool, queue_size = self.QUEUE_SIZE)
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_BOTTLE
        self.state_pub.publish(self.robot_state)

        # Set initial servo positions
        self.servo1_pub.publish(self.CLAW_OPEN_ANGLE)
        self.servo2_pub.publish(self.ARM_UP_ANGLE)

        print("="*50)
        print("= Initialize TrashBot")
        print("="*50)

    def euler_to_quaternion(self,roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    '''
    Stop the robot's movement
    '''
    def stop(self):
        self.set_vel(0.0, 0.0)

        while self.robot_state is self.STATE_STOP and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

    def set_dropoff(self):
        # Just set at origin for now
        self.dropoff_pose = Pose()
        self.dropoff_pose.position.x = 0.0
        self.dropoff_pose.position.y = 0.0
        self.dropoff_pose.position.z = 0.0
        qx,qy,qz,qw = self.euler_to_quaternion(0.0,0.0,0.0)
        self.dropoff_pose.orientation.x = qx
        self.dropoff_pose.orientation.y = qy
        self.dropoff_pose.orientation.z = qz
        self.dropoff_pose.orientation.w = qw

    '''
    Rotate in place until a bottle is seen, then centre it in the robots camera
    '''
    def find_bottle(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes, self.find_bottle_callback)
        self.set_vel(self.FIND_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        time.sleep(self.STARTUP_TRACKER_DELAY)
        #bool_msg = Bool()
        #bool_msg.data = True
        #self.tracker_flag.publish(bool_msg)
        #time.sleep(self.STARTUP_TRACKER_DELAY)

    '''
    Callback function for finding bottle whenever a new bouding box is published
    '''
    def find_bottle_callback(self, data):
        boxes = data.bounding_boxes
        box = next(iter(list(filter(lambda x : x.Class == "bottle", boxes))), None)

        if box != None:
            # Determine bottle position relative to 0, in range [-1, 1]
            xpos = ((box.xmax + box.xmin) / 2.0 - self.IMAGE_HALF_WIDTH) / self.IMAGE_HALF_WIDTH
            print("Bottle X Position = {}".format(xpos))

            # Exit state when the bottle is centered
            if abs(xpos) < self.FORWARD_THRESHOLD:
                self.robot_state = self.STATE_NAV_BOTTLE

    '''
    Navigate to the first classified bottle in view until it's close enough to
    be picked up
    '''
    def navigate_bottle(self):
        #self.box_sub = rospy.Subscriber('/object_tracker/bounding_box',
        #                                BoundingBox, self.navigate_bottle_callback)
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()

    '''
    Callback function for navigating to a bottle whenever a new bounding box is published
    '''
    def navigate_bottle_callback(self, data):
        boxes = data.bounding_boxes
        box = next(iter(list(filter(lambda x : x.Class == "bottle", boxes))), None)

        if box != None:
            # Navigate to first bottle seen
            # Determine size of bottle
            size = box.xmax - box.xmin + 1
            print("Bottle Size = {}".format(size))
            if size > self.GRAB_SIZE_THRESHOLD:
                self.robot_state = self.STATE_PICKUP_BOTTLE

            # Determine bottle position relative to 0, in range [-1, 1], scaled by xpos
            xpos = ((box.xmax + box.xmin) / 2.0 - self.IMAGE_HALF_WIDTH) / self.IMAGE_HALF_WIDTH * (size / self.IMAGE_HALF_WIDTH)
            print("Bottle X Position = {}".format(xpos))

            # Go forward at constant speed
            if abs(xpos) < self.FORWARD_THRESHOLD:
                #xvel = self.FORWARD_SPEED * (self.GRAB_SIZE_THRESHOLD / size) - (self.FORWARD_SPEED)
                xvel = self.FORWARD_SPEED
                print("Robot Forward Speed = {}".format(xvel))
                self.set_vel(0.0, xvel)
            # Rotate in place
            else:
                turn = -xpos * self.PROPORTIONAL
                turn = turn if abs(turn) > self.MINIUMUM_TURN else math.copysign(self.MINIUMUM_TURN, turn)
                print("Robot Turn Speed = {}".format(turn))
                self.set_vel(turn, 0.0)

    '''
    Pickup a bottle
    '''
    def pickup_bottle(self):
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.servo1_pub.publish(20)
        self.servo2_pub.publish(90)

        time.sleep(self.STARTUP_QR_DELAY)

    '''
    Rotate in place until a QR code is seen
    '''
    def navigate_dropoff(self):
        pass


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
            elif bot.robot_state == bot.STATE_SET_DROPOFF:
                bot.set_dropoff()
            elif bot.robot_state == bot.STATE_FIND_BOTTLE:
                bot.find_bottle()
            elif bot.robot_state == bot.STATE_NAV_BOTTLE:
                bot.navigate_bottle()
            elif bot.robot_state == bot.STATE_PICKUP_BOTTLE:
                bot.pickup_bottle()
            elif bot.robot_state == bot.NAGIVATE_DROPOFF:
                bot.navigate_dropoff()
            elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
                bot.dropoff_bottle()
            else:
                bot.stop()

    except rospy.ROSInterruptException:
        bot.stop()

    bot.stop()
