#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
import math
import time
import copy
from std_msgs.msg import Int8, UInt16, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    FORWARD_THRESHOLD = 0.18
    FORWARD_SPEED = 1.4
    GRAB_Z_THRESHOLD = 0.004
    DROP_Z_THRESHOLD = 0.0010
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    IMAGE_HALF_WIDTH = 320.0
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.5
    FIND_TURN = 1.25
    VEL_PUBLISH_RATE = 10.0
    SERVO_PUBLISH_RATE = 1.0
    QUEUE_SIZE = 10
    CLAW_DELAY = 0.5
    STARTUP_TRACKER_DELAY = 2.0
    STARTUP_QR_DELAY = 1.0
    ARM_DOWN_ANGLE = 30.0
    ARM_UP_ANGLE = 50.0
    CLAW_CLOSED_ANGLE = 30.0
    CLAW_OPEN_ANGLE = 40.0

    # Different robot states
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

        # Zero velocity message
        self.zero_vel = copy.deepcopy(self.vel)

        #  Create this ROS Py node
        rospy.init_node('Navigator', anonymous=True)

        # Published topics and publish rates
        self.vel_pub = rospy.Publisher("/intermediate_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.claw_pub = rospy.Publisher("/claw", UInt16, queue_size = self.QUEUE_SIZE)
        self.arm_pub = rospy.Publisher("/arm", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        #self.tracker_flag = rospy.Publisher("/tracker_flag", Bool, queue_size = self.QUEUE_SIZE)
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_QR
        self.state_pub.publish(self.robot_state)

        # Set initial servo positions
        self.claw_pub.publish(self.CLAW_OPEN_ANGLE)
        self.arm_pub.publish(self.ARM_DOWN_ANGLE)

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
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes, self.find_bottle_callback)
        self.set_vel(self.FIND_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        #time.sleep(self.STARTUP_TRACKER_DELAY)
        #bool_msg = Bool()
        #bool_msg.data = True
        #self.tracker_flag.publish(bool_msg)
        #time.sleep(self.STARTUP_TRACKER_DELAY)

    '''
    Callback function for finding bottle whenever a new bounding box is published
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
        self.pose_sub = rospy.Subscriber('/object_tracker/pose',
                                        Pose, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.pose_sub.unregister()

    '''
    Callback function for navigating to a bottle whenever a new bottle pose is published
    by the object_tracker
    '''
    def navigate_bottle_callback(self, data):
        zpos = data.position.z
        print("Bottle Z Position = {}".format(zpos))

        if zpos > 0:
            xpos = min(max(-1.0, data.position.x / (data.position.z * self.IMAGE_HALF_WIDTH)), 1.0)
            print("Bottle X Position = {}".format(xpos))

            if zpos < self.GRAB_Z_THRESHOLD:
                self.robot_state = self.STATE_PICKUP_BOTTLE
            else:
                # Go forward at constant speed
                if abs(xpos) < self.FORWARD_THRESHOLD:
                    xvel = self.FORWARD_SPEED
                    print("Robot Forward Speed = {}".format(xvel))
                    self.set_vel(0.0, xvel)
                # Rotate in place
                else:
                    turn = -xpos * self.PROPORTIONAL
                    print("Robot Turn Speed = {}".format(turn))
                    self.set_vel(turn, 0.0)

    '''
    Pickup a bottle
    '''
    def pickup_bottle(self):
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        self.claw_pub.publish(self.CLAW_CLOSED_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.arm_pub.publish(self.ARM_UP_ANGLE)

        time.sleep(self.STARTUP_QR_DELAY)
        self.robot_state = self.STATE_FIND_QR

    '''
    Rotate in place until a QR code is seen
    '''
    def find_qr(self):
        self.qr_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
                                        PoseStamped, self.find_qr_callback)
        self.set_vel(self.MINIUMUM_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_QR and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.qr_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

    '''
    Callback function for finding QR code whenever a new QR pose is published
    '''
    def find_qr_callback(self, data):
        if data.pose.position.z > 0:
            #xpos = min(max(-1.0, data.pose.position.x * 2/data.pose.position.z),1.0)
            xpos = min(max(-1.0, data.position.x / (data.position.z * self.IMAGE_HALF_WIDTH)), 1.0)
            print("QR X Position = {}".format(xpos))

            # Exit state when the bottle is centered
            if abs(xpos) < self.FORWARD_THRESHOLD:
                self.robot_state = self.STATE_NAV_QR

    '''
    Navigate to the QR code until it's close enough to drop off the bottle
    '''
    def navigate_qr(self):
        self.qr_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
                                        PoseStamped, self.navigate_qr_callback)

        while self.robot_state is self.STATE_NAV_QR and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.vel_rate.sleep()

        self.qr_sub.unregister()

    '''
    Callback function for navigating to a QR codewhenever a new QR pose is published
    by visp_auto_tracker
    '''
    def navigate_qr_callback(self, data):
        zpos = data.pose.position.z
        print("QR Z Position = {}".format(zpos))

        if zpos > 0:
            xpos = min(max(-1.0, data.position.x / (data.position.z * self.IMAGE_HALF_WIDTH)), 1.0)
            print("QR X Position = {}".format(xpos))

            if zpos < self.DROP_Z_THRESHOLD:
                self.robot_state = self.STATE_DROPOFF_BOTTLE
            else:
                # Go forward at constant speed
                if abs(xpos) < self.FORWARD_THRESHOLD:
                    xvel = self.FORWARD_SPEED
                    print("Robot Forward Speed = {}".format(xvel))
                    self.set_vel(0.0, xvel)
                # Rotate in place
                else:
                    turn = -xpos * self.PROPORTIONAL
                    print("Robot Turn Speed = {}".format(turn))
                    self.set_vel(turn, 0.0)

    '''
    Drop off a bottle
    '''
    def dropoff_bottle(self):
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        self.arm_pub.publish(self.ARM_DOWN_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.claw_pub.publish(self.CLAW_OPEN_ANGLE)


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

    except rospy.ROSInterruptException:
        bot.stop()

    bot.stop()
