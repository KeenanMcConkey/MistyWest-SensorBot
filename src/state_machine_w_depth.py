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

print("="*50)
print("= Environment Ready")

"""
Navigator class for trash bot
"""
class TrashBot:
    ### CLASS CONSTANTS ###

    # Camera
    GRAB_SIZE_THRESHOLD = 125.0
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    IMAGE_HALF_WIDTH = 320.0

    # Publish/subscribe
    VEL_PUBLISH_RATE = 20.0 #10.0 #7.0
    SERVO_PUBLISH_RATE = 20.0 #10.0 #7.0
    QUEUE_SIZE = 10

    # Motor control
    FORWARD_THRESHOLD = 0.18
    FORWARD_SPEED = 0.0 #0.25
    BACKWARD_SPEED = -0.25
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.0
    FIND_TURN = 0.5
    QUEUE_SIZE = 10

    # Delays
    CLAW_DELAY = 0.5
    TURN_DELAY = 0.08
    REVERSE_DELAY = 0.5
    TURNAROUND_DELAY = 1.0
    STARTUP_TRACKER_DELAY = 0.5
    STARTUP_DROPOFF_DELAY = 1.0
    SET_DROPOFF_DELAY = 1.0

    # Claw angles
    ARM_DOWN_ANGLE = 115.0
    ARM_UP_ANGLE = 100.0
    CLAW_CLOSED_ANGLE = 40.0
    CLAW_OPEN_ANGLE = 80.0

    # Robot states
    STATE_STOP = 0
    STATE_SET_DROPOFF = 1
    STATE_FIND_BOTTLE = 2
    STATE_NAV_BOTTLE = 3
    STATE_PICKUP_BOTTLE = 4
    STATE_NAV_DROPOFF = 5
    STATE_DROPOFF_BOTTLE = 6

    def __init__(self):

        #  Create this ROSPy node
        rospy.init_node('Navigator', anonymous=True)
        self.bridge = CvBridge()

        # Published topics
        self.goal_simple_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = self.QUEUE_SIZE)
        self.vel_pub = rospy.Publisher("/intermediate_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.claw_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.arm_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)

        # And publish rates
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Subscribed topics
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.dnet_box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
        self.ot_box_sub = message_filters.Subscriber('/object_tracker/bounding_box', BoundingBox)
        self.goal_reached_sub = message_filters.Subscriber('/rtabmap/goal_reached', Bool)
        self.robot_pose_sub = message_filters.Subscriber('/robot_pose', Pose)

        # Time synchronizer between depth image and object tracker box
        self.ts = message_filters.TimeSynchronizer([self.depth_image_sub, self.ot_box_sub], self.QUEUE_SIZE)

        # Initially set dropoff
        self.robot_state = self.STATE_SET_DROPOFF

        # Set initial servo positions
        self.claw_pub.publish(self.CLAW_OPEN_ANGLE)
        self.arm_pub.publish(self.ARM_DOWN_ANGLE)

        # Velocity message, sent to /cmd_vel at VEL_PUBLISH_RATE
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0

        print("= Initialize TrashBot")
        print("="*50)

    '''
    Convert Euler rotation to Quaternion
    '''
    def euler_to_quaternion(self, roll, pitch, yaw):

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
            print("Robot Stopped")
            self.vel_pub.publish(self.vel)
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

    '''
    Set bottle dropoff location
    '''
    def set_dropoff(self):

        print("Setting Dropoff Point")
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        self.state_pub.publish(self.robot_state)
        time.sleep(self.SET_DROPOFF_DELAY)

        # Set at dropoff for now
        self.dropoff_pose = PoseStamped()
        self.dropoff_pose.header.seq = 0
        self.dropoff_pose.header.stamp = rospy.Time.now()
        self.dropoff_pose.header.frame_id = "_dropoff_"
        self.dropoff_pose.pose.position.x = 0.0
        self.dropoff_pose.pose.position.y = 0.0
        self.dropoff_pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, 0.0)
        self.dropoff_pose.pose.orientation.x = float(qx)
        self.dropoff_pose.pose.orientation.y = float(qy)
        self.dropoff_pose.pose.orientation.z = float(qz)
        self.dropoff_pose.pose.orientation.w = float(qw)

        self.robot_state = self.STATE_FIND_BOTTLE

    '''
    Randomly navigate around room until a bottle is found (i.e. until the bottle queue
    is not empty anymore)
    '''
    def find_bottle(self):

        print("Finding Bottle")
        self.dnet_box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes, self.find_bottle_callback)
        self.set_vel(self.FIND_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

        self.dnet_box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        time.sleep(self.STARTUP_TRACKER_DELAY)

        # Startup the object tracker
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

        print("Navigating to Bottle")
        self.ts.registerCallback(self.navigate_bottle_callback)
        #self.ot_box_sub = rospy.Subscriber('/object_tracker/bounding_box',
        #                                BoundingBox, self.navigate_bottle_callback)
        #self.dnet_box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

        # Instead of unregister
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
        self.ts.registerCallback(self.dummy_navigate_bottle_callback)

    '''
    Callback function for navigating to a bottle whenever a new bottle pose is published
    by the object_tracker
    '''
    def navigate_bottle_callback(self, depth, box):

        #boxes = data.bounding_boxes
        #box = next(iter(list(filter(lambda x : x.Class == "bottle", boxes))), None)
        #box = data

        # Get distance to bottle from depth image
        depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        depth = np.array(depth_image, dtype = np.dtype('f8'))

        height, width = self.IMAGE_HEIGHT,self.IMAGE_WIDTH
        expected = 300.0
        scale = height / expected
        crop_start = 0.0
        xmin_depth = int((box.xmin * expected + crop_start) * scale)
        ymin_depth = int((box.ymin * expected) * scale)
        xmax_depth = int((box.xmax * expected + crop_start) * scale)
        ymax_depth = int((box.ymax * expected) * scale)

        depth = depth[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

        # Get data scale from the device and convert to meters
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale

        distance, _, _, _ = cv2.mean(depth)
        print("Bottle Distance {}".format(distance))
        if distance < self.GRAB_DIST_THRESHOLD:
            self.robot_state = self.STATE_PICKUP_BOTTLE
            return

        # Determine bottle position relative to 0, in range [-1, 1], scaled by xpos
        xpos = ((box.xmax + box.xmin) / 2.0 - self.IMAGE_HALF_WIDTH) / self.IMAGE_HALF_WIDTH * (size / self.IMAGE_HALF_WIDTH)
        print("Bottle X Position (-1 to 1) = {}".format(xpos))

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
    Dummy navigation callback, does nothing
    '''
    def dummy_navigate_bottle_callback(self, depth, box):
        # Do nothing

    '''
    Pickup a bottle
    '''
    def pickup_bottle(self):

        print("Picking Up Bottle")
        self.state_pub.publish(self.robot_state)

        self.claw_pub.publish(self.CLAW_CLOSED_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.arm_pub.publish(self.ARM_UP_ANGLE)
        time.sleep(self.CLAW_DELAY)

        time.sleep(self.STARTUP_DROPOFF_DELAY)
        self.robot_state = self.STATE_NAV_DROPOFF


    '''
    Navigate to the dropoff location
    '''
    def navigate_dropoff(self):

        self.goal_simple_pub.publish(self.dropoff_pose)
        print("Published Goal, ID = ", self.dropoff_pose.header.frame_id)
        self.goal_reached_sub = rospy.Subscriber('/rtabmap/goal_reached', Bool, self.navigate_dropoff_callback)

        while self.robot_state is self.STATE_NAV_DROPOFF and not rospy.is_shutdown():
            print("Navigating to Dropoff")
            self.state_pub.publish(self.robot_state)
            rospy.spin()

    '''
    Callback function for navigating to dropoff location, checks if goal reached
    '''
    def navigate_dropoff_callback(self, data):

        if data == True:
            self.robot_state = STATE_DROPOFF_BOTTLE


    '''
    Drop off a bottle
    '''
    def dropoff_bottle(self):

        print("Letting Go of Bottle")
        self.state_pub.publish(self.robot_state)
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.arm_pub.publish(self.ARM_DOWN_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.claw_pub.publish(self.CLAW_OPEN_ANGLE)
        time.sleep(self.CLAW_DELAY)


        print("Reversing Direction")
        curr_pose = rospy.wait_for_message(robot_pose_sub, 'Pose')
        self.set_vel(0.0, BACKWARD_SPEED)
        time.sleep(self.REVERSE_DELAY)
        qx = curr_pose.orientation.x
        qy = curr_pose.orientation.y
        qz = curr_pose.orientation.z
        qw = curr_pose.orientation.w
        curr_pose.orientation.x = -qw
        curr_pose.orientation.y = qz
        curr_pose.orientation.z = -qy
        curr_pose.orientation.w = qx
        self.goal_simple_pub.publish(self.dropoff_pose)
        time.sleep(TURNAROUND_DELAY)

        print("Finished All States: Stopping Robot")
        self.robot_state = self.STATE_STOP
        while not rospy.is_shutdown():
            self.state_pub.publish(self.robot_state)


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
            elif bot.robot_state == bot.STATE_NAV_DROPOFF:
                bot.navigate_dropoff()
            elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
                bot.dropoff_bottle()
            else:
                bot.stop()

    except Exception as e:
        print(e)
