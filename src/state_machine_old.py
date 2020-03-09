#!/usr/bin/python2
from __future__ import print_function

import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

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
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes

from bottle_queue import BottleQueue

print("Environment Ready")

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    GRAB_SIZE_THRESHOLD = 250.0
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    HFOV = 69.4 # degrees
    CAMERA_HEIGHT = 0.5 # height in meters from ground
    CAMERA_ANGLE = 30.0 # angle with respect to the plane paralell to the ground 
                        # (where angle below the plane is positive)
    SERVO_PUBLISH_RATE = 1.0
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
        self.bridge = CvBridge()

        # Published topics and publish rates
        self.goal_simple_pub = rospy.Publisher("/move_base/goal_simple", PoseStamped, queue_size = self.QUEUE_SIZE)
        self.servo1_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.servo2_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Subscribed topics
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes)

        self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub,self.depth_image_sub,self.box_sub], 10)
        self.ts.registerCallback(self.bottle_callback)
        self.listener = tf.TransformListener()


        # Custom queue structure for storing bottle pose information
        DIFFERENT_BOTTLE_THRESHOLD_RADIUS = 1.0
        self.bqueue = BottleQueue(DIFFERENT_BOTTLE_THRESHOLD_RADIUS)

        # Initially search for a bottle
        self.robot_state = self.STATE_FIND_BOTTLE
        self.state_pub.publish(self.robot_state)

        # Set initial servo positions
        self.servo1_pub.publish(90)
        self.servo2_pub.publish(50)

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
    Callback function for finding bottle whenever a new bouding box is published
    '''
    def bottle_callback(self,rgb,depth,bboxes):
        try:
            time = rospy.Time(0)
            (trans1,rot1) = self.listener.lookupTransform('/map', '/odom', time)
            (trans2,rot2) = self.listener.lookupTransform('/odom', '/camera_link', time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        boxes = bboxes.bounding_boxes
        bottles = list(filter(lambda x : x.Class == "bottle" ,boxes))
        if len(bottles) != 0:
            for box in bottles:
                try:
                    depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
                    color_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
                except CvBridgeError as e:
                    print(e)
                image_postion = tuple([(-1)((box.xmax + box.xmin) / 2.0 - self.IMAGE_WIDTH / 2.0) 
                                                                 / (self.IMAGE_WIDTH / 2.0),
                                   (-1)((box.ymax + box.ymin) / 2.0 - self.IMAGE_WIDTH / 2.0) 
                                                                 / (self.IMAGE_WIDTH / 2.0)])
                
                # Convert the depth image to a Numpy array since most cv2 functions
                # require Numpy arrays.
                depth = np.array(depth_image, dtype = np.dtype('f8'))
                color = np.array(color_image, dtype = np.dtype('f8'))
                
                # Get distance to bottle
                height, width = self.IMAGE_HEIGHT,self.IMAGE_WIDTH
                expected = 300
                scale = height / expected
                xmin_depth = int((box.xmin * expected + crop_start) * scale)
                ymin_depth = int((box.ymin * expected) * scale)
                xmax_depth = int((box.xmax * expected + crop_start) * scale)
                ymax_depth = int((box.ymax * expected) * scale)

                depth = depth[xmin_depth:xmax_depth,ymin_depth:ymax_depth].astype(float)

                # Get data scale from the device and convert to meters
                depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
                depth = depth * depth_scale

                distance,_,_,_ = cv2.mean(depth)
                proj_distance = distance*math.cos(self.CAMERA_ANGLE)

                theta = xpos*(self.HFOV/2)
                x = trans1[0] + trans2[0] + math.sin() + math.cos()
                y = 0.0
                
                yaw = 0.0

                self.bqueue.push(x,y,yaw)

    '''
    Stop the robot's movement
    '''
    def stop(self):
        pass

    '''
    Randomly navigate around room until a bottle is found (i.e. until the bottle queue
    is not empty anymore)
    '''
    def find_bottle(self):
        # If no bottles are in the queue set a random navigation waypoint
        while self.bqueue.isEmpty():
            try:
                time = rospy.Time(0)
                (trans1,rot1) = self.listener.lookupTransform('/map', '/odom', time)
                (trans2,rot2) = self.listener.lookupTransform('/odom', '/camera_link', time)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rospy.spin()
            pass

    '''
    Navigate to the first classified bottle in view until it's close enough to
    be picked up
    '''
    def navigate_bottle(self):
        bottle_pose = self.bqueue.pop()

        # Set navigation goal
        goal_pose = PoseStamped()

        goal_pose.header.stamp = rospy.Time()

        goal_pose.pose.position.x = bottle_pose[0]
        goal_pose.pose.position.y = bottle_pose[1]
        goal_pose.pose.position.y = 0.0

        x,y,z,w = self.euler_to_quaternion(0.0,0.0,bottle_pose[2])
        goal_pose.pose.orientation.x = x
        goal_pose.pose.orientation.x = y
        goal_pose.pose.orientation.x = z
        goal_pose.pose.orientation.x = w

        self.goal_simple_pub.publish(goal_pose)
           

    '''
    Pickup a bottle
    '''
    def pickup_bottle(self):

        self.servo1_pub.publish(20)
        self.servo2_pub.publish(90)


    '''
    Drop-off bottle
    '''
    def dropoff_bottle(self):
        pass


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
            elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
                bot.dropoff_bottle()
            else:
                bot.stop()

    except rospy.ROSInterruptException:
        bot.stop()

    bot.stop()
