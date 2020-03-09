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
from std_srvs.msg import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes

from bottle_queue import BottleQueue

print("Environment Ready")

class BottleFinder:
    def __init__(self):
        # Class constants
        self.GRAB_SIZE_THRESHOLD = 250.0
        self.IMAGE_HEIGHT = 480.0
        self.IMAGE_WIDTH = 640.0
        self.HFOV = 69.4 # degrees
        self.CAMERA_HEIGHT = 0.5 # height in meters from ground
        self.CAMERA_ANGLE = 30.0 # angle with respect to the plane paralell to the ground 
                            # (where angle below the plane is positive)
        self.SERVO_PUBLISH_RATE = 1.0
        self.QUEUE_SIZE = 10

        # Bottle Queue Setup
        self.UNIQUE_BOTTLE_RADIUS = 1.0
        self.bqueue = BottleQueue(self.UNIQUE_BOTTLE_RADIUS)

        # 

        # Suscribed Topics
        self.pop_trigger_sub = rospy.Subscriber('bottle_queue/pop', Trigger)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes)

        self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub,self.depth_image_sub,self.box_sub], 10)
        self.ts.registerCallback(self.bottle_callback)
        self.listener = tf.TransformListener()
    
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
