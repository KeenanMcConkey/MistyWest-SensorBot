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
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes

print("="*50)
print("Environment Ready")

"""
Navigator class for trash bot
"""
class TrashBot:
    # Class constants
    FORWARD_THRESHOLD = 0.18
    FORWARD_SPEED = 1.3
    GRAB_SIZE_THRESHOLD = 300.0
    GRAB_DIST_THRESHOLD = 0.12 # m
    IMAGE_HEIGHT = 480.0
    IMAGE_WIDTH = 640.0
    IMAGE_HALF_WIDTH = 320.0

    VEL_PUBLISH_RATE = 7.0
    SERVO_PUBLISH_RATE = 7.0
    QUEUE_SIZE = 10

    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.5
    FIND_TURN = 1.4
    CLAW_DELAY = 0.5
    TURN_DELAY = 0.08
    STARTUP_TRACKER_DELAY = 1.0
    STARTUP_DROPOFF_DELAY = 1.0
    ARM_DOWN_ANGLE = 30.0
    ARM_UP_ANGLE = 50.0
    CLAW_CLOSED_ANGLE = 30.0
    CLAW_OPEN_ANGLE = 40.0

    # Different robot states
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
        self.goal_simple_pub = rospy.Publisher("/move_base/goal_simple", PoseStamped, queue_size = self.QUEUE_SIZE)
        self.vel_pub = rospy.Publisher("/intermediate_vel", Twist, queue_size = self.QUEUE_SIZE)
        self.claw_pub = rospy.Publisher("/servo1", UInt16, queue_size = self.QUEUE_SIZE)
        self.arm_pub = rospy.Publisher("/servo2", UInt16, queue_size = self.QUEUE_SIZE)
        self.state_pub = rospy.Publisher("/robot_state", Int8, queue_size = self.QUEUE_SIZE)
        #self.tracker_flag = rospy.Publisher("/tracker_flag", Bool, queue_size = self.QUEUE_SIZE)

        # And publish rates
        self.vel_rate = rospy.Rate(self.VEL_PUBLISH_RATE)
        self.servo_rate = rospy.Rate(self.SERVO_PUBLISH_RATE)

        # Subscribed topics
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.box_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes)
        self.goal_reached_sub = message_filters.Subscriber('/rtabmap/goal_reached', Bool)

        # Time synchronizer
        self.ts = message_filters.TimeSynchronizer([self.depth_image_sub, self.box_sub], self.QUEUE_SIZE)
        self.ts.registerCallback(self.navigate_bottle_callback)

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

        # Zero velocity message
        self.zero_vel = copy.deepcopy(self.vel)

        print("= Done Initializing TrashBot")
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
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

    def set_dropoff(self):
        self.state_pub.publish(self.robot_state)
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        # Just set at origin for now
        self.dropoff_pose = Pose()
        self.dropoff_pose.position.x = 0.0
        self.dropoff_pose.position.y = 0.0
        self.dropoff_pose.position.z = 0.0
        qx,qy,qz,qw = self.euler_to_quaternion(0.0, 0.0, 0.0)
        self.dropoff_pose.orientation.x = qx
        self.dropoff_pose.orientation.y = qy
        self.dropoff_pose.orientation.z = qz
        self.dropoff_pose.orientation.w = qw

        self.robot_state = self.STATE_FIND_BOTTLE

    '''
    Randomly navigate around room until a bottle is found (i.e. until the bottle queue
    is not empty anymore)
    '''
    def find_bottle(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes, self.find_bottle_callback)
        self.set_vel(self.FIND_TURN, 0.0)

        while self.robot_state is self.STATE_FIND_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

        self.box_sub.unregister()
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)
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
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.navigate_bottle_callback)

        while self.robot_state is self.STATE_NAV_BOTTLE and not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            self.state_pub.publish(self.robot_state)
            self.vel_rate.sleep()

        self.box_sub.unregister()

    '''
    Callback function for navigating to a bottle whenever a new bottle pose is published
    by the object_tracker
    '''
    def navigate_bottle_callback(self, depth, bboxes):
        boxes = bboxes.bounding_boxes
        box = next(iter(list(filter(lambda x : x.Class == "bottle", boxes))), None)

        # Just look at first bottle (no queue)
        if box != None:
            # Navigate to first bottle seen
            # Determine size of bottle
            #size = box.xmax - box.xmin + 1
            #print("Bottle Size = {}".format(size))

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
            #if size > self.GRAB_SIZE_THRESHOLD:
            if distance < self.GRAB_DIST_THRESHOLD:
                self.robot_state = self.STATE_PICKUP_BOTTLE
                return

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
        self.state_pub.publish(self.robot_state)
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.claw_pub.publish(self.CLAW_CLOSED_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.arm_pub.publish(self.ARM_UP_ANGLE)
        time.sleep(self.STARTUP_DROPOFF_DELAY)
        self.robot_state = self.STATE_NAV_DROPOFF


    '''
    Navigate to the dropoff location
    '''
    def navigate_dropoff(self):
        self.goal_simple_pub.publish(dropoff_pose)
        self.goal_reached_sub = rospy.Subscriber('/rtabmap/goal_reached', Bool, self.navigate_dropoff_callback)

        while self.robot_state is self.STATE_NAV_DROPOFF and not rospy.is_shutdown():
            self.state_pub.publish(self.robot_state)
            rospy.spin()

        self.box_sub.unregister()


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
        self.state_pub.publish(self.robot_state)
        self.set_vel(0.0, 0.0)
        self.vel_pub.publish(self.vel)

        self.arm_pub.publish(self.ARM_DOWN_ANGLE)
        time.sleep(self.CLAW_DELAY)
        self.claw_pub.publish(self.CLAW_OPEN_ANGLE)
        time.sleep(self.STARTUP_DROPOFF_DELAY)
        self.robot_state = self.STATE_STOP


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
            elif bot.robot_state == bot.NAGIVATE_DROPOFF:
                bot.navigate_dropoff()
            elif bot.robot_state == bot.STATE_DROPOFF_BOTTLE:
                bot.dropoff_bottle()
            else:
                bot.stop()

    except rospy.ROSInterruptException:
        bot.stop()
