#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox

class Navigator:
    FORWARD_THRESHOLD = 75
    IMAGE_HEIGHT = 480
    IMAGE_WIDTH = 640
    PROPORTIONAL = 2.0
    MINIUMUM_TURN = 1.5
    """
    Navigator class for trash bot
    """
    def __init__(self):
        # By default, send zero velocity
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0

        print("="*50)
        print("= Initialize Trashbot Navigator")
        print("="*50)
        print("Waiting for bottle bounding box")

        rospy.init_node('navigator', anonymous=True)
        rospy.Subscriber('/object_tracker/bounding_box', BoundingBox, self.navigate_bottle)
        self.send_vel_message()
            
    # Navigate to the nearest bottle
    def navigate_bottle(self, data):
        # Navigate to first bottle
        box = data
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


    # Set velocity
    def set_vel(self, turn, forward):
        self.vel.angular.z = turn
        self.vel.linear.x = forward
        #self.vel_pub.publish(self.vel)

    # Publish to cmd_vel topic
    def send_vel_message(self):
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        r = rospy.Rate(7)
        while not rospy.is_shutdown():
            vel_pub.publish(self.vel)
            r.sleep()

if __name__ == '__main__':
    try:
        d = Navigator()

    except rospy.ROSInterruptException:
        pass
