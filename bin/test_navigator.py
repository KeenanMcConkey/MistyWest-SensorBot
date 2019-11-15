#!/usr/bin/python2
from __future__ import print_function

import roslib
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes


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
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.navigate_bottle)
        self.send_vel_message()
            
    # Navigate to the nearest bottle
    def navigate_bottle(self, data):
        # Navigate to first bottle
        boxes =  list(filter(lambda x: x.Class == "bottle", data.bounding_boxes))
        if boxes:
            box = boxes[0]
            #print("See bottle in x range = {}-{}".format(box.xmin, box.xmax))
            xpos = (box.xmax + box.xmin)/2 - self.IMAGE_WIDTH/2
            print("xpos {}".format(xpos))

            if xpos > -self.FORWARD_THRESHOLD and xpos < self.FORWARD_THRESHOLD:
	        # Go forward
                self.set_vel(0.0, 2.0)
            else:
                # Rotate 
                turn = -xpos / (self.IMAGE_WIDTH/2) * self.PROPORTIONAL
                turn = turn if abs(turn) > self.MINIUMUM_TURN else self.MINIUMUM_TURN
                print("turn {}".format(turn))
                #if turn > 2:
                #    turn = 2
                #elif turn < -2:
                #    turn = -2
                #turn = -(abs(xpos)/xpos) * 1.5
                self.set_vel(turn, 0.0)
        else:
            self.set_vel(0.0, 0.0)

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
