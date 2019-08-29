#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2


class AbstractRulo(object):
    __metaclass__ = ABCMeta

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name

        # bumper state

        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # velocity publisher
        self.vel_pub = rospy.Publisher('/Rulo/cmd_vel', Twist,queue_size=1)
	self.mode_pub = rospy.Publisher('/mobile_base/command/mode', String,queue_size=1)
        # bumper subscrivre

        # camera subscriver
        # please uncoment out if you use camera
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.imageCallback)

    # bumper topic call back sample
    # update bumper state
   

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    @abstractmethod
    def strategy(self):
        pass