#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

import math
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import select
import termios
import tty


state_x = -1
count = 0

DISTINATION = [[-1, 0, 0], [-0.5, 0, 0]]
DISTINATION2 = [[0, 1.5, 315], [1.5, 0, 270], [0, -1.5, 225], [-1.5, 0, 135]]
DISTINATION3 = [[-0.7, 0.7, 45], [0, 1, 315], [0.7, 0.7, 315], [1, 0,
                                                                225], [0.7, -0.7, 225], [0, -1, 225], [-0.7, -0.7, 135], [-1, 0, 315]]
FLG_GOAL = 0
FLG_Timer = 0
mode = 0  # 0:wall running, 1:color attack


class STATUS():
    position = [0, 0]  # recent odom ([x,z])
    orient=0
    vel = 0  # recent velocity (m/s)
    twist = [0, 0]  # recent twist ([x,z])
    targetangle=0
    mode = 0  # 0:navi 1:avoid 2:attack 3:escape 4:emergensy 5:avoid 6:stragit
    distination = [[-1.1, 0, 45], [0, 1.1, 315], [1.1, 0, 315], [0, -1.1,
                                                                 225], ]
    flgbusy = False
    flgrotete = 0
    distination_now = 1
    distination_num = 4
    

    def __init__(self):
        pass

    @staticmethod
    def sleep(time, callback, oneshot=True):
        rospy.Timer(rospy.Duration(time), callback, oneshot)

    @staticmethod
    def setMode(mode,robot):
        if(not STATUS.flgbusy):
            STATUS.mode = mode
            print('status -> '+str(mode))
            if(STATUS.mode==0):
                robot.togoal(*STATUS.distination[STATUS.distination_now])
            if(STATUS.mode==1):
               robot.cancel.publish() 
            if(STATUS.mode==5):
               robot.cancel.publish() 
            
                            

    @staticmethod
    def setBusy(time):
        STATUS.flgbusy = True
        STATUS.sleep(time, STATUS.resetStatusFlg)
    
    @staticmethod
    def resetStatusFlg(data):
        STATUS.flgbusy = False
    

class Trutlerun(object):

    def __init__(self):
        #global IMU
        # self.feedvback=rospy.Subscriber('/red_bot/move_base/feedback',MoveBaseActionFeedback,self.feedbackCallback,queue_size=1)
        # IMU["sub"]=rospy.Subscriber('/red_bot/imu',Imu,self.imuCallback,queue_size=1)
        self.odom = rospy.Subscriber(
            '/red_bot/odom', Odometry, self.odomCallback, queue_size=1)
        self.scan = rospy.Subscriber(
            '/red_bot/scan', LaserScan, self.LaserScanCallback, queue_size=1)
        self.result = rospy.Subscriber(
            '/red_bot/move_base/result', MoveBaseActionResult, self.goalcallback, queue_size=1)
        # self.result = rospy.Subscriber(
        #     '/red_bot/war_state', String, self.warstatecallback, queue_size=1)
        self.cmd_vel_pub = rospy.Subscriber(
            'cmd_vel', Twist, self.speedCallback, queue_size=1)

        self.slam = rospy.Publisher(
            '/red_bot/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel = rospy.Publisher(
            '/red_bot/move_base/cancel', GoalID, queue_size=1)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def warstatecallback(self,data):
        print(data)

    def odomCallback(self, data):
        STATUS.position[1] = STATUS.position[0]
        STATUS.position[0] = np.array(
            [data.pose.pose.position.x, data.pose.pose.position.z])
        vel = STATUS.position[0]-STATUS.position[1]
        STATUS.vel = np.linalg.norm(vel, ord=2)*30
        STATUS.orient=np.rad2deg(euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))[2])

    def imuCallback(self, data):
        print(data)
        global IMU
        IMU["data"] = data.pose.position
        IMU["Vel"].unregister()

    # def feedbackCallback(self, data):
    #     print(data)

    def speedCallback(self, data):
        STATUS.twist[0] = data.linear.x
        STATUS.twist[1] = data.angular.z

    def togoal(self, x, y, radians):
        '''
        概要
        目的地を指定して移動する
        x : 上下方向の座標 (-1<x<1)
        y : 左右方向の座標 (-1<y<1)
        radians : 目標姿勢。0度が上。半時計回りに増加する方向。 
        '''
        goal = PoseStamped()
        goal.header.frame_id = "red_bot/map"        # 世界座標系で指定する
        goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, math.radians(radians))
        goal.pose.orientation = Quaternion(*q)
        print('呼び出し')
        self.slam.publish(goal)

    def goalcallback(self, data):
        if(data.status.status == 3 and FLG_Timer == 0):
            global FLG_GOAL
            FLG_GOAL = 1

    def LaserScanCallback(self, data):
        direction = STATUS.twist[0] >= 0
        self.range = np.array(data.ranges)

        if(self.checkSafeRoute(self.range, direction)):
            if(STATUS.mode == 0):
                STATUS.setMode(1,self)
                STATUS.setBusy(1)
            if(STATUS.mode == 1):
                self.avoidobstacle2(self.range, direction)
            if(STATUS.mode == 6):
                STATUS.setMode(5,self)
                self.avoidobstacle3(self.range, direction)

        else:
            if(STATUS.mode == 1):
                STATUS.setMode(0,self)
            if(STATUS.mode == 6):
                vel = Twist()
                vel.linear.x = 0.3
                vel.angular.z=0
                self.vel_pub.publish(vel)
        if(STATUS.mode==5):
            if(STATUS.flgrotete == -1):
                if(STATUS.orient<STATUS.targetangle):
                    STATUS.flgrotete =0
                    print(STATUS.orient,STATUS.targetangle)
                    STATUS.setMode(6,self)
                    
            if(STATUS.flgrotete == 1):
                if(STATUS.orient>STATUS.targetangle):
                    STATUS.flgrotete =0
                    print(STATUS.orient,STATUS.targetangle)
                    STATUS.setMode(6,self)

    def checkSafeRoute(self, laser, direction, distance=0.4, width=0.12):
        th = np.linalg.norm([distance, width], ord=2)
        angle = np.ceil(np.rad2deg(np.arctan2(width, distance)))
        if(direction):
            checkresult = (np.any(laser[0:angle] < th)
                           or np.any(laser[-angle:360] < th))
        else:
            checkresult = np.any(laser[180-angle:180+angle] < th)
        
        return checkresult

    def avoidobstacle(self, laser, direction):
        watchangle = 60
        speedweight = 0.5
        curveweight = 1.5
        reverselength = 0.2
        curvemax=1#3.5
        speedmax=0.22
        vel = Twist()
        obs_fr = np.median(self.range[0:watchangle])
        obs_fl = np.median(self.range[-watchangle:360])
        obs_br = np.median(self.range[180-watchangle:180])
        obs_bl = np.median(self.range[180:180+watchangle])
        obs_f = min(self.range[0:20].min(), self.range[340:359].min())
        obs_b = self.range[140:220].min()
        if((direction and (obs_f < reverselength)) or (not direction and (obs_b < reverselength))):
            direction = not direction
            obs_f = obs_b = 0.5

        fcurveweight = abs(obs_fr - obs_fl)
        bcurveweight = abs(obs_br - obs_bl)
        if obs_fl > obs_fr:
            fcurveweight = -fcurveweight
        if obs_bl > obs_br:
            bcurveweight = -bcurveweight
        if(direction):
            vel.linear.x = obs_f*speedweight
            vel.angular.z = fcurveweight*1/obs_f
        else:
            vel.linear.x = -obs_b*speedweight
            vel.angular.z = bcurveweight*1/obs_b
        if (fcurveweight < 0.2):
            fcurveweight = 0.2
        if(bcurveweight < 0.2):
            bcurveweight = 0.2
        print(vel.angular.z)
        
        if (vel.linear.x>speedmax):
            vel.linear.x=speedmax
        if (vel.linear.x<-speedmax):
            vel.linear.x=-speedmax
        if (vel.angular.z>curvemax):
            vel.angular.z=curvemax
            vel.linear.x=0
        if (vel.angular.z<-curvemax):
            vel.angular.z=-curvemax
            vel.linear.x=0


        self.vel_pub.publish(vel)
        #curveweight = curveweight if obs_r > obs_l else - curveweight
        # STATUS.twist[0] = STATUS.twist[0] * \
        #     obs_f if STATUS.twist[0] > 0 else STATUS.twist[0] * obs_b
    
    def avoidobstacle2(self, laser, direction):
        watchangle = 60
        speedweight = 0.5
        curveweight = 1.5
        reverselength = 0.2
        curvemax=2.84#3.5
        speedmax=0.22
        vel = Twist()
        obs_fr = np.median(self.range[0:watchangle])
        obs_fl = np.median(self.range[-watchangle:360])
        obs_br = np.median(self.range[180-watchangle:180])
        obs_bl = np.median(self.range[180:180+watchangle])
        obs_f = min(self.range[0:20].min(), self.range[340:359].min())
        obs_b = self.range[140:220].min()
        
        fcurveweight = abs(obs_fr - obs_fl)
        bcurveweight = abs(obs_br - obs_bl)
        if obs_fl > obs_fr:
            fcurveweight = -fcurveweight
        if obs_bl > obs_br:
            bcurveweight = -bcurveweight
        if(direction):
            vel.angular.z = fcurveweight*1/obs_f
        else:
            vel.angular.z = bcurveweight*1/obs_b
        print(vel.angular.z)
        
        if (vel.angular.z>curvemax):
            vel.angular.z=curvemax
        if (vel.angular.z<-curvemax):
            vel.angular.z=-curvemax

        self.vel_pub.publish(vel)
    
    def avoidobstacle3(self, laser, direction):
        vel = Twist()
        laser[laser>1]=0
        edgeind=np.where(abs(laser-np.roll(laser,1))>0.5)[0]
        print(edgeind)
        r=max(edgeind)-360
        l=min(edgeind)

        targetangle=r + 15 if r<abs(l) else l -15
        print(targetangle)
        STATUS.flgrotete=1 if r<abs(l) else -1
        STATUS.targetangle=STATUS.orient+targetangle
        if (STATUS.targetangle>180):
            STATUS.targetangle=STATUS.targetangle-360
        if (STATUS.targetangle<-180):
            STATUS.targetangle=360+STATUS.targetangle
        print(STATUS.targetangle,STATUS.orient)
        vel.angular.z= 0.1 if targetangle < 0 else -0.1
        vel.linear.x = 0
        self.vel_pub.publish(vel)

    def checkImage(self):
        '''
        mode=1->color attack. 
        mode=0 ->wall running.
        '''
        global state_x
        global mode
        if state_x == -1:
            mode = 0
        else:
            mode = 1

    def calcTwist(self):
        global state_x
        global mode
        mode = 1
        if state_x >= 360:
            # print("right")
            x = 0
            th = -0.2
        elif state_x <= 260 and state_x >= 0:
            # print("left")
            x = 0
            th = 0.2
        elif 260 < state_x and state_x < 360:  # 対象が範囲内
            # print("center")
            x = 0.2
            th = 0
        elif state_x == -1:  # 対象が範囲内にないとき
            return

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1)  # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        twist = self.calcTwist()
        # print(twist)
        self.vel_pub.publish(twist)

        r.sleep()

    def resetTimer(self, event):
        global FLG_Timer
        FLG_Timer = 0

    def changeDestination(self, event):
        if(FLG_Timer == 0):
            global FLG_GOAL
            FLG_GOAL = 1


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/red_bot/image_raw", Image, self.callback)

    def callback(self, data):
        global state_x
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imwrite("/home/nryk/デスクトップ/dst/1-test.png", cv_image)

        # RGB表色系からHSV表色系に変換
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        cv2.imwrite("/home/nryk/デスクトップ/dst/2-hsv.png", hsv_image)

        # 赤
        # color_min = np.array([150,100,150])
        # color_max = np.array([180,255,255])

        # 青
        # color_min = np.array([110, 50, 50])
        # color_max = np.array([130, 255, 255])

        # 緑
        color_min = np.array([30, 100, 200])
        color_max = np.array([60, 255, 250])

        # マスクの画像
        mask = cv2.inRange(hsv_image, color_min, color_max)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        cv2.imwrite("/home/lsic/デスクトップ/dst/3-blue-mask.png", res)

        # グレースケール化（色抽出）
        img = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        mu = cv2.moments(img, False)
        if mu["m00"] != 0:
            state_x, y = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
        else:
            state_x, y = -1, -1

        # print(state_x,y)


if __name__ == '__main__':
    rospy.init_node('onigiri_run')
    image   = image_converter()
    r = rospy.Rate(1)  # change speed 1fps
    myrobot = Trutlerun()
    rospy.timer.sleep(2)
    #myrobot.togoal(*STATUS.distination[STATUS.distination_now])
    STATUS.setMode(0,myrobot)
    while not rospy.is_shutdown():
        # a.checkImage()

        if mode:
            myrobot.strategy()
            pass
        else:
            # print(FLG_GOAL)
            if(FLG_GOAL):
                STATUS.distination_now += 1
                print("Distination: " + str(STATUS.distination_now))
                if(STATUS.distination_now == 4):
                    STATUS.distination_now = 0
                myrobot.togoal(*STATUS.distination[STATUS.distination_now])
                FLG_GOAL = 0
                FLG_Timer = 1
                # rospy.Timer(rospy.Duration(10),
                #             myrobot.changeDestination, oneshot=True)
                rospy.Timer(rospy.Duration(
                    1), myrobot.resetTimer, oneshot=True)

    rospy.spin()
