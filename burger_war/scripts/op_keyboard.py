#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
from abstractRulo import *
import numpy as np
from pprint import pprint
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""
speed=[1,1]
IMU={'data':0,'sub':0,'new':0}


moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

class OpKey():
	
	def __init__(self, bot_name):
		IMU["sub"]=rospy.Subscriber('/red_bot/imu',Imu,self.imuCallback,queue_size=1)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
		self.scan = rospy.Subscriber('/red_bot/scan', LaserScan, self.LaserScanCallback, queue_size=1)
		self.cmd_vel_pub = rospy.Subscriber('cmd_vel', Twist,self.speedCallback, queue_size=1)
		
	def speedCallback(self,data):
		global speed
		speed[0]=data.linear.x
		speed[1]=data.angular.z

		
	def imuCallback(self,data):
		global IMU
		IMU["data"]=data
		#IMU["sub"].unregister()
	
	def LaserScanCallback(self,data):
		vel = Twist()
		global speed
		global IMU
		print(IMU['data'].linear_acceleration.x,IMU['data'].linear_acceleration.y,IMU['data'].linear_acceleration.z)
		print('\n\r')
		CURVE_FINDLENGTH = 0.5
		CURVE_FINDLENGTH = 0.5
		self.range=np.array(data.ranges)
		obs_r = np.median(self.range[0:60])
		obs_l = np.median(self.range[300:359])
		obs_f = min(self.range[0:30].min(),self.range[330:359].min())
		obs_b = self.range[150:210].min()
		# print(obs_l, obs_r)
		# print(self.range[0], self.range[180])
		if (obs_r > CURVE_FINDLENGTH and obs_l > CURVE_FINDLENGTH):
			speed[1]=0
			return 0
		curveweight = abs(obs_r - obs_l)
		curveweight = curveweight if obs_r > obs_l else - curveweight
		#print(str(obs_b)+'\n\r')
		vel.angular.z = speed[1]+curveweight
		
		speed[0]=speed[0]*obs_f if speed[0]>0 else speed[0] * obs_b
		vel.linear.x = speed[0]
		
		#self.vel_pub.publish(vel)
        
		
		
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

	def vels(self,speed,turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)

	def strategy(self):
		speed = rospy.get_param("~speed", 0.5)
		turn = rospy.get_param("~turn", 1.0)
		x = 0
		y = 0
		z = 0
		th = 0
		status = 0

		try:
			print(msg)
			print(self.vels(speed,turn))
			while(1):
				key = self.getKey()
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					y = moveBindings[key][1]
					z = moveBindings[key][2]
					th = moveBindings[key][3]
				elif key in speedBindings.keys():
					speed = speed * speedBindings[key][0]
					turn = turn * speedBindings[key][1]

					print(self.vels(speed,turn))
					if (status == 14):
						print(msg)
					status = (status + 1) % 15
				else:
					x = 0
					y = 0
					z = 0
					th = 0
					if (key == '\x03'):
						break

				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				self.vel_pub.publish(twist)
				

		except Exception as e:
			print(e)

		finally:
			twist = Twist()
			twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			pub.publish(twist)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('teleop_twist_keyboard_red')
	pub = rospy.Publisher('red_bot/cmd_vel', Twist, queue_size = 1)
	
	op_key = OpKey("op_key")
	op_key.strategy()
	

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)