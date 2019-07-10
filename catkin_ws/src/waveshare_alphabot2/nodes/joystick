#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time

# Joystick
CTR = 7 # Center
A = 8   # Up
B = 9   # Right
C = 10  # Left
D = 11  # Down
	
class joystick_driver:
	def __init__(self, ctr=7, a=8, b=9, c=10, d=11):
		rospy.init_node("joystick_driver")
		rospy.loginfo("Node 'joystick' configuring driver")
		self.CTR = ctr
		self.A = a
		self.B = b
		self.C = c
		self.D = d

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.CTR,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.A,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.B,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.C,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.D,GPIO.IN,GPIO.PUD_UP)

		self.rate = rospy.Rate(rospy.get_param('~rate', 10))

		# Setup publisher for obstacle detection
		self.pub = rospy.Publisher('joystick', String, queue_size=4)
		rospy.loginfo("Node 'joystick' configured.")

	def __del__(self):
		GPIO.cleanup()

	def run(self):
		rospy.loginfo("Node 'ir_control' running.")
		while not rospy.is_shutdown():
			if GPIO.input(self.CTR) == 0:
				# center
				while GPIO.input(self.CTR) == 0:
					self.pub.publish(String("Center"))
					rospy.loginfo("Node 'joystick' Center.")
			elif GPIO.input(self.A) == 0:
				# up
				while GPIO.input(self.A) == 0:
					self.pub.publish(String("Up"))
					rospy.loginfo("Node 'joystick' Up.")
			elif GPIO.input(self.B) == 0:
				# right
				while GPIO.input(self.B) == 0:
					self.pub.publish(String("Right"))
					rospy.loginfo("Node 'joystick' Right.")
			elif GPIO.input(self.C) == 0:
				# left
				while GPIO.input(self.C) == 0:
					self.pub.publish(String("Left"))
					rospy.loginfo("Node 'joystick' Left.")
			elif GPIO.input(self.D) == 0:
				# down
				while GPIO.input(self.D) == 0:
					self.pub.publish(String("Down"))
					rospy.loginfo("Node 'joystick' Down.")

			self.rate.sleep()

def main():
	rospy.loginfo("Starting node 'joystick'")
	driver = joystick_driver()
	driver.run()
	rospy.loginfo("Node 'joystick' Stopped")

if __name__ == '__main__':
	main()
