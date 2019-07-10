#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time

# Buzzer
# Speaker - TODO
# Microphone - TODO

BUZ = 4

class sound_driver:
	def __init__(self):
		rospy.init_node("sound_driver")
		rospy.loginfo("Node 'sound' configuring driver.")

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(BUZ,GPIO.OUT)

		self.rate = rospy.Rate(rospy.get_param('~rate', 10))

		# Setup subscriber for velocity twist message
		rospy.Subscriber('buzzer', Float32, self.buzzer_callback)

		rospy.loginfo("Node 'sound' configuration complete.")

	def __del__(self):
		GPIO.cleanup()

	def run(self):
		rospy.loginfo("Node 'sound' running.")
		while not rospy.is_shutdown():
			self.rate.sleep()

	# Beep for duration seconds
	def buzzer_callback(self, message):
		rospy.loginfo("Node 'sound' beeping for "+str(message.data)+" seconds.")
		GPIO.output(BUZ,GPIO.HIGH)
		time.sleep(message.data)
		GPIO.output(BUZ,GPIO.LOW)

def main():
	rospy.loginfo("Starting node 'sound'")
	driver = sound_driver()
	driver.run()
	rospy.loginfo("Node 'sound' Stopped")

if __name__ == '__main__':
	main()
