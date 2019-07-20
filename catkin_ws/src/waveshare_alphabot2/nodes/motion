#!/usr/bin/env python

'''
Controls the Waveshare Alphabot2 robot wheel motors
Updated code from DingoOz: https://github.com/DingoOz/Alphabot2_Pi_ROS/blob/master/src/alphabot2/src/driver_node
Author: Shaun Price (https://github.com/ShaunPrice)
'''
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time

# Raspberry PI GPIO Function to pin mapping
IN1 = 13
IN2 = 12
IN3 = 21
IN4 = 20
ENA = 6
ENB = 26
PA  = 50
PB  = 50

_FREQUENCY = 22000

def _clip(value, minimum, maximum):
	if value < minimum:
		return minimum
	elif value > maximum:
		return maximum
	return value

class Motor:
	def __init__(self, forward_pin, backward_pin):
		self._forward_pin = forward_pin
		self._backward_pin = backward_pin
		GPIO.setup(forward_pin, GPIO.OUT)
		GPIO.setup(backward_pin, GPIO.OUT)

		self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
		self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

	def move(self, speed_percent):
		speed = _clip(abs(speed_percent), 0, 100)

		# A positive speed moves wheels forward, negative moves backwards
		if speed_percent >= 0:
			self._forward_pwm.start(speed)
			self._backward_pwm.start(0)
		else:
			self._backward_pwm.start(speed)
			self._forward_pwm.start(0)

class motion_driver:
	def __init__(self, in1=13, in2=12, in3=21, in4=20, ena=6, enb=26, pa=50, pb=50):
		rospy.init_node("motion_driver")
		rospy.loginfo("Node 'motion' configuring driver.")
		self.IN1 = in1
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENA = ena
		self.ENB = enb
		self.PA = pa
		self.PB = pb

		#Configure GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.IN1,GPIO.OUT)
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.PWMA = GPIO.PWM(self.ENA,500)
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(self.PA)
		self.PWMB.start(self.PB)

		# Set ENA/ENB to high
		GPIO.output(self.ENA, GPIO.HIGH)
		GPIO.output(self.ENB, GPIO.HIGH)

		# Set forward
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

		rospy.loginfo("Node 'motion' GPIO configured.")

		self._last_received = rospy.get_time()
		self._timeout = rospy.get_param('~timeout', 2)
		self.rate = rospy.Rate(rospy.get_param('~rate', 10))
		self._max_speed = rospy.get_param('~max_speed', 0.25)
		self._wheel_base = rospy.get_param('~wheel_base', 0.093)
		self._left_motor = Motor(self.IN1, self.IN2)
		self._right_motor = Motor(self.IN3, self.IN4)
		self._left_speed_percent = 0
		self._right_speed_percent = 0

		# Setup subscriber for velocity twist message
		rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)

		# Setup subscriber for collision detection
		rospy.Subscriber("collision", String, self.collision_callback)

		# Setup publisher for velocity. Used in collision to stop motors
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=4)

		rospy.loginfo("Node 'motion' configuration complete.")

	def __del__(self):
		GPIO.cleanup()

	def velocity_received_callback(self, message):
		self._last_received = rospy.get_time()
		rospy.loginfo("Node 'motion' velocity message received")
		# Extract linear and angular velocities from the message
		linear = message.linear.x
		angular = message.angular.z

		rospy.loginfo("Node 'motion' linear=  "+str(linear)+".")
		rospy.loginfo("Node 'motion' angular= "+str(angular)+".")

		# Calculate wheel speeds in m/s
		left_speed = linear - angular*self._wheel_base/2
		right_speed = linear + angular*self._wheel_base/2

		# Ideally we'd now use the desired wheel speeds along
		# with data from wheel speed sensors to come up with the
		# power we need to apply to the wheels, but we don't have
		# wheel speed sensors. Instead, we'll simply convert m/s
		# into percent of maximum wheel speed, which gives us a
		# duty cycle that we can apply to each motor.
		self._left_speed_percent = (100 * left_speed/self._max_speed)
		self._right_speed_percent = (100 * right_speed/self._max_speed)
		rospy.loginfo("Node 'motion' velocity message Left= "+str(self._left_speed_percent)+"%")
		rospy.loginfo("Node 'motion' velocity message Right="+str(self._right_speed_percent)+"%.")

	def run(self):
		rospy.loginfo("Node 'motion' running.")

		while not rospy.is_shutdown():
			# If we haven't received new commands for a while, we
			# may have lost contact with the commander-- stop moving
			delay = rospy.get_time() - self._last_received
			if delay < self._timeout:
				self._left_motor.move(self._left_speed_percent)
				self._right_motor.move(self._right_speed_percent)
			else:
				self._left_motor.move(0)
				self._right_motor.move(0)
			
			self.rate.sleep()
	
	def collision_callback(self, message):
		rospy.loginfo("Node 'motion' collision detected. Stopping")
		if(message.data == 'obstacle_right' or message.data == 'obstacle_left'):
			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

def main():
	rospy.loginfo("Starting node 'motion'")
	driver = motion_driver(IN1, IN2, IN3, IN4, ENA, ENB, PA, PB)
	driver.run()
	rospy.loginfo("Node 'motion' Stopped")

if __name__ == '__main__':
	main()
