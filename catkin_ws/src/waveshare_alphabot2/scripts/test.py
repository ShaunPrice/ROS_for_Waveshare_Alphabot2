#!/usr/bin/env python

import time
import rospy
import roslib

from waveshare_alphabot2.msg import Pan_Tilt, IR, RGB_LED
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32

rospy.init_node("test")

# Message publishers
pantilt = rospy.Publisher('pan_tilt', Pan_Tilt, queue_size=4)
rgb = rospy.Publisher('rgb_leds', RGB_LED, queue_size=4)
buzz = rospy.Publisher('buzzer', Float32, queue_size=4)
move = rospy.Publisher('cmd_vel', Twist, queue_size=4)

time.sleep(1)

# Messages
pantilt_msg = Pan_Tilt()
rgb_led_msg = RGB_LED()

# beep
print('buzz......')
buzz.publish(0.5)

# Flash LEDs
print('Flashing LED\s')
rgb_led_msg.function =  'theaterChaseRainbow'
rgb_led_msg.color = ''
rgb_led_msg.led1_color = ''
rgb_led_msg.led2_color = ''
rgb_led_msg.led3_color = ''
rgb_led_msg.led4_color = ''
rgb_led_msg.delay = 50

rgb.publish(rgb_led_msg)

# Centre
print('Head Pan/Tilt - Centre')
pantilt_msg.pan = 0.0
pantilt_msg.tilt = 0.0
pantilt.publish(pantilt_msg)
time.sleep(1.0)

#  Head Up/Right
print('Head Pan/Tilt - Look Up and Right')
pantilt_msg.pan = 1.0
pantilt_msg.tilt = 1.0
pantilt.publish(pantilt_msg)
time.sleep(1.0)

#  Head Down/Left
print('Head Pan/Tilt - Look Down and Left')
pantilt_msg.pan = -1.0
pantilt_msg.tilt = -1.0
pantilt.publish(pantilt_msg)
time.sleep(1.0)

# Centre
print('Head Pan/Tilt - Centre')
pantilt_msg.pan = 0.0
pantilt_msg.tilt = 0.0
pantilt.publish(pantilt_msg)
time.sleep(1.0)

# beep
print('buzz......')
buzz.publish(0.5)

# Now we're going to move the robot
print('Moving forward for 0.5 seconds')
move_msg = Twist()
move_msg.linear.x = 0.5
move.publish(move_msg)
time.sleep(1.0)

print('Moving backward for 0.5 seconds')
move_msg = Twist()
move_msg.linear.x = -0.5
move.publish(move_msg)
time.sleep(1.0)

print('Turning robot right for 0.5 seconds')
move_msg = Twist()
move_msg.linear.x = 0.0
move_msg.angular.z = 0.5
move.publish(move_msg)
time.sleep(1.0)

print('Turning robot left for 0.5 seconds')
move_msg = Twist()
move_msg.angular.z = -0.5
move.publish(move_msg)
time.sleep(1.0)
# beep
print('buzz......')
buzz.publish(0.5)

print('Turning robot left for 0.5 seconds')
move_msg = Twist()
move_msg.angular.z = 0.0
move.publish(move_msg)
time.sleep(0.5)

# beep
print('buzz......')
buzz.publish(0.5)

# Turn LED's Off
print('LED\'s Off')
rgb_led_msg.function =  'setLED'
rgb_led_msg.color = ''
rgb_led_msg.led1_color = '000000'
rgb_led_msg.led2_color = '000000'
rgb_led_msg.led3_color = '000000'
rgb_led_msg.led4_color = '000000'
rgb_led_msg.delay = 50

rgb.publish(rgb_led_msg)