#!/usr/bin/env python

import rospy
import socket
from waveshare_alphabot2.msg import IR

LIRCPATH = "/var/run/lirc/lircd"

class ir_driver:

    def __init__(self):
        rospy.init_node("ir_control")
        rospy.loginfo("Node 'ir_control' configuring driver.")

        self.rate = rospy.Rate(rospy.get_param('~rate', 10))

        try:
            self.lirc_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.lirc_socket.connect(LIRCPATH)

            rospy.loginfo("Node 'ir_control' lirc connection established.")

            # Setup publisher for ir_remote
            self.pub = rospy.Publisher('ir_remote', IR, queue_size=4)
            
            rospy.loginfo("Node 'ir_driver' configured.")

        except AssertionError as error:
            rospy.loginfo("Node 'ir_control' ERROR: lirc connection failled. ")
            raise RuntimeError(error)

    def run(self):
        rospy.loginfo("Node 'ir_control' running.")
        while not rospy.is_shutdown():
            data = self.lirc_socket.recv(128)
            data = data.strip()
            if data:
                words = data.split()
                key_name = words[2]
                count_pressed = int(words[1],16)
                IR_message = IR()
                IR_message.key_name = key_name
                IR_message.count_pressed = count_pressed

                self.pub.publish(IR_message)
                rospy.loginfo("Node 'ir_control' received key "+str(words[2])+" ("+str(count_pressed)+").")

             
            self.rate.sleep()

def main():
    rospy.loginfo("Starting Node 'ir_control'.")
    driver = ir_driver()
    driver.run()
    rospy.loginfo("Node 'ir_control' Stopped.")

if __name__ == '__main__':
    main()

