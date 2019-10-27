#! /usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

key_pub = rospy.Publisher('keys', String, queue_size = 1) # the topic name is key
rospy.init_node("key_publisher") # the node name is key_publisher
rate = rospy.Rate(100)

old_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

print ("Publishing keystrokes. Press Ctrl-C to exit")

while not rospy.is_shutdown():
    if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
        key_pub.publish(sys.stdin.read(1))
    rate.sleep()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
