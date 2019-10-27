#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan


def callback(msg):
    j = 270
        
    for i in range(90,180):
        
        print(msg.ranges[i])
        
    pub.publish(msg)

rospy.init_node('scan_reverse')
pub = rospy.Publisher('/scan2',LaserScan,queue_size = 1)
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

