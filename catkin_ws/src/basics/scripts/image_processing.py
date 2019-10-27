#! /usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


bridge = CvBridge()

queue = [] 

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)
    
    
def call_back(msg):
    #rospy.loginfo(msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        
    queue.append(cv_image)
    if len(queue) > 2:
        queue.pop(0)
        
    if len(queue) == 2:
        gray1 = cv2.cvtColor(queue[1], cv2.COLOR_BGR2GRAY) # most recent image
        gray0 = cv2.cvtColor(queue[0], cv2.COLOR_BGR2GRAY) # second most recent image
     
        delta = gray1-gray0
                
        show_image(delta)
 
        
            
      

rospy.init_node('image_processing')
rospy.Subscriber('usb_cam/image_raw', Image, call_back, queue_size=1)
rospy.spin()
