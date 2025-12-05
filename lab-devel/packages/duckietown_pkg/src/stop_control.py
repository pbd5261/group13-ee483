#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import Segment, SegmentList, Vector2D, LanePose, Twist2DStamped, BoolStamped
from cv_bridge import CvBridge
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float32
import time 


class Stop: 
    def __init__(self):
       rospy.Subscriber("/ee483mm13/stop_line_filter_node/at_stop_line", BoolStamped,self.stop)
        
    def stop(self,msg):
        if msg.data == True:
            rospy.loginfo("no")
            rospy.set_param("stop","True")
            rospy.sleep(1)




if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("Stop", anonymous=True)
    Stop()
    rospy.spin()