#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import Segment, SegmentList, Vector2D, LanePose
#from sensor_msgs.msgs import Twist2DStamped
from cv_bridge import CvBridge
from std_srvs.srv import SetBool, SetBoolResponse



class LaneDetector: 
    def __init__(self):
        self.bridge = CvBridge()
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/lane_filter_node/lane_pose", LanePose, self.callback, queue_size=1, buff_size= 2**24)
        rospy.pub_1 = rospy.Publisher('/ee483mm13/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        rospy.pub_phi = rospy.Publisher("avg_phi", LanePose, queue_size=32)
        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        
 
    def callback(self,LanePose):
        self.movingavg(LanePose)


    def movingAvg(self, phi):
        newPhi = 0
        temp = 0
        for temp in range(5):
            if temp < 5:
                newPhi = newPhi + phi
            else:
                newPhi = newPhi/5
                temp = 0
            temp = temp + 1
    self.pub_phi(newPhi)
    


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("LaneDetector", anonymous=True)
    LaneDetector()
    rospy.spin()