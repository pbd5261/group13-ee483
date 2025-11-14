#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import Segment, SegmentList, Vector2D, LanePose, Twist2DStamped
from cv_bridge import CvBridge
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float32
import time 



class LaneDetector: 
    def __init__(self):
        self.bridge = CvBridge()
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/lane_filter_node/lane_pose", LanePose, self.callback, queue_size=1, buff_size= 2**24)
        self.pub_1 = rospy.Publisher('/ee483mm13/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        self.pub_phi = rospy.Publisher("avg_phi", Float32, queue_size=32)
        self.car_cmd = Twist2DStamped()

        self.state = rospy.get_param("stateIsStatic")
        if self.state == "True":
            rospy.Subscriber("/ee483mm13/avg_phi", Float32, self.static_PID)
        else:
            rospy.Subscriber("/ee483mm13/avg_phi", Float32, self.dynamic_PID)

        self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
        self.total_error = 0
        self.prev_error = 0
        self.K = rospy.get_param("K")
        self.Ki = rospy.get_param("Ki")
        self.Kd = rospy.get_param("Kd")
        self.v = rospy.get_param("v")
        self.prev_time = rospy.Time.now().to_sec()

        self.newPhi = 0
        self.temp = 0
        
 
    def callback(self,phi):
       self.movingAvg(phi.phi)


    def movingAvg(self, phi):


        self.newPhi = self.newPhi + phi
        self.temp = self.temp + 1
        if self.temp == 5:
            self.newPhi = self.newPhi/5
            self.pub_phi.publish(self.newPhi)
            self.newPhi = 0
            self.temp = 0

    def static_PID(self,msg):
        current_time = rospy.Time.now().to_sec()

        self.car_cmd.v = 0
        msg.data = -1*msg.data
        control_signal_p = rospy.get_param("K") * msg.data
        control_signal_i = rospy.get_param("Ki") * ((current_time-self.prev_time) * 1*msg.data + self.total_error)
        control_signal_d = rospy.get_param("Kd") * (msg.data-self.prev_error)/(current_time-self.prev_time)

        control_signal = control_signal_p+control_signal_i+control_signal_d
        control_signal_msg = Float32(control_signal)
        self.pub.publish(control_signal_msg)
        self.prev_error = msg.data
        self.total_error = self.total_error + msg.data
        self.prev_time = current_time

        self.car_cmd.omega = control_signal
        self.pub_1.publish(self.car_cmd)

    def dynamic_PID(self,msg):
        current_time = rospy.Time.now().to_sec()

        self.car_cmd.v = rospy.get_param("v")
        msg.data = -1*msg.data
        control_signal_p = rospy.get_param("K") * msg.data
        control_signal_i = rospy.get_param("Ki") * ((current_time-self.prev_time) * 1*msg.data + self.total_error)
        control_signal_d = rospy.get_param("Kd") * (msg.data-self.prev_error)/(current_time-self.prev_time)

        control_signal = control_signal_p+control_signal_i+control_signal_d
        control_signal_msg = Float32(control_signal)
        self.pub.publish(control_signal_msg)
        self.prev_error = msg.data
        self.total_error = self.total_error + msg.data
        self.prev_time = current_time

        self.car_cmd.omega = control_signal
        self.pub_1.publish(self.car_cmd)


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("LaneDetector", anonymous=True)
    LaneDetector()
    rospy.set_param("controller_ready",'true')
    rospy.spin()