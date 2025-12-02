#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber
from duckietown_msgs.msg import Segment, SegmentList, Vector2D, StopLineReading, BoolStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_srvs.srv import SetBool, SetBoolResponse



class ImageProcessor: 
    def __init__(self):
        # Instatiate the converter class once by using a class member
        rospy.Service('line_detector_node/switch', SetBool, self.ld_switch)
        rospy.Service('lane_filter_node/switch',SetBool, self.lf_switch)
        self.bridge = CvBridge()
        veh_name = os.environ['VEHICLE_NAME']
        rospy.Subscriber(f"/{veh_name}/camera_node/image/compressed", CompressedImage, self.processor, queue_size=1, buff_size= 2**24)
        #rospy.Subscriber("image", Image, self.processor, queue_size=1, buff_size= 2**24)
        self.pub_edges = rospy.Publisher("image_edges", Image, queue_size=10)
        self.pub_yellow_mask = rospy.Publisher("image_mask_yellow", Image, queue_size=10)
        self.pub_red_mask = rospy.Publisher("image_mask_red", Image, queue_size=10)
        self.pub_white_mask = rospy.Publisher("image_mask_white", Image, queue_size=10)
        self.pub_yellow_lines = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.pub_red_lines = rospy.Publisher("image_lines_red", Image, queue_size=10)
        self.pub_white_lines = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.pub_lines = rospy.Publisher("image_lines", Image, queue_size=10)
        self.pub_segments = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)



        self.pub_stop_line_reading = rospy.Publisher("stop_line_reading", StopLineReading, queue_size=1, latch=True)
        self.pub_at_stop_line = rospy.Publisher("at_stop_line", BoolStamped, queue_size=1)


        self.stop_distance = 1
        self.min_segs = 1
        self.off_time = 1
        self.max_y = 5


        #variables used in class
        self.hsv_image = None
        self.cv_img = None
        self.cv_crop = None
        self.white_mask = None
        self.yellow_mask = None
        self.red_mask = None

    def processor(self, msg):
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.crop() #Crop Image
        self.hsv_image = cv2.cvtColor(self.cv_crop, cv2.COLOR_BGR2HSV)
        self.white_filter() #Create white filter
        self.yellow_filter() #create yellow filter
        self.red_filter()
        self.segment() #Create segments from filters

    # DO NOT CHANGE THIS FUNCTION
    def crop(self):
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(self.cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        self.cv_crop= new_image[offset:, :]

    def white_filter(self):
        #Finds all pixels within the given range. In this scenario its red
        # CHANGE TO YOUR VALUES

        mask = cv2.inRange(self.hsv_image, (70,0,145),(140,120,255))
        # mask2 = cv2.inRange(self.hsv_image, (13,0,154),(33,255,255))
        # mask = cv2.bitwise_or(mask,mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        image_dilate = cv2.dilate(image_erode, kernel2)
        self.white_mask = image_dilate
        #Converting image from cv format to ROS msg using 
        white_mask = self.bridge.cv2_to_imgmsg(self.white_mask, "mono8")
        #publishing cropped image
        self.pub_white_mask.publish(white_mask)


    def yellow_filter(self):
        # CHANGE VALUES IF NEEDED
        #Finds all pixels within the given range. In this scenario its red
        mask = cv2.inRange(self.hsv_image, (20,33,100), (86,176,222))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        image_dilate = cv2.dilate(image_erode, kernel2)
        self.yellow_mask = image_dilate
        #Converting image from cv format to ROS msg using 
        white_mask = self.bridge.cv2_to_imgmsg(self.yellow_mask, "mono8")
        #publishing cropped image

        self.pub_yellow_mask.publish(white_mask)

    def red_filter(self):
        # CHANGE VALUES IF NEEDED
        #Finds all pixels within the given range. In this scenario its red
        mask = cv2.inRange(self.hsv_image, (0,101,191), (21,179,255))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        image_dilate = cv2.dilate(image_erode, kernel2)
        self.red_mask = image_dilate
        #Converting image from cv format to ROS msg using 
        white_mask = self.bridge.cv2_to_imgmsg(self.red_mask, "mono8")
        #publishing cropped image

        self.pub_red_mask.publish(white_mask)

    def segment(self):
        #Convert image from BGR to HSV
        # CHANGE VALUES TO YOUR VALUES

        edges = cv2.Canny(self.cv_crop, 100, 200)

        #Combinining edges with mask white 
        edges_white = cv2.bitwise_and(edges,self.white_mask)
        ros_edges = self.bridge.cv2_to_imgmsg(edges_white, "mono8")
        self.pub_white_lines.publish(ros_edges)

        #Combinining edges with mask yellow 
        edges_yellow = cv2.bitwise_and(edges,self.yellow_mask)
        ros_edges = self.bridge.cv2_to_imgmsg(edges_yellow, "mono8")
        self.pub_yellow_lines.publish(ros_edges)

        edges_red = cv2.bitwise_and(edges,self.red_mask)
        ros_edges = self.bridge.cv2_to_imgmsg(edges_red, "mono8")
        self.pub_red_lines.publish(ros_edges)

        
        #Finding Line Segments
        # CHANGE VALUES TO YOUR VALUES
        hough_white = cv2.HoughLinesP(edges_white, 1, np.pi / 180, 10,minLineLength=20,maxLineGap=3)
        
        # CHANGE VALUES IF NEEDED 
        hough_yellow = cv2.HoughLinesP(edges_yellow, 1, np.pi / 180, 0,minLineLength=20,maxLineGap=10)

        hough_red = cv2.HoughLinesP(edges_red, 1, np.pi / 180, 0,minLineLength=20,maxLineGap=10)

        # DO NOT CHANGE
        #Publish and  Segments
        final_lines = self.output_lines(self.cv_crop,hough_white,hough_yellow,hough_red)
        ros_lines = self.bridge.cv2_to_imgmsg(final_lines, "bgr8")
        self.pub_lines.publish(ros_lines)
        self.convert_lines(hough_white,hough_yellow,hough_red)


    def convert_lines(self,lines_white,lines_yellow,lines_red):
        if lines_white is not None:
            lines_reshaped =lines_white.reshape((-1,4))
        else:
            lines_reshaped =[]
        if lines_yellow is not None:
            lines_reshaped_yellow =lines_yellow.reshape((-1,4))
        else:
            lines_reshaped_yellow =[]
        if lines_red is not None:
            lines_reshaped_red =lines_red.reshape((-1,4))
        else:
            lines_reshaped_red =[]
        offset = 40
        image_size = (160,120)
        arr_cutoff = np.array([0,offset, 0, offset])
        arr_ratio = np.array([([1. /image_size[0], 1. /image_size[1], 1. /image_size[0], 1. /image_size[1]])])
        if lines_white is not None:
            line_normalized_white = (lines_reshaped + arr_cutoff) * arr_ratio
        else:
            line_normalized_white= np.empty(shape= [0,0,])
            
        if lines_yellow is not None:
            line_normalized_yellow = (lines_reshaped_yellow + arr_cutoff) * arr_ratio
        else:
            line_normalized_yellow= np.empty(shape= [0,0,])

        if lines_red is not None:
            line_normalized_red = (lines_reshaped_red + arr_cutoff) * arr_ratio
        else:
            line_normalized_red= np.empty(shape= [0,0,])
        
        Line_List = SegmentList()
        Line_Array = []
        # print(line_normalized_white)
        for i in range(line_normalized_white.shape[0]):
        # for i in range(1):

            line = Segment()
            line.color = line.WHITE
            Initial_Line = Vector2D(line_normalized_white[i,0], line_normalized_white[i,1] )
            End_Line = Vector2D(line_normalized_white[i,2], line_normalized_white[i,3] )
            line.pixels_normalized[0] = Initial_Line
            line.pixels_normalized[1] = End_Line
            Line_Array.append(line)
        for i in range(line_normalized_yellow.shape[0]):
        # for i in range(1):

            line = Segment()
            line.color = line.YELLOW
            Initial_Line = Vector2D(line_normalized_yellow[i,0], line_normalized_yellow[i,1] )
            End_Line = Vector2D(line_normalized_yellow[i,2], line_normalized_yellow[i,3] )
            line.pixels_normalized[0] = Initial_Line
            line.pixels_normalized[1] = End_Line
            Line_Array.append(line)

        for i in range(line_normalized_red.shape[0]):
        # for i in range(1):

            line = Segment()
            line.color = line.RED
            Initial_Line = Vector2D(line_normalized_red[i,0], line_normalized_red[i,1] )
            End_Line = Vector2D(line_normalized_red[i,2], line_normalized_red[i,3] )
            line.pixels_normalized[0] = Initial_Line
            line.pixels_normalized[1] = End_Line
            Line_Array.append(line)
            cb_segments(line)

        Line_List.segments= Line_Array
        Line_List.header.stamp = rospy.Time.now()
        self.pub_segments.publish(Line_List)


    def output_lines(self, original_image, lines,lines_yellow,lines_red):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        if lines_yellow is not None:
            for i in range(len(lines_yellow)):
                l = lines_yellow[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,255,255))
        if lines_red is not None:
            for i in range(len(lines_red)):
                l = lines_red[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,255,255))
        return output
       
       
    
    def ld_switch(self,msg):
        return True,""
    
    def lf_switch(self,msg):
        return True,""

    def cb_segments(self, segment_list_msg):


        good_seg_count = 0
        stop_line_x_accumulator = 0.0
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:  # the point is behind us
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])
            avg_x = 0.5 * (p1_lane[0] + p2_lane[0])
            avg_y = 0.5 * (p1_lane[1] + p2_lane[1])

            # If the line is more than max_y offset in the y direction then it is
            # not a stop line in our lane and we shouldn't count it
            if np.abs(avg_y) > self.max_y.value:
                continue
            stop_line_x_accumulator += avg_x
            good_seg_count += 1.0

        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.header.stamp = segment_list_msg.header.stamp
        if good_seg_count < self.min_segs.value:
            stop_line_reading_msg.stop_line_detected = False
            stop_line_reading_msg.at_stop_line = False
            self.pub_stop_line_reading.publish(stop_line_reading_msg)

        else:
            stop_line_reading_msg.stop_line_detected = True
            stop_pose = Pose2D()
            stop_pose.x = - stop_line_x_accumulator / good_seg_count
            stop_pose.y = self.lane_pose.d
            stop_pose.theta = self.lane_pose.phi
            stop_line_reading_msg.stop_pose = stop_pose

            # Only detect redline if y is within max_y distance:
            stop_line_reading_msg.at_stop_line = \
                -stop_pose.x < self.stop_distance.value

            self.pub_stop_line_reading.publish(stop_line_reading_msg)
            if stop_line_reading_msg.at_stop_line:
                msg = BoolStamped()
                msg.header.stamp = stop_line_reading_msg.header.stamp
                msg.data = True
                self.pub_at_stop_line.publish(msg)

        #rospy.set_param("stop",stop_line_reading_msg.stop_line_detected)

    def to_lane_frame(self, point):
        p_homo = np.array([point.x, point.y, 1])
        phi = self.lane_pose.phi
        d = self.lane_pose.d
        T = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), d], [0, 0, 1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new


        

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("ImageProcessor", anonymous=True)
    ImageProcessor()
    rospy.spin()