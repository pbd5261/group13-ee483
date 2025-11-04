#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class lanedetector:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("ee483mm13/camera_node/image/compressed", CompressedImage, self.callback, queue_size = 1, buff_size = 1000000)
        self.pub = rospy.Publisher("final_image", Image, queue_size=10)


    def callback(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #Getting image height
        height = cv_img.shape[0]
        # Calculate the midpoint
        midpoint = int(height * 0.3)
        # Crop the image to keep only the bottom 70%
        cropped_img = cv_img[midpoint:, :]
      
        #Convert image from BGR to HSV
        hsv_image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        #Finds all pixels within the given range. In this scenario its red
        white_mask = cv2.inRange(hsv_image, (70,0,145),(140,120,255))
        yellow_mask = cv2.inRange(hsv_image, (21,0,97), (97,174,198))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # Kernel size
        image_erode = cv2.erode(yellow_mask, kernel)
        # Apply erosion to image
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        yellow_dilate = cv2.dilate(image_erode, kernel)


        cv_img_canny = cv2.Canny(cropped_img, 50,400)

        #combine image edge detecton image with mask
        edges_white = cv2.bitwise_and(cv_img_canny,white_mask)
        edges_yellow = cv2.bitwise_and(cv_img_canny,yellow_dilate)

        #publish image
        #ros_image_edges = self.bridge.cv2_to_imgmsg(output_image_edges, "mono8")
        #self.pub.publish(ros_image_edges)
        
        #perform hough transform on combined images
        lines_white = cv2.HoughLinesP(edges_white,1,np.pi/180,10,minLineLength=2,maxLineGap=10)
        lines_yellow = cv2.HoughLinesP(edges_yellow,1,np.pi/180,10,minLineLength=2,maxLineGap=10)

        #draw lines on Hough transform 
        img_combined_white = self.output_lines(cropped_img,lines_white, "white")
        img_combined_white_yellow = self.output_lines(img_combined_white,lines_yellow, "yellow")

        #publish image
        ros_image_final = self.bridge.cv2_to_imgmsg(img_combined_white_yellow, "bgr8")
        self.pub.publish(ros_image_final)

    def output_lines(self, original_image, lines, color):
            output = np.copy(original_image)
            if lines is not None:
                for i in range(len(lines)):
                    l = lines[i][0]
                    if color == "white":
                        cv2.line(output, (l[0],l[1]),(l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                    elif color == "yellow":
                        cv2.line(output, (l[0],l[1]),(l[2],l[3]), (255,255,0), 2, cv2.LINE_AA)
                    
                    cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                    cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
            return output



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("white_filter", anonymous=True)
    img_filter = lanedetector()
    rospy.spin()