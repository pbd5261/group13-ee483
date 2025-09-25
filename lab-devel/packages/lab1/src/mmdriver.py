#!/usr/bin/env python3

import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm
import time

class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher(f"/{self.veh_name}/wheels_driver_node/wheels_cmd",WheelsCmdStamped, queue_size = 10)
        # USING PARAMETER TO GET THE NAME OF THE VEHICLE
        # THIS WILL BE USEFUL TO SPECIFY THE NAME OF THE TOPIC
        
        # INITIALIZE YOUR VARIABLES HERE (SUBSCRIBERS OR PUBLISHERS)
        self.state = "init"
        self.turn_number = 0
        self.stop_time = 5
        self.forward_time = 2.25
        self.turn_time = 1.25
    
    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION
        
        #WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        cmd_1 = WheelsCmdStamped()
        cmd_1.header.stamp = rospy.Time.now()

        if self.state == "init":
            rospy.sleep(10)
            cmd_1.vel_left = 0
            cmd_1.vel_right = 0
            rospy.sleep(self.forward_time)
            self.state = "forward"
        elif self.state == "stopped":
            rospy.sleep(self.forward_time)
            cmd_1.vel_left = 0
            cmd_1.vel_right = 0
            
            self.state = "turn"

        elif self.turn_number >= 3:
            cmd_1.vel_right = 0
            cmd_1.vel_left = 0
            self.state = "end"

        elif self.state == "forward":
            rospy.sleep(self.turn_time)
            cmd_1.vel_right = 0.5
            cmd_1.vel_left = 0.5
            
            self.state = "stopped"

        elif (self.state == "turn") and (self.turn_number < 3):
            rospy.sleep(self.stop_time)
            cmd_1.vel_right = -0.4
            cmd_1.vel_left = 0.4
            
            self.turn_number = self.turn_number + 1
            self.state = "forward"

        

        print("Current State:" + self.state) # Just for testin
        self.pub.publish(cmd_1)
        
if __name__ == "__main__": ## The main function which will be called when your python sc
    # Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        # Keep the line above - you might be able to reduce the delay a bit,
        while not rospy.is_shutdown(): # Run ros forever - you can change
            # this as well instead of running forever
            drive.drive() # calling your node function
    except rospy.ROSInterruptException:
        pass