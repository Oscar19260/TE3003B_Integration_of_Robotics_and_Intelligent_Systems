#!/usr/bin/env python  

import rospy  
import numpy as np
   
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


#Class desc

class ClosestDetectorClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ PRINCIPAL CONSTANTS & VARIABLES ################  
        
        self.clst_range, self.clst_index, self.clst_angle, self.angle_min, self.ranges, self.msg, self.intensities = 0.0, 0, 0.0, 0.0, [0], LaserScan(), [0]
        
        self.linear_vel, self.angular_vel, ed, self.flag = 0.0, 0.0, 1, 0
        
        ###******* INIT PUBLISHERS *******###  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_gtg_vel = rospy.Publisher('gtg_vel', Twist, queue_size=1)   

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("base_scan", LaserScan, self.LS_cb) 

        #********** INIT NODE **********###  
        
        vel = Twist()

        freq = 10.0 

        rate = rospy.Rate(freq) # freq Hz  

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
        
            self.ranges             = self.msg.ranges
            self.angle_min          = self.msg.angle_min
            self.angle_increment    = self.msg.angle_increment
            self.intensities        = self.msg.intensities
            
            if (len(self.ranges) == 0): self.ranges, self.angle_min, self.angle_increment, self.clst_range, self.clst_index, self.clst_angle, self.linear_vel, self.angular_vel = "EMPTY", "EMPTY", "EMPTY", "EMPTY", "EMPTY", "EMPTY", "EMPTY", "EMPTY"
            else:
                self.clst_range = min(self.ranges)
                self.clst_index = self.ranges.index(self.clst_range)
                self.clst_angle = self.angle_min + self.clst_index * self.angle_increment
                self.clst_angle = np.arctan2(np.sin(self.clst_angle), np.cos(self.clst_angle))
                
                ed = abs(self.clst_range)
                
                #Kl, Kt = 0.1, 0.25
                Kt, alpha, kmax = 0.3, 1.0, 0.3
                Kl = kmax * ((1.0-np.exp(-alpha*(abs(ed))**2))/(abs(ed)))
                
                self.linear_vel, self.angular_vel = Kl*self.clst_range, Kt*self.clst_angle
                
                vel.linear.x, vel.angular.z = self.linear_vel, self.angular_vel
                
                if np.isposinf(self.clst_range): vel = Twist()
                
                if (ed >= 0 and ed <= 1.0): vel.linear.x = 0.0
                
                self.pub_cmd_vel.publish(vel)
                 
            
            #"""
            print("                         ")
            print("=========================")
            #print("ranges:          " + str(self.ranges)) 
            #print("angle_min:       " + str(self.angle_min)) 
            #print("angle_inc:       " + str(self.angle_increment)) 
            print("linear_vel  :    " + str(self.linear_vel)) 
            print("angular_vel :    " + str(self.angular_vel)) 
            #print("                         ")
            print("clst_RANGE  :    " + str(self.clst_range)) 
            #print("clst_index:      " + str(self.clst_index))
            print("clst_ANGLE  :    " + str(self.clst_angle))
            print("ed          :    " + str(ed))
            print("flag        :    " + str(self.flag))
            print("=========================")
            #"""

            rate.sleep()  

    def LS_cb(self, LS_msg):  
        ## This function receives a LaserScan message and saves it
        self.msg = LS_msg

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        vel = Twist()
        self.pub_cmd_vel.publish(vel) #publish the robot's speed

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("closest_object_detector", anonymous=True)  
    ClosestDetectorClass()
