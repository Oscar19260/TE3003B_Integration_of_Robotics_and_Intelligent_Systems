#!/usr/bin/env python  

import rospy  
import numpy as np 
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist 

# This class receives a LaserScan, finds the closest object and avoid tries to avoid it

class AvoidObstacleClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

        vel_msg = Twist() 

        ############ CONSTANTS ################  

        kv=0.4 #Constant to change the linear speed 

        kw=0.333 #Angular velocity gain 

        self.closest_range = 0.0 #Distance to the closest object 

        self.closest_angle = 0.0 #Angle to the closest object 

        #********** INIT NODE **********###  

        r = rospy.Rate(10) #10Hz is the lidar's frequency  

        print("Node initialized 1hz") 

        while not rospy.is_shutdown():  

            range_c = self.closest_range 
            ed = abs(range_c)
            theta   = self.closest_angle 

            #limit the angle to [-pi,pi] 

            theta       = np.arctan2(np.sin(theta),np.cos(theta))
            theta_ao    = theta - np.pi
            theta_ao    = np.arctan2(np.sin(theta_ao),np.cos(theta_ao)) 

            if np.isposinf(range_c):

                print(" ")
                print("No object detected")
                print(" ") 

                vel_msg.linear.x    = 0.0
                vel_msg.angular.z   = 0.0 

            else:
                if (ed >= 1.0): vel_msg.linear.x, vel_msg.angular.z = kv, 0.0
                else:
                    if (ed <= 0.19*2): vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0
                    else:
                        if (theta >= -np.pi/2 and theta <= np.pi/2):
                            vel_msg.linear.x    = kv
                            vel_msg.angular.z   = kw*theta_ao
                        else: vel_msg.linear.x, vel_msg.angular.z = kv, 0.0

                #print("I'm working") 

            self.cmd_vel_pub.publish(vel_msg) 
            
            print("                                            ")
            #print("c_range              : " + str(range_c))
            print("ed                   : " + str(ed))
            print("theta                : " + str(theta))
            #print("theta_ao             : " + str(theta_ao))
            print("vel_msg.linear.x     : " + str(vel_msg.linear.x)) 
            print("vel_msg.angular.z    : " + str(vel_msg.angular.z))
            print("===================================================") 

            r.sleep()  

             

             

             

    def laser_cb(self, msg):  

        ## This function receives a number   

        #For hls lidar  

        closest_range = min(msg.ranges)

        idx = msg.ranges.index(closest_range) 

        closest_angle = msg.angle_min + idx * msg.angle_increment

        self.closest_range = closest_range 

        self.closest_angle = closest_angle 

        #print("closest object distance: " + str(closest_range)) 

        #print("closest object direction: " + str(closest_angle)) 

         

    def cleanup(self):  
    
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    

        vel_msg = Twist() 

        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("avoid_obstacle", anonymous=True)  

    AvoidObstacleClass() 
