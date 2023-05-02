#!/usr/bin/env python  

import rospy  

import numpy as np 

from sensor_msgs.msg import LaserScan   
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

# This class implements a simple obstacle avoidance algorithm 

class WallFollowingControlClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

 

        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  

        rospy.Subscriber("scan", LaserScan, self.laser_cb) #base_scan
        rospy.Subscriber("gtg_vel", Twist, self.gtg_cb)
        rospy.Subscriber("ed", Float32, self.ed_cb)
        rospy.Subscriber("ed_theta", Float32, self.ed_theta_cb) 

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

         

        ######################## CONSTANTS AND VARIABLES ##############################  

        self.laser_received             = False 
        self.control_received           = False
        self.avoid_obstacle_received    = False
        self.ed_received                = False
        self.ed_theta_received          = False
        self.flag_wf                    = False
        self.flag                       = "go to goal"

        #v_desired  = 0.4 #[m/s] desired speed when there are no obstacles 

        self.closest_angle = 0.0 #Angle to the closest object 

        self.closest_range = np.inf #Distance to the closest object 

        vel_msg = Twist() 
        
        ## Speeds from topics
        
        self.ao_msg     = Twist()
        self.gtg_msg    = Twist()
        self.ed_msg     = Float32()
        self.ed_theta_msg     = Float32()
        
        epsilon     = 0.3
        self.ed_tau = 0.0

        rate = rospy.Rate(10) #10Hz is the lidar's frequency  

        print("Node initialized 1hz") 

        ############################### MAIN LOOP ##################################### 

        while not rospy.is_shutdown():  

            ############################### YOUR CODE HERE ############################## 

            if self.laser_received and self.control_received and self.ed_received and self.ed_theta_received:

                self.laser_received = False
                dfw = min(self.lidar_msg.ranges)
                
                vel_msg = Twist()
                
                # Closest range and theta_ao
                
                l_msg = self.lidar_msg
                    
                closest_range = min(l_msg.ranges)                               # closest_range

                idx = l_msg.ranges.index(closest_range) 

                closest_angle = l_msg.angle_min + idx * l_msg.angle_increment   # closest_angle
                theta = closest_angle
                
                theta       = np.arctan2(np.sin(theta),np.cos(theta))
                theta_ao    = theta - np.pi
                
                theta_ao    = np.arctan2(np.sin(theta_ao),np.cos(theta_ao))     # theta_ao
                
                vel_gtg     = self.gtg_msg.linear.x
                w_gtg       = self.gtg_msg.angular.z
                
                theta_gtg = self.ed_theta_msg.data
                
                if (dfw <= 0.5):
                    theta_fwc   = -np.pi/2 + theta_ao
                    theta_fwcc  = np.pi/2 + theta_ao
                    
                    theta_fwc   = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))
                    theta_fwcc  = np.arctan2(np.sin(theta_fwcc), np.cos(theta_fwcc))
                    
                    if (self.flag_wf == False):
                        if (abs(theta_fwc - theta_gtg) <= np.pi/2): self.flag = "wall following - clockwise"
                        else: self.flag = "wall following - counter-clockwise"
                        self.flag_wf = True
                        
                        self.ed_tau = self.ed_msg.data      # dGTG(tau)
                        
                    if (self.flag == "wall following - clockwise"): theta_fwf = theta_fwc
                    else: theta_fwf = theta_fwcc
                    
                    if ((abs(theta_ao - theta_gtg) < np.pi/2) and (self.ed_msg.data < abs(self.ed_tau - epsilon))):
                        vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg
                        self.flag       = "go to goal"
                        self.flag_wf    = False
                    else:
                        Kw = 1.0
                        vel_msg.linear.x, vel_msg.angular.z = 0.1, Kw * theta_fwf
                
                else:
                    vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg
                    self.flag       = "go to goal"
                    self.flag_wf    = False
                
                
                print(" ")
                print("vel_x        : " + str(vel_msg.linear.x))
                print("vel_z        : " + str(vel_msg.angular.z))
                print("dfw          : " + str(dfw))
                print("goal         : " + self.flag)
                print("ed_TAU       : " + str(self.ed_tau))
                print("ed_actual    : " + str(self.ed_msg.data))
                print("================")

            self.cmd_vel_pub.publish(vel_msg)

            rate.sleep()  


    def laser_cb(self, msg):  

        ## This function receives a message of type LaserScan and computes the closest object direction and range 

        self.lidar_msg = msg 

        self.laser_received = True
        
    def gtg_cb(self, msg):
        self.gtg_msg = msg
        self.control_received = True
        
    def ed_cb(self, msg):
        self.ed_msg = msg
        self.ed_received = True
        
    def ed_theta_cb(self, msg):
        self.ed_theta_msg = msg
        self.ed_theta_received = True

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        vel_msg = Twist() 

        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("wall_following", anonymous=True)  

    WallFollowingControlClass() 

 
