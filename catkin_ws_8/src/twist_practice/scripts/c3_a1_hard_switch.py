#!/usr/bin/env python  

import rospy  

import numpy as np 

from sensor_msgs.msg import LaserScan   

from geometry_msgs.msg import Twist 

# This class implements a simple obstacle avoidance algorithm 

class HardSwitchControlClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

 

        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  

        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  
        rospy.Subscriber("ao_vel", Twist, self.ao_cb) 
        rospy.Subscriber("gtg_vel", Twist, self.gtg_cb) 

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

         

        ######################## CONSTANTS AND VARIABLES ##############################  

        self.laser_received             = False 
        self.control_received           = False
        self.avoid_obstacle_received    = False

        #v_desired  = 0.4 #[m/s] desired speed when there are no obstacles 

        self.closest_angle = 0.0 #Angle to the closest object 

        self.closest_range = np.inf #Distance to the closest object 

        vel_msg = Twist() 
        
        ## Speeds from topics
        
        self.ao_msg     = Twist()
        self.gtg_msg    = Twist()

        rate = rospy.Rate(10) #10Hz is the lidar's frequency  

        print("Node initialized 1hz") 

        ############################### MAIN LOOP ##################################### 

        while not rospy.is_shutdown():  

            ############################### YOUR CODE HERE ############################## 

            if self.laser_received and self.avoid_obstacle_received and self.control_received:

                self.laser_received = False
                clst_r = min(self.lidar_msg.ranges)
                
                vel_msg = Twist() 
                
                if (clst_r <= 1.0):
                    vel_msg.linear.x, vel_msg.angular.z = self.ao_msg.linear.x, self.ao_msg.angular.z
                    flag = "avoid"
                else:
                    vel_msg.linear.x, vel_msg.angular.z = self.gtg_msg.linear.x, self.gtg_msg.angular.z
                    flag = "go to goal"
                
                print(" ")
                print("clsr_r : " + str(clst_r))
                print("goal   : " + flag)
                print("================")

            self.cmd_vel_pub.publish(vel_msg)

            rate.sleep()  


    def laser_cb(self, msg):  

        ## This function receives a message of type LaserScan and computes the closest object direction and range 

        self.lidar_msg = msg 

        self.laser_received = True 

    def ao_cb(self, msg):
        self.ao_msg = msg
        self.avoid_obstacle_received = True
        
    def gtg_cb(self, msg):
        self.gtg_msg = msg
        self.control_received = True

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        vel_msg = Twist() 

        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("hard_switch_control", anonymous=True)  

    HardSwitchControlClass() 

 
