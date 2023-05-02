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
        
        blended_distance    = 1.5   # distance to activate the blended controller [m]
        ao_distance         = 0.7   # distance to activate the obstacle avoidance [m]
        stop_distance       = 0.15  # distance to stop the robot [m]
        
        flag = "stop"

        rate = rospy.Rate(10) #10Hz is the lidar's frequency  

        print("Node initialized 10hz") 

        ############################### MAIN LOOP ##################################### 

        while not rospy.is_shutdown():  

            ############################### YOUR CODE HERE ############################## 

            if self.laser_received and self.avoid_obstacle_received and self.control_received: 

                self.laser_received = False
                clst_r = min(self.lidar_msg.ranges)
                
                vel_msg = Twist()
                    
                if (clst_r > blended_distance): vel_msg.linear.x, vel_msg.angular.z, flag = self.gtg_msg.linear.x, self.gtg_msg.angular.z, "go to goal"
                elif (clst_r > ao_distance and clst_r <= blended_distance):
                    
                    alpha   = 0.2
                    v_blend = alpha * self.ao_msg.linear.x  + (1 - alpha) * self.gtg_msg.linear.x
                    w_blend = alpha * self.ao_msg.angular.z + (1 - alpha) * self.gtg_msg.angular.z
                    
                    vel_msg.linear.x, vel_msg.angular.z, flag = v_blend, w_blend, "blended controller"
                    
                elif (clst_r > stop_distance and clst_r <= ao_distance): vel_msg.linear.x, vel_msg.angular.z, flag = self.ao_msg.linear.x, self.ao_msg.angular.z, "avoid"
                elif (clst_r <= stop_distance): vel_msg, flag = Twist(), "stop"
                
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

 
