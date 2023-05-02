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

        #kv = 0.000005 #Constant to change the linear speed

        kw = 0.3 # Angular speed gain 
        
        self.msg = LaserScan()

        #********** INIT NODE **********###  

        r = rospy.Rate(50) #100 Hz is the lidar's frequency  

        print("Node initialized 100 hz") 

        while not rospy.is_shutdown():
            
            ranges          = self.msg.ranges
            a_min, a_inc    = self.msg.angle_min, self.msg.angle_increment
            r_max           = self.msg.range_max
            
            dt, theta_t = self.vw_control(ranges, a_min, a_inc, r_max)
            
            kvmax = 0.3 # Linear speed maximum gain
            alpha = 1.0 # Constant to adjust the exponential's growth rate
            
            kv = kvmax*(1-np.exp(-alpha*dt**2))/dt
            
            v = kv * dt
            w = kw * theta_t
            
            vel_msg = Twist()
            
            vel_msg.linear.x    = v
            vel_msg.angular.z   = w

            self.cmd_vel_pub.publish(vel_msg)
            
            print("                                            ")
            #print("===================================================") 
            print("dt                   : " + str(dt))
            print("theta_t              : " + str(theta_t))
            print("kv                   : " + str(kv))
            print("kw                   : " + str(kw))
            #print("===================================================")
            print("vel_msg.linear.x     : " + str(vel_msg.linear.x)) 
            print("vel_msg.angular.z    : " + str(vel_msg.angular.z))
            print("===================================================") 

            r.sleep()

    def laser_cb(self, msg):
    
        ## This function receives a LaserScan message and saves it
        self.msg = msg
        
    def vw_control(self, ranges, angle_min, angle_increment, range_max):
        
        ranges = list(ranges)
        angles = np.zeros_like(ranges)
        xi = np.zeros_like(ranges)
        yi = np.zeros_like(ranges)
        
        for i in range(0, len(ranges)):
            if np.isposinf(ranges[i]): ranges[i] = range_max
            angles[i] = angle_min + i * angle_increment
            #if (angles[i] >= -np.pi/2 and angles[i] <= np.pi/2):
            xi[i] = ranges[i]*np.cos(angles[i])
            yi[i] = ranges[i]*np.sin(angles[i])
        
        xt = np.sum(xi)
        yt = np.sum(yi)
        
        theta_t = np.arctan2(yt, xt)
        dt      = np.sqrt(xt**2 + yt**2)
        
        return dt, theta_t

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
