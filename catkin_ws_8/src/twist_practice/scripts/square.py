#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import Twist   
from std_msgs.msg import Float32
from numpy import pi 

#This class will make the puzzlebot move following a square 

class SquareClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.17 #wheel separation [m] 

        self.d=0 #distance  
        self.theta=0 #angle 

        vel=Twist() #Robot's speed 

        self.wR=0.0 
        self.wL=0.0 
        
        self.t = float(0)
        d_des = 1.0                        # 1 m
        theta_des = pi/2        # 90 degs  1.0471975512 1.57079632679 1.2853981634 3.14159265359
        self.flag, self.state, self.c = 1, 0, 0
        error = 0.0001
        
        self.d_d_des = 0.0
        self.t_t_des = 0.0
        
        ###******* INIT PUBLISHERS *******###  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("wl", Float32, self.wL_cb)  
        rospy.Subscriber("wr", Float32, self.wR_cb)  

        #********** INIT NODE **********###  

        freq = 100 

        rate = rospy.Rate(freq) # freq Hz  

        dT = 1/float(freq) #Dt is the time between one calculation and the next one 

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
        
        
            if (self.c != 8):
            
                if   (self.flag == 1):
                
                    if (self.state == 0 and self.d >= d_des):
                        self.flag, self.state, self.d, self.theta = 0, 1, 0.0, 0.0
                        self.c += 1
                    elif (self.state == 1 and self.theta >= theta_des):
                        self.flag, self.state, self.d, self.theta = 0, 0, 0.0, 0.0
                        self.c += 1
                    elif (self.state == 0):
                        vel = Twist()
                        vel.linear.x  = 0.05
                    elif (self.state == 1):
                        vel = Twist()
                        vel.angular.z = 0.05
                
                elif (self.flag == 0):
                
                    if ((self.wR >= 0.0-error and self.wR <= 0.0+error) and (self.wL >= 0.0-error and self.wL <= 0.0+error)): self.flag, self.d, self.theta = 1, 0.0, 0.0
                    vel = Twist()
                else:
                    pass
                
            else:
            
                vel = Twist()
            
            v=r*(self.wR+self.wL)/2 
            w=r*(self.wR-self.wL)/L 

            self.d=v*dT+self.d 
            self.theta=w*dT+self.theta 
            
            self.d_d_des = self.d
            self.t_t_des = self.theta

            self.pub_cmd_vel.publish(vel) #publish the robot's speed  

            print("V:          " + str(v)) 
            print("W:          " + str(w)) 
            print("D:          " + str(self.d)) 
            print("THETA:      " + str(self.theta)) 
            print("dT:         " + str(dT)) 
            print("d_LAST:     " + str(self.d_d_des))
            print("theta_LAST: " + str(self.t_t_des))

            rate.sleep()  

    def wL_cb(self, wL):  
        ## This function receives a number   
        self.wL = wL.data 

    def wR_cb(self, wR):  
        ## This function receives a number.  
        self.wR = wR.data  

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        vel=Twist()
        self.pub_cmd_vel.publish(vel) #publish the robot's speed  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("square", anonymous=True)  

    SquareClass()
