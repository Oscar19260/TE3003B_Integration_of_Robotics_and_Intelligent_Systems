#!/usr/bin/env python  

import rospy 
import numpy as np
 
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

#This class will make the puzzlebot move following a square 

class JCClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        self.d=0 #distance
        self.dx=0 #distance in x
        self.dy=0 #distance in y
        self.theta=0 #angle 

        vel=Twist() #Robot's speed 

        self.wR=0.0 
        self.wL=0.0
        
        # Last activity
        
        self.f_v = 0.0
        self.f_w = 0.0
        
        self.action = "None"
        
        # desired goals
        
        #goals = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]
        goals = [[0.0, 1.0]]
        #goal_dx, goal_dy = -1.0, 0.0
        
        self.index = 0
        
        self.flag, self.counter = 0, 0
        
        ###******* INIT PUBLISHERS *******###  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("wl", Float32, self.wL_cb)  
        rospy.Subscriber("wr", Float32, self.wR_cb)  
        rospy.Subscriber("action", String, self.action_cb)  

        #********** INIT NODE **********###  

        freq = 20.0 

        rate = rospy.Rate(freq) # freq Hz  

        dT = 1/float(freq) #Dt is the time between one calculation and the next one 

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
            
            try:
                goal = goals[self.counter]
                self.index = self.counter
            except:
                goal = goals[self.index]
                self.flag = 1
                #self.action = "stop" 
            
            v=r*(self.wR+self.wL)/2 
            w=r*(self.wR-self.wL)/L 

            self.d      = v*dT + self.d 
            self.dx     = v*dT*np.cos(self.theta) + self.dx
            self.dy     = v*dT*np.sin(self.theta) + self.dy
            self.theta  = w*dT + self.theta
            self.etheta = np.arctan2(goal[1]-self.dy, goal[0]-self.dx) - self.theta
            self.etheta = np.arctan2(np.sin(self.etheta), np.cos(self.etheta))
            self.ed     = np.sqrt((goal[0] - self.dx)**2 + (goal[1] - self.dy)**2)
            
            #Kd, Kt = 0.1, 0.25 -- Kt, alpha, kmax = 0.6, 1.0, 0.7
            alpha   = 2.0
            kmax    = 0.4
            Kd = kmax * ((1.0-np.exp(-alpha*(abs(self.ed))**2))/(abs(self.ed)))
            #Kd = 0.1
            Kt = 0.5
            
            self.f_v = Kd * self.ed
            self.f_w = Kt * self.etheta
            
            max_v, max_w = 0.82, 9.0
            
            if (self.f_v >= max_v): self.f_v = max_v
            if (self.f_v <= -max_v): self.f_v = -max_v
            #if (self.f_w >= max_w): self.f_w = max_w
            #if (self.f_w <= -max_w): self.f_w = -max_w
            
            if (self.ed >= 0 and self.ed <= 0.2): self.counter+=1
            else:
                if (self.action == "forward"):
                    vel = Twist()
                    vel.linear.x, vel.angular.z    = self.f_v, self.f_w
                elif (self.action == "stop"): vel = Twist()
                elif (self.action == "None"): pass
                else: pass
            
            if (self.flag == 1): vel, self.f_v, self.f_w, self.flag = Twist(), 0.0, 0.0, 0
            
            self.pub_cmd_vel.publish(vel) #publish the robot's speed  

            print("                         ")
            #print("vel:          " + str(vel))
            print("flag:         " + str(self.flag)) 
            #print("V:          " + str(v)) 
            #print("W:          " + str(w)) 
            #print("D:          " + str(self.d)) 
            #print("Dx:         " + str(self.dx)) 
            #print("Dy:         " + str(self.dy))  
            #print("dT:         " + str(dT)) 
            print("F_v:         " + str(self.f_v)) 
            print("F_w:         " + str(self.f_w))
            #print("THETA:      " + str(self.theta))
            print("==============================")
            print("ed    : " + str(self.ed))
            print("etheta: " + str(self.etheta))
            print("Action  : ", self.action)
            print("Goal(x)   : ", goal[0])
            print("Goal(y)   : ", goal[1])

            rate.sleep()

    def wL_cb(self, wL):  
        ## This function receives a number   
        self.wL = wL.data 

    def wR_cb(self, wR):  
        ## This function receives a number.  
        self.wR = wR.data  
    
    def action_cb(self, msg):  
        ## This function receives a string (action)   
        self.action = msg.data 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        vel=Twist()
        self.pub_cmd_vel.publish(vel) #publish the robot's speed  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("jetson_control_rn", anonymous=True)  

    JCClass()
