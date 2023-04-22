#!/usr/bin/env python 

import rospy 
import numpy as np
from std_msgs.msg import String  
from geometry_msgs.msg import Twist

# Review Act 0 

class ReviewControlClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ###******* INIT PUBLISHERS *******### 
        self.pub_msg = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("string_topic", String, self.string_c_cb) 
        
        ############ CONSTANTS ################ 
        self.string_command = "None" #  The string needed to execute an spec. com.
        self.f_msg = Twist()         #  Final message
        
        #********** INIT NODE **********### 
        r = rospy.Rate(10) #10Hz 
        print("Node initialized 10hz")
        
        period = 5.0 # How much time the robot will move [seconds]
        
        flag = 0
        sto, sto_2 = 0.0, 0.0
        
        while not rospy.is_shutdown():
        
            self.f_msg = Twist() 
            
            if   (self.string_command == "MF"):
                self.f_msg.linear.x = 0.1
                flag = 0
            elif (self.string_command == "MB"):
                self.f_msg.linear.x = -0.1
                flag = 0
            elif (self.string_command == "TR"):
                self.f_msg.angular.z = -0.5
                flag = 0
            elif (self.string_command == "TL"):
                self.f_msg.angular.z = 0.5
                flag = 0
            elif (self.string_command == "STOP"):
                pass
            elif (self.string_command == "EXTRA"):
                
                #"""
                
                print("SI EXTRA")
                
                if flag == 0:
                    flag = 1
                    start_time = rospy.get_time()
                
                if (rospy.get_time() - start_time) <= period:
                
                    print("mf")
                    
                    self.f_msg.linear.x = 0.2           # Our forward speed in [m/s]. (0.2[m/s]*5[s]) = 1[m]
                    self.f_msg.angular.z = 0     
                    
                    sto =  rospy.get_time()                     

                elif ((rospy.get_time() - start_time) > period and (rospy.get_time() - sto) <= period ):

                    print("t")
                    
                    self.f_msg.linear.x = 0.0   
                    self.f_msg.angular.z = np.pi/5.0    # Our angular speed in [rad/s]. (pi/5[rad/s]*5[s]) = pi[rad] ---> Media vuelta
                    
                    sto_2 = rospy.get_time()   
                    
                elif ((rospy.get_time() - start_time) > period and (rospy.get_time() - sto) > period and (rospy.get_time() - sto_2) <= period):
                    
                    print("mf_2")
                    
                    self.f_msg.linear.x = 0.2           # Our forward speed in [m/s]. (0.2[m/s]*5[s]) = 1[m]
                    self.f_msg.angular.z = 0.0
                    
                else:
                    
                    print("stop")
                    
                    self.f_msg.linear.x = 0.0    # Full Stop 
                    self.f_msg.angular.z = 0.0
                    
                #"""
                    
                #self.f_msg.linear.x     = 0.5
                #self.f_msg.angular.z    = 2
                
            else:
                pass
                        
            self.pub_msg.publish(self.f_msg) #publish the final message 
            
            print(self.f_msg)
            print("String command:")
            print("  "+ self.string_command)
            r.sleep()

            
    def string_c_cb(self, string_command): 
        ## This function receives a string command  
        self.string_command = string_command.data
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        self.f_msg = Twist()                # STOP
        self.pub_msg.publish(self.f_msg)    # publish the STOP instruction
        
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("dass_robot_control", anonymous=True) 
    ReviewControlClass()
    
