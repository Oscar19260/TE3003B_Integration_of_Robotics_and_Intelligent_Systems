#!/usr/bin/env python 

import rospy 
import numpy as np
from std_msgs.msg import String  
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

# Review Act 0 

class ReviewControlClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ###******* INIT PUBLISHERS *******### 
        self.pub_msg = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("string_topic", String, self.string_c_cb) 
        rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_states_cb)
        
        ############ CONSTANTS ################ 
        self.string_command = "None" #  The string needed to execute an spec. com.
        self.f_msg = []         #  Final message
        
        self.awas_msg = [0]*3
        
        #********** INIT NODE **********### 
        r = rospy.Rate(10) #10Hz 
        print("Node initialized 10hz")
        
        period = 5.0 # How much time the robot will move [seconds]
        
        flag = 0
        sto, sto_2 = 0.0, 0.0
        
        while not rospy.is_shutdown(): 
            print(" ")
            print(self.awas_msg[0])
            print(self.awas_msg[1])
            r.sleep()

            
    def string_c_cb(self, string_command): 
        ## This function receives a string command  
        self.string_command = string_command.data
        
    def gazebo_states_cb(self, data):
	    aux_idx = data.name.index("puzzlebot")
	    #print(" ")
	    #print(data.pose[aux_idx].position.x)
	    #print(data.pose[aux_idx].orientation.x)
	    
	    self.awas_msg[0] = data.pose[aux_idx].position.x
	    self.awas_msg[1] = data.pose[aux_idx].orientation.x
        
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
    
