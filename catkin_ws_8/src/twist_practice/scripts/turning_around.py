#!/usr/bin/env python 

import rospy 
from std_msgs.msg import Int32  
from geometry_msgs.msg import Twist

#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 

class TurnClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_msg = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        
        ############ CONSTANTS ################ 
        self.f_msg = Twist() # Final message
        
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        print("Node initialized 1hz")
        
        while not rospy.is_shutdown(): 
            self.f_msg.angular.z = 0.5 # Counterclockwise --> Z+
            self.pub_msg.publish(self.f_msg) #publish the instruction
            print(self.f_msg)
            r.sleep()
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        self.f_msg = Twist()
        self.pub_msg.publish(self.f_msg) #publish the instruction

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("turning_around", anonymous=True) 
    TurnClass() 
    
