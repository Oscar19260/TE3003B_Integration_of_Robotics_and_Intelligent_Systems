#!/usr/bin/env python 

import rospy 
from std_msgs.msg import String  
from geometry_msgs.msg import Twist

#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 

class ControlClass(): 
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
        
        while not rospy.is_shutdown(): 
            
            if   (self.string_command == "MF"):
                self.f_msg = Twist()
                self.f_msg.linear.x = 0.1
            elif (self.string_command == "MB"):
                self.f_msg = Twist()
                self.f_msg.linear.x = -0.1
            elif (self.string_command == "TR"):
                self.f_msg = Twist()
                self.f_msg.angular.z = -0.5
            elif (self.string_command == "TL"):
                self.f_msg = Twist()
                self.f_msg.angular.z = 0.5
            elif (self.string_command == "Stop"):
                self.f_msg = Twist()
            elif (self.string_command == "EXTRA"):
                self.f_msg = Twist()
                self.f_msg.linear.x     = 0.5
                self.f_msg.angular.z    = 2
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
        self.f_msg = Twist()
        self.pub_msg.publish(self.f_msg) # publish the STOP instruction
        
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("dass_robot_control", anonymous=True) 
    ControlClass() 
    
 
