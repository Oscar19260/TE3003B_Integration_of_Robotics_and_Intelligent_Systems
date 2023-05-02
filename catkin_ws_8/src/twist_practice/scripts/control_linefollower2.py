#!/usr/bin/env python  

import rospy 
import numpy as np
 
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

#This class will make the puzzlebot move following a red ball/object
class ColorFollowerClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ CONSTANTS ################

        vel = Twist() #Robot's speed
        
        self.f_v = 0.0  # Final linear velocity
        self.f_w = 0.0  # Final angular velocity
        
        self.flag = 0   # Flag to identify when we are close enough to the desired object
        
        self.center_x   = 1280.0/2.0 # Center of the blob inside the acquired image
        
        ###******* INIT PUBLISHERS *******### 
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel publisher

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("center", Int32, self.center_cb)  # center subscriber

        #********** INIT NODE **********###  

        freq = 20.0     # frequency

        rate = rospy.Rate(freq) # freq Hz

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
            
            xm = 1280.0/2.0  # Inicializamos nuestra referencia/centro de la imagen/origen
            
            v = 0.02                # v control
            w = xm - self.center_x  # w control
            
            Kv, Kw = 1.0, 0.0002    # Constantes para control P propuestas
            
            self.f_v = Kv * v       # Control P para velocidad lineal
            self.f_w = Kw * w       # Control P para velocidad angular
            
            vel = Twist()
            vel.linear.x, vel.angular.z    = self.f_v, self.f_w     # Pasar los valores de velocidad al objeto tipo Twist
            
            self.pub_cmd_vel.publish(vel) #publish the robot's speed
                    
                    ## Imprimir variables requeridas para verificacion
            print("                         ")
            print("F_v          :" + str(self.f_v)) 
            print("F_w          :" + str(self.f_w))
            print("==============================")
            print("velX         :" + str(vel.linear.x))
            print("velZ         :" + str(vel.angular.z))
            print("==============================")
            print("centerX      :" + str(self.center_x))

            rate.sleep()

    def center_cb(self, msg):  
        ## This function receives the position of the middle of the center line   
        self.center_x = msg.data

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        vel=Twist()
        self.pub_cmd_vel.publish(vel) #publish the robot's speed  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("color_follower", anonymous=True)  

    ColorFollowerClass()
