#!/usr/bin/env python  

import rospy 
import numpy as np
 
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point

#This class will make the puzzlebot move following a red ball/object
class ColorFollowerClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ CONSTANTS ################

        vel = Twist() #Robot's speed
        
        self.f_v = 0.0  # Final linear velocity
        self.f_w = 0.0  # Final angular velocity
        
        self.flag = 0   # Flag to identify when we are close enough to the desired object
        
        self.center_x   = 600.0/2.0 # Center of the blob inside the acquired image
        self.radius     = 150.0     # Radius variable
        
        ###******* INIT PUBLISHERS *******### 
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel publisher

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("center", Point, self.center_cb)  # center subscriber
        rospy.Subscriber("radius", Int32, self.radius_cb)  # radius subscriber

        #********** INIT NODE **********###  

        freq = 10.0     # frequency

        rate = rospy.Rate(freq) # freq Hz

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
            
            xm = 600.0/2.0  # Inicializamos nuestra referencia/centro de la imagen/origen
            
            if (self.radius <= 0.0): self.radius = 150  # Evitar division por cero
            
            v = 1.0/self.radius     # Variable de control para modular la velocidad lineal si el radio es pequeno o grande
            w = xm - self.center_x  # Variable de control para modular la velocidad angular si la distancia entre el centro del blob y el centro de la imagen es pequena o grande
            
            Kv, Kw = 5.0, 0.006     # Constantes para control P propuestas
            
            self.f_v = Kv * v       # Control P para velocidad lineal
            self.f_w = Kw * w       # Control P para velocidad angular
            
            if (self.radius >= 150): self.flag = 1  # Asegurar que el robot se detenga a una distancia definida
            else:
                vel = Twist()
                vel.linear.x, vel.angular.z    = self.f_v, self.f_w     # Pasar los valores de velocidad al objeto tipo Twist 
            
            if (self.flag == 1): vel, self.f_v, self.f_w, self.flag = Twist(), 0.0, 0.0, 0  # Si se detecta que el robot esta suficientemente cerca se detiene
            
            self.pub_cmd_vel.publish(vel) #publish the robot's speed
                    
                    ## Imprimir variables requeridas para verificacion
            print("                         ")
            #print("vel:          " + str(vel))
            print("flag:         " + str(self.flag)) 
            #print("V:          " + str(v)) 
            #print("W:          " + str(w)) 
            #print("D:          " + str(self.d)) 
            #print("Dx:         " + str(self.dx)) 
            #print("Dy:         " + str(self.dy))  
            #print("dT:         " + str(dT)) 
            print("==============================")
            print("F_v:         " + str(self.f_v)) 
            print("F_w:         " + str(self.f_w))
            #print("THETA:      " + str(self.theta))
            print("==============================")
            print("velX:       " + str(vel.linear.x))
            print("velZ:       " + str(vel.angular.z))
            print("==============================")
            print("centerX:      " + str(self.center_x))
            print("radius:       " + str(self.radius))
            #print("Action       : ", self.action)
            #print("Goal(x)      : ", goal[0])
            #print("Goal(y)      : ", goal[1])

            rate.sleep()

    def center_cb(self, msg):  
        ## This function receives a Point   
        self.center_x = msg.x

    def radius_cb(self, msg):  
        ## This function receives the radius  
        self.radius = msg.data

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
