#!/usr/bin/env python  

import rospy 
import numpy as np
 
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import String

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
        
        self.colour = "green"
        self.line = "line"
        self.id_prediction = "69"
        
        self.colour_received        = False
        self.line_received          = False
        self.center_received        = False
        self.id_prediction_received = False
        
        self.turn_on_off_delay = False
        self.x = 0
        self.y = 0
        self.z = 0
        
        self.cs = 0
        self.past_id = "3"
        
        ###******* INIT PUBLISHERS *******### 
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # cmd_vel publisher

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("center", Int32, self.center_cb)  # center subscriber
        rospy.Subscriber("red_string", String, self.colour_cb)
        rospy.Subscriber("line", String, self.line_cb)
        rospy.Subscriber("prediction", String, self.prediction_cb)   

        #********** INIT NODE **********###  

        freq = 20.0     # frequency

        rate = rospy.Rate(freq) # freq Hz

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
        
            if (self.center_received == True and self.colour_received == True and self.line_received == True):            # remove as True -- and self.id_prediction_received == True
            
                a = self.x
                b = self.y
                c = self.z
                
                p_id = self.past_id
                
                change_s = self.cs
                
                self.colour_received        = False
                self.line_received          = False
                self.center_received        = False
                self.id_prediction_received = False
                
                d_line  = self.line
                id_pred = self.id_prediction
                color   = self.colour
            
                xm = 1280.0/2.0  # Inicializamos nuestra referencia/centro de la imagen/origen
                
                w = xm - self.center_x  # w control
                
                if (change_s == 0): 
                    Kv, Kw = 1.0, 0.00055    # Constantes para control P propuestas -- 0.00055 -- 0.0006
                    v = 0.08                # v control          --- 0.08 -- 0.1
                
                if (change_s == 1):
                    Kv, Kw = 1.0, 0.0006
                    v = 0.1                # v control          --- 0.08 -- 0.1
                
                vel.linear.x    = Kv * v       # Control P para velocidad lineal
                vel.angular.z   = Kw * w      # Control P para velocidad angular
                
                    
                if (d_line == "line" and b == 0):
                
                    print("HAY LINEA")
                
                    self.turn_on_off_delay = False
                    
                    if (color == "green"):
                    
                        print("VEO VERDE, SIGUE")
                        
                        if (id_pred == "1" and (p_id == "3" or p_id == "1")):
                            self.cs = 1
                            print("PRED FUE NSL, CAMBIA SPEED")
                    
                    elif (color == "red" and c == 0):
                        print("VEO ROJO ANTES DE GIRAR A LA DERECHA, PARATE")
                        vel = Twist()
                    
                    
                elif (d_line == "no_line" or b == 1):
                
                    print("NO HAY LINEA")
                    
                    self.turn_on_off_delay = False
                    
                    if (id_pred == "0" and (p_id == "2" or p_id == "0")):
                        print("PRED FUE STOP, TERMINALO")
                        vel = Twist()
                        print(" ")
                        print("Ya termine TODO WE")
                        print(" ")
                        break
                    elif (id_pred == "3" and p_id == "3"):
                        print("PRED FUE AHEAD, SIGUETE")
                        pass
                    elif (id_pred == "2" and (p_id == "0" or p_id == "1" or p_id = "2")):
                    
                        print("ENTRA A GIRAR en " + str(a))
                    
                        self.turn_on_off_delay = True
                        self.y = 1
                    
                        if (a == 0):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.x = 1
                            print("S0")
                            pass
                        
                        if (a == 1):
                            vel.linear.x, vel.angular.z, delay = 0.08, 0, 4.3
                            self.x = 2
                            print("S1")
                            pass
                            
                        if (a == 2):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.x = 3
                            print("S2")
                            pass
                        
                        if (a == 3):
                            vel.linear.x, vel.angular.z, delay = 0, -0.08, 4
                            self.x = 4
                            print("S3")
                            pass
                            
                        if (a == 4):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.x = 5
                            print("S4")
                            pass
                            
                        if (a == 5):
                            vel.linear.x, vel.angular.z, delay = 0.08, 0, 2.3
                            self.x = 6
                            print("S5")
                            pass
                            
                        if (a == 6):
                            print("S6")
                            self.y = 0
                            self.z = 1
                            delay = 0
                            self.turn_on_off_delay = False
                            pass  
                
                self.pub_cmd_vel.publish(vel) #publish the robot's speed
                
                if (self.turn_on_off_delay == True): rospy.sleep(delay)
                        
                        ## Imprimir variables requeridas para verificacion
                print("                         ")
                print("F_v          :" + str(self.f_v)) 
                print("F_w          :" + str(self.f_w))
                print("==============================")
                print("velX         :" + str(vel.linear.x))
                print("velZ         :" + str(vel.angular.z))
                print("==============================")
                print("centerX      :" + str(self.center_x))
                print("==============================")
                print("colour       :" + str(self.colour))
                print("==============================")
                print("line_no_line :" + str(d_line))
                print("==============================")
                print("id_pred      :" + str(id_pred))
                print("==============================")
                print("a            :" + str(a) + " " + str(self.x))
                print("b            :" + str(b) + " " + str(self.y))
                print("c            :" + str(c) + " " + str(self.z))
                
            rate.sleep()

    def center_cb(self, msg):  
        ## This function receives the position of the middle of the center line   
        self.center_x = msg.data
        self.center_received = True
        
    def colour_cb(self, msg_ac):  
        ## This function receives a string (colour)   
        self.colour = msg_ac.data
        self.colour_received = True
        
    def line_cb(self, msg_ac):  
        ## This function receives a string (line)   
        self.line = msg_ac.data
        self.line_received = True
        
    def prediction_cb(self, msg_ac):  
        ## This function receives a string (prediction)   
        self.id_prediction = msg_ac.data
        self.id_prediction_received = True

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
