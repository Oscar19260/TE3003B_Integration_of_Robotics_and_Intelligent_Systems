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
        self.f = 0
        self.g = 0
        self.h = 0
        
        self.k = 0
        
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
                
            fg = False
            
            a = self.f
            b = self.g
            c = self.h
            d = self.k
            
            p = self.past_id
        
            if (self.center_received == True and self.colour_received == True and self.line_received == True):            # remove as True  and self.id_prediction_received == True
                
                self.colour_received        = False
                self.line_received          = False
                self.center_received        = False
                self.id_prediction_received = False
                
                a = self.f
                b = self.g
                c = self.h
                d = self.k
                
                p = self.past_id
                
                d_line  = self.line
                id_pred = self.id_prediction
                color = self.colour
            
                xm = 1280.0/2.0  # Inicializamos nuestra referencia/centro de la imagen/origen
                
                v = 0.08                # v control          --- 0.08 -- 0.1
                w = xm - self.center_x  # w control
                
                Kv, Kw = 1.0, 0.00055    # Constantes para control P propuestas -- 0.00055 -- 0.0006
                
                self.f_v = Kv * v       # Control P para velocidad lineal
                self.f_w = Kw * w      # Control P para velocidad angular
                
                    
                if (d_line == "line" and b == 0):
                
                    print("a")
                    self.turn_on_off_delay = False

                    self.f = 0
                    self.g = 0
                    
                    if (color == "green"):
                        print("b")
                        
                        if (id_pred == "1" and (p == "1" or p == "0")):
                            
                            v = 0.1                # v control
                            w = xm - self.center_x
                            
                            Kv = 1.0
                            Kw = 0.0006            # constante angular proporcional
                            
                            self.f_v = Kv * v      # Control P para velocidad lineal
                            self.f_w = Kw * w      # Control P para velocidad angular
                            print("ENTRO 1 PAI")
                            #print(str(self.f_v) + " " + str(self.f_w))
                            print("c")
                            
                        vel.linear.x, vel.angular.z = self.f_v, self.f_w
                    
                    elif (color == "red" and c == 0):
                        print("Entro WE")
                        vel = Twist()
                        print("d")
                    else: 
                        vel.linear.x, vel.angular.z = self.f_v, self.f_w
                        print("e")
                    
                    
                elif (d_line == "no_line" or b == 1):
                    
                    self.turn_on_off_delay = False
                
                    vel = Twist()
                    print("f")
                    
                    if (id_pred == "0" and (p == "2")):
                        vel = Twist()
                        print(" ")
                        print("Ya termine WE")
                        print(" ")
                        break
                        print("g")
                    elif (id_pred == "3"):
                        """
                        self.turn_on_off_delay = True
                    
                        if (d == 0):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 1
                            self.k = 1
                            print("A")
                            pass
                        
                        if (d == 1):
                            vel.linear.x, vel.angular.z, delay = 0.08, 0, 9.0
                            self.k = 2
                            print("B")
                            pass
                            
                        if (d == 2):
                            self.k = 0
                            print("C")
                            self.g = 0
                            self.turn_on_off_delay = False
                            delay = 0
                            pass
                        """
                        self.g = 0
                        print("h")
                        
                        vel.linear.x, vel.angular.z = self.f_v, self.f_w
                        
                    elif ((id_pred == "2") and (p == "2" or p == "1" or p == "0")):
                        print("i")
                    
                        print("Entro 2")
                    
                        self.g = 1
                    
                        self.turn_on_off_delay = True
                    
                        if (a == 0):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.f = 1
                            print("a")
                            pass
                        
                        elif (a == 1):
                            vel.linear.x, vel.angular.z, delay = 0.08, 0, 3.3
                            self.f = 2
                            print("b")
                            pass
                            
                        elif (a == 2):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.f = 3
                            print("c")
                            pass
                        
                        elif (a == 3):
                            vel.linear.x, vel.angular.z, delay = 0, -0.08, 4
                            self.f = 4
                            print("d")
                            pass
                            
                        elif (a == 4):
                            vel.linear.x, vel.angular.z, delay = 0, 0, 2
                            self.f = 5
                            print("e")
                            pass
                            
                        elif (a == 5):
                            vel.linear.x, vel.angular.z, delay = 0.08, 0, 2.3
                            self.f = 6
                            print("f")
                            pass
                            
                        elif (a == 6):
                            self.turn_on_off_delay = False
                            print("h")
                            self.f = 0
                            self.g = 0
                            self.h = 1
                            delay = 0
                            pass 
                            
                self.past_id = d_line 
                
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
                print("colour       :" + str(color))
                print("==============================")
                print("line_no_line :" + str(d_line))
                print("==============================")
                print("id_pred      :" + str(id_pred))
                print("b            :" + str(b))
                print("a            :" + str(a))
                
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
