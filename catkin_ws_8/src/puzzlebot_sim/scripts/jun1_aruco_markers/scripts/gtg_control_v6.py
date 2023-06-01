#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from numpy import pi, sin, cos, arctan2, sqrt, exp

#This class will make the puzzlebot move following a square 

class GTGClass():  

    def __init__(self):  
        
        rospy.on_shutdown(self.cleanup) 
        
        ############ CONSTANTS ################
        
        self.vector_d = Vector3()

        self.d=0 #distance
        self.dx=0 #distance in x
        self.dy=0 #distance in y
        self.theta=0 #angle 

        vel=Twist() #Robot's speed
        
        # Last activity
        
        self.f_v = 0.0
        self.f_w = 0.0
        
        #goals = [ [0.35, 2.8], [0.38, 3.86], [2.0, 2.8], [1.5, 0.2], [0.60, 0.15] ]
        #goals = [ [0.55, 0.80] ]
        goals = [ [2.06, 1.50], [0.36, 2.70], [2.10, 3.58], [1.19, 4.17], [2.10, 3.58], [2.39, 0.0000000000001] ]   #2.06, 1.50
        self.id_goal_act    = 0
        self.id_goal_prev   = -1
        
        self.flag_id_change = 0
        self.flag = "Go to Goal"
        
        delay = 2.0
        goal_reached = 1
        actual_time = 0.0
        
        ###******* INIT PUBLISHERS *******###
          
        self.pub_gtg_vel = rospy.Publisher('gtg_vel', Twist, queue_size=1)
        self.pub_ed      = rospy.Publisher('ed', Float32, queue_size=1)
        self.pub_ed_theta      = rospy.Publisher('ed_theta', Float32, queue_size=1) 
        
        self.pub_goal_x     = rospy.Publisher('goal_x', Float32, queue_size=1)
        self.pub_goal_y     = rospy.Publisher('goal_y', Float32, queue_size=1)
        
        self.pub_xp         = rospy.Publisher('xp', Float32, queue_size=1)
        self.pub_yp         = rospy.Publisher('yp', Float32, queue_size=1)
        
        self.start_pos_pub   = rospy.Publisher('start_pos', Vector3, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("est_pose_robot", Vector3, self.vd_cb)
        rospy.Subscriber("start_pos_zero", Vector3, self.sp_cb)
        
        #################### VARIABLES AND CONSTANTS ####################
        
        self.vector_sp_zero = Vector3()
        start_pos           = Vector3()

        #********** INIT NODE **********###  

        freq = 50 

        rate = rospy.Rate(freq) # freq Hz  

        dT = 1/float(freq) #Dt is the time between one calculation and the next one 

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
            
            ### Actual GOAL selected  ###
            
            id_now  = self.id_goal_act
            id_prev = self.id_goal_prev
            
            print(id_now)
            print(id_prev)
            print("")
            
            try:
                ## ENOUGH GOALS
                goal_dx, goal_dy = goals[id_now][0], goals[id_now][1]
                
                th_id_sp = id_now-1
                if (th_id_sp < 0):
                    sp_act_dx, sp_act_dy = self.vector_sp_zero.x, self.vector_sp_zero.y
                else:
                    sp_act_dx, sp_act_dy = goals[th_id_sp][0], goals[th_id_sp][1]
                
                self.flag = "Go to Goal"
                
                if (id_prev != id_now):
                    self.flag_id_change = 1
                    print("ID_DIFF")
                    print("")
                
                self.id_goal_prev = id_now
                
            except:
                ## NOT ENOUGH GOALS (outside len of goals) --> AFTER END
                self.flag = "Stop"
                
                ### DEFAULT VALUES ###
                val = 0.00001
                
                self.dx     = val
                self.dy     = val
                self.theta  = val
                
                goal_dx, goal_dy        = val, val
                sp_act_dx, sp_act_dy    = val, val
                
                edo = val
                self.etheta = val
                self.ed = val
                
            
            if (self.flag == "Stop"):
                
                vel, self.f_v, self.f_w = Twist(), 0.0, 0.0
                
            else:
                
                ### Acquire GTG_dxS values ###
                d_gtg = self.vector_d
                
                self.dx     = d_gtg.x
                self.dy     = d_gtg.y
                self.theta  = d_gtg.z
                
                vel = Twist()
            
                ### Errors and P control ###
                self.etheta = arctan2(goal_dy-self.dy, goal_dx-self.dx) - self.theta
                self.etheta = arctan2(sin(self.etheta), cos(self.etheta))
                self.ed     = sqrt((goal_dx - self.dx)**2 + (goal_dy - self.dy)**2)
                
                edo = abs(self.ed)
                
                print(abs(self.etheta))
                print("")
                
                #Kl, Kt = 0.1, 0.25
                #Kt, alpha, kmax = 1.6, 0.5, 0.2 # 0.5, 1.0, 0.2
                
                ######
                # Angular
                Kt          = 0.3 # 0.5 - 1.6, 0.4
                # Linear
                alpha, kmax = 1.1, 0.12 # 1.0, 0.15
                ######
                            
                Kl = kmax * ((1.0-exp(-alpha*(abs(edo))**2))/(abs(edo)))
                
                self.f_v = Kl * self.ed
                self.f_w = Kt * self.etheta
                
                ### Limit angular velocity ###
                
                #thresh_fw = 0.15
                
                #if (self.f_w >= thresh_fw): self.f_w = thresh_fw
                #if (self.f_w <= -thresh_fw): self.f_w = -thresh_fw
                
                #print(vel.linear.x)
                #print(vel.angular.z)
                
                ### Rotate then do usual ###
                
                thresh_theta = pi/8
                
                print(thresh_theta)
                print("")
                
                if (self.flag_id_change == 1):
                    print("F_ID_CHANGE = 1")
                    if ( abs(self.etheta) > 0.0 and abs(self.etheta) < thresh_theta ):
                        self.flag_id_change = 0
                        print("WELL ALIGNED")
                    else:
                        print("JUST FW")
                        self.f_v = 0.0
                        
                        ### Limit angular velocity ###
                
                        thresh_fw = 0.2
                        
                        if (self.f_w >= thresh_fw): self.f_w = thresh_fw
                        if (self.f_w <= -thresh_fw): self.f_w = -thresh_fw
                        
                else:
                    print("F_ID_CHANGE = 0")
                    pass
                    
                #print(vel.linear.x)
                #print(vel.angular.z)
            
                ### Errors and P control ###
                
                if (self.ed >= 0 and self.ed <= 0.10):  # IF CLOSE ENOUGH TO THE GOAL --> CHANGE ID TO NEXT GOAL
                    if(goal_reached == 1):
                        time_goal = rospy.get_time()
                        goal_reached = 0
                        
                    if ((actual_time - time_goal) >= delay):
                        self.id_goal_act +=1
                        goal_reached = 1
                    else:
                        actual_time = rospy.get_time()
                    
                else:                                   # ELSE JUST USE v AND w velocities for GTG calculated
                    vel.linear.x, vel.angular.z = self.f_v, self.f_w
                    
                #print(vel.linear.x)
                #print(vel.angular.z)
            
            # pub goals and actual positions
            
            start_pos = Vector3()
            start_pos.x = sp_act_dx
            start_pos.y = sp_act_dy
            
            self.start_pos_pub.publish(start_pos)
            
            self.pub_goal_x.publish(goal_dx)
            self.pub_goal_y.publish(goal_dy)
            self.pub_xp.publish(self.dx)
            self.pub_yp.publish(self.dy)
            
            ###
            
            self.pub_gtg_vel.publish(vel) # publish the robot's speed
            self.pub_ed.publish(edo)      # publish the distance between robot and goal
            self.pub_ed_theta.publish(self.etheta)
            
            #self.pub_cmd_vel.publish(vel) #publish the robot's speed  

            print("                         ")
            print("GOAL X, Y       :    " + str(goal_dx) + ", " + str(goal_dy))
            #print("vel:          " + str(vel))
            print("flag:         " + self.flag) 
            #print("V:          " + str(v)) 
            #print("W:          " + str(w)) 
            #print("D:          " + str(self.d)) 
            #print("Dx:         " + str(self.dx)) 
            #print("Dy:         " + str(self.dy))  
            #print("dT:         " + str(dT)) 
            print("F_v:         " + str(self.f_v)) 
            print("F_w:         " + str(self.f_w))
            #print("THETA:      " + str(self.theta))
            print("etheta:     " + str(self.etheta))
            print("ed:          " + str(self.ed))
            print("============================")

            rate.sleep()  
            
            if (self.flag == "Stop"): exit()

    def sp_cb(self, msg):
        self.vector_sp_zero = msg
    
    def vd_cb(self, msg):
        self.vector_d = msg

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        vel=Twist()
        self.pub_gtg_vel.publish(vel) #publish the robot's speed  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("gtg_control", anonymous=True)  

    GTGClass()
