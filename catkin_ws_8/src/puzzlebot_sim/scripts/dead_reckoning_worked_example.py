#!/usr/bin/env python

from tf.transformations import quaternion_from_euler

import numpy as np

np.set_printoptions(suppress=True) 

np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

############ ROBOT CONSTANTS ################  

r = 0.065 #puzzlebot wheel radius [m] or 0.05
L = 0.19 #puzzlebot wheel separation [m] or 0.18

############ Variables ############### 

vk, wk  = 1.0, 1.0
delta_t = 0.1 # [s]

wr, wl = 0.0, 0.0

x_act, x_ant          = 0.0, 0.0
y_act, y_ant          = 0.0, 0.0
theta_act, theta_ant  = 0.0, 0.0

kr, kl = 0.01, 0.01

############ MATRIXES ###############

U_act, U_ant = np.zeros([3,1]), np.zeros([3,1])

Sigma_act, Sigma_ant = np.zeros([3,3]), np.zeros([3,3])

Qk = np.array([[0.5 , 0.01, 0.01], 
               [0.01,  0.5, 0.01], 
               [0.01, 0.01,  0.2]])
               
times = 2

print("")
print("==================================")
    
print("\nU(%i):" %(0))
print(U_act)

print("")
print("Qk:")
print(Qk)

print("")
print("E_sigma(%i):" %(0))
print(Sigma_act)
print("")

for i in range(0,times):
    
    ############ U (miu) ROBOT POSE   ################

    x_act = x_ant + (vk * np.cos(theta_ant) * delta_t)

    y_act = y_ant + (vk * np.sin(theta_ant) * delta_t) 

    theta_act = theta_ant + (wk * delta_t)
    
    #theta = np.arctan2(np.sin(theta), np.cos(theta)) #Make theta from -pi to pi
    
    
    U_act[0] = x_act
    U_act[1] = y_act
    U_act[2] = theta_act
    
    ############ H ################
    
    Hk = np.array([[1, 0, -delta_t * vk * np.sin(theta_ant)], 
                   [0, 1, delta_t * vk * np.cos(theta_ant)], 
                   [0, 0, 1]])
                   
    ############ E (sigma) COVARIANCE POSE MATRIX ################
    
    Sigma_act = Hk.dot(Sigma_ant).dot(Hk.T) + Qk
    
    ############ UPDATE VALUES   ################ 
    
    x_ant, y_ant, theta_ant = x_act, y_act, theta_act
    Sigma_ant = Sigma_act
    
    
    
    print("==================================")
    
    print("\nU(%i):" %(i+1))
    print(U_act)
    
    print("")
    print("H(%i):" %(i+1))
    print(Hk)
    
    print("")
    print("E_sigma(%i):" %(i+1))
    print(Sigma_act)
    print("")
    
