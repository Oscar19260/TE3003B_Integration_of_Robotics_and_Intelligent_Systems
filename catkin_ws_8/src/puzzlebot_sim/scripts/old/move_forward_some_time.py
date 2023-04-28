#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist  
#This class will publish the speed to the /cmd_vel topic to make the robot move for some period of time 
# Then stops

class MoveFClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  

        ###******* INIT PUBLISHERS *******###  

        # create the publisher to cmd_vel topic 

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 

        my_twist = Twist() # create a twist message, fill in the details

         

        period = 5.0 #How much time the robot will move [seconds] 

        rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 

        rospy.loginfo("About to be moving forward!") 

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        start_time = rospy.get_time()  #Get the current time in float seconds

     

        while not rospy.is_shutdown(): 

             

            #if (rospy.get_time() - start_time) <= period: # If we haven't reached the desired "period" of time [s] then move. 

                #rospy.loginfo("moving forward!") 

                # Fill in the message with the required data 

                # If we move at 0.2 m/s for 5.0 seconds we will move in the end 1m.  

                #my_twist.linear.x = 0.2   # our forward speed in [m/s]. (0.2[m/s]*5[s]) = 1[m] 

                #my_twist.angular.z = 0    # Our angular speed in [rad/s], (In this case the robot does not rotate) 
                
                #sto =  rospy.get_time()                     

            #elif ((rospy.get_time() - start_time) > period and (rospy.get_time() - sto) <= period ):
                
                #rospy.loginfo("turning!")
                #print(sto)
                #print(rospy.get_time()) 

                #my_twist.linear.x = 0.0   # our angular speed in [rad/s]. (pi/10[rad/s]*5[s]) = 1[m] 
                #my_twist.angular.z = np.pi/10.0
                
            
            if (rospy.get_time() - start_time) <= period: # If we haven't reached the desired "period" of time [s] then move. 

                rospy.loginfo("turning!") 

                my_twist.linear.x = 0.0
                my_twist.angular.z = np.pi/10.0     # our angular speed in [rad/s]. (pi/10[rad/s]*5[s]) = pi/2[rad]
                
            else:

                rospy.loginfo("stop!") 

                my_twist.linear.x = 0.0    # our forward speed in [m/s]; 0 => stops the robot. 
                my_twist.angular.z = 0.0

            self.cmd_vel_pub.publish(my_twist) # Send the speed to the robot 

            # wait enough time to keep the required rate (50Hz) 

            rate.sleep() 

    

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.  

        print("Stopping the robot") 

        print("Bye bye!!!")
        
        my_twist = Twist()
        self.cmd_vel_pub.publish(my_twist)
 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    # first thing, init a node! 

    rospy.init_node('move_forward_some_time')  

    MoveFClass()

