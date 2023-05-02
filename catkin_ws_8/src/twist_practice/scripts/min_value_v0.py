#!/usr/bin/env python  

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#Class desc

class IMGProcessingNodeClass():  

    def __init__(self):
    
        #rospy.on_shutdown(self.cleanup)
        
        ############ PRINCIPAL CONSTANTS & VARIABLES ################  
        
        self.image_received = 0 #Flag to indicate that we have already received an image 

        self.blobs  = 0  # Number of red and green circles
        self.colour = "none"  # Flag used to identify the colour of the circle detected \
                    # (none, red, green)
                    # (0, 1, 2)
                    
        self.per_g, self.per_r = 0.0, 0.0
        self.action = "continue"
        
        self.msg_string = String() #Action
        self.msg_string.data = "continue"
        
        ###******* INIT PUBLISHERS *******###  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        
        #self.msg_pub = rospy.Publisher('action', String, queue_size=1)  
        self.img_pub = rospy.Publisher('image_viewer', Image, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("video_source/raw", Image, self.img_cb) 

        #********** INIT NODE **********###  
        
        self.bridge = CvBridge()

        freq = 20.0 

        rate = rospy.Rate(freq) # freq Hz  

        print("Node initialized " + str(freq) + " hz") 

        while not rospy.is_shutdown():
            
            if self.image_received:
                #
                ## Activity: Pre-processing
                
                image_x = self.msg_img
            
                cv_image = self.bridge.imgmsg_to_cv2(image_x, "bgr8")
                
                #per = 50
                #r_img = cv2.resize(cv_image, (int(cv_image.shape[1]*per/100), int(cv_image.shape[0]*per/100)))
                
                    # Convert BGR to GRAY
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)     # change image to work with
                    # Apply Gaussian blur
                blur_gi = cv2.GaussianBlur(gray_image,(7,7),0)
                
                dims = blur_gi.shape
                
                    # Crop it (rectangulo del fondo, desde 2/3)
                cp_img = blur_gi[int(dims[0]*2/3):,:]
                
                #cp_img_f = self.bridge.cv2_to_imgmsg(cp_img, "mono8")

                #self.img_pub.publish(cp_img_f)
                
                cv2.imshow('robot_image', cp_img)
            
            cv2.waitKey(1)
            
            rate.sleep()  

    def img_cb(self, msg_image):
        
        ## This function receives a ROS image
        try: 
            #print("received ROS image")
            self.msg_img = msg_image
            self.image_received = 1 #Turn the flag on 
        except CvBridgeError as e: 
            print(e)


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("min_value_v0", anonymous=True)  
    IMGProcessingNodeClass()
