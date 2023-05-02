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

        # Canny threshold values

        l_t = 0
        u_t = 255

        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars',800,500)

        cv2.createTrackbar('lower_threshold_value','Trackbars',0,255,self.track)
        cv2.createTrackbar('upper_threshold_value','Trackbars',0,255,self.track)

        cv2.setTrackbarPos('lower_threshold_value','Trackbars',l_t)
        cv2.setTrackbarPos('upper_threshold_value','Trackbars',u_t)
        
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

                # Trackbars

                low_t = cv2.getTrackbarPos('lower_threshold_value','Trackbars')
                upp_t = cv2.getTrackbarPos('upper_threshold_value','Trackbars')

                #
                ## Activity: Pre-processing
                
                image_x = self.msg_img
            
                cv_image = self.bridge.imgmsg_to_cv2(image_x, "bgr8")
                
                    # Rotate 180 degrees clockwise
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

                    # Resize image
                #cv_image = cv2.resize(cv_image, (480,480))

                #per = 50
                #r_img = cv2.resize(cv_image, (int(cv_image.shape[1]*per/100), int(cv_image.shape[0]*per/100)))
                
                    # Convert BGR to GRAY
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)     # change image to work with
                    # Apply Gaussian blur
                blur_gi = cv2.GaussianBlur(gray_image,(7,7),0)
                
                dims = blur_gi.shape
                
                    # Crop it (rectangulo del fondo, desde 2/3 de la imagen)
                cp_img = blur_gi[int(dims[0]*2/3):,:]

                    # Sum by columns
                b = np.sum(cp_img, axis=0)

                #print(" ")		
                #print(b)

                    # Find index of minimum value
                result = np.where(b == np.amin(b))
                try:
                    a = result[0][1]
                except:
                    a = result[0][0]
                idx_r  = int(a)

                #print(idx_r)

                #img_wc = cv2.circle(cp_img, (idx_r, int(cp_img.shape[0]/2)), 10, (255,255,255), -1) # Print circle in oimg
                
                img_canny = cv2.Canny(cv_image, low_t, upp_t)

                cp_img_f = self.bridge.cv2_to_imgmsg(img_canny, "mono8")

                self.img_pub.publish(cp_img_f)
                
                #cv2.imshow('robot_image', cp_img)
            
            #cv2.waitKey(1)
            
            rate.sleep()  

    def img_cb(self, msg_image):
        
        ## This function receives a ROS image
        try: 
            #print("received ROS image")
            self.msg_img = msg_image
            self.image_received = 1 #Turn the flag on 
        except CvBridgeError as e: 
            print(e)

    def track(self,x):
        pass


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("min_value_v0", anonymous=True)  
    IMGProcessingNodeClass()
