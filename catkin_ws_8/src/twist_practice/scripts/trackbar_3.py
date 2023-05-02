#!/usr/bin/env python  

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#This class will receive a ROS image and transform it to opencv format  

class TrackbarClass():  

    def __init__(self):  
    
        rospy.on_shutdown(self.cleanup)

        ############ CONSTANTS ################  

        self.bridge_object = CvBridge() # create the cv_bridge object 

        self.image_received = 0 #Flag to indicate that we have already received an image 
        
        # Canny threshold values

        l_t = 0
        u_t = 255

        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars',800,500)

        cv2.createTrackbar('lower_threshold_value','Trackbars',0,255,self.track)
        cv2.createTrackbar('upper_threshold_value','Trackbars',0,255,self.track)

        cv2.setTrackbarPos('lower_threshold_value','Trackbars',l_t)
        cv2.setTrackbarPos('upper_threshold_value','Trackbars',u_t)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(20) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:

                low_t = cv2.getTrackbarPos('lower_threshold_value','Trackbars')
                upp_t = cv2.getTrackbarPos('upper_threshold_value','Trackbars')
                
                cv_img = self.cv_image
                gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                
                blur_gi = cv2.GaussianBlur(gray_image,(7,7),0)
                
                img_canny = cv2.Canny(blur_gi, low_t, upp_t)
                    

                #Showing the original and masked images
                cv2.imshow("Image", blur_gi)

            cv2.waitKey(1) 

            r.sleep()  

        cv2.destroyAllWindows()

    def image_cb(self, ros_image):  

        ## This function receives a ROS image and transforms it into opencv format   

        try:
            #print("received ROS image, I will convert it to opencv") 

            # We select bgr8 because it is the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            self.image_received = 1 #Turn the flag on 

        except CvBridgeError as e:
            print(e) 

    def cleanup(self):  
        #This function is called just before finishing the node
        #cv2.imwrite(self.filename, self.cv_image)
        cv2.destroyAllWindows()
        print("Ciao")
        
    def track(self,x):
        pass

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("trackbar", anonymous=True)  

    TrackbarClass() 
