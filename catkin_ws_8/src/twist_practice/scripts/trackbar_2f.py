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
        
        er = 1
        erro = 7
        
        thresh_value = 128
        thresh_value2 = 255

        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars',700,500)

        cv2.createTrackbar('lower_threshold_value','Trackbars',0,255,self.track)
        cv2.createTrackbar('upper_threshold_value','Trackbars',0,255,self.track)
        cv2.createTrackbar('size_kernel','Trackbars',1,255,self.track)
        cv2.createTrackbar('size_kernel2','Trackbars',1,255,self.track)
        cv2.createTrackbar('thresh_value','Trackbars',0,255,self.track)
        cv2.createTrackbar('thresh_value2','Trackbars',0,255,self.track)

        cv2.setTrackbarPos('lower_threshold_value','Trackbars',l_t)
        cv2.setTrackbarPos('upper_threshold_value','Trackbars',u_t)
        cv2.setTrackbarPos('size_kernel','Trackbars',er)
        cv2.setTrackbarPos('size_kernel2','Trackbars',erro)
        cv2.setTrackbarPos('thresh_value','Trackbars',thresh_value)
        cv2.setTrackbarPos('thresh_value2','Trackbars',thresh_value2)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(20) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:

                low_t   = cv2.getTrackbarPos('lower_threshold_value','Trackbars')
                upp_t   = cv2.getTrackbarPos('upper_threshold_value','Trackbars')
                er_f    = cv2.getTrackbarPos('size_kernel','Trackbars')
                er_2    = cv2.getTrackbarPos('size_kernel2','Trackbars')
                tv      = cv2.getTrackbarPos('thresh_value','Trackbars')
                tv2      = cv2.getTrackbarPos('thresh_value2','Trackbars')
                
                if (er_f % 2 == 0): er_f += 1
                if (er_2 % 2 == 0): er_2 += 1
                
                cv_img = self.cv_image
                cv_img = cv2.resize(cv_img, (cv_img.shape[1]/2, cv_img.shape[0]/2))
                gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                
                (thresh, im_bw) = cv2.threshold(gray_image, tv, tv2, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
                
                # Creating kernel
                kernel = np.ones((er_f, er_f), np.uint8)
                  
                # Using cv2.erode() method 
                img_g_w_k = cv2.erode(im_bw, kernel) 
                
                blur_gi = cv2.GaussianBlur(img_g_w_k,(er_2,er_2),0)
                
                img_canny = cv2.Canny(blur_gi, low_t, upp_t)
                    

                #Showing the original and masked images
                cv2.imshow("Bitwise", im_bw)
                cv2.imshow("Eroded", img_g_w_k)
                cv2.imshow("Blur_Canny", img_canny)

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
