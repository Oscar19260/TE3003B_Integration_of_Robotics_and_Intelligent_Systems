#!/usr/bin/env python3  

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
        
        #HSV initial max and min values
        hue_min_ini = 0
        hue_max_ini = 15
        hue_min_ini2 = 165
        hue_max_ini2 = 179
        sat_min_ini = 82
        sat_max_ini = 255
        val_min_ini = 92
        val_max_ini = 255
        er = 0

        #Create and configure a window for the Trackbars
        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars',800,500)

        #Create the Trackbars for the HSV max and min values
        cv2.createTrackbar('hue_min','Trackbars',0,179,self.track)
        cv2.createTrackbar('hue_max','Trackbars',0,179,self.track)
        cv2.createTrackbar('hue_min2','Trackbars',0,179,self.track)
        cv2.createTrackbar('hue_max2','Trackbars',0,179,self.track)
        cv2.createTrackbar('sat_min','Trackbars',0,255,self.track)
        cv2.createTrackbar('sat_max','Trackbars',0,255,self.track)
        cv2.createTrackbar('val_min','Trackbars',0,255,self.track)
        cv2.createTrackbar('val_max','Trackbars',0,255,self.track)
        cv2.createTrackbar('red_green','Trackbars',0,1,self.track)
        cv2.createTrackbar('size_kernel1','Trackbars',0,255,self.track)
        cv2.createTrackbar('size_kernel2','Trackbars',0,255,self.track)
        cv2.createTrackbar('kernel_erode','Trackbars',0,255,self.track)

        #Set initial values for the HSV max and min
        cv2.setTrackbarPos('hue_min','Trackbars',hue_min_ini)
        cv2.setTrackbarPos('hue_max','Trackbars',hue_max_ini)
        cv2.setTrackbarPos('sat_min','Trackbars',sat_min_ini)
        cv2.setTrackbarPos('sat_max','Trackbars',sat_max_ini)
        cv2.setTrackbarPos('val_min','Trackbars',val_min_ini)
        cv2.setTrackbarPos('val_max','Trackbars',val_max_ini)
        cv2.setTrackbarPos('hue_min2','Trackbars',hue_min_ini2)
        cv2.setTrackbarPos('hue_max2','Trackbars',hue_max_ini2)
        cv2.setTrackbarPos('size_kernel1','Trackbars',er)
        cv2.setTrackbarPos('size_kernel2','Trackbars',er)
        cv2.setTrackbarPos('red_green','Trackbars',0)
        cv2.setTrackbarPos('kernel_erode','Trackbars',er+0)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(20) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:

                h_min = cv2.getTrackbarPos('hue_min','Trackbars')
                h_max = cv2.getTrackbarPos('hue_max','Trackbars')
                s_min = cv2.getTrackbarPos('sat_min','Trackbars')
                s_max = cv2.getTrackbarPos('sat_max','Trackbars')
                v_min = cv2.getTrackbarPos('val_min','Trackbars')
                v_max = cv2.getTrackbarPos('val_max','Trackbars')
                h_min2 = cv2.getTrackbarPos('hue_min2','Trackbars')
                h_max2 = cv2.getTrackbarPos('hue_max2','Trackbars')
                red_green = cv2.getTrackbarPos('red_green','Trackbars')
                er_1 = cv2.getTrackbarPos('size_kernel1','Trackbars')
                er_2 = cv2.getTrackbarPos('size_kernel2','Trackbars')
                er_4 = cv2.getTrackbarPos('kernel_erode','Trackbars')
                
                if (er_1 % 2 == 0): er_1 += 1
                if (er_2 % 2 == 0): er_2 += 1
                if (er_4 % 2 == 0): er_4 += 1
                
                dim = (500,500)
                
                cv_img_o = self.cv_image
                cv_img_o = cv2.rotate(cv_img_o, cv2.ROTATE_180)
                img = cv2.resize(cv_img_o, dim)
                
                img = cv2.GaussianBlur(img, (er_1, er_1), 0)

                #Converting to HSV space
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                if (red_green == 1):    # green
                    color_min0 = np.array([h_min, s_min, v_min])
                    color_max0 = np.array([h_max, s_max, v_max])
                        #Applying mask for the selected color
                    mask = cv2.inRange(hsv_frame, color_min0, color_max0)
                elif (red_green == 0):  # red
                    color_min0 = np.array([h_min, s_min, v_min])
                    color_max0 = np.array([h_max, s_max, v_max])
                    color_min1 = np.array([h_min2, s_min, v_min])
                    color_max1 = np.array([h_max2, s_max, v_max])
                        #Applying mask for the selected color
                    mask0 = cv2.inRange(hsv_frame, color_min0, color_max0)
                    mask1 = cv2.inRange(hsv_frame, color_min1, color_max1)
                    mask = cv2.bitwise_or(mask0, mask1)
                
                # Creating kernel
                kernele = np.ones((er_4, er_4), np.uint8)
                  
                # Using cv2.erode() method 
                mask_e = cv2.erode(mask, kernele)
                    
                # Creating kernel
                kernel = np.ones((er_2, er_2), np.uint8)
                  
                # Using cv2.erode() method 
                img_g_w_k = cv2.dilate(mask_e, kernel)

                #Showing the original and masked images
                #cv2.imshow("Image", img)
                cv2.imshow("Color Mask", img_g_w_k)	

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
