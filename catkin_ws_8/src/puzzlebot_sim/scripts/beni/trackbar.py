#!/usr/bin/env python  

#import rospy
import cv2
import numpy as np

#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError

#This class will receive a ROS image and transform it to opencv format  

class TrackbarClass():  

    def __init__(self):  

        ############ CONSTANTS ################  

        #self.bridge_object = CvBridge() # create the cv_bridge object 

        #self.image_received = 0 #Flag to indicate that we have already received an image 
        
        #HSV initial max and min values
        #hue_min_ini = 0
        #hue_max_ini = 15
        #sat_min_ini = 128
        #sat_max_ini = 255
        #val_min_ini = 128
        #val_max_ini = 255
        
        
        # BLUE
        #hue_min_ini = 92
        #hue_max_ini = 110
        #sat_min_ini = 212
        #sat_max_ini = 255
        #val_min_ini = 145
        #val_max_ini = 255
        rt = 1
        
        
        """
        # YELLOW
        hue_min_ini = 20
        hue_max_ini = 33
        sat_min_ini = 131
        sat_max_ini = 255
        val_min_ini = 153
        val_max_ini = 255
        rt = 1
        """
        
        # """
        # RED
        hue_min_ini = 0
        hue_max_ini = 15
        sat_min_ini = 128
        sat_max_ini = 255
        val_min_ini = 128
        val_max_ini = 255
        rt = 0
        # """
        
        #Create and configure a window for the Trackbars
        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars',800,500)

        #Create the Trackbars for the HSV max and min values
        cv2.createTrackbar('hue_min','Trackbars',0,179,self.track)
        cv2.createTrackbar('hue_max','Trackbars',0,179,self.track)
        cv2.createTrackbar('sat_min','Trackbars',0,255,self.track)
        cv2.createTrackbar('sat_max','Trackbars',0,255,self.track)
        cv2.createTrackbar('val_min','Trackbars',0,255,self.track)
        cv2.createTrackbar('val_max','Trackbars',0,255,self.track)
        cv2.createTrackbar('hue_min2','Trackbars',0,179,self.track)
        cv2.createTrackbar('hue_max2','Trackbars',0,179,self.track)
        cv2.createTrackbar('red_green','Trackbars',0,1,self.track)

        #Set initial values for the HSV max and min
        cv2.setTrackbarPos('hue_min','Trackbars',hue_min_ini)
        cv2.setTrackbarPos('hue_max','Trackbars',hue_max_ini)
        cv2.setTrackbarPos('sat_min','Trackbars',sat_min_ini)
        cv2.setTrackbarPos('sat_max','Trackbars',sat_max_ini)
        cv2.setTrackbarPos('val_min','Trackbars',val_min_ini)
        cv2.setTrackbarPos('val_max','Trackbars',val_max_ini)
        cv2.setTrackbarPos('hue_min2','Trackbars',hue_min_ini)
        cv2.setTrackbarPos('hue_max2','Trackbars',hue_max_ini)
        cv2.setTrackbarPos('red_green','Trackbars',rt)

        ############################### SUBSCRIBERS #####################################  

        #image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)
        
        ### VARIABLES AND CONSTANTS ###
        
        cap = cv2.VideoCapture(0)

        #********** INIT NODE **********###  

        #r = rospy.Rate(20) #10Hz  

        while True:
        
            ###
            # Capture frame-by-frame
            ret, frame = cap.read()
            ###
            
            if ret:

                h_min = cv2.getTrackbarPos('hue_min','Trackbars')
                h_max = cv2.getTrackbarPos('hue_max','Trackbars')
                s_min = cv2.getTrackbarPos('sat_min','Trackbars')
                s_max = cv2.getTrackbarPos('sat_max','Trackbars')
                v_min = cv2.getTrackbarPos('val_min','Trackbars')
                v_max = cv2.getTrackbarPos('val_max','Trackbars')
                h_min2 = cv2.getTrackbarPos('hue_min2','Trackbars')
                h_max2 = cv2.getTrackbarPos('hue_max2','Trackbars')
                red_green = cv2.getTrackbarPos('red_green','Trackbars')
                
                dim = (500,500)
                
                cv_img_o = frame
                img = cv2.resize(cv_img_o, dim)

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
                    

                #Showing the original and masked images
                #cv2.imshow("Image", img)
                
                pixel_area = cv2.countNonZero(mask)
                
                print(pixel_area)
                
                if pixel_area >= 11000: print("Detected")
                else: print("Missed")
                
                cv2.imshow("Color Mask", mask)	

            cv2.waitKey(1)

        cap.release()
        cv2.destroyAllWindows()
        
    def track(self,x):
        pass

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":

    TrackbarClass() 
    
