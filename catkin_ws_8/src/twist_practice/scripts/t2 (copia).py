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
        
        # Trackbars

        p1 = 100
        p2 = 48
        min_rad = 40
        max_rad = 71

        #Create and configure a window for the Trackbars
        cv2.namedWindow('Parameters')
        cv2.resizeWindow('Parameters', 640, 240)

        cv2.createTrackbar('param1','Parameters',1,255,self.track)
        cv2.createTrackbar('param2','Parameters',1,255,self.track)
        cv2.createTrackbar('min_rad','Parameters',0,200,self.track)
        cv2.createTrackbar('max_rad','Parameters',0,200,self.track)

        cv2.setTrackbarPos('param1','Parameters',p1)
        cv2.setTrackbarPos('param2','Parameters',p2)
        cv2.setTrackbarPos('min_rad','Parameters',min_rad)
        cv2.setTrackbarPos('max_rad','Parameters',max_rad)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(20) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:
                p1      = cv2.getTrackbarPos('param1','Parameters')
                p2      = cv2.getTrackbarPos('param2','Parameters')   
                min_rad = cv2.getTrackbarPos('min_rad','Parameters')   
                max_rad = cv2.getTrackbarPos('max_rad','Parameters')
                
                h_min = 0
                h_max = 15
                h_min2 = 165
                h_max2 = 179
                s_min = 48
                s_max = 255
                v_min = 110
                v_max = 255
                #red_green = 0
                er_1 = 21
                er_2 = 15

                
                dim = (500,500)
                
                cv_img_o = self.cv_image
                img = cv2.resize(cv_img_o, dim)
                img2 = img.copy()
                imgg = img.copy()
                
                img = cv2.GaussianBlur(img, (er_1, er_1), 0)
                imgg = cv2.GaussianBlur(imgg, (19, 19), 0)

                #Converting to HSV space
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                hsv_frameg = cv2.cvtColor(imgg, cv2.COLOR_BGR2HSV)
                    
                #red
                color_min0 = np.array([h_min, s_min, v_min])
                color_max0 = np.array([h_max, s_max, v_max])
                color_min1 = np.array([h_min2, s_min, v_min])
                color_max1 = np.array([h_max2, s_max, v_max])
                    #Applying mask for the selected color
                mask0 = cv2.inRange(hsv_frame, color_min0, color_max0)
                mask1 = cv2.inRange(hsv_frame, color_min1, color_max1)
                maskr = cv2.bitwise_or(mask0, mask1)
                
                    # Creating kernel
                kernel = np.ones((er_2, er_2), np.uint8)
                  
                # Using cv2.erode() method 
                img_mkr = cv2.dilate(maskr, kernel)
                
                #green
                color_mina = np.array([46, 30, 28])
                color_maxb = np.array([98, 255, 255])
                    #Applying mask for the selected color
                maskg = cv2.inRange(hsv_frameg, color_mina, color_maxb)
                img_mkg = maskg.copy()
                    
                mask = cv2.bitwise_or(img_mkr, img_mkg)
                
                img_g_w_k = mask
                
                # houghCircles
                
                circles = cv2.HoughCircles(img_g_w_k, cv2.HOUGH_GRADIENT, 1, img_g_w_k.shape[0]/8, param1 = p1, param2 = p2, minRadius = min_rad , maxRadius = max_rad)
                
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for i in circles[0,:]:
                        center = (i[0], i[1])   # x and y coordinates
                        radius = i[2]           # radius
                        # draw the outer circle
                        cv2.circle(img2, center, radius, (255,0,0), 2)
                        # draw the center of the circle
                        cv2.circle(img2, center, 2, (0,0,255), 3)

                #Showing the original and masked images
                #cv2.imshow("Image", img)
                
                cv2.imshow("Color Mask", img2)	

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
