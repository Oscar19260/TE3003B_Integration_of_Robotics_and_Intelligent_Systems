#!/usr/bin/env python  

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

#Class desc

class IMGProcessingNodeClass():  

    def __init__(self):
    
        #rospy.on_shutdown(self.cleanup)
        
        ############ PRINCIPAL CONSTANTS & VARIABLES ################  
        
        center_final = Int32()
        
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
        self.img_pub    = rospy.Publisher('image_viewer', Image, queue_size=1)
        self.center_pub = rospy.Publisher('center', Int32, queue_size=1)

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
                
                    # Rotate 180 degrees clockwise
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

                    # Resize image
                #cv_image = cv2.resize(cv_image, (cv_image.shape[1]/2, cv_image.shape[0]/2))

                #per = 50
                #r_img = cv2.resize(cv_image, (int(cv_image.shape[1]*per/100), int(cv_image.shape[0]*per/100)))
                
                    # Convert BGR to GRAY
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)     # change image to work with
                
                (thresh, im_bw) = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
                
                sz = 30 #69

                # Creating kernel
                kernel = np.ones((sz, sz), np.uint8)
                  
                # Using cv2.erode() method 
                img_g_w_k = cv2.erode(im_bw, kernel)
                
                    # Apply Gaussian blur
                #img_g_w_k = cv2.GaussianBlur(img_g_w_k,(7,7),0)
                
                dims = img_g_w_k.shape
                
                    # Apply canny
                img_canny = cv2.Canny(img_g_w_k, 0, 255)       # 0, 60 -- 68, 46
                
                    # Crop it (rectangulo del fondo, desde 2/3 de la imagen)
                cp_img = img_canny[int(dims[0]*8/10):,:]

                    # Sum by columns
                b = abs(np.sum(cp_img, axis=0))

                #print(" ")		
                #print(b)

                    # Process 2
                    # ==================
                    # Find indexes of the maximum N values
                N = 31  # N maximum values SIEMPRE IMPAR
                result = np.argpartition(b, -N)[-N:]

                idx_r = np.sort(result, axis=None)   # Sort it
                
                ##
                
                idx_r_2 = np.zeros_like(idx_r)
    
                for i in range(0, idx_r.shape[0]):
                    if ((idx_r[i] >= int(dims[1]*1/3)) and (idx_r[i] <= int(dims[1]*2/3))): idx_r_2[i] = idx_r[i]
                
                try:
                    xmen = np.trim_zeros(idx_r_2)
                    xmen = int(np.mean(xmen))
                except:
                    xmen = 1280/2

                #print(xmen)
                
                ##img_wc_0 = cv2.circle(cp_img, (xmen, int(cp_img.shape[0]/2)), 5, (255,255,255), -1) # Print circle in oimg
                #img_wc_1 = cv2.circle(img_wc_0, (right_edge, int(cp_img.shape[0]/2)), 10, (255,255,255), -1) # Print circle in oimg
                #img_wc_2 = cv2.circle(img_wc_1, (m_3, int(cp_img.shape[0]/2)), 15, (255,255,255), -1) # Print circle in oimg

                ##cp_img_f = self.bridge.cv2_to_imgmsg(img_wc_0, "mono8")
                
                center_final = Int32()
                center_final.data = xmen         # Test point xmen

                ##self.img_pub.publish(cp_img_f)
                self.center_pub.publish(center_final)
                
                print("=====================")                
                print("          Px         ")
                print("[" + str(center_final.data) + "]")
                print(" ")
                
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
    rospy.init_node("min_value_v10", anonymous=True)  
    IMGProcessingNodeClass()
