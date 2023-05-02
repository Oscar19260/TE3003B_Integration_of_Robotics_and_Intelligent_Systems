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
                #cv_image = cv2.resize(cv_image, (480,480))

                #per = 50
                #r_img = cv2.resize(cv_image, (int(cv_image.shape[1]*per/100), int(cv_image.shape[0]*per/100)))
                
                    # Convert BGR to GRAY
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)     # change image to work with
                    # Apply Gaussian blur
                blur_gi = cv2.GaussianBlur(gray_image,(7,7),0)
                
                dims = blur_gi.shape
                
                    # Apply canny
                img_canny = cv2.Canny(blur_gi, 68, 46)       # 0, 60 -- 68, 46
                
                    # Crop it (rectangulo del fondo, desde 2/3 de la imagen)
                cp_img = img_canny[int(dims[0]*2/3):,:]

                    # Sum by columns
                b = abs(np.sum(cp_img, axis=0))

                #print(" ")		
                #print(b)

                    # Find indexes of the maximum N values
                N = 15  # N maximum values
                result = np.argpartition(b, -N)[-N:]

                idx_r = np.sort(result, axis=None)   # Sort it

                a0 = idx_r.copy()
                a1 = idx_r.copy()
                
                    # Slice indexes array by EDGE
                diff = 50
                left_edge, right_edge = [], []                
                
                prev = idx_r[0]
                flag = 0
                for i in idx_r:
                    act = i
                    if (act-prev >= diff): flag = 1
                    if (flag == 1): right_edge.append(i)
                    else:   left_edge.append(i)
                
                try:
                    left_edge, right_edge = int(round(sum(left_edge)/len(left_edge))), int(round(sum(right_edge)/len(right_edge)))

                except:
                    left_edge, right_edge = int(round(sum(left_edge)/8)), int(round(sum(right_edge)/7))

                #print(idx_r)
                """
                print("=====================")                
                print("Left edge, Right edge")
                print("[" + str(left_edge) + ", " + str(right_edge) + "]")
                print(" ")
                """

                    # Original center estimation
                        # Crop it (rectangulo del fondo, desde 2/3 de la imagen)
                cp_img2 = blur_gi[int(dims[0]*2/3):,:]

                        # Sum by columns
                ba = abs(np.sum(cp_img2, axis=0))

                        # Find index of minimum value
                res = np.where(ba == np.amin(ba))
                
                try:
                    d = res[0][1]
                except:
                    d = res[0][0]

                idx_2 = int(d)

                    # Obtain middle point
                m_1 = int(round(abs(right_edge-left_edge)/2)) + left_edge
                m_2 = idx_2
                m_3 = int((m_1+m_2)/2)
                """
                print("=====================")                
                print("       P1, P2        ")
                print("[" + str(m_1) + ", " + str(m_2) + "]")
                print("mean: " + str(m_3))
                print(" ")
                """
                img_wc_0 = cv2.circle(cp_img, (m_1, int(cp_img.shape[0]/2)), 5, (255,255,255), -1) # Print circle in oimg
                img_wc_1 = cv2.circle(img_wc_0, (m_2, int(cp_img.shape[0]/2)), 10, (255,255,255), -1) # Print circle in oimg
                img_wc_2 = cv2.circle(img_wc_1, (m_3, int(cp_img.shape[0]/2)), 15, (255,255,255), -1) # Print circle in oimg

                cp_img_f = self.bridge.cv2_to_imgmsg(img_wc_2, "mono8")
                
                center_final = Int32()
                center_final.data = m_1         # Test point 1

                self.img_pub.publish(cp_img_f)
                self.center_pub.publish(center_final)
                
                print("=====================")                
                print("          P2         ")
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
    rospy.init_node("min_value_v2", anonymous=True)  
    IMGProcessingNodeClass()
