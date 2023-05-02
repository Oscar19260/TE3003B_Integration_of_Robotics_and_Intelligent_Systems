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
    
        rospy.on_shutdown(self.cleanup)
        
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
        
        self.msg_pub = rospy.Publisher('action', String, queue_size=1)  
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
                ## Activity: Colour filtering
                
                image_x = self.msg_img
            
                cv_image = self.bridge.imgmsg_to_cv2(image_x, "bgr8")
                
                per = 25
                cv_image_2 = cv2.resize(cv_image, (int(cv_image.shape[1]*per/100), int(cv_image.shape[0]*per/100)), interpolation = cv2.INTER_AREA)
                cv_image_f = self.bridge.cv2_to_imgmsg(cv_image_2, "bgr8") 
                
                noisy_cv_image = cv_image
                
                    # Convert BGR to HSV
                hsv_image = cv2.cvtColor(noisy_cv_image, cv2.COLOR_BGR2HSV)
                    # define range of RED color in HSV
                ##lower_red = np.array([0,88,179])
                ##upper_red = np.array([33,255,255])
                lower_red0 = np.array([0,82,92])
                upper_red0 = np.array([15,255,255])
                lower_red1 = np.array([165,82,92])
                upper_red1 = np.array([179,255,255])

                # define range of GREEN color in HSV
                lower_green = np.array([49,62,60])
                upper_green = np.array([98,255,255])


                    # Threshold the HSV image to get only RED colors
                mask_red0    = cv2.inRange(hsv_image, lower_red0, upper_red0)
                mask_red1    = cv2.inRange(hsv_image, lower_red1, upper_red1)
                    # Threshold the HSV image to get only RED colors
                mask_green  = cv2.inRange(hsv_image, lower_green, upper_green)

                    # Bitwise-AND mask and original image
                mask_red    = cv2.bitwise_or(mask_red0, mask_red1)
                mask_final  = cv2.bitwise_or(mask_red, mask_green)
                res         = cv2.bitwise_and(cv_image, cv_image, mask= mask_final)
                
                ## Blobs
                max_perc = 90.0
                min_perc = 10.0
                d_img = mask_final
                
                ## Blobs detection

                    # Setup SimpleBlobDetector parameters.
                params = cv2.SimpleBlobDetector_Params()       
                
                # Change thresholds
                params.minThreshold = 0;
                params.maxThreshold = 255;

                # Filter by Area.
                params.filterByArea = True
                #params.minArea = float(d_img.shape[0]*d_img.shape[1])*percentage/100.0
                params.minArea = float(d_img.shape[0]*d_img.shape[1])*min_perc/100.0
                params.maxArea = float(d_img.shape[0]*d_img.shape[1])*max_perc/100.0

                # Filter by Color (black=0)
                params.filterByColor = True
                params.blobColor = 255

                # Filter by Circularity
                params.filterByCircularity = True
                params.minCircularity = 0.3
                params.maxCircularity = 1

                # # Filter by Convexity
                # params.filterByConvexity = True
                # params.minConvexity = 0.5
                # params.maxConvexity = 1

                # # Filter by Inertia
                # params.filterByInertia = True
                # params.minInertiaRatio = 0
                # params.maxInertiaRatio = 1

                # Distance Between Blobs
                params.minDistBetweenBlobs = 0

                # """
                # # Create a detector with the parameters
                # ver = (cv2.__version__).split('.')
                # if int(ver[0]) < 3 :
                # 	detector = cv2.SimpleBlobDetector(params)
                # else : 
                # 	detector = cv2.SimpleBlobDetector_create(params)
                # """

                detector = cv2.SimpleBlobDetector_create(params)
                 
                # Detect blobs.
                keypoints = detector.detect(d_img)

                # Draw detected blobs as red circles.
                # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
                #im_with_keypoints = cv2.drawKeypoints(d_img, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                #cv2_imshow(im_with_keypoints)
                # imshow_colour(im_with_keypoints, (10, 10))

                self.blobs = len(keypoints)
                
                # Identify colour of the circle(s) detected
                
                # From BGR to HSV
                
                new = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

                # Extract Hue channel
                H, S, V = cv2.split(new)

                # Make mask of zeroes in which we will set greens to 1
                mask_g = np.zeros_like(H, dtype=np.uint8)
                mask_r = mask_g.copy()

                # Set all green and red pixels to 1
                mask_g[(H>=lower_green[0]) & (H<=upper_green[0]) & (V!=0)] = 1
                mask_r[(H>=lower_red0[0]) & (H<=upper_red0[0]) & (V!=0)] = 1
                mask_r[(H>=lower_red1[0]) & (H<=upper_red1[0]) & (V!=0)] = 1

                self.per_g, self.per_r = mask_g.mean()*100, mask_r.mean()*100

                if (self.per_g > self.per_r): self.colour = "green"
                elif (self.per_g < self.per_r): self.colour = "red"
                
                if (self.blobs != 0 and self.colour != "none"):
                    if (self.colour == "green"): self.action = "forward"
                    elif (self.colour == "red"): self.action = "stop"
                
                # 
               
                #f_img = self.bridge.cv2_to_imgmsg(res, "bgr8") 
                ##final_img = self.bridge.cv2_to_imgmsg(mask_red, "mono8") 
                #self.image_pub.publish(f_img)
                
                #if (self.action == "forward"): self.vel.linear.x = 0.1
                #elif (self.action == "stop"): self.vel = Twist()
                #elif (self.action == "None"): pass
                #else: pass
                
                self.msg_string.data = self.action
                
                self.img_pub.publish(cv_image_f)
                self.msg_pub.publish(self.msg_string)
                
                print("                              ")
                print("Number of blobs detected are: ", self.blobs)
                
                # Now print percentage of green and red pixels
                print("Percentage of green in final image with mask : ", self.per_g)
                print("Percentage of red in final image with mask   : ", self.per_r)
                
                # Print colour of circle identified
                print("Colour of circle identified: ", self.colour)
                print("==============================")
                print("Action       : ", self.action)
                print("MSG_Action   : ", self.msg_string.data)
            
            rate.sleep()  

    def img_cb(self, msg_image):
    
        ## Activity: Image processing framework
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        
        (rows,cols,channels) = cv_image.shape
        
        scale_percent = 50 # percent of original size
        width = int(cols * scale_percent / 100)
        height = int(rows * scale_percent / 100)
        dim = (width, height)
          
        # resize image
        cv_image = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)
        
        # apply guassian blur on src image
        cv_image = cv2.GaussianBlur(cv_image, (9,9), cv2.BORDER_DEFAULT)
        
        final_img = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")    
        
        self.image_pub.publish(final_img)
        """
        
        ## This function receives a ROS image and saves it
        try: 
            #print("received ROS image")
            self.msg_img = msg_image
            self.image_received = 1 #Turn the flag on 
        except CvBridgeError as e: 
            print(e) 
            
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        
        a=String()
        a.data = "stop"
        self.msg_pub.publish(a) #publish the robot's speed

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("act0_img_pro_node", anonymous=True)  
    IMGProcessingNodeClass()
