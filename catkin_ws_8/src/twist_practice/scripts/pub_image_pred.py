#!/usr/bin/env python  

import rospy  

from sensor_msgs.msg import Image 

from cv_bridge import CvBridge, CvBridgeError 

import cv2  

#This class will receive a ROS image and transform it to opencv format  

class SendImagePred():  

    def __init__(self):  
    
        rospy.on_shutdown(self.cleanup)

        ############ CONSTANTS ################  

        self.bridge_object = CvBridge() # create the cv_bridge object 

        self.image_received = 0 #Flag to indicate that we have already received an image
        
        self.flag = 0
        
        ############################### PUBLISHERS #####################################        
        
        self.prediction_pub = rospy.Publisher('prediction_image', Image, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(10) #10Hz  

        while not rospy.is_shutdown():  

            if self.image_received:
                
                self.image_received = 0
            
                img = self.cv_image
                cv_image = cv2.rotate(img, cv2.ROTATE_180)
                img2 = cv2.resize(cv_image, (224, 224))
                cp_img_f = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                
                self.prediction_pub.publish(cp_img_f)

            r.sleep()    

    def image_cb(self, ros_image):  

        ## This function receives a ROS image and transforms it into opencv format   

        try: 

            print("received ROS image, I will convert it to opencv") 

            # We select bgr8 because it is the OpenCV encoding by default 

            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 

            self.image_received = 1 #Turn the flag on 

        except CvBridgeError as e: 

            print(e) 

    def cleanup(self):  
        #This function is called just before finishing the node
        #cv2.imwrite(self.filename, self.cv_image)
        #cv2.destroyAllWindows()
        print("Ciao")

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("send_image_pred", anonymous=True)  

    SendImagePred() 
