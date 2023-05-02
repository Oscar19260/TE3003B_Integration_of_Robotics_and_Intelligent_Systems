#!/usr/bin/env python  

import rospy  

from sensor_msgs.msg import Image 

from cv_bridge import CvBridge, CvBridgeError 

import cv2  

#This class will receive a ROS image and transform it to opencv format  

class ShowImage():  

    def __init__(self):  
    
        rospy.on_shutdown(self.cleanup)

        ############ CONSTANTS ################  

        self.bridge_object = CvBridge() # create the cv_bridge object 

        self.image_received = 0 #Flag to indicate that we have already received an image
        
        self.flag = 0 
        
                
        # Filename
        self.filename = "robot_image.jpg"

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_cb) 

           

        #********** INIT NODE **********###  

        r = rospy.Rate(10) #10Hz  

        while not rospy.is_shutdown():  

            if self.image_received: 

                cv2.imshow('robot_image', self.cv_image)
                
                if (self.flag == 1):
                    cv2.imwrite(self.filename, self.cv_image)
                    cv2.destroyAllWindows()

            cv2.waitKey(1) 

            r.sleep()  

        cv2.destroyAllWindows()  

 

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

    rospy.init_node("cv_bridge_example", anonymous=True)  

    ShowImage() 
