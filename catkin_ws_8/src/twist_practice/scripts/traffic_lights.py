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
        
        # Trackbars

        t0 = 0
        t1 = 100
        self.min_area = 700
        self.min_area_2 = 2100
        
        self.flag = 0
        self.counter = 0
        self.colour = "green"
        
        er = 0

        #Create and configure a window for the Trackbars
        
        cv2.namedWindow('Parameters')
        cv2.resizeWindow('Parameters', 640, 240)
        
        """
        cv2.createTrackbar('t0','Parameters',0,255,self.track)
        cv2.createTrackbar('t1','Parameters',0,255,self.track)
        cv2.createTrackbar('min_area','Parameters',0,1000,self.track)
        cv2.createTrackbar('size_kernel0','Parameters',0,255,self.track)
        cv2.createTrackbar('size_kernel1','Parameters',0,255,self.track)
        cv2.createTrackbar('size_kernel2','Parameters',0,255,self.track)
        cv2.createTrackbar('size_kernel3','Parameters',0,255,self.track)
        cv2.createTrackbar('kernel_erode','Parameters',0,255,self.track)
        cv2.createTrackbar('kernel_erode2','Parameters',0,255,self.track)

        cv2.setTrackbarPos('t0','Parameters',t0)
        cv2.setTrackbarPos('t1','Parameters',t1)
        cv2.setTrackbarPos('min_area','Parameters',min_area)
        cv2.setTrackbarPos('size_kernel0','Parameters',er+21)
        cv2.setTrackbarPos('size_kernel1','Parameters',er+9)
        cv2.setTrackbarPos('size_kernel2','Parameters',er+27)
        cv2.setTrackbarPos('size_kernel3','Parameters',er+9)
        cv2.setTrackbarPos('kernel_erode','Parameters',er+0)
        cv2.setTrackbarPos('kernel_erode2','Parameters',er+0)
        """
        
        cv2.createTrackbar('min_area','Parameters',0,1000,self.track)
        cv2.setTrackbarPos('min_area','Parameters',0)
        
        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)

        #********** INIT NODE **********###  

        r = rospy.Rate(20) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:
                #t0          = cv2.getTrackbarPos('t0','Parameters')
                #t1          = cv2.getTrackbarPos('t1','Parameters')   
                mino    = cv2.getTrackbarPos('min_area','Parameters')
                
                #er_0 = cv2.getTrackbarPos('size_kernel0','Parameters')
                #er_1 = cv2.getTrackbarPos('size_kernel1','Parameters')
                #er_2 = cv2.getTrackbarPos('size_kernel2','Parameters')
                #er_3 = cv2.getTrackbarPos('size_kernel3','Parameters')
                #er_4 = cv2.getTrackbarPos('kernel_erode','Parameters')
                #er_5 = cv2.getTrackbarPos('kernel_erode2','Parameters')
                
                #if (er_0 % 2 == 0): er_0 += 1
                #if (er_1 % 2 == 0): er_1 += 1
                #if (er_2 % 2 == 0): er_2 += 1
                #if (er_3 % 2 == 0): er_3 += 1
                #if (er_4 % 2 == 0): er_4 += 1
                #if (er_5 % 2 == 0): er_5 += 1
                
                er_0 = er+21
                er_1 = er+15
                er_2 = er+27
                er_3 = er+10
                er_4 = er+1
                er_5 = er+1

                
                dim = (500,500)
                
                cv_img_o = self.cv_image
                cv_img_o = cv2.rotate(cv_img_o, cv2.ROTATE_180)
                img = cv2.resize(cv_img_o, dim)
                im3  = img.copy()
                img = img[:,:int(dim[0]*5/6)]
                img2 = img.copy()
                imgg = img.copy() 
                
                img = cv2.GaussianBlur(img, (er_0, er_0), 0)
                imgg = cv2.GaussianBlur(imgg, (er_2, er_2), 0)

                #Converting to HSV space
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                hsv_frameg = cv2.cvtColor(imgg, cv2.COLOR_BGR2HSV)
                
                ## REEEEEEEEEEEEEEEEEEEED
                #######################################
                    
                #red
                color_min0 = np.array([0, 48, 110])
                color_max0 = np.array([15, 255, 255])
                color_min1 = np.array([165, 48, 110])
                color_max1 = np.array([179, 255, 255])
                    #Applying mask for the selected color
                mask0 = cv2.inRange(hsv_frame, color_min0, color_max0)
                mask1 = cv2.inRange(hsv_frame, color_min1, color_max1)
                maskr = cv2.bitwise_or(mask0, mask1)
                
                    # Creating kernel
                kernele = np.ones((er_4, er_4), np.uint8)
                  
                # Using cv2.erode() method 
                mkr_e = cv2.erode(maskr, kernele)
                
                    # Creating kernel
                kernel = np.ones((er_1, er_1), np.uint8)
                  
                # Using cv2.erode() method 
                img_mkr = cv2.dilate(mkr_e, kernel)
                
                im2r = img_mkr.copy()
                im3_r = img_mkr.copy()
                
                imgCanny_red = cv2.Canny(img_mkr, t0, t1)
                
                self.flag = 0
                
                self.getContours(imgCanny_red, self.flag)
                
                #######################################
                
                """
                #green
                color_mina = np.array([49, 62, 100])
                color_maxb = np.array([98, 255, 255])
                    #Applying mask for the selected color
                maskg = cv2.inRange(hsv_frameg, color_mina, color_maxb)
                
                    # Creating kernel
                kernelo = np.ones((er_5, er_5), np.uint8)
                  
                # Using cv2.erode() method 
                mkg_e = cv2.erode(maskg, kernelo)
                
                    # Creating kernel
                kernel2 = np.ones((er_3, er_3), np.uint8)
                  
                # Using cv2.erode() method 
                img_mkg = cv2.dilate(mkg_e, kernel2)
                    
                im2g = img_mkg.copy()
                img3 = img_mkg.copy()
                
                imgCanny_green = cv2.Canny(img_mkg, t0, t1)
                
                self.flag = 1
                
                self.getContours(imgCanny_green, self.flag)
                
                print("colour: ", str(self.colour))
                print("counter: ", str(self.counter))
                print(" ")
                
                maskf = cv2.bitwise_or(im3_r, img3)
                """
                
                maskf = im3_r
                
                maskk = maskf.copy()
                
                imgCanny_final = cv2.Canny(maskf, t0, t1)
                
                img_final_f = self.getContours2(imgCanny_final, maskk, mino)
                
                print("colour: ", str(self.colour))
                print("counter: ", str(self.counter))
                print(" ")
                
                if (self.colour == "green" and self.counter == 2):
                    print("YA WE")
                    #break
                
                #cv2.imshow("Color Mask", img_final_f)	

            #cv2.waitKey(1) 

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
            
    def getContours(self, img, flago):
        
        agg = []
        
        f0 = 0
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(imgContour, contours, -1, (255, 0, 255), 3)
            
        for cnt in contours:
            area = cv2.contourArea(cnt)
            print("Area", area)
            
            if (area >= self.min_area and area <= 1000):
                self.counter = 1
                area_min = self.min_area
            if (area >= self.min_area_2 and area <= 2300):
                self.counter = 2
                area_min = self.min_area_2
            else:
                area_min = 5000
                
            
            if (area >= area_min and f0 == 0):
                    
                #print(area)
                
                if (flago == 0): self.colour = "red"
            
                #cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 1)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                #print("len", len(approx))
                x, y, w, h = cv2.boundingRect(approx)
                er = 5
                x0, y0, x1, y1 = x, y-er, x+w, y+h+er
                #cv2.rectangle(imgContour, (x0, y0), (x1, y1), (255,255,255), 3)
                
                #cv2.putText(imgContour, "Points : " + str(len(approx)), (x+w+20, y+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                #cv2.putText(imgContour, "Area   : " + str(int(area)), (x+w+20, y+45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                f0 = 1
                
            else:
                self.colour = "green"
                
                
        #return imgContour
        
        
    def getContours2(self, img, imgContour, area_min):
        
        agg = []
        
        f0 = 0
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(imgContour, contours, -1, (255, 0, 255), 3)
            
        for cnt in contours:
            area = cv2.contourArea(cnt)
            #print("Area", area)
            
            if (area >= area_min and f0 == 0):
            
                cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 1)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                #print("len", len(approx))
                x, y, w, h = cv2.boundingRect(approx)
                er = 5
                x0, y0, x1, y1 = x, y-er, x+w, y+h+er
                cv2.rectangle(imgContour, (x0, y0), (x1, y1), (255,255,255), 3)
                
                cv2.putText(imgContour, "Points : " + str(len(approx)), (x+w+20, y+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(imgContour, "Area   : " + str(int(area)), (x+w+20, y+45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                f0 = 1
                
                
        return imgContour

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
