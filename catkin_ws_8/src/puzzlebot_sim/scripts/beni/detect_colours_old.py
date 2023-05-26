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
        hue_min_ini_b = 92
        hue_max_ini_b = 110
        sat_min_ini_b = 212
        sat_max_ini_b = 255
        val_min_ini_b = 100
        val_max_ini_b = 255
        rt = 0
        
        
        
        # YELLOW
        hue_min_ini_y = 10
        hue_max_ini_y = 33
        sat_min_ini_y = 90
        sat_max_ini_y = 255
        val_min_ini_y = 153
        val_max_ini_y = 255
        rt = 1
        
        
        # """
        # RED
        hue_min_ini_r = 167
        hue_max_ini_r = 179
        sat_min_ini_r = 95
        sat_max_ini_r = 255
        val_min_ini_r = 138
        val_max_ini_r = 255
        
        hue_min_ini_1_r = 0
        hue_max_ini_1_r = 27
        rt = 2
        # """
        
        flag = 9
        
        ### VIDEO CAPTURE ###
        
        cap = cv2.VideoCapture(0)

        while True:
        
            ###
            # Capture frame-by-frame
            ret, frame = cap.read()
            ###
            
            if ret:
                
                dim = (500,500)
                
                cv_img_o = frame
                img = cv2.resize(cv_img_o, dim)

                # Converting to HSV space
                hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                # Apply masks for selected colours   
                #BLUE
                color_min_0_b = np.array([hue_min_ini_b, sat_min_ini_b, val_min_ini_b])
                color_max_0_b = np.array([hue_max_ini_b, sat_max_ini_b, val_max_ini_b])
                
                mask_b = cv2.inRange(hsv_frame, color_min_0_b, color_max_0_b)
                
                #YELLOW
                color_min_0_y = np.array([hue_min_ini_y, sat_min_ini_y, val_min_ini_y])
                color_max_0_y = np.array([hue_max_ini_y, sat_max_ini_y, val_max_ini_y])
                
                mask_y = cv2.inRange(hsv_frame, color_min_0_y, color_max_0_y)
                
                #RED
                color_min_0_r = np.array([hue_min_ini_r,    sat_min_ini_r, val_min_ini_r])
                color_max_0_r = np.array([hue_max_ini_r,    sat_max_ini_r, val_max_ini_r])
                color_min_1_r = np.array([hue_min_ini_1_r,  sat_min_ini_r, val_min_ini_r])
                color_max_1_r = np.array([hue_max_ini_1_r,  sat_max_ini_r, val_max_ini_r])
                                
                mask0 = cv2.inRange(hsv_frame, color_min_0_r, color_max_0_r)
                mask1 = cv2.inRange(hsv_frame, color_min_1_r, color_max_1_r)
                mask_r = cv2.bitwise_or(mask0, mask1)
                
                # Merge masks
                mask_all_0  = cv2.bitwise_or(mask_r, mask_y)
                mask_all    = cv2.bitwise_or(mask_all_0, mask_b)

                #Showing the original and masked images
                #cv2.imshow("Image", img)
                
                # Count pixel area per mask colour
                pixel_area_r = cv2.countNonZero(mask_r)
                pixel_area_y = cv2.countNonZero(mask_y)
                pixel_area_b = cv2.countNonZero(mask_b)
                
                pa = [pixel_area_r, pixel_area_y, pixel_area_b]
                max_pa = max(pa)
                idx_pa = pa.index(max_pa)
                
                print(" ")
                
                print(pixel_area_r)
                print(pixel_area_y)
                print(pixel_area_b)
                
                str_col_det = "None"
                min_area = 8500
                
                #print(min_area)
                
                if idx_pa == 0:
                    str_col_det = "RED"
                    
                    if pixel_area_y >= min_area:
                        print("YELLOW" + " READY TO PICK!")
                        flag = 1 #yellow
                    else:
                        print("NOT ENOUGH " + "YELLOW")
                        if pixel_area_r >= min_area:
                            print(str_col_det + " READY TO PICK!")
                            flag = 0 # red
                        else: print("NOT ENOUGH " + str_col_det)
                    
                elif idx_pa == 1:
                    str_col_det = "YELLOW"
                    
                    if pixel_area_y >= min_area:
                        print(str_col_det + " READY TO PICK!")
                        flag = 1 # yellow
                    else: print("NOT ENOUGH " + str_col_det)
                
                elif idx_pa == 2:
                    str_col_det = "BLUE"
                    
                    if pixel_area_b >= min_area:
                        print(str_col_det + " READY TO PICK!")
                        flag = 2 # blue
                    else: print("NOT ENOUGH " + str_col_det)
                
                else:
                    print(str_col_det + "DETECTED")
                    flag = 9 # none
                
                print("flag : " + str(flag))
                
                cv2.imshow("Color Mask", mask_all)	

            cv2.waitKey(1)

        cap.release()
        cv2.destroyAllWindows()
        
    def track(self,x):
        pass

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":

    TrackbarClass() 
    
