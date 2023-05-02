#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import PIL

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tensorflow import keras
from std_msgs.msg import String


#This class will receive a ROS image and transform it to opencv format  

class PredictionsClass():  

    def __init__(self):  
    
        rospy.on_shutdown(self.cleanup)

        ############ CONSTANTS ################  

        self.bridge_object = CvBridge() # create the cv_bridge object 

        self.image_received = 0 #Flag to indicate that we have already received an image
        self.counter = 0
        self.counter2 = 0
        self.flag = "s"
        self.past_score = 0
        self.past_id_class = 0
        self.past_scores_all = np.array([[0.0, 0.0, 0.0, 0.0]])
        self.data_ids = []
        self.data_ids = np.array(self.data_ids)
        self.data_score_id = []
        self.data_score_id = np.array(self.data_score_id)
        self.past_ids_last_ten = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.past_final_id_prediction = "a"
        
        self.msg_string = String() #Action
        self.msg_string.data = "69"
        
        #print(self.past_ids_last_ten.shape[1])
        
        ############################### PUBLISHERS #####################################        
        
        self.prediction_pub = rospy.Publisher('prediction', String, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)
        
        ############################### LOAD MODEL ######################################
        
        model = keras.models.load_model('/home/dassdinho/puzzlebot_ws/src/twist_practice/scripts/models/op5_from_zero_merged_all_images')
        print("Hi")
        print(" ")

        #********** INIT NODE **********###  

        r = rospy.Rate(1) #10Hz  

        while not rospy.is_shutdown():
            
            if self.image_received:
                
                img_original = self.cv_image
                img = img_original.copy()
                img = cv2.rotate(img, cv2.ROTATE_180)
                #img = img[:, 1*int(img.shape[1]/2):]              # image crop - ALL
                
                #print(img.shape)
                
                
                img = cv2.resize(img, (224, 224))
                img = np.array(img)
                
                image_to_predict = np.expand_dims(img, axis=0)
    
                image_to_predict_scores = model.predict(image_to_predict)
                scores = image_to_predict_scores                            # scores
                id_class_prediction = np.argmax(image_to_predict_scores)    # id_class_prediction

                prediction = self.getClassName(id_class_prediction)         # class_name
                
                #print(" ")
                #print("No. class predicted  :", no_class_prediction)
                #print("Name class predicted : " + prediction)
                
                #cv2.imshow("ggs", img)
                #print(" ")
                #print(str(id_class_prediction) + " "+ str(prediction) + " " + str(scores))
                
                #print(scores[0][no_class_prediction])
                #print(self.past_score)
                
                ## Case 0 - Initial test
                
                #print(self.past_scores_all)
                #print(scores)
                np.array_equal(scores, self.past_scores_all)
                
                if (np.array_equal(scores, self.past_scores_all)): pass
                else:
                    #print("ALL: " + str(id_class_prediction) + " "+ str(prediction) + " " + str(scores))
                    
                    
                    self.past_ids_last_ten = np.roll(self.past_ids_last_ten, 1)
                    self.past_ids_last_ten[0][0] = id_class_prediction
                    
                    print("NPM: " + str(self.past_ids_last_ten))
                    
                
                    (unique, counts) = np.unique(self.past_ids_last_ten, return_counts=True)
                    frequencies = np.asarray((unique, counts)).T
                    
                    #print(frequencies)
                    #print(frequencies.shape)
                    #print(frequencies[0])
                    #print(frequencies[1])
                    #print(frequencies[2][0])
                    
                    n_resta = 2
                    perc = int(self.past_ids_last_ten.shape[1] - n_resta)
                    #print(perc)
                    
                    for i in range(0, frequencies.shape[0]):
                        x       = int(frequencies[i][1])
                        id_y    = int(frequencies[i][0])
                        if (x >= perc):
                            if (self.past_final_id_prediction != str(int(id_y))):
                                print(" ")
                                print(str(perc) + " or more preds: " + str(int(id_y)) + " "+ str(self.getClassName(int(id_y))))
                            
                                print(" ")
                                self.past_final_id_prediction = str(int(id_y))
                                self.msg_string.data = self.past_final_id_prediction
                        
                    
                self.past_score         = scores[0][id_class_prediction]
                self.past_id_class      = id_class_prediction
                self.past_scores_all    = np.array([scores[0]])
                #print(self.past_scores_all)
                
                
                self.prediction_pub.publish(self.msg_string.data)
                
                #cv2.imshow("ggs", img)	
                
                

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
        
    def getClassName(self, classNo):
        if classNo == 0: return 'Stop'                                            ## Clase 0 
        elif classNo == 1: return 'End of all speed and passing limits'           ## Clase 1
        elif classNo == 2: return 'Turn right ahead'                              ## Clase 2
        elif classNo == 3: return 'Ahead only'                                    ## Clase 3
        

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("predictions", anonymous=True)  

    PredictionsClass() 
