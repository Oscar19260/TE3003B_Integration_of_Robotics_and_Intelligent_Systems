from djitellopy import tello
import cv2
import numpy as np

#Trackbar Callback - Need to be defined
def track(x):
	pass

#HSV initial max and min values
hue_min_ini = 0
hue_max_ini = 30
sat_min_ini = 235
sat_max_ini = 255
val_min_ini = 130
val_max_ini = 210

#Create and configure a window for the Trackbars
cv2.namedWindow('Trackbars')
cv2.resizeWindow('Trackbars',800,500)

#Create the Trackbars for the HSV max and min values
cv2.createTrackbar('hue_min','Trackbars',0,179,track)
cv2.createTrackbar('hue_max','Trackbars',0,179,track)
cv2.createTrackbar('sat_min','Trackbars',0,255,track)
cv2.createTrackbar('sat_max','Trackbars',0,255,track)
cv2.createTrackbar('val_min','Trackbars',0,255,track)
cv2.createTrackbar('val_max','Trackbars',0,255,track)

#Set initial values for the HSV max and min
cv2.setTrackbarPos('hue_min','Trackbars',hue_min_ini)
cv2.setTrackbarPos('hue_max','Trackbars',hue_max_ini)
cv2.setTrackbarPos('sat_min','Trackbars',sat_min_ini)
cv2.setTrackbarPos('sat_max','Trackbars',sat_max_ini)
cv2.setTrackbarPos('val_min','Trackbars',val_min_ini)
cv2.setTrackbarPos('val_max','Trackbars',val_max_ini)

#Connect to tello drone
me = tello.Tello()
me.connect()



#Initializing camera stream
me.streamon()

#main cycle
while True:	

	#Reading HSV values from Trackbars
	h_min = cv2.getTrackbarPos('hue_min','Trackbars')
	h_max = cv2.getTrackbarPos('hue_max','Trackbars')
	s_min = cv2.getTrackbarPos('sat_min','Trackbars')
	s_max = cv2.getTrackbarPos('sat_max','Trackbars')
	v_min = cv2.getTrackbarPos('val_min','Trackbars')
	v_max = cv2.getTrackbarPos('val_max','Trackbars')
	print(f'HUE_MIN : {h_min} HUE_MAX : {h_max} SAT_MIN : {s_min} SAT_MAX : {s_max} VAL_MIN : {v_min} VAL_MAX : {v_max}')
	
	#Creating max and min HSV values arrays
	color_min = np.array([h_min, s_min, v_min])
	color_max = np.array([h_max, s_max, v_max])
	
	#Obtaining a new frame
	img = me.get_frame_read().frame
	img = cv2.resize(img,(500,500))
		
	#Converting to HSV space
	hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	#Applying mask for the selected color
	mask = cv2.inRange(hsv_frame, color_min, color_max)

	#Showing the original and masked images
	cv2.imshow("Image",img)
	cv2.imshow("Color Mask",mask)	

	cv2.waitKey(1)

	#Obtaining the remaining battery %
	print(me.get_battery())


