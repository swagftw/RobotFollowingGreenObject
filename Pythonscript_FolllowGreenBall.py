
#*******************************************************************************************************************************************
#Author:			Anel Music - anel.music@tum.de
#This code tracks a green ball and makes a Rapberry PI based, car like robot following 
#
#Part 1:
#In the first part an image from the camera is captured. I blured the image to make the
#object detection more stabe. I found that the here presented approach of tresholding the color green
#tracks the ball way more precisely and stable than compared to my first approach where I used a trackbar
#to adjust the color paramets dynamically. As an indicator for the object distance I used the radius but
#the area of the found contour works also fine
#
#Part 2: 
#To make the robot move smooth and precisely i used updated the PWM-parameters via PID-Ctrl
#Please keep in my the the absolute image centers X-coordinate = 320. That means that if the calls center_x > 320 the robot
#must turn right and if the balls centers_x is <320 the robot must turn left. If the balls radius gets too big
#(happens at about 5cm) the robot stops. Also if the robot loses the ball it must stop untill the ball is located again
#
#Additional Part:
#I extended the project a little bit and used a servo motor to move robot eyes and fix them always on the ball (to make my robot more likeable)
# 
#Components used for this project:
#Raspberry PI 3, 2 x NoName 6V Dc-Motors (China), L298N H-bridge motor driver for wheels, SG90 Micro servo Motor for eyes, 
#4 x AA Batteries to power the motor driver, 10000mAh power bank to power Raspberry PI, Umox Camera
#*******************************************************************************************************************************************

import cv2
import time 
import RPi.GPIO as GPIO
import cv2.cv as cv
import numpy as np

#Open CV Window setting
WINDOW_NAME = 'HSA final Project' 

#Setting up Raspberry Pi
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT) #servo 21 BCM 40 Board

#Setting up DC Motors
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

GPIO.setup(7, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)

#Setting up PWM
#p back  q forward
pR = GPIO.PWM(16,100)
pR.start(0)
qR = GPIO.PWM(18,100)
qR.start(0)
pL = GPIO.PWM(7,100)
pL.start(0)
qL = GPIO.PWM(12,100)
qL.start(0)


#PID-Ctrl params and variables
lastPropotional_param = 0
propotional_param = 0
integral_param = 0
differential_param = 0
powerDIFF = 0

#Dc Motor params
defaultSPEED = 0
motormaneuvrSPEED =0
const_param = 0.8 

#Servo Motor params

#Additional variables
radiusList = []
objectSIZE = 0
objectLocation = 0



def findGreenBall(image):
	#Instead of making these variables global for this particular function I could have
	#used them as members via an additional class but for this project OOP would be an overkill
    global radiusList	
    global objectSIZE
    global propotional_param, integral_param, differential_param, lastPropotional_param
    global powerDIFF
    global groundSPEED,motorCtrlSpeed 
    global objectLocation
    global const_param
    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    #Conv RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    #Define color range
    greenMIN = np.array([40,70,70])
    greenMAX = np.array([80,200,200])

    #Threshold HSV image for green
    my_mask = cv2.inRange(hsv, greenMIN, greenMAX)
    
    #Blur the mask to improve image processing
    blurmask = cv2.GaussianBlur(my_mask, (5,5),0)
	
    #Take the moments to get the centerpoint 
    moments = cv2.moments(blurmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)

  
    contours, hierarchy = cv2.findContours(
    blurmask,
    cv2.cv.CV_RETR_LIST,
    cv2.cv.CV_CHAIN_APPROX_SIMPLE
    )

    #Due to noise several contours were detected. Because I'm only interested in the biggest one
    #I stored all contours radius and used the biggest one
    for contour in contours:
    	radiusdummy = 0
	area = cv2.contourArea(contour)
	(x,y),radi = cv2.minEnclosingCircle(contour)
  	mycenter = (int(x),int(y))
	myradius = int(radi)
	radiusList.append(myradius)
	#Drawing i circle in bmask for debugging purposes
	cv2.circle(bmask,mycenter,myradius, (0,255,0),2)
    contourFound = True
    if contourFound:
	 objectSIZE = max(radiusList)
	 radiusList = list()
	 contourFound = False


    #Assume that there is no centerpoint
    ctr = (-1,-1)

    #Update robot eyes
    dutyCycleOld = dutyCycle
    dutyCycleOld = (-6.00*centerpoint_x + 6120.00) / (560.00) # linear function for mapping pixelvalues to servo dutycycle values (See. report)
    s.ChangeDutyCycle(dutyCycle)
    
    #If Ball was found use it's center
    if centerpoint_x != None and centerpoint_y != None:

        centerpointXY = (centerpoint_X, centerpoint_y)

        #Visualize detection with circle around center
        cv2.circle(image, ctr, 5, (0,0,0))
	
	##Debugging
	#print("objectSIZE : =  ",objectSIZE)
	#print("Radius : = ",myradius)

	#Stop about 5 cm away from ball
	if objectSIZE >= 40:
                print("Stop!!! Object too close")
				qLR.ChangeDutyCycle(0)
                qL.ChangeDutyCycle(0)
                time.sleep(0.1)		
		
	if objectSIZE <40:
		print("RADIUS == ", myradius)
		if(objectSIZE >=35):
			defaultSPEED = 50
		else:
			defaultSPEED = 60
		
		lastPropotional_param = propotional_param
		propotional_param = ( int(round(centerpoint_x))-320)
		
		differential_param = propotional_param - lastPropotional_param
		integral_param += propotional_param
		powerDIFF = propotional_param/10 + integral_param/10000 + differential_param*3/2
		
		if(powerDIFF >defaultSPEED):
			powerDIFF = defaultSPEED
		if(powerDIFF <=-defaultSPEED):
			powerDIFF = -defaultSPEED
		#If ball in front (within a certain deltaX range) move forward	
		if(propotional >-50 and propotional <50):
			qL.ChangeDutyCycle(35)
            qR.ChangeDutyCycle(35)
			time.sleep(0.02)
			print ("Move forward---Ball in front")

		#If Ball to the left
		elif(centerpoint_x <320): #elif propotional >-50 and...
			objectLocation = 0
			#print("links")
			motormaneuvrSPEED = (defaultSPEED+powerDIFF)*const_param
			if(motormaneuvrSPEED < 25):
				motormaneuvrSPEED = 25
				
			#print("qL.ChangeDuryCycleL = ",motormaneuvrSPEED)
			#print("qR.ChangeDutyCycleR = ", defaultSPEED)
			qL.ChangeDutyCycle(motormaneuvrSPEED)
			qR.ChangeDutyCycle(defaultSPEED)	
			time.sleep(0.02)	
		#If Ball to the right	
		elif(centerpoint_x >320): 
			print("Ball right = Speed up left Motor")	
			motormaneuvrSPEED = (defaultSPEED-powerDIFF)*const_param
            if(motormaneuvrSPEED < 25):
            	motormaneuvrSPEED = 25
			
			print(" qL.ChangeDuryCycleL = ",defaultSPEED)
            print(" qR.ChangeDutyCycleR = ", motormaneuvrSPEED)
			qL.ChangeDutyCycle(defaultSPEED)
            qR.ChangeDutyCycle(motormaneuvrSPEED)    
			
			time.sleep(0.02) 
		
		#Code didnt work but im gonna let it uncommeted
		#time.sleep(1)
		#if centroid_y <280:
		#	print("Zu hoch zu Stop")
		#	qR.ChangeDutyCycle(0)
		#	qL.ChangeDutyCycle(0)
		#	time.sleep(0.1)
		#if centroid_x >280 and centroid_x <=320:
			#print("Objekt Voraus  === ",ctr)
            #qR.ChangeDutyCycle(40)
        	#qL.ChangeDutyCycle(40)
        	#time.sleep(0.1)
		#elif centroid_x <280:
			#print("Objekt Links")
            #qR.ChangeDutyCycle(40)
       		#qL.ChangeDutyCycle(25)
       		#time.sleep(0.1)
		#elif centroid_x >320:
			#print("Object rechts")
			#qR.ChangeDutyCycle(25)
        	#qL.ChangeDutyCycle(40)

			#time.sleep(0.1)
	
   	#Display full-color image
	#cv2.imshow(WINDOW_NAME, image)

    	# Force/Set centerpoint to None if ESC key was pressed
    	if cv2.waitKey(1) & 0xFF == 27:
        	centerpointXY = None
    
	return centerpointXY

#Main
if __name__ == '__main__':

    capture = cv2.VideoCapture(0)
		
    while True:

        okay, image = capture.read()

        if okay:

            if not findGreenBall(image):
	
		print("Stop object lost")
                qR.ChangeDutyCycle(0)
                qL.ChangeDutyCycle(0)
                time.sleep(0.1)
          
            if cv2.waitKey(1) & 0xFF == 27:
           	print("Waitkey")

        else:

           print('Couldnt capture image - webcam failed')
