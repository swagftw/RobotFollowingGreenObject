# RobotFollowingGreenObject


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

Demonstration video:

https://youtu.be/9njUdVDlwN4

https://youtu.be/9MIgMDpZxdw

https://youtu.be/KkVSJUBkuxY

https://youtu.be/5TTddz3KNTQ
