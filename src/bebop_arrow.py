#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

import numpy as np
import math

variable_rotate = 0

bridge = CvBridge()

def stop_arrow():
    arrow_sub.unregister()

def land():
    pub = rospy.Publisher('/bebop/land', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(4.)

def image_callback(ros_image):
  #print('Processing frame / Delay:%6.3f' % (rospy.Time.now() - ros_image.header.stamp).to_sec())
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  (rows, cols, channels) = cv_image.shape

  #print ( rows/2, cols/2)
  #lower_red = (30,150,80) # entre 0 et 60 // 2 pour opencv 
  #upper_red = (255,255,180)

  #binary_image_mask = filter_color(cv_image, lower_red, upper_red)
  #getContours(binary_image_mask, cv_image)

  imgGray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
  imgBlur = cv2.GaussianBlur(imgGray, (7,7), 1)
  imgCanny = cv2.Canny(imgBlur,50,50)
  getContours(imgCanny, cv_image)
  #cv2.imshow("Image window", cv_image)
  cv2.waitKey(3)

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
  #convert the image into the HSV color space
  hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
  cv2.imshow("Image window", hsv_image)
  mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
  return mask

def distance (x,y,a,b):
    return math.sqrt((x-a)**2 + (y-b)**2)

def getContours(imgCanny, img):

    print("salut")

    global variable_rotate

    if (variable_rotate > 0):
        variable_rotate = variable_rotate -1

    (rows, cols, channels) = img.shape

    largeur_min = 0.2*rows
    largeur_max= 0.3*rows

    d_gauche= 0.4*rows
    d_droite = 1.2*rows
    d_audessus=0.2*cols
    d_endessous= 0.6*cols

    _, contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>500:
            cv2.drawContours(img, cnt, -1, (0,0,255),3)  
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            if objCor == 7:
                '''
                if 2000>area: 
                    if x <d_gauche:
                        print("Go left")
                    if x+w>d_droite:
                        print("Go right") 
                '''
                '''
                if y <d_audessus:
                    cv2.putText(img, 'DESCENDRE',(300,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
                if y+h>d_endessous:
                    cv2.putText(img, 'MONTER',(300,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
                '''
                coteGauche = distance(x,y,x,y+h)
                coteHaut = distance(x,y,x+w,y)
                
                if coteGauche > coteHaut:  
                    min_x = 1000000000
                    max_x = 0
                    mid = 0
                    cmt = 0
                    for point in approx:
                        if point[0][1] < min_x:
                            min_x = point[0][1]
                        if point[0][1] > max_x:
                            max_x = point[0][1]
                        else:
                            if cmt <= 5:
                                mid = point[0][1]
                        cmt += 1
                    dMin = abs(min_x-mid)
                    dMax = abs(max_x-mid)
                        
                    if dMin < dMax:
                        objectType = 'verticale haut'
                        #rospy.loginfo('haut')  
                    else:
                        objectType = "verticale bas" 
                        #print (w, h)
                        #rospy.loginfo('droite')
                        if 2500>area:
                            print("avance")
                            moveX(True)

                        elif area > 2500:
                            print("Land")
                            #land()
                         
                else : 
                    min_x = 1000000000
                    max_x = 0
                    mid = 0
                    cmt = 0
                    for point in approx:
                        if point[0][0] < min_x:
                            min_x = point[0][0]
                        if point[0][0] > max_x:
                            max_x = point[0][0]
                        else : 
                            if cmt <=5:
                                mid = point[0][0]
                        cmt += 1
                    dMin = abs(min_x-mid)
                    dMax = abs(max_x-mid)
                        
                    if dMin < dMax:
                        objectType = 'horizontale gauche'
                        #rospy.loginfo('gauche')
                        if 2500>area:
                            print("avance")
                            moveX(True)

                        elif area > 2500:
                            print("stop et tourne")
                            moveX(False)
                            print(variable_rotate)
                            if (variable_rotate < 20):
                                rotate(10, 65, False)
                                variable_rotate += 40
                    else:
                        objectType = "horizontale droite" 
                        #print (w, h)
                        #rospy.loginfo('droite')
                        if 2500>area:
                            print("avance")
                            moveX(True)

                        elif area > 2500:
                            print("stop et tourne")
                            moveX(False)
                            print(variable_rotate)
                            if (variable_rotate < 20):
                                rotate(10, 65, True)
                                variable_rotate += 40
                            

                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
                cv2.putText(img, objectType, (x,y-10),cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,0),2)     
                
        else: 
            objectType = "None"

def moveX(Verif):      
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()
    if (Verif):
        velocity_message.linear.x = 0.1
    else: 
        velocity_message.linear.x = 0
    
    velocity_publisher.publish(velocity_message)

def moveY(direction, Verif):      
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()
    if (Verif):
        if (direction == "left"):
            velocity_message.linear.Y = -0.1
        else:
            velocity_message.linear.Y = 0.1
    else: 
        velocity_message.linear.Y = 0
    
    velocity_publisher.publish(velocity_message)

def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=0)
    velocity_message = Twist()
    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(5) 
    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("bebop rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def arrow():

    global arrow_sub
    arrow_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)


