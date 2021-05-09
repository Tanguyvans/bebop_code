#!/usr/bin/env python

import rospy
import math
import time

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# categorie d'objet
objType = 'none'

# dimension de l'objet
width = 0
height = 0

# position precedente de l'objet
objCenterX = 0
objCenterY = 0

rowWindow = 0
colWindow = 0

z = 0

def stop_tracking():
    width = 0
    heigth =0
    image_tracking.unregister()

def image_callback(ros_image):
    global bridge
    global rowWindow
    global colWindow

    print('je rentre dans image callback')
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    (rowWindow, colWindow, channels) = cv_image.shape
    image_sub.unregister()

def Odometry_Callback(msg):
    global z

    z = msg.pose.pose.position.z

def Bounding_Callback(msg):

    #print(objType)
    #print('in bounding callback')

    global width 
    global height 

    global objCenterX
    global objCenterY

    #recupere le milieu de l'image
    x_middle_frame = colWindow/2
    y_middle_frame = rowWindow/2

    # hauteur souhaitee 
    y_good_heigh_frame = rowWindow*0.35

    #initialise les listes contenant les personnes
    listPerson = []
    listPosition = []

    # On recupere l'ensemble des personnes avec leurs positions 
    for box in msg.bounding_boxes:
        if (box.Class == objType):
            
            print (objType)
            listPosition.append(box.xmax) 
            listPosition.append(box.xmin) 
            listPosition.append(box.ymax)
            listPosition.append(box.ymin)

            listPerson.append(listPosition)
            listPosition = []

    if listPerson : 
        centerX = colWindow/2
        centerY = rowWindow/2
        DistCenter = 1000000
        posePerson = 0

        if (width==0 and height==0):
            for i in range(len(listPerson)): 
                # position du centre de l'objet dans l'espace
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                # distance entre l'objet et le centre de l'image
                DistCenter0 = (math.sqrt((centerX-centerX0)**2+(centerY-centerY0)**2))

                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i
            
            # declare les dimensions de l'images
            width = listPerson[posePerson][0]-listPerson[posePerson][1]
            height = listPerson[posePerson][2]-listPerson[posePerson][3]

            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        else : 
            for i in range(len(listPerson)): 
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                DistCenter0 = (math.sqrt((objCenterX-centerX0)**2+(objCenterY-centerY0)**2))
                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i

            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        width0 = listPerson[posePerson][0]-listPerson[posePerson][1]
        height0 = listPerson[posePerson][2]-listPerson[posePerson][3]

        # Position voulue - position actuel 
        x_difference = width - width0
        y_difference = height - height0

        # milieu en fonction de x
        x_middle_person = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
        x_rot_difference = x_middle_frame - x_middle_person

        #calcul en fonction de Z
        z_difference = (listPerson[posePerson][3]-y_good_heigh_frame)
        
        #initialise pub et msg
        velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        velocity_message = Twist()

        # deplacement selon X
        if ((abs(y_difference)-10) > 0):
            x_speed = round(y_difference*0.001, 1)
            print('x_speed', x_speed)
            velocity_message.linear.x = x_speed
        else:
            velocity_message.linear.x = 0

        # orientation du drone
        if ((abs(x_rot_difference)-20) > 0):
            rot_speed = round(x_rot_difference*0.001, 1)
            velocity_message.angular.z = rot_speed
        else: 
            velocity_message.angular.z = 0

        # hauteur du drone 
        if (abs(z_difference)-10 > 0):
            z_speed = round(z_difference*0.001,1)
            velocity_message.linear.z = -z_speed
        else:
            velocity_message.linear.z = 0
      
        velocity_publisher.publish(velocity_message)

            

def tracking(name):

    global objType 
    objType= name

    global width 
    global height 

    width = 0
    height = 0


    global image_sub
    global image_tracking

    darknet_topic = "/darknet_ros/bounding_boxes"
    image_tracking = rospy.Subscriber(darknet_topic, BoundingBoxes, Bounding_Callback)
    #print('in the tracking')
    image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)
