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

width = 0
height = 0

rowWindow = 0
colWindow = 0

z = 0

def image_callback(ros_image):
  #print('Processing frame / Delay:%6.3f' % (rospy.Time.now() - ros_image.header.stamp).to_sec())
  global bridge
  global rowWindow
  global colWindow
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
    
    global width 
    global height 

    #print(rowWindow, colWindow)
    x_middle_frame = colWindow/2

    for box in msg.bounding_boxes:
        if (box.Class == "person"):
            #rospy.loginfo(box.Class)
            #rospy.loginfo("Xmin {} Xmax {} Ymin {} Ymax {}".format(box.xmin, box.xmax, box.ymin, box.ymax))

            width0 = box.xmax-box.xmin
            height0 = box.ymax-box.ymin

            if (width==0 and height==0):
                width = box.xmax-box.xmin
                height = box.ymax-box.ymin

            # Position voulue - position actuel 
            x_difference = width - width0

            # milieu 
            x_middle_person = (box.xmax +box.xmin )/2
            x_rot_difference = x_middle_frame - x_middle_person
            print(x_rot_difference)
            
            #if (x_difference+10 > 0):
            #    moveX('avance', x_difference)
            #elif ( 0 > x_difference-10):
            #    moveX('recule', x_difference)
            #else:
            #    moveX('stop')

            if ((x_rot_difference-20) > 0):
                rotate('left', x_rot_difference)
            elif ((x_rot_difference+20) < 0):
                rotate('right', x_rot_difference)
            else: 
                rotate('stop', x_rot_difference)



def takeoff():
    pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(2.)

def moveX(Verif):  
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()
    if (Verif == 'avance'):
        rospy.loginfo("Il faut avancer")
        velocity_message.linear.x = 0.2
    elif (Verif == 'stop'): 
        rospy.loginfo("tu es bien")
        velocity_message.linear.x = 0
    elif (Verif == 'recule'):
        rospy.loginfo("Il faut reculer")
        velocity_message.linear.x = -0.2

    #velocity_publisher.publish(velocity_message)

def moveZ(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()

    global z
    z0=z

    if (is_forward):
        velocity_message.linear.z =abs(speed)
    else:
    	velocity_message.linear.z =-abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("bebop goes UP")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt((z-z0) ** 2))
            print  distance_moved               
            if  not (distance_moved<distance):
                rospy.loginfo("reached")
                break

    
    velocity_publisher.publish(velocity_message)

def rotate (Verif):
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()
    
    if (Verif == 'left'):
        rospy.loginfo("gauche")
        velocity_message.angular.z = 0.2
    elif (Verif == 'right'):
        rospy.loginfo("droite")
        velocity_message.angular.z = -0.2
    elif (Verif == 'good'):
        rospy.loginfo("ok")
        velocity_message.angular.z = 0

    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        
        rospy.init_node('detection_sub', anonymous=True)

        darknet_topic = "/darknet_ros/bounding_boxes"
        pose_subscriber = rospy.Subscriber(darknet_topic, BoundingBoxes, Bounding_Callback)

        image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback) 
        
        position_topic = "/bebop/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry,Odometry_Callback)     
        time.sleep(2)

        takeoff()
        moveZ(0.4, 1.5, True)
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
