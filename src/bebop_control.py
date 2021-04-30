#!/usr/bin/env python

import rospy
import math
import time
import sys 

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError

x = 0
y = 0
z = 0
yaw = 0

record_bridge = CvBridge()

def Odometry_Callback(msg):
    global x
    global y
    global z

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    yaw = msg.pose.pose.orientation.z
    #rospy.loginfo('valeur de x est: %f', x)
    #rospy.loginfo('valeur de y est: %f', y)
    #rospy.loginfo('valeur de z est: %f', z)
    
def takeoff():
    pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(2.)

def land():
    print('land')
    pub = rospy.Publisher('/bebop/land', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(4.)

def moveX(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()

    global x, y
    x0=x
    y0=y

    if (is_forward):
        velocity_message.linear.x =abs(speed)
    else:
    	velocity_message.linear.x =-abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("bebop moves forwards")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print  distance_moved               
            if  not (distance_moved<distance):
                rospy.loginfo("reached")
                break
        
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)
    
def moveY(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()

    global x, y
    x0=x
    y0=y

    if (is_forward):
        velocity_message.linear.y =abs(speed)
    else:
    	velocity_message.linear.y =-abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("bebop moves forwards")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print  distance_moved               
            if  not (distance_moved<distance):
                rospy.loginfo("reached")
                break
        
    velocity_message.linear.y =0
    velocity_publisher.publish(velocity_message)

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
        
    velocity_message.linear.z =0
    velocity_publisher.publish(velocity_message)

def rotate2 (angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
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

def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))
    relative_angle = math.radians(relative_angle_degree)

    global yaw
    yaw0 = yaw

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    rotation_moved = 0.0
    loop_rate = rospy.Rate(10) 

    while True :
        rospy.loginfo("bebop rotates")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        rotation_moved = yaw-yaw0

        if  not (rotation_moved < relative_angle):
            rospy.loginfo("reached")
            break

    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def home():

    global x, y, yaw

    desired_angle = 0.0
    delta_y = 0-y
    delta_x = 0-x
    desired_distance = abs(math.sqrt((x ** 2) + (y ** 2))) 

    if (x < 0 and y < 0):
        desired_angle = math.asin(delta_y/desired_distance)
        desired_angle = math.degrees(desired_angle)

    elif (x < 0 and y > 0):
        desired_angle = math.asin(delta_y/desired_distance)
        desired_angle = math.degrees(desired_angle)

    elif (x > 0 and y < 0):
        angle = math.asin(delta_y/desired_distance)
        angle = math.degrees(desired_angle)
        desired_angle = 180 - angle

    elif (x > 0 and y > 0):
        angle = math.asin(delta_y/desired_distance)
        angle = math.degrees(desired_angle)
        desired_angle = 180 + angle
    elif (x == 0 and y>0):
        desired_angle = 270
    elif (x == 0 and y < 0):
        desired_angle = 90
    elif (y == 0 and x > 0):
        desired_angle = 180
    elif (y == 0 and x < 0):
        desired_angle = 0

    else: 
        print("you are allready at base")

    if (x !=0 and y != 0):
        relative_angle = desired_angle - yaw

        rotate(0.1, relative_angle, True)
        time.sleep(2)
        moveX(0.1, desired_distance, True)


def record_callback(ros_image):
    #print('Processing frame / Delay:%6.3f' % (rospy.Time.now() - ros_image.header.stamp).to_sec())
    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = record_bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    (rows, cols, channels) = cv_image.shape
    writer= cv2.VideoWriter('basicvideo.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 20, (width,height))

    writer.write(cv_image)
    cv2.imshow('frame', cv_image)

def stop_record():
    cv2.destroyAllWindows()
    record_sub.unregister()

  
def record():

    global record_sub
    record_sub = rospy.Subscriber("/bebop/image_raw",Image, record_callback)
