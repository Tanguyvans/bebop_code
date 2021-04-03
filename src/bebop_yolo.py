#!/usr/bin/env python

import rospy
import math
import time

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes

width = 0
height = 0

def Bounding_Callback(msg):
    
    global width 
    global height 

    for box in msg.bounding_boxes:
        if (box.Class == "person"):
            rospy.loginfo(box.Class)
            #rospy.loginfo("Xmin {} Xmax {} Ymin {} Ymax {}".format(box.xmin, box.xmax, box.ymin, box.ymax))

            width0 = box.xmax-box.xmin
            height0 = box.ymax-box.ymin

            if (width==0 and height==0):
                width = box.xmax-box.xmin
                height = box.ymax-box.ymin

            if ( width > width0):
                rospy.loginfo("Il faut avancer")

            elif ( width < width0):
                rospy.loginfo("Il faut reculer")


if __name__ == '__main__':
    try:
        
        rospy.init_node('detection_sub', anonymous=True)

        darknet_topic = "/darknet_ros/bounding_boxes"
        pose_subscriber = rospy.Subscriber(darknet_topic, BoundingBoxes, Bounding_Callback) 
        time.sleep(2)
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")