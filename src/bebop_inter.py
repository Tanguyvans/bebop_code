#!/usr/bin/env python

import rospy
import math
import time
import sys 

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

x = 0
y = 0
z = 0

from Tkinter import *
import threading 
import bebop_control
 
window= Tk()
window.config(background= "#41B77F")

    
def fleches_background():
    t = threading.Thread(target= 'y a r')
    t.start()
def takeoff_background():
    t = threading.Thread(target= bebop_control.takeoff())
    t.start()
def land_background():
    t = threading.Thread(target= bebop_control.land())
    t.start()
def avancer_background():
    t = threading.Thread(target= bebop_control.moveX(0.1, 1.0, True))
    t.start()
def reculer_background():
    t = threading.Thread(target= bebop_control.moveX(0.1, 1.0, False))
    t.start()
def gauche_background():
    t = threading.Thread(target= bebop_control.moveY(0.1, 1.0, True))
    t.start()
def droite_background():
    t = threading.Thread(target= bebop_control.moveY(0.1, 1.0, False))
    t.start()
def tracking_background():
    t = threading.Thread(target= '')
    t.start()
def camera_background():
    t = threading.Thread(target= 'c')
    t.start()
def monter_background():
    t = threading.Thread(target= bebop_control.moveZ(0.1, 0.5, True))
    t.start()
def descendre_background():
    t = threading.Thread(target= bebop_control.moveZ(0.1, 1.0, False))
    t.start()
def rotationgauche_background():
    t = threading.Thread(target= 'rg')
    t.start()    
def rotationdroite_background():
    t = threading.Thread(target= 'rd')
    t.start()
    

if __name__ == '__main__':
    try:
        
        rospy.init_node('bebop_controler', anonymous=True)

        position_topic = "/bebop/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, bebop_control.Odometry_Callback) 
        time.sleep(2)

        label_title= Label(window, text= "Drone", font=("Courrier",40), bg = "#41B77F", fg= "white")
        label_title.pack()

        window.title("Drone")
        window.geometry("1080x600")
        window.minsize(1000,500)

        take_off_button = Button(window, text="Take-OFF", height= "3", width="20",command = takeoff_background).place(x=30,y=100)
        land_button= Button(window, text="Land", height= "3", width="20",command= land_background).place(x=30,y=300)
        avancer_button = Button(window, text="Avancer", height= "3", width="20",command = avancer_background).place(x=450,y=150)
        reculer_button= Button(window, text="Reculer", height= "3", width="20", command = reculer_background).place(x=450,y=350)
        gauche_button= Button(window, text="Gauche", height= "3", width="20", command= gauche_background).place(x=350,y=250)
        droite_button = Button(window, text="Droite", height= "3", width="20", command = droite_background).place(x=550,y=250)
        fleches_button = Button(window, text="Detection de fleches", height= "3", width="20",command= fleches_background).place(x=800,y=100)
        tracking_button = Button(window, text="Tracking", height= "3", width="20", command = tracking_background).place(x=800,y=300)
        camera_button = Button(window, text= "Camera du drone", height = "3", width= "20", command = camera_background).place(x=30, y= 450)
        monter_button = Button(window, text= "Monter", height="3", width="20", command= monter_background).place(x = 450, y = 450)
        descendre_button = Button(window, text= "Descendre", height="3", width= "20", command= descendre_background).place(x= 450, y =520)
        rotationgauche_button= Button(window, text= "Rotation gauche", height="3", width= "20", command= rotationgauche_background).place(x=800, y=450)
        rotationdroite_button = Button(window, text = "Rotation droite", height="3", width= "20", command= rotationdroite_background).place(x=800,y=520)
        
        window.mainloop()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")