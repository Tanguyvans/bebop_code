#!/usr/bin/env python

#fichier qui reprend toutes les commandes pour contrôler le drone 

#on importe les modules nécessaires
import rospy
import math
import time
import sys 
import cv2

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

#initialisation de la position et de l'angle du drone
x = 0
y = 0
z = 0
yaw = 0

#appel à la fonction pour récupérer l'image du drone
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

#décollage
def takeoff():
    pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(2.)

#atterrissage
def land():
    print('land')
    pub = rospy.Publisher('/bebop/land', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(4.)

#déplacement selon X
def moveX(speed, distance, is_forward):
    # recupere la distance a parcourir
    # la viesse de deplacement
    # le sens de deplacement
    
    # cree le publisher permettant d envoyer les commandes aux drones
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    # le publisher a besion d un message de type twist 
    velocity_message = Twist()

    # on recupere et definit comme globale les variable de position
    global x, y
    # ces variables servirons pour retenir la position initiale 
    x0=x
    y0=y

    # regarde si la le drone va vers l avant ou l arriere 
    # sert uniquement comme securite
    if (is_forward):
        velocity_message.linear.x =abs(speed)
    else:
    	velocity_message.linear.x =-abs(speed)

    # initialise la variable distance moved 
    # cette variable permet de stocker la distance parcoure par le drone
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    #tant qu'on n'a pas atteint la distance voulue, on boucle
    while True :
            # equivalent de print 
            rospy.loginfo("bebop moves forwards")
            # on envoie la commande de deplacement au drone
            velocity_publisher.publish(velocity_message)
            # on attent un moment tant que le drone se deplace
            loop_rate.sleep()   
            # on calcul la distance parcourue par le drone
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            
            #si la distance parcoure n est pas plus grande que la distance que l on doit parcourir on continue a boucler
            if  not (distance_moved<distance):
                #quand la distance voulue est atteinte, on affiche 'reached'
                rospy.loginfo("reached")
                break
    #declare la commande de deplacement comme etant nulle
    velocity_message.linear.x =0
    # envoie la commande de deplacement au drone 
    velocity_publisher.publish(velocity_message)

#déplacement selon Y 
#reprend le meme principe que pour le deplacment selon X
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

#déplacement selon Z
#reprend le meme principe que pour le deplacment selon X
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

#rotation du drone
def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    # recupere la vitesse angulaire 
    # la vitesse angulaire 
    # le sens de deplacement 
    
    # cree le publisher permettant d envoyer des commande de deplacement au drone
    velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=0)
    # le message a envoyer est de type twist
    velocity_message = Twist()
    # converti la vitesse angualaire degree --> radians
    angular_speed=math.radians(abs(angular_speed_degree))

    # securite pour s assurer que le drone troune dans le sens voulu
    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    # on publie la vitesse a 5Hz
    loop_rate = rospy.Rate(5) 
    # on regarde le temps actuel
    # sert a calculer la distance parcourue
    t0 = rospy.Time.now().to_sec()

    # boucle pour envoyer le deplacement au drone 
    while True :
        # equivalent de print
        rospy.loginfo("bebop rotates")
        # envoie la commande de deplacement au drone
        velocity_publisher.publish(velocity_message)

        # regarde le temps actuel
        t1 = rospy.Time.now().to_sec()
        # calcule la distance angulaire parcourue 
        # on fait delta t * vitesse
        current_angle_degree = (t1-t0)*angular_speed_degree
        # attent que le drone se deplace
        loop_rate.sleep()

        # si le drone a parcouru une plus grande distance que prevue on quitte la boucle 
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    # definit le commande de deplacement comme etant nulle 
    velocity_message.angular.z =0
    # envoie la commande de deplacement au drone 
    velocity_publisher.publish(velocity_message)

#retour au point de départ
def home():
    
    global x, y, yaw
    
    # cree la valeur d angle a tourner
    desired_angle = 0.0
    # variable delta x et y
    delta_y = 0-y
    delta_x = 0-x
    # calcul la distance a parcourir par pythagore
    # la position de depart est 0,0 et la position actuel est x,y
    desired_distance = abs(math.sqrt((x ** 2) + (y ** 2))) 

    # on doit visualiser la grille spacial du drone comme le cercle trigonometrique
    # le drone se trouve dans le premier quadrant
    if (x < 0 and y < 0):
        # calcule l'angle a tourner
        # l'angle obtenu est en dregre
        desired_angle = math.asin(delta_y/desired_distance)
        
    # drone se trouve dans le deuxieme quadrant
    elif (x < 0 and y > 0):
        # calcule l'angle a tourner
        # l'angle obtenu est en dregre
        desired_angle = math.asin(delta_y/desired_distance)

    # droen se toruve dans le 4 quadrant
    elif (x > 0 and y < 0):
        angle = math.asin(delta_y/desired_distance)
        desired_angle = 180 - angle

    # drone se trouve dans le premier quadrant
    elif (x > 0 and y > 0):
        angle = math.asin(delta_y/desired_distance)
        desired_angle = 180 + angle
    # drone se toruve sur l axe y positif
    elif (x == 0 and y>0):
        desired_angle = 270
    # drone se trouve sur l axe y negatif
    elif (x == 0 and y < 0):
        desired_angle = 90
    # drone se trouve sur l axe x positif
    elif (y == 0 and x > 0):
        desired_angle = 180
    # drone se trouve sur l axe x negatif
    elif (y == 0 and x < 0):
        desired_angle = 0

    # droen se trouve deja a la base
    else: 
        print("you are allready at base")

    # si le drone n est pas a la base on appelle des fonction de deplacement
    if (x !=0 and y != 0):
        relative_angle = desired_angle - yaw

        rotate(10, relative_angle, True)
        time.sleep(2)
        moveX(0.1, desired_distance, True)
        land()

#on récupère le flux vidéo
def record_callback(ros_image):
    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        # conversion du format en un format lisible par opencv
        # on choisit le format bgr8
        cv_image = record_bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    # montre le flux video sur un ecran
    cv2.imshow('frame', cv_image)

#on s'unregister du topic
def stop_record():
    # ferme toutes les fenetre 
    cv2.destroyAllWindows()
    # on n est plus subscriber de record_sub
    record_sub.unregister()

  
def record():
    global record_sub
    # cree un subscriber pour recuperer le flux video du drone
    record_sub = rospy.Subscriber("/bebop/image_raw",Image, record_callback)
