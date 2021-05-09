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

# largeur et longueur de la fenetre 
rowWindow = 0
colWindow = 0

# fonction permettant d arrete le tracking
def stop_tracking():
    # on reinitialise les dimension pour le prochain objet
    width = 0
    heigth =0
    # on n est plus subscriber du topic /bounding_callback
    image_tracking.unregister()

# fonction image callback sert uniquement a recuperer les dimensions de l'image
def image_callback(ros_image):
    # appelle le bridge pour le traitement opencv
    global bridge
    # appelle les variable pour les dimension de la fenetre
    global rowWindow
    global colWindow
    
    # conversion imageROS --> en image lisible par opencv
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    # exeption lorsque une arrive lors d une conversion
    except CvBridgeError as e:
        print(e)
    # recupere les dimension de l image
    (rowWindow, colWindow, channels) = cv_image.shape
    # on veut plus etre subscriber de image_raw
    image_sub.unregister()

# fonction utilisant l algorithme tracking
def Bounding_Callback(msg):
    
    # appelle les variables de dimension
    global width 
    global height 

    # appelle les variables du centre de la bounding box
    global objCenterX
    global objCenterY

    #recupere le milieu de l'image
    x_middle_frame = colWindow/2
    y_middle_frame = rowWindow/2

    # hauteur souhaitee 
    y_good_heigh_frame = rowWindow*0.35

    #initialise la liste contenant les personnes/objet traque
    listPerson = []
    # initialise la liste contenant les positions
    listPosition = []

    # On recupere l'ensemble des personnes/objet traquer avec leurs positions 
    for box in msg.bounding_boxes:
        # si la bounding box est de la meme categorie de l objet traque
        if (box.Class == objType):
            # met les variables de positions d un objet dans une liste
            listPosition.append(box.xmax) 
            listPosition.append(box.xmin) 
            listPosition.append(box.ymax)
            listPosition.append(box.ymin)

            # met la liste contenant les position dans la liste de personne/objet traque
            listPerson.append(listPosition)
            # vide la liste regroupant les positions
            listPosition = []

    # si la liste d objet traque/ personne n est pas vide on peut continuer le traking
    if listPerson : 
        # recupere le centre de l image 
        centerX = colWindow/2
        centerY = rowWindow/2
        # initialise la variable distance minimale avec un tres grande valeur ainsi on sait qu elle sera change
        DistCenter = 1000000
        # initialise la variable permettant de retrouver l element de la liste ayant la plus courte deistance
        posePerson = 0
        
        # si les dimensions sont egale a 0 l objet traque n est aps encore paru a l ecran 
        if (width==0 and height==0):
            # on cherche l objet le plus au centre de l ecran en parcourant la liste d objet
            for i in range(len(listPerson)): 
                # position du centre de l'objet dans l'espace
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                # distance entre l'objet et le centre de l'image
                DistCenter0 = (math.sqrt((centerX-centerX0)**2+(centerY-centerY0)**2))
                
                # si la distance de l element i est inferieur a la distance minimal actuel 
                # on met a jour les information (position de l objet et la distance minimal)
                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i
            
            # declare les dimensions de l'images
            width = listPerson[posePerson][0]-listPerson[posePerson][1]
            height = listPerson[posePerson][2]-listPerson[posePerson][3]

            # met a jour la derniere position enregistre du centre de la bounding box
            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        # si les variable width et height ne son pas egale a zero 
        # ce n est aps la premiere fois que l objet parait a l ecran
        else : 
            # regarde l objet ayant la plus courte distance entre la position auctuel et la derniere position enregistre
            for i in range(len(listPerson)): 
                # centre de la bounding box i
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                # distance entre le centre de labounding box i et la derniere  position enregistre
                DistCenter0 = (math.sqrt((objCenterX-centerX0)**2+(objCenterY-centerY0)**2))
                # si la distance de l objet i est inferieur a la distance minimal on met a jour les variables
                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i

            # met a jour la derniere position de l objet enregistre
            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        # calcul des dimenssions actuel de l objet
        width0 = listPerson[posePerson][0]-listPerson[posePerson][1]
        height0 = listPerson[posePerson][2]-listPerson[posePerson][3]

        # difference entre les dimensions initiales et actuel
        x_difference = width - width0
        y_difference = height - height0

        # calcul du centre selon x de l objet traque
        x_middle_person = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
        x_rot_difference = x_middle_frame - x_middle_person

        # difference en hauteur souhaitee et la hauteur maximum de la bounding box
        z_difference = (listPerson[posePerson][3]-y_good_heigh_frame)
        
        #cree un publiser pour envoyer des commandes de deplacement au drone 
        velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        # les commande de deplacement doivent doivent etre sous la forme twist
        velocity_message = Twist()

        # deplacement lineaire selon X
        # la difference entre les dimension sont suffisament grande il faut envoyer une commande de deplacement au drone
        if ((abs(y_difference)-10) > 0):
            # calcul la vitesse de deplacement a partir de la difference multiplier par un coef de prportionalite
            x_speed = round(y_difference*0.001, 1)
            # gare la commande en memoire 
            velocity_message.linear.x = x_speed
        # si la difference sont trop petite la commande de deplacement en nulle
        else:
            # enregistre une commande de deplacement egale a 0
            velocity_message.linear.x = 0

        # deplacement yaw
        # la difference entre les dimension sont suffisament grande il faut envoyer une commande de deplacement au drone
        if ((abs(x_rot_difference)-20) > 0):
            # calcul la vitesse angulaire a l aide de la difference multiplier par un coef de propportionnalite
            rot_speed = round(x_rot_difference*0.001, 1)
            # enregistre la commande de deplacement
            velocity_message.angular.z = rot_speed
        # si la difference est trop petite la commande de deplacement est nulle
        else: 
            # enregistre la commande de deplacement est nulle
            velocity_message.angular.z = 0

        # deplacement du drone selon z 
        # si la difference entre la hauteur voulue et la hauteur maximal de la bounding box est suffisament grande 
        # on doit envoyer une commande de deplacement au drone 
        if (abs(z_difference)-10 > 0):
            # calcul de la commandede deplacement a partir de la difference multiplier par un coef de proportionnalité
            z_speed = round(z_difference*0.001,1)
            # enregistrement de la commande de deplacement
            velocity_message.linear.z = -z_speed
        # si la difference est trop petit le drone de doit pas effecuter de deplacement
        else:
            # enregistre de la commande de deplacement comme etant nulle
            velocity_message.linear.z = 0
      
        # envoi de la commande au drone
        velocity_publisher.publish(velocity_message)

            
# fonction demarrant le tracking
def tracking(name):
    
    # definit la variable categorie d objet traque comme global
    #recupere la valeur de la variable
    global objType 
    objType= name

    # definit les variable de dimension comme global
    global width 
    global height 

    # par securite un definit comme les variables de dimesion comme nulle 
    width = 0
    height = 0
    
    # variable global pour les subscriber 
    # ainsi il est possible de unregister
    global image_sub
    global image_tracking

    # variable avec le nom du topic
    darknet_topic = "/darknet_ros/bounding_boxes"
    # cree le subscriber pour le tracking
    image_tracking = rospy.Subscriber(darknet_topic, BoundingBoxes, Bounding_Callback)
    # cree le connaitre les dimensions de l image
    image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)
