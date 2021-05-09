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

# catégorie d'objet
objType = 'none'

# dimension de l'objet
width = 0
height = 0

# position précédente de l'objet
objCenterX = 0
objCenterY = 0

# largeur et longueur de la fenêtre 
rowWindow = 0
colWindow = 0

# fonction permettant d'arreter le tracking
def stop_tracking():
    # on reinitialise les dimensions pour le prochain objet
    width = 0
    heigth =0
    # on n est plus subscriber du topic /bounding_callback
    image_tracking.unregister()

# fonction image callback sert uniquement à recuperer les dimensions de l'image
def image_callback(ros_image):
    # appelle le bridge pour le traitement opencv
    global bridge
    # appelle les variables pour les dimensions de la fenêtre
    global rowWindow
    global colWindow
    
    # conversion imageROS --> en image lisible par opencv
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    # exception lorsqu'une erreur arrive lors d une conversion
    except CvBridgeError as e:
        print(e)
    # recupere les dimensions de l image
    (rowWindow, colWindow, channels) = cv_image.shape
    # on ne veut plus etre subscriber de image_raw->unregister
    image_sub.unregister()

# fonction utilisant l'algorithme tracking
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

    #initialise la liste contenant les personnes/objet traqués
    listPerson = []
    # initialise la liste contenant les positions
    listPosition = []

    # On recupere l'ensemble des personnes/objet traqué(e)s avec leurs positions 
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

    # si la liste d objet traque/ personne n est pas vide on peut continuer le tracking
    if listPerson : 
        # recupere le centre de l image 
        centerX = colWindow/2
        centerY = rowWindow/2
        # initialise la variable distance minimale avec un tres grande valeur ainsi on sait qu elle sera change
        DistCenter = 1000000
        # initialise la variable permettant de retrouver l element de la liste ayant la plus courte distance
        posePerson = 0
        
        # si les dimensions sont egales à 0 l objet traque n est pas encore apparu a l'écran 
        if (width==0 and height==0):
            # on cherche l objet le plus au centre de l ecran en parcourant la liste d objets
            for i in range(len(listPerson)): 
                # position du centre de l'objet dans l'espace
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                # distance entre l'objet et le centre de l'image
                DistCenter0 = (math.sqrt((centerX-centerX0)**2+(centerY-centerY0)**2))
                
                # si la distance de l element i est inferieure a la distance minimale actuelle 
                # on met a jour les informations (position de l objet et la distance minimale)
                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i
            
            # declare les dimensions de l'image
            width = listPerson[posePerson][0]-listPerson[posePerson][1]
            height = listPerson[posePerson][2]-listPerson[posePerson][3]

            # met a jour la derniere position enregistrée du centre de la bounding box
            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        # si les variables width et height ne son pas egales a zero 
        # ce n est pas la premiere fois que l objet parait a l ecran
        else : 
            # regarde l objet ayant la plus courte distance entre la position actuelle et la derniere position enregistrée
            for i in range(len(listPerson)): 
                # centre de la bounding box i
                centerX0 = (listPerson[i][0] +listPerson[i][1] )/2
                centerY0 = (listPerson[i][2] +listPerson[i][3] )/2
                # distance entre le centre de labounding box i et la derniere  position enregistre
                DistCenter0 = (math.sqrt((objCenterX-centerX0)**2+(objCenterY-centerY0)**2))
                # si la distance de l objet i est inferieure à la distance minimale on met à jour les variables
                if (DistCenter > DistCenter0):
                    DistCenter = DistCenter0
                    posePerson = i

            # met a jour la derniere position de l'objet enregistrée
            objCenterX = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
            objCenterY = (listPerson[posePerson][2] +listPerson[posePerson][3] )/2

        # calcul des dimensions actuelles de l objet
        width0 = listPerson[posePerson][0]-listPerson[posePerson][1]
        height0 = listPerson[posePerson][2]-listPerson[posePerson][3]

        # difference entre les dimensions initiales et actuelles
        x_difference = width - width0
        y_difference = height - height0

        # calcul du centre selon x de l objet traqué
        x_middle_person = (listPerson[posePerson][0] +listPerson[posePerson][1] )/2
        x_rot_difference = x_middle_frame - x_middle_person

        # difference en hauteur souhaitee et la hauteur maximum de la bounding box
        z_difference = (listPerson[posePerson][3]-y_good_heigh_frame)
        
        #cree un publisher pour envoyer des commandes de deplacement au drone 
        velocity_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        # les commandes de deplacement doivent doivent être sous la forme twist
        velocity_message = Twist()

        # deplacement lineaire selon X
        # la difference entre les dimensions est suffisament grande-> il faut envoyer une commande de deplacement au drone
        if ((abs(y_difference)-10) > 0):
            # calcul la vitesse de deplacement a partir de la difference multipliée par un coef de proportionnalité
            x_speed = round(y_difference*0.001, 1)
            # garde la commande en mémoire 
            velocity_message.linear.x = x_speed
        # si la difference est trop petite la commande de deplacement est nulle
        else:
            # enregistre une commande de deplacement égale à 0
            velocity_message.linear.x = 0

        # deplacement yaw (angulaire)
        # la difference entre les dimensions est suffisament grande -> il faut envoyer une commande de déplacement au drone
        if ((abs(x_rot_difference)-20) > 0):
            # calcul de la vitesse angulaire à l'aide de la différence multipliée par un coef de proportionnalité
            rot_speed = round(x_rot_difference*0.001, 1)
            # enregistre la commande de deplacement
            velocity_message.angular.z = rot_speed
        # si la difference est trop petite la commande de deplacement est nulle
        else: 
            # enregistre la commande de deplacement est nulle
            velocity_message.angular.z = 0

        # deplacement du drone selon z 
        # si la difference entre la hauteur voulue et la hauteur maximale de la bounding box est suffisament grande 
        # on doit envoyer une commande de deplacement au drone 
        if (abs(z_difference)-10 > 0):
            # calcul de la commande de deplacement à partir de la différence multipliée par un coef de proportionnalité
            z_speed = round(z_difference*0.001,1)
            # enregistrement de la commande de déplacement
            velocity_message.linear.z = -z_speed
        # si la difference est trop petit le drone ne doit pas effctuer de deplacement
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

    # definit les variables de dimension comme global
    global width 
    global height 

    # par securite un definit les variables de dimension comme nulle 
    width = 0
    height = 0
    
    # variable globale pour les subscriber 
    # ainsi il est possible de unregister
    global image_sub
    global image_tracking

    # variable avec le nom du topic
    darknet_topic = "/darknet_ros/bounding_boxes"
    # cree le subscriber pour le tracking
    image_tracking = rospy.Subscriber(darknet_topic, BoundingBoxes, Bounding_Callback)
    # cree le connaitre les dimensions de l image
    image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)
