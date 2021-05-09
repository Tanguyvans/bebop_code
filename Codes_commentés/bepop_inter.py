#!/usr/bin/env python
# -*- coding: utf-8 -*-

#version du code commentée; 2 versions car parfois les commentaires dans le code pouvaient poser problème lors de l'exécution

#on importe les modules nécessaires et les fonctions définies dans d'autres fichiers
import rospy
import math
import time
import sys 
from Tkinter import *
import ttk
import threading 
import bebop_control
import bebop_tracking
import bebop_arrow

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#on initialise la position du drone en (0,0,0)
x = 0
y = 0
z = 0


#on crée la fenêtre tkinter et on définit la couleur du background 
window= Tk()
window.config(background= "#41B77F")

global image_tracking
global arrow_sub
global record_sub

#fonction qui permet de récupérer la distance sélectionnée par l'utilisateur dans la combobox
def getdistance():
    distance = float(distance_choisie.get())
    return distance

#fonction qui permet de récupérer la vitesse sélectionnée par l'utilisateur dans la combobox
def getvitesse():
    vitesse = float(vitesse_choisie.get())
    return vitesse

#fonction qui permet de récupérer l'objet sélectionné par l'utilisateur dans la combobox pour le tracking
def getobjet():
    objet = objet_choisi.get()
    return objet


#on définit toutes les fonctions nécessaires pour le tracking en utilisant des threads
#on fait un thread pour la fonction qui permet de lancer la reconnaissance de flèches
#(comme toutes les fonctions, elle est définie dans un autre fichier, on l'a importé et on peut l'appeler maintenant pour instancier le thread lors de son appel)
def fleches_background():
    t = threading.Thread(target= bebop_arrow.arrow())
    t.start()

#stop flèches qui permet de s'unregister du topic 
def stop_fleches_background():
    t = threading.Thread(target= bebop_arrow.stop_arrow())
    t.start()

#décollage du drone
def takeoff_background():
    t = threading.Thread(target= bebop_control.takeoff())
    t.start()

#atterrissage du drone
def land_background():
    t = threading.Thread(target= bebop_control.land())
    t.start()

#avancer/reculer/gauche/droite/monter/descendre
# -> on récupère la distance et la vitesse que l'utilisateur a sélectionnées dans la combobox grâce aux deux fonctions définies juste avant
#et on fait appel à la fonction avec les bons paramètres
def avancer_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveX(vitesse, distance, True))
    t.start()

def reculer_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveX(vitesse, distance, False))
    t.start()

def gauche_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveY(vitesse, distance, True))
    t.start()

def droite_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveY(vitesse, distance, False))
    t.start()

def monter_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveZ(vitesse, distance, True))
    t.start()

def descendre_background():
    vitesse= getvitesse()
    distance=getdistance()
    t = threading.Thread(target= bebop_control.moveZ(vitesse, distance, False))
    t.start()

#rotation du drone
def rotationgauche_background():
    t = threading.Thread(target= bebop_control.rotate(10, 65, True))
    t.start()    

def rotationdroite_background():
    t = threading.Thread(target= bebop_control.rotate(10, 65, False))
    t.start()



#lancement du tracking avec l'objet sélectionné par l'utilisateur
#on récupère l'objet sélectionné avec la fonction getObjet() définie avant
#et on fait appel à la fonction tracking avec cet objet en paramètre
def tracking_background():
    objet= getobjet()
    t = threading.Thread(target= bebop_tracking.tracking(objet))
    t.start()

#unregister du topic tracking
def stop_tracking_background():
    t = threading.Thread(target= bebop_tracking.stop_tracking())
    t.start()

#recupération du flux vidéo de la caméra du drone
def camera_background():
    t = threading.Thread(target= bebop_control.record())
    t.start()

#unregister du topic
def stop_camera_background():
    t = threading.Thread(target= bebop_control.stop_record())
    t.start()

#fonction qui permet de revenir au point de départ (il y a une marge d'erreur, à améliorer)
def move_background():
    t = threading.Thread(target= bebop_control.home())
    t.start()



if __name__ == '__main__':
    try:
        #initialisation du node 
        rospy.init_node('bebop_controler', anonymous=True)

        position_topic = "/bebop/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, bebop_control.Odometry_Callback) 
        time.sleep(2)

        label_title= Label(window, text= "Drone", font=("Courrier",40), bg = "#41B77F", fg= "white")
        label_title.pack()

        #paramètre de la fenêtre
        window.title("Drone")
        window.geometry("1080x600")
        window.minsize(1000,500)

        #choix de la vitesse et de la distance par l'utilisateur
        vitesse_label= Label(window,text = "Vitesse").place(x= 30,y=50)
        distance_label= Label(window, text= "Distance").place(x=230,y=50)
        
        #combobox pour la distance 
        #initialisation de la variable
        distance_var= StringVar()
        #définition du combobox
        distance_choisie= ttk.Combobox(window, width=20,textvariable=distance_var)
        #valeurs du combobox -> voir ça comme une liste, on peut rajouter facilement des valeurs si nécessaires
        distance_choisie['values']=('1','2','3','5')
        #placement sur la fenêtre
        distance_choisie.place(x=80,y=50)
        #valeur par défaut
        distance_choisie.current(0)
        

        #combobox pour la vitesse
        #initialisation de la variable
        vitesse_var= StringVar()
        #définition combobox
        vitesse_choisie= ttk.Combobox(window, width=20,textvariable=vitesse_var)
        #valeurs possibles
        vitesse_choisie['values']=('0.1','0.2','0.3','0.5')
        #placement sur la fenêtre
        vitesse_choisie.place(x=300,y=50)
        #valeur par défaut
        vitesse_choisie.current(0)
        

        #combobox pour l'objet traqué
        #initialisation de la variable
        objet_var= StringVar()
        #définition combobox
        objet_choisi= ttk.Combobox(window, width=21,textvariable=objet_var)
        #valeurs possibles
        objet_choisi['values']=('person','car','dog')
        #placement sur la fenêtre
        objet_choisi.place(x=800,y=250)
        #valeur par défaut
        objet_choisi.current(0)
        

        #définitions de tous les boutons, à chaque fois on entre en paramètre le texte affiché, les dimensions du boutons, le placement et surtout,
        #la commande qui s'exécute lorsqu'on appuie sur le bouton. Celle-ci fait référence aux fonctions background définies auparavant et vont 
        #permettre d'instancier les threads

        #boutons décollage et atterrissage
        take_off_button = Button(window, text="Take-OFF", height= "3", width="20",command = takeoff_background).place(x=30,y=100)
        land_button= Button(window, text="Land", height= "3", width="20",command= land_background).place(x=30,y=150)
        
        #boutons pour le déplacement du drone
        avancer_button = Button(window, text="Avancer", height= "3", width="20",command = avancer_background).place(x=450,y=150)
        reculer_button= Button(window, text="Reculer", height= "3", width="20", command = reculer_background).place(x=450,y=350)
        gauche_button= Button(window, text="Gauche", height= "3", width="20", command= gauche_background).place(x=350,y=250)
        droite_button = Button(window, text="Droite", height= "3", width="20", command = droite_background).place(x=550,y=250)
        monter_button = Button(window, text= "Monter", height="3", width="20", command= monter_background).place(x = 450, y = 450)
        descendre_button = Button(window, text= "Descendre", height="3", width= "20", command= descendre_background).place(x= 450, y =520)

        #boutons pour les flèches
        fleches_button = Button(window, text="Detection de fleches", height= "3", width="20",command= fleches_background).place(x=800,y=70)
        stop_fleches_button = Button(window, text="Stop fleches", height= "3", width="20",command= stop_fleches_background).place(x=800,y=140)
        
        #boutons pour le tracking
        tracking_button = Button(window, text="Tracking", height= "3", width="20", command = tracking_background).place(x=800,y=300)
        stop_tracking_button = Button(window, text="Stop Tracking", height= "3", width="20", command = stop_tracking_background).place(x=800,y=380)
        
        #bouton pour le retour au point de départ
        move_button = Button(window, text= "BASE", height = "3", width= "20", command = move_background).place(x=30, y= 300)
        
        #boutons pour la caméra du drone
        cam_button = Button(window, text= "camera", height = "3", width= "20", command = camera_background).place(x=30, y= 350)
        stop_cam_button = Button(window, text= "stop cam", height = "3", width= "20", command = stop_camera_background).place(x=30, y= 400)
        
        #boutons pour les rotations gauche/droite du drone
        rotationgauche_button= Button(window, text= "Rotation gauche", height="3", width= "20", command= rotationgauche_background).place(x=800, y=450)
        rotationdroite_button = Button(window, text = "Rotation droite", height="3", width= "20", command= rotationdroite_background).place(x=800,y=520)
        
        #permet de boucler et d'afficher la fenêtre avec tkinter
        window.mainloop()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
