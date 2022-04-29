#! /usr/bin/env python

from numpy.core.einsumfunc import _compute_size_by_dict
import rospy # Import the Python library for ROS
from std_msgs.msg import Float32 # Import the Float32 message from the std_msgs package
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster 


#///////////////////////////////////////////////////////////////////

from tkinter import *
import webbrowser
import can
from tinymovr import Tinymovr
from tinymovr.iface.can_bus import CANBus, guess_channel
from time import sleep
from tinymovr.units import get_registry
from tinymovr.plotter import plot
import tkinter as tk
import os 
from PIL import Image, ImageTk
import threading
import time
from math import pi

#///////////////////////////////////////////////////////////////////

ureg = get_registry()
mA = ureg.milliampere
rad = ureg.radian
s = ureg.second
channel = guess_channel(bustype_hint='slcan')
bus = can.Bus(bustype="slcan", channel=channel, bitrate=1000000)
iface = CANBus(bus)
tm1 = Tinymovr(node_id=1, iface=iface)
variable = "Information du programme"
retour = "retour position"
toto = 1
retour_cal = 0
com_courbe = 0


#///////////////////////////////////////////////////////////////////
def application():
    ######################################################################################
    ######################## Initialisation de la fenêtre ################################
    def create_widgets():
        #create_title()
        Retour_programme(variable)
        create_subtitle()
        #image_moteur()
        Retour_position(retour)
        Commande_position()
        #codeur()
        Commande_vitesse()
        Commande_stop()
        Documentation()
        Graphique()
        Initialisation()
        Maintien_position()
        val = case()
        instruction_position()
        instruction_vitesse()
        instruction_acceleration()
        instruction_decceleration()
        #acquisition_position()
        


    ################################################################
    ######################## Titire ################################
    def create_title():
        label_title = Label(frame, text="Mad 5010 tinymovr", font=("Arial", 40), bg='#bf6e0c', fg='white')                    
        label_title.grid(row=0, column=0, sticky=E, columnspan = 2, pady= 30)

    def create_subtitle():
        label_subtitle = Label(frame, text="Position du moteur", font=("Courrier", 20), bg='#041d38',fg='white')                          
        label_subtitle.grid(row=1, column=0, sticky=W, pady=10)
        label_subtitle2 = Label(frame, text="Consigne position (degrès)", font=("Courrier", 15), bg='#041d38',fg='white')                          
        label_subtitle2.grid(row=2, column=0, sticky=W, pady=10)
        label_subtitle3 = Label(frame, text="Consigne vitesse (trs/sec)", font=("Courrier", 15), bg='#041d38',fg='white')                          
        label_subtitle3.grid(row=2, column=1, pady=10)
        label_subtitle4 = Label(frame, text="Consigne accélération", font=("Courrier", 15), bg='#041d38',fg='white')                          
        label_subtitle4.grid(row=3, column=0, sticky=W, pady=10)
        label_subtitle5 = Label(frame, text="Consigne deccélération", font=("Courrier", 15), bg='#041d38',fg='white')                          
        label_subtitle5.grid(row=3, column=1, pady=10)


    def image_moteur():   
        monimage = Image.open("/home/philippe/mad.png")    ## Chargement d'une image à partir de PIL
        photo = ImageTk.PhotoImage(monimage)   ## Création d'une image compatible Tkinter
        label = Label(image=photo)    ## Insertion de l'image de l
        label.image = photo 			## Maintient en vie de photo dans un objet non détruit par le garbage                                   ## pour pas que l'image disparaisse du label
        label.pack()


    ################################################################
    ######################## Champs ################################
    def Retour_position(retour):
        # Commande à envoyer au moteur
        commande_position = Entry(frame, font=("Arial", 15))
        commande_position.grid(row=1, column=1, ipadx = 250, sticky=W, pady=10)
        # affichage de la fonction dans le champs : 
        commande_position.delete(0, END)
        commande_position.insert(0, retour)

    def instruction_position():
        global position_moteur
        value = tk.StringVar(frame)
        # value.set("Saisir une vitesse en tours/min")   
        position_moteur = tk.Entry(frame, textvariable=value, font=("Arial", 20), width=5)
        position_moteur.grid(row=2, column=1, sticky=W, pady=5)
        #position_moteur.bind("<Return>", print_name)
    
    def instruction_vitesse():
        global vitesse_moteur
        defaut = "0.2"
        value = tk.StringVar(frame)
        # value.set("Saisir une vitesse en tours/min")   
        vitesse_moteur = tk.Entry(frame, textvariable=value, font=("Arial", 20), width=5)
        vitesse_moteur.grid(row=2, column=1, sticky=E, pady=5)
        vitesse_moteur.delete(0, END)
        vitesse_moteur.insert(0, defaut)
        #position_moteur.bind("<Return>", print_name)

    def instruction_acceleration():
        global acc_moteur
        defaut = "40"
        value = tk.StringVar(frame)
        acc_moteur = tk.Entry(frame, textvariable=value, font=("Arial", 20), width=5)
        acc_moteur.grid(row=3, column=1, sticky=W, pady=5) 
        acc_moteur.delete(0, END)
        acc_moteur.insert(0, defaut)
        #commande_moteur.bind("<Return>", print_name)
    
    def instruction_decceleration():
        global decc_moteur
        defaut = "40"
        value = tk.StringVar(frame)
        # value.set("Saisir une vitesse en tours/min")
        decc_moteur = tk.Entry(frame, textvariable=value, font=("Arial", 20), width=5)
        decc_moteur.grid(row=3, column=1, sticky=E, pady=5) 
        decc_moteur.delete(0, END)
        decc_moteur.insert(0, defaut)
        #commande_moteur.bind("<Return>", print_name)

    def Retour_programme(variable):
        information = Entry(frame, font=("Arial", 20))
        information.grid(row=4, column=0, ipadx = 230, columnspan = 2,sticky=W, pady=30)
        # affichage de la fonction dans le champs : 
        information.delete(0, END)
        information.insert(0, variable)

    def case():
        global choix1
        global choix2
        choix1 = IntVar()
        choix2 = IntVar()
        bouton = Checkbutton(frame, text="Position/Velocity", variable = choix1)
        bouton.grid(row=6, column=0, sticky=W)
        bouton = Checkbutton(frame, text="Position/Couple", variable = choix2)
        bouton.grid(row=7, column=0, sticky=W)

    #################################################################
    ######################## Boutons ################################
    def Commande_stop():
        print("Commande stop")
        def stop():
            print("stop")
            global com_courbe
            #tm1.estop() 
            if (com_courbe == 1):
                com_courbe = 0 
                print("Com_courbe : ", com_courbe)



        button = Button(frame, text="STOP", font=("Courrier", 30), bg='red', fg='white', command = stop)                
        button.grid(row=0, column=2, sticky=W, padx=5)

    def acquisition_position():
        button = Button(frame, text="acquisition", font=("Courrier", 15), bg='white', fg='#41B77F', command = codeur)                  
        button.grid(row=1, column=2, padx=5)
        # Commande à envoyer au moteur

    def Commande_position():
        button = Button(frame, text="Validation", font=("Courrier", 15), bg='white', fg='#41B77F', command = consigne_position_vit)                  
        button.grid(row=2, column=2, padx=5)
        # Commande à envoyer au moteur

    def Commande_vitesse():
        button = Button(frame, text="Validation", font=("Courrier", 15), bg='white', fg='#41B77F', command = consigne_acc_decc)                  
        button.grid(row=3, column=2, pady=5)
        # Commande à envoyer au moteur

    def Documentation():
        button = Button(frame, text="Documentation", font=("Courrier", 15), bg='white', fg='darkblue',command=open_tinymovr_documentation)                  
        button.grid(row=8, column=0, sticky=W, ipadx = 80, pady= 20)

    def Graphique():
        button = Button(frame, text="Graphique", font=("Courrier", 15), bg='white', fg='darkred', command=pyplot)                  
        button.grid(row=5, column=0, sticky=W, ipadx = 50, pady= 2)

    def Maintien_position():
        button = Button(frame, text="Maintien Position", font=("Courrier", 15), bg='white', fg='darkred', command=Maintien_pos)                  
        button.grid(row=5, column=1, ipadx = 50, pady= 2)

    def Initialisation():
        button = Button(frame, text="Initialisation", font=("Courrier", 15), bg='white', fg='darkred', command=init)                  
        button.grid(row=5, column=2, sticky=E, ipadx = 50, pady= 2)
        
    ###################################################################
    ######################## Fonctions ################################
    def init() :
        #lancement calibration
        tm1.calibrate()
        variable2 = 'Initialisation'
        Retour_programme(variable2)
        print("initialisation en cours")
        
    def Maintien_pos() :
        #lancement calibration
        tm1.position_control()
        sleep(0.1)
        print('Moteur en mode controle de position')
        tm1.set_limits(velocity=300000.0, current=15.0)
        variable = 'Moteur en mote controle de position'
        sleep(0.1)
        Retour_programme(variable)
        tm1.set_gains(position=80.0, velocity=0.00005)
        sleep(0.1)
        print("Consigne de maintien effecté")
        variable = "Consigne de maintien effectuée"
        Retour_programme(variable)

    def consigne_acc_decc():
        acc = acc_moteur.get()
        decc = decc_moteur.get()
        int_acc = int(acc) # 40
        int_decc = int(decc) # 40
        tm1.set_max_plan_acc_dec( int_acc* 3.14 * ureg('rad/s/s'), int_decc* 3.14 * ureg('rad/s/s'))
        sleep(0.1)
        variable = 'Acceleration et déceleration initialisées'
        Retour_programme(variable)

    def consigne_position_vit():
        global com_courbe
        pos = position_moteur.get()
        vit = vitesse_moteur.get()
        if (pos == "999"):
            if (com_courbe == 0):
                com_courbe = 1 
            print("Com_courbe : ", com_courbe)
        int_vit = float(vit)
        int_pos = float(pos)
        if (int_pos > 0):
            pos_degres = 360/int_pos
            pos_degres2 = 1 / pos_degres
        else:
            pos_degres2 = 0


            
        if (int_vit < 0.5):
            tm1.plan_v_limit(pos_degres2*251.3274* ureg('rad'),int_vit*251.3274* ureg('rad/s'))
            sleep(0.1)
        variable = 'Commande position et vitesse envoyées'
        Retour_programme(variable)

    def pyplot():
        val = choix1.get()
        print(val)
        if (val == 1):
            plot(lambda: [tm1.encoder_estimates])

    def codeur():
        global retour_cal
        global toto
        global com_vel 
        global com_courbe
        com_vel = 0

        while toto == 1:

            #retour = tm1.encoder_estimates
            #retour_str = str(retour)
            #retour_net = retour_str[23:29]
            #retour_int = float(retour_net)
            #retour_cal = (retour_int/327600)*360
            retour_cal = tm1.Iq
            Retour_position(retour_cal)

            #if (retour_cal < 3 and com_courbe ==1):
             #   com_vel = 30
              #  print("Envoi commande1")
               # tm1.plan_v_limit(com_vel*251.3274* ureg('rad'),0.1*251.3274* ureg('rad/s'))
            #if (retour_cal > 29 and com_courbe == 1):
             #   com_vel = 0
              #  print("Envoi commande1")
               # tm1.plan_v_limit(com_vel*251.3274* ureg('rad'),0.1*251.3274* ureg('rad/s')) 
     
            time.sleep(1/20)

    def open_tinymovr_documentation():
        webbrowser.open_new("https://tinymovr.readthedocs.io/en/latest/")

    ###########################################################################
    ######################## Paramêtre fenêtre ################################
    window = Tk()
    window.title("Contrôle moteur")
    w = window.winfo_screenwidth()
    h = window.winfo_screenheight()
    window.geometry("%dx%d" % (w,h))
    #window.geometry("480x360")
    window.minsize(480, 360)
    window.config(background='#041d38')


    # initialization des composants
    frame = Frame(window, bg='#041d38')

    # creation des composants
    create_widgets()
    th1 = threading.Thread(target = codeur)
    th1.start()



    # empaquetage
    frame.pack(expand=YES)

    # afficher
    window.mainloop()
    toto = 0
    christo = 0

th2 = threading.Thread(target = application)
th2.start()


#///////////////////////////////////////////////////////////////////
rospy.init_node('topic_publisher') # Initiate a Node named 'topic_publisher'
pub = rospy.Publisher('/valeur_moteur', JointState, queue_size=1) # Create a Publisher object, that will publish on the /valeur_moteur topic messages of type Float32
rate = rospy.Rate(20) # Set a publish rate of 2 Hz
count = Float32() # Create a var of type Int32
count.data = 0 # Initialize 'count' variable

# Robot state
joint_Base_Axe1 = 0  
joint_Axe1_Axe2 = 0  
joint_Axe2_Axe3 = 0
joint_Axe3_Axe4 = 0  
joint_Axe4_Axe5 = 0  
joint_Axe5_Axe6 = 0  



# Message declarations
joint_state = JointState()  

joint_state.name.insert(0,"joint_Base_Axe1")
joint_state.position.insert(0,joint_Base_Axe1)

joint_state.name.insert(1,"joint_Axe1_Axe2")
joint_state.position.insert(1,joint_Axe1_Axe2)

joint_state.name.insert(2,"joint_Axe2_Axe3")
joint_state.position.insert(2,joint_Axe2_Axe3)

joint_state.name.insert(3,"joint_Axe3_Axe4")
joint_state.position.insert(3,joint_Axe3_Axe4)

joint_state.name.insert(4,"joint_Axe4_Axe5")
joint_state.position.insert(4,joint_Axe4_Axe5)

joint_state.name.insert(5,"joint_Axe5_Axe6")
joint_state.position.insert(5,joint_Axe5_Axe6)

while not rospy.is_shutdown():  # Create a loop that will go until someone stops the program execution
    joint_state.header.stamp = rospy.get_rostime()



    # Send the joint state and transform

    joint_state.position[1] = -(retour_cal / 360)*2*pi
    pub.publish(joint_state)


    # if(joint_Axe2_Axe3 >= 2.36):
    # joint_Axe2_Axe3 = -2.36
    
    # joint_Axe1_Axe2 = joint_Axe1_Axe2 - 0.0031415
    # joint_Axe2_Axe3 = joint_Axe2_Axe3 + 0.00236

    # print("Valeur joint_Axe1_Axe2 : ", joint_Axe1_Axe2)
    # print("Valeur joint_Axe1_Axe2 : ", joint_Axe2_Axe3)
    print("Valeur : ", retour_cal)

    # This will adjust as needed per iteration
    rate.sleep()