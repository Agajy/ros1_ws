#!/usr/bin/env python

"""
======================================================
 Fichier     : pose_recorder_gui.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : Interface graphique pour le noeud ROS pose_recorder, Permet de démarrer/arrêter l'enregistrement de poses
======================================================
"""

import rospy
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from std_msgs.msg import String
import threading
import os
from datetime import datetime

class PoseRecorderGUI:
    def __init__(self):
        # Initialiser ROS
        rospy.init_node('pose_recorder_gui', anonymous=True)
        
        # Variables d'état
        self.is_recording = False
        self.current_status = "Deconnecte"
        self.message_count = 0
        self.recording_start_time = None
        
        # Publishers et Subscribers ROS
        self.command_pub = rospy.Publisher(
            '/pose_recorder/command', String, queue_size=1
        )
        
        self.status_sub = rospy.Subscriber(
            '/pose_recorder/status', String, self.status_callback
        )
        
        # Création de l'interface graphique
        self.setup_gui()
        
        # Démarrer le thread ROS
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def setup_gui(self):
        """Créer l'interface graphique"""
        self.root = tk.Tk()
        self.root.title("ROS Pose Recorder")
        self.root.geometry("480x1000")
        self.root.configure(bg='white')
        
        # Frame principal avec padding
        main_frame = tk.Frame(self.root, bg='white', padx=20, pady=20)
        main_frame.pack(fill='both', expand=True)
        
        # Titre simple
        title_label = tk.Label(main_frame, text="ROS Pose Recorder", 
                              font=('Arial', 14, 'bold'), bg='white')
        title_label.pack(pady=(0, 15))
        
        # Configuration des topics
        topics_frame = tk.LabelFrame(main_frame, text="Configuration", 
                                   font=('Arial', 10), bg='white', padx=10, pady=10)
        topics_frame.pack(fill='x', pady=(0, 10))
        
        # tk.Label(topics_frame, text="Topics surveilles:", 
        #         font=('Arial', 9), bg='white').pack(anchor='w')
        
        # self.topics_var = tk.StringVar(value="/pose1, /pose2, /pose3, /pose4")
        # topics_entry = tk.Entry(topics_frame, textvariable=self.topics_var, 
        #                        font=('Arial', 9), width=50)
        # topics_entry.pack(fill='x', pady=(5, 10))
        
        # Sélection du dossier de sortie
        tk.Label(topics_frame, text="Dossier de sauvegarde:", 
                font=('Arial', 9), bg='white').pack(anchor='w')
        
        folder_frame = tk.Frame(topics_frame, bg='white')
        folder_frame.pack(fill='x', pady=(5, 0))
        
        self.output_dir_var = tk.StringVar(value=os.path.expanduser("~/pose_recordings"))
        self.folder_entry = tk.Entry(folder_frame, textvariable=self.output_dir_var, 
                                    font=('Arial', 9), width=35)
        self.folder_entry.pack(side='left', fill='x', expand=True)
        
        browse_btn = tk.Button(folder_frame, text="Parcourir", 
                              command=self.browse_folder, font=('Arial', 9))
        browse_btn.pack(side='right', padx=(5, 0))
        
        # Statut
        status_frame = tk.LabelFrame(main_frame, text="Statut", 
                                   font=('Arial', 10), bg='white', padx=10, pady=10)
        status_frame.pack(fill='x', pady=(0, 10))
        
        self.status_label = tk.Label(status_frame, text="Statut: Deconnecte", 
                                    font=('Arial', 10, 'bold'), bg='white')
        self.status_label.pack(anchor='w')
        
        self.message_label = tk.Label(status_frame, text="Messages: 0", 
                                     font=('Arial', 9), bg='white')
        self.message_label.pack(anchor='w', pady=(5, 0))
        
        self.time_label = tk.Label(status_frame, text="Duree: 00:00:00", 
                                  font=('Arial', 9), bg='white')
        self.time_label.pack(anchor='w', pady=(5, 0))
        
        # Boutons de contrôle
        control_frame = tk.LabelFrame(main_frame, text="Controle", 
                                    font=('Arial', 10), bg='white', padx=10, pady=10)
        control_frame.pack(fill='x', pady=(0, 10))
        
        button_frame = tk.Frame(control_frame, bg='white')
        button_frame.pack(fill='x')
        
        self.start_btn = tk.Button(button_frame, text="DEMARRER", 
                                  command=self.start_recording,
                                  font=('Arial', 11, 'bold'), bg='#4CAF50', fg='white',
                                  width=15, height=2)
        self.start_btn.pack(side='left', padx=(0, 10), pady=5)
        
        self.stop_btn = tk.Button(button_frame, text="ARRETER", 
                                 command=self.stop_recording,
                                 font=('Arial', 11, 'bold'), bg='#f44336', fg='white',
                                 width=15, height=2, state='disabled')
        self.stop_btn.pack(side='right', padx=(10, 0), pady=5)
        
        # Log des activités
        log_frame = tk.LabelFrame(main_frame, text="Journal des activites", 
                                 font=('Arial', 10), bg='white', padx=10, pady=10)
        log_frame.pack(fill='both', expand=True, pady=(0, 10))
        
        # Frame pour text et scrollbar
        text_frame = tk.Frame(log_frame, bg='white')
        text_frame.pack(fill='both', expand=True)
        
        self.log_text = tk.Text(text_frame, height=8, width=55, 
                               font=('Courier', 9), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Bouton de fermeture
        close_btn = tk.Button(main_frame, text="Fermer", command=self.close_application,
                             font=('Arial', 10))
        close_btn.pack(pady=(10, 0))
        
        # Protocole de fermeture
        self.root.protocol("WM_DELETE_WINDOW", self.close_application)
        
        # Ajouter un message initial
        self.add_log("Interface graphique initialisee")
        self.add_log("En attente de connexion avec le noeud pose_recorder...")
        
        # Timer pour mettre à jour la durée d'enregistrement
        self.update_timer()
        
    def browse_folder(self):
        """Ouvrir un dialogue pour choisir le dossier de sauvegarde"""
        try:
            folder = filedialog.askdirectory(
                title="Choisir le dossier de sauvegarde",
                initialdir=self.output_dir_var.get()
            )
            if folder:
                self.output_dir_var.set(folder)
                self.add_log("Dossier de sauvegarde change: {}".format(folder))
        except Exception as e:
            self.add_log("Erreur lors de la selection du dossier: {}".format(str(e)))
    
    def status_callback(self, msg):
        """Callback pour le statut du recorder"""
        status_data = msg.data
        
        # Mettre à jour l'interface dans le thread principal
        self.root.after(0, self.update_status_display, status_data)
    
    def update_status_display(self, status_data):
        """Mettre à jour l'affichage du statut"""
        try:
            if "recording" in status_data:
                if not self.is_recording:
                    self.is_recording = True
                    self.recording_start_time = datetime.now()
                    self.start_btn.config(state='disabled', bg='#cccccc')
                    self.stop_btn.config(state='normal', bg='#f44336')
                    self.status_label.config(text="Statut: ENREGISTREMENT EN COURS", fg='red')
                    self.add_log("Enregistrement demarre")
                
                # Extraire le nombre de messages s'il est présent
                if " - " in status_data:
                    try:
                        msg_info = status_data.split(" - ")[1]
                        if "messages" in msg_info:
                            self.message_count = int(msg_info.split(" ")[0])
                            self.message_label.config(text="Messages: {}".format(self.message_count))
                    except:
                        pass
                        
            elif status_data == "stopped" or status_data == "idle":
                if self.is_recording:
                    self.is_recording = False
                    self.recording_start_time = None
                    self.start_btn.config(state='normal', bg='#4CAF50')
                    self.stop_btn.config(state='disabled', bg='#cccccc')
                    self.status_label.config(text="Statut: ARRETE", fg='black')
                    self.time_label.config(text="Duree: 00:00:00")
                    self.add_log("Enregistrement arrete et sauvegarde")
                else:
                    self.status_label.config(text="Statut: EN ATTENTE", fg='blue')
            
            self.current_status = status_data
            
        except Exception as e:
            self.add_log("Erreur mise a jour statut: {}".format(str(e)))
    
    def start_recording(self):
        """Démarrer l'enregistrement"""
        try:
            # Vérifier que le dossier de sortie existe ou peut être créé
            output_dir = self.output_dir_var.get()
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                self.add_log("Dossier cree: {}".format(output_dir))
            
            # Envoyer la commande start avec le dossier de sauvegarde
            # Le backend doit gérer la synchronisation des topics et l'enregistrement
            msg = String()
            msg.data = "start:{}".format(output_dir)
            self.command_pub.publish(msg)
            
            self.add_log("Commande 'start' envoyee avec dossier: {}".format(output_dir))
            
        except Exception as e:
            try:
                messagebox.showerror("Erreur", "Impossible de demarrer l'enregistrement:\n{}".format(str(e)))
            except:
                pass
            self.add_log("Erreur: {}".format(str(e)))
    
    def stop_recording(self):
        """Arrêter l'enregistrement"""
        try:
            # Envoyer la commande stop
            msg = String()
            msg.data = "stop"
            self.command_pub.publish(msg)
            
            self.add_log("Commande 'stop' envoyee")
            
        except Exception as e:
            try:
                messagebox.showerror("Erreur", "Impossible d'arreter l'enregistrement:\n{}".format(str(e)))
            except:
                pass
            self.add_log("Erreur: {}".format(str(e)))
    
    def add_log(self, message):
        """Ajouter un message au journal"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_message = "[{}] {}\n".format(timestamp, message)
            
            self.log_text.insert(tk.END, log_message)
            self.log_text.see(tk.END)  # Scroll vers le bas
        except Exception as e:
            print("Erreur log: {}".format(str(e)))
    
    def update_timer(self):
        """Mettre à jour le timer d'enregistrement"""
        try:
            if self.is_recording and self.recording_start_time:
                elapsed = datetime.now() - self.recording_start_time
                hours, remainder = divmod(elapsed.seconds, 3600)
                minutes, seconds = divmod(remainder, 60)
                time_str = "{:02d}:{:02d}:{:02d}".format(hours, minutes, seconds)
                self.time_label.config(text="Duree: {}".format(time_str))
            
            # Programmer la prochaine mise à jour
            self.root.after(1000, self.update_timer)
        except Exception as e:
            print("Erreur timer: {}".format(str(e)))
            self.root.after(1000, self.update_timer)
    
    def ros_spin(self):
        """Thread pour ROS spin"""
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except:
                break
    
    def close_application(self):
        """Fermer l'application"""
        try:
            if self.is_recording:
                try:
                    result = messagebox.askyesno(
                        "Enregistrement en cours", 
                        "Un enregistrement est en cours. Voulez-vous l'arreter avant de fermer?"
                    )
                    if result:
                        self.stop_recording()
                        self.root.after(1000, self.force_close)
                        return
                except:
                    pass
            
            self.force_close()
        except Exception as e:
            print("Erreur fermeture: {}".format(str(e)))
            self.force_close()
    
    def force_close(self):
        """Fermer l'application de force"""
        try:
            rospy.signal_shutdown("GUI fermee")
        except:
            pass
        try:
            self.root.destroy()
        except:
            pass
    
    def run(self):
        """Lancer l'interface graphique"""
        try:
            self.root.mainloop()
        except Exception as e:
            print("Erreur interface: {}".format(str(e)))

def main():
    try:
        gui = PoseRecorderGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Erreur lors du demarrage de l'interface: {}".format(e))

if __name__ == '__main__':
    main()