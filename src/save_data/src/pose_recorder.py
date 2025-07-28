#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import json
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading

class PoseRecorder:
    def __init__(self):
        rospy.init_node('pose_recorder', anonymous=True)
        
        # Paramètres configurables
        self.topic_names = rospy.get_param('~topics', [
            '/pose1', '/pose2', '/pose3', '/pose4'
        ])
        self.output_dir = rospy.get_param('~output_dir', '/tmp/pose_recordings')
        
        # État d'enregistrement
        self.is_recording = False
        self.recording_data = {}
        self.individual_data = {}  # Nouveau: données individuelles par topic
        self.lock = threading.Lock()
        self.session_dir = None
        self.sync_buffer = {}  # Pour la synchronisation des topics
        self.last_written_stamp = None
        
        # Créer le dossier de sortie s'il n'existe pas
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Initialiser les structures de données
        for topic in self.topic_names:
            self.recording_data[topic] = []
            self.individual_data[topic] = []  # Nouveau: liste pour chaque topic
            self.sync_buffer[topic] = None
            
        # Subscribers pour les poses
        self.pose_subscribers = {}
        for topic in self.topic_names:
            self.pose_subscribers[topic] = rospy.Subscriber(
                topic, PoseStamped, 
                lambda msg, topic_name=topic: self.pose_callback(msg, topic_name)
            )
            
        # Subscriber pour les commandes start/stop
        self.command_sub = rospy.Subscriber(
            '/pose_recorder/command', String, self.command_callback
        )
        
        # Publisher pour le statut
        self.status_pub = rospy.Publisher(
            '/pose_recorder/status', String, queue_size=1
        )
        
        rospy.loginfo(f"pose_recorder.py : initialisé")
        rospy.loginfo(f"pose_recorder.py : Topics surveillés: {self.topic_names}")
        rospy.loginfo(f"pose_recorder.py : Dossier de sortie: {self.output_dir}")
        rospy.loginfo("pose_recorder.py : Envoyez 'start' ou 'stop' sur /pose_recorder/command")
        
    def pose_callback(self, msg, topic_name):
        """Callback pour les messages PoseStamped"""
        if self.is_recording:
            with self.lock:
                pose_data = {
                    'timestamp': msg.header.stamp.to_sec(),
                    'frame_id': msg.header.frame_id,
                    'position': {
                        'x': msg.pose.position.x,
                        'y': msg.pose.position.y,
                        'z': msg.pose.position.z
                    },
                    'orientation': {
                        'x': msg.pose.orientation.x,
                        'y': msg.pose.orientation.y,
                        'z': msg.pose.orientation.z,
                        'w': msg.pose.orientation.w
                    }
                }
                
                # Nouveau: Sauvegarder individuellement chaque message
                self.individual_data[topic_name].append(pose_data.copy())
                
                # Code existant pour la synchronisation
                self.sync_buffer[topic_name] = pose_data

                # Vérifier si tous les topics ont une nouvelle valeur
                if all(self.sync_buffer[t] is not None for t in self.topic_names):
                    # Synchronisation : on prend le plus grand timestamp
                    sync_stamp = max(self.sync_buffer[t]['timestamp'] for t in self.topic_names)
                    # Éviter d'écrire deux fois pour le même timestamp
                    if self.last_written_stamp != sync_stamp:
                        sync_row = {
                            t: self.sync_buffer[t] for t in self.topic_names
                        }
                        # Ajout à la liste d'enregistrements (ligne synchronisée)
                        self.recording_data.setdefault('rows', []).append(sync_row)
                        self.last_written_stamp = sync_stamp
                    # Réinitialiser le buffer pour la prochaine synchronisation
                    for t in self.topic_names:
                        self.sync_buffer[t] = None

    def command_callback(self, msg):
        """Callback pour les commandes start/stop"""
        command = msg.data.strip()
        if command.startswith('start:'):
            # Extraire le dossier de sauvegarde
            output_dir = command[6:]
            if output_dir:
                self.output_dir = output_dir
            self.start_recording()
        elif command.lower() == 'start':
            self.start_recording()
        elif command.lower() == 'stop':
            self.stop_recording()
        else:
            rospy.logwarn(f"Commande inconnue: {command}. Utilisez 'start' ou 'stop'")

    def start_recording(self):
        """Démarrer l'enregistrement"""
        if not self.is_recording:
            with self.lock:
                self.is_recording = True
                self.recording_data = {}
                self.individual_data = {topic: [] for topic in self.topic_names}  # Nouveau: réinitialiser
                self.last_written_stamp = None
                for topic in self.topic_names:
                    self.sync_buffer[topic] = None
                # Créer le dossier de session
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.session_dir = os.path.join(self.output_dir, f"recording_{timestamp}")
                if not os.path.exists(self.session_dir):
                    os.makedirs(self.session_dir)
            rospy.loginfo(f"pose_recorder.py : Enregistrement démarré dans {self.session_dir}")
            self.status_pub.publish(String("recording"))
        else:
            rospy.logwarn("Enregistrement déjà en cours")

    def stop_recording(self):
        """Arrêter l'enregistrement et sauvegarder"""
        if self.is_recording:
            with self.lock:
                self.is_recording = False
            self.save_data()
            rospy.loginfo("pose_recorder.py : Enregistrement arrêté et données sauvegardées")
            self.status_pub.publish(String("stopped"))
        else:
            rospy.logwarn("Aucun enregistrement en cours")

    def save_data(self):
        """Sauvegarder les données synchronisées et individuelles"""
        if not self.session_dir:
            rospy.logwarn("Aucune session à sauvegarder")
            return
        
        # Sauvegarder les données synchronisées (code existant)
        rows = self.recording_data.get('rows', [])
        filepath = os.path.join(self.session_dir, "poses_sync.json")
        with open(filepath, 'w') as f:
            json.dump(rows, f, indent=2)
        rospy.loginfo(f"pose_recorder.py : Sauvegardé {len(rows)} lignes synchronisées dans {filepath}")

        # Nouveau: Sauvegarder les données individuelles pour chaque topic
        total_individual_msgs = 0
        for topic_name in self.topic_names:
            topic_data = self.individual_data[topic_name]
            if topic_data:  # Seulement si il y a des données
                # Nettoyer le nom du topic pour le nom de fichier
                clean_topic_name = topic_name.replace('/', '_').lstrip('_')
                individual_filepath = os.path.join(self.session_dir, f"{clean_topic_name}_individual.json")
                
                with open(individual_filepath, 'w') as f:
                    json.dump(topic_data, f, indent=2)
                
                rospy.loginfo(f"pose_recorder.py : Sauvegardé {len(topic_data)} messages individuels pour {topic_name} dans {individual_filepath}")
                total_individual_msgs += len(topic_data)

        # Sauvegarder un fichier de métadonnées mis à jour
        metadata = {
            'recording_time': datetime.now().strftime("%Y%m%d_%H%M%S"),
            'topics': self.topic_names,
            'total_synchronized_rows': len(rows),
            'total_individual_messages': total_individual_msgs,
            'individual_counts': {topic: len(self.individual_data[topic]) for topic in self.topic_names}
        }
        metadata_file = os.path.join(self.session_dir, 'metadata.json')
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        rospy.loginfo(f"pose_recorder.py : Session sauvegardée dans: {self.session_dir}")
        rospy.loginfo(f"pose_recorder.py : Total messages individuels: {total_individual_msgs}")

    def run(self):
        """Boucle principale"""
        rate = rospy.Rate(1)  # 1 Hz pour les messages de statut
        
        while not rospy.is_shutdown():
            # Publier le statut actuel
            if self.is_recording:
                sync_msgs = len(self.recording_data.get('rows', []))
                individual_msgs = sum(len(self.individual_data[topic]) for topic in self.topic_names)
                status_msg = f"recording - {sync_msgs} sync, {individual_msgs} individual"
            else:
                status_msg = "idle"
                
            self.status_pub.publish(String(status_msg))
            rate.sleep()

def main():
    try:
        recorder = PoseRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("pose_recorder.py : Noeud PoseRecorder arrêté")

if __name__ == '__main__':
    main()