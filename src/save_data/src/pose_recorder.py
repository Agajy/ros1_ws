#!/usr/bin/env python

"""
======================================================
 Fichier     : pose_recorder.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : Noeud ROS pour enregistrer des poses synchronisées à partir de plusieurs topics PoseStamped.
              Permet l'enregistrement individuel des messages et la synchronisation périodique.
              Gère les commandes de démarrage/arrêt via un topic dédié.
              Enregistre les données dans un dossier spécifié, avec des statistiques de synchronisation.
======================================================
"""

import rospy
import os
import json
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading
import time

class PoseRecorder:
    def __init__(self):
        rospy.init_node('pose_recorder', anonymous=True)
        
        # Paramètres configurables
        self.topic_names = rospy.get_param('~topics', [
            '/pose1', '/pose2', '/pose3', '/pose4'
        ])
        self.output_dir = rospy.get_param('~output_dir', '/tmp/pose_recordings')
        self.sync_timeout = rospy.get_param('~sync_timeout', 0.1)  # Timeout en secondes pour la synchronisation
        self.sync_rate = rospy.get_param('~sync_rate', 10.0)  # Hz - fréquence d'enregistrement synchronisé
        
        # État d'enregistrement
        self.is_recording = False
        self.recording_data = {}
        self.individual_data = {}
        self.lock = threading.Lock()
        self.session_dir = None
        self.sync_buffer = {}
        self.sync_timestamps = {}  # Timestamps pour chaque topic
        self.last_sync_time = 0
        
        # Créer le dossier de sortie s'il n'existe pas
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Initialiser les structures de données
        for topic in self.topic_names:
            self.recording_data[topic] = []
            self.individual_data[topic] = []
            self.sync_buffer[topic] = None
            self.sync_timestamps[topic] = 0
            
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
        
        # Timer pour la synchronisation périodique
        self.sync_timer = rospy.Timer(
            rospy.Duration(1.0 / self.sync_rate), 
            self.sync_timer_callback
        )
        
        rospy.loginfo(f"pose_recorder.py : initialisé")
        rospy.loginfo(f"pose_recorder.py : Topics surveillés: {self.topic_names}")
        rospy.loginfo(f"pose_recorder.py : Dossier de sortie: {self.output_dir}")
        rospy.loginfo(f"pose_recorder.py : Timeout de sync: {self.sync_timeout}s, Rate: {self.sync_rate}Hz")
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
                
                # Sauvegarder individuellement chaque message
                self.individual_data[topic_name].append(pose_data.copy())
                
                # Mettre à jour le buffer de synchronisation avec la dernière valeur
                self.sync_buffer[topic_name] = pose_data
                self.sync_timestamps[topic_name] = time.time()

    def sync_timer_callback(self, event):
        """Callback du timer pour l'enregistrement synchronisé périodique"""
        if not self.is_recording:
            return
            
        with self.lock:
            current_time = time.time()
            sync_row = {}
            
            # Pour chaque topic, prendre la dernière valeur disponible
            for topic in self.topic_names:
                if self.sync_buffer[topic] is not None:
                    # Vérifier si la donnée n'est pas trop ancienne
                    age = current_time - self.sync_timestamps[topic]
                    if age <= self.sync_timeout:
                        sync_row[topic] = self.sync_buffer[topic].copy()
                    else:
                        # Donnée trop ancienne, marquer comme manquante
                        sync_row[topic] = None
                        rospy.logwarn_throttle(5, f"Topic {topic} trop ancien ({age:.3f}s)")
                else:
                    # Aucune donnée disponible pour ce topic
                    sync_row[topic] = None
            
            # Enregistrer même si certains topics manquent
            if any(sync_row[t] is not None for t in self.topic_names):
                # Au moins un topic a des données
                sync_row['sync_timestamp'] = current_time
                self.recording_data.setdefault('rows', []).append(sync_row)
            
            # Optionnel : nettoyer les buffers trop anciens
            for topic in self.topic_names:
                if (self.sync_buffer[topic] is not None and 
                    current_time - self.sync_timestamps[topic] > self.sync_timeout * 2):
                    self.sync_buffer[topic] = None

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
                self.individual_data = {topic: [] for topic in self.topic_names}
                self.last_sync_time = time.time()
                for topic in self.topic_names:
                    self.sync_buffer[topic] = None
                    self.sync_timestamps[topic] = 0
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
        
        # Sauvegarder les données synchronisées
        rows = self.recording_data.get('rows', [])
        filepath = os.path.join(self.session_dir, "poses_sync.json")
        with open(filepath, 'w') as f:
            json.dump(rows, f, indent=2)
        
        # Calculer les statistiques de synchronisation
        total_sync_points = len(rows)
        missing_stats = {topic: 0 for topic in self.topic_names}
        for row in rows:
            for topic in self.topic_names:
                if row.get(topic) is None:
                    missing_stats[topic] += 1
        
        rospy.loginfo(f"pose_recorder.py : Sauvegardé {total_sync_points} points synchronisés dans {filepath}")
        for topic, missing_count in missing_stats.items():
            if missing_count > 0:
                rospy.loginfo(f"pose_recorder.py : {topic}: {missing_count}/{total_sync_points} points manquants ({100*missing_count/total_sync_points:.1f}%)")

        # Sauvegarder les données individuelles pour chaque topic
        total_individual_msgs = 0
        for topic_name in self.topic_names:
            topic_data = self.individual_data[topic_name]
            if topic_data:
                clean_topic_name = topic_name.replace('/', '_').lstrip('_')
                individual_filepath = os.path.join(self.session_dir, f"{clean_topic_name}_individual.json")
                
                with open(individual_filepath, 'w') as f:
                    json.dump(topic_data, f, indent=2)
                
                rospy.loginfo(f"pose_recorder.py : Sauvegardé {len(topic_data)} messages individuels pour {topic_name}")
                total_individual_msgs += len(topic_data)

        # Sauvegarder les métadonnées avec statistiques de synchronisation
        metadata = {
            'recording_time': datetime.now().strftime("%Y%m%d_%H%M%S"),
            'topics': self.topic_names,
            'sync_settings': {
                'rate_hz': self.sync_rate,
                'timeout_s': self.sync_timeout
            },
            'total_synchronized_points': total_sync_points,
            'total_individual_messages': total_individual_msgs,
            'individual_counts': {topic: len(self.individual_data[topic]) for topic in self.topic_names},
            'missing_data_stats': missing_stats
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