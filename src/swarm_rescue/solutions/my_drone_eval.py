from typing import Optional
from solutions.my_drone_random import MyDroneRandom
from solutions.my_drone_lidar_communication import MyDroneLidarCommunication, MiscData
import random
from random import uniform
import math
import numpy as np
import time

class MyDroneEval(MyDroneLidarCommunication):

    def _init_(self):
        super()._init_()
        self.target_position = None

    def set_target(self, target_x, target_y):
        """ Définit la cible à atteindre """
        self.target_position = np.array([target_x, target_y])

    def get_position(self):
        """ Récupère la position actuelle du drone """
        return np.array(self.get_drone_position())  # Méthode héritée

    def compute_direction(self):
        """ Calcule la direction normalisée vers la cible """
        current_pos = self.get_position()
        direction = self.target_position - current_pos
        norm = np.linalg.norm(direction)
        return direction / norm if norm > 0 else np.array([0, 0])

    def move_to_target(self, speed=1.0):
        """ Déplace le drone vers la cible en évitant les obstacles """
        while np.linalg.norm(self.target_position - self.get_position()) > 0.5:  # Seuil de tolérance
            direction = self.compute_direction()
            self.set_drone_orientation(np.arctan2(direction[1], direction[0]))  # Ajuste l'orientation
            lidar_data = self.get_lidar_data()

            if min(lidar_data) < 1.0:  # Si un obstacle est détecté à moins de 1m
                self.avoid_obstacle(lidar_data)
            else:
                self.set_drone_speed(speed)  # Avance normalement

            time.sleep(0.1)  # Rafraîchissement

    def avoid_obstacle(self, lidar_data):
        """ Stratégie simple pour éviter un obstacle """
        self.set_drone_speed(0)  # Stoppe le drone
        time.sleep(0.5)  # Pause avant décision
        self.set_drone_orientation(self.get_drone_orientation() + np.pi / 4)  # Tourne à droite
        self.set_drone_speed(0.5)  # Avance lentement
        time.sleep(1)  # Laisse le temps au drone d'éviter

