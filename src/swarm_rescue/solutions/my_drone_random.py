"""
Simple random controller
The Drone will move forward and turn for a random angle when an obstacle is hit
"""
import math
import random
from typing import Optional

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle


class MyDroneRandom(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         display_lidar_graph=False,
                         **kwargs)
        self.counterStraight = 0 #compteur utilisé pour suivre le nombre d'itérations òù le drone se déplace en ligne droite.
        self.angleStopTurning = random.uniform(-math.pi, math.pi) #définit un angle aléatoire entre −π−π et ππ (en radians), qui représente la direction vers laquelle le drone doit arrêter de tourner
        self.distStopStraight = random.uniform(10, 50) #Définit une distance aléatoire entre 10 et 50  que le drone doit parcourir en ligne droite avant de changer de comportement
        self.isTurning = False #Un booléen qui indique si le drone est en train de tourner ou non

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def process_lidar_sensor(self): #fonction pour détecter une collision
        """
        Returns True if the drone collided an obstacle
        """
        if self.lidar_values() is None: #return les valeurs du capteur lidar
            return False

        collided = False #on suppose qu'il n'y a pas de collision
        dist = min(self.lidar_values()) #pour trouver la distance la plus petite entre le drone et un obstacle

        if dist < 40: #indique que le drone est trop proche d'un obstacle
            collided = True #alors il y a une collision

        return collided #indique si une collision a été detectée

    def control(self): # Pour gérer une collision : une rotation quand le drone rencontre un obstacle
        """
        The Drone will move forward and turn for a random angle when an obstacle is hit
        """
        command_straight = {"forward": 1.0,
                            "lateral": 0.0,
                            "rotation": 0.0,
                            "grasper": 0}

        command_turn = {"forward": 0.0,
                        "lateral": 0.0,
                        "rotation": 1.0,
                        "grasper": 0}

        collided = self.process_lidar_sensor()

        self.counterStraight += 1

        if collided and not self.isTurning and self.counterStraight > self.distStopStraight:
            self.isTurning = True
            self.angleStopTurning = random.uniform(-math.pi, math.pi)

        measured_angle = 0
        if self.measured_compass_angle() is not None:
            measured_angle = self.measured_compass_angle()

        diff_angle = normalize_angle(self.angleStopTurning - measured_angle)
        if self.isTurning and abs(diff_angle) < 0.2:
            self.isTurning = False
            self.counterStraight = 0
            self.distStopStraight = random.uniform(10, 50)

        if self.isTurning:
            return command_turn
        else:
            return command_straight
