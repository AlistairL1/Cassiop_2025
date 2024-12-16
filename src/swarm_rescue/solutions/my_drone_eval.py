from typing import Optional
from solutions.my_drone_random import MyDroneRandom
from solutions.my_drone_lidar_communication import MyDroneLidarCommunication, MiscData
import random
from random import uniform
import math

class MyDroneEval(MyDroneLidarCommunication):
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







