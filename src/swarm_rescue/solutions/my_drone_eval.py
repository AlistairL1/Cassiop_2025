from typing import Optional
from solutions.my_drone_random import MyDroneRandom
from solutions.my_drone_lidar_communication import MyDroneLidarCommunication, MiscData
import random
from random import uniform
import math

class MyDroneEval(MyDroneLidarCommunication):
    pass

#on a notre position , une trajectoire de reference que le drone doit suivre on veut aller de a vers b => suivre une trajectoire
#on a deux points a et b et il faut le rejoindre , si trajectoire constante (voir lien chat) - graphe ligne pointillée qui est la référence , en utilisant le régulateur - refefrence se deplace vers le point B - si on deplace la reference on ralenti et si on ne l'atteint pas on accelère
#le truc le plus compliqué : comment choisir A et B de façon à ce que ça suivre notre trajectoire
#se baser sur des techniques qui sont basées sur des slams
#plutôt position de référence ou vitesse de référence ? si on connaît la dérivée, c'est plus précis






