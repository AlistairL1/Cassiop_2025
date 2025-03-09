from spg_overlay.entities.drone_abstract import DroneAbstract
import numpy as np
from typing import Optional
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import clamp, normalize_angle, rad2deg

class MyDroneEval(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         display_lidar_graph=False,
                         **kwargs)
        self.goal = np.array([100.0, 100.0])  # Point B

    def define_message_for_all(self):
        pass

    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0}

        current_position = self.measured_gps_position()
        current_x_angle = self.measured_compass_angle()  # Angle avec l'axe des abscisses
        if current_position is None or np.any(np.isnan(current_position)) \
                                    or current_x_angle is None \
                                    or np.any(np.isnan(current_x_angle)):
            return command

        direction_vector = self.goal - current_position  # Vecteur de direction
        distance_to_target = np.linalg.norm(direction_vector)  # Distance à l'objectif (norme euclidienne)

        direction_angle_x = np.arctan2(direction_vector[1], direction_vector[0])  # Angle target avec l'axe (Ox) positif

        angle_diff_x = ((direction_angle_x - current_x_angle) + np.pi) % (2*np.pi) - np.pi

        # Contrôle forward (x)
        Kp_forward = 0.75
        a_forward = max(0, 8/np.pi * angle_diff_x*distance_to_target)
        command["forward"] = clamp(Kp_forward * a_forward * distance_to_target, -1.0, 1.0)

        # Contrôle latéral (y)
        # Kp_lateral = 0.7
        # a_latetal = max(0, 4/np.pi * (angle_diff_x))
        # command["lateral"] = clamp(Kp_lateral * a_latetal * distance_to_target, -1.0, 1.0)

        # Contrôle rotation (θ)
        Kp_rotation = .25
        command["rotation"] = clamp(angle_diff_x*Kp_rotation, -1.0, 1.0)

        return command
