from spg_overlay.entities.drone_abstract import DroneAbstract
import numpy as np
from typing import Optional
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import clamp

class MyDroneEval(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         display_lidar_graph=False,
                         **kwargs)
        self.goal = np.array([100.0, 100.0])  # Example target position (point B)
        self.threshold = 10.0  # Distance threshold to consider the target reached

    def define_message_for_all(self):
        pass

    def control(self):
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0}

        current_position = self.measured_gps_position()
        if current_position is None or np.any(np.isnan(current_position)):
            return command

        direction_vector = self.goal - current_position
        distance_to_target = np.linalg.norm(direction_vector)

        if np.isnan(distance_to_target) or distance_to_target < self.threshold:
            return command

        direction_angle = np.arctan2(direction_vector[1], direction_vector[0])
        current_angle = self.measured_compass_angle()

        if current_angle is None or np.isnan(current_angle):
            return command

        angle_diff = direction_angle - current_angle
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize angle to [-pi, pi]

        # Proportional control for rotation
        Kp_rotation = 1.0
        command["rotation"] = clamp(Kp_rotation * angle_diff, -1.0, 1.0)

        # Proportional control for forward movement
        Kp_forward = 0.5
        command["forward"] = clamp(Kp_forward * distance_to_target, -1.0, 1.0)

        return command