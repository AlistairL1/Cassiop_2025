from spg_overlay.entities.drone_abstract import DroneAbstract

import math
import os
import sys
from pathlib import Path
from typing import List, Type, Tuple

import arcade
import numpy as np

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg_overlay.utils.path import Path
from spg_overlay.utils.pose import Pose
from spg_overlay.utils.utils import clamp, normalize_angle
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.utils.misc_data import MiscData


class Test_PID(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Translation control attributes
        self.distance = 150.0
        self.position_setpoint = np.array([-self.distance, 0.0])
        self.counter_change_setpoint = 180
        self.prev_diff_position = 0.0
        self.translation_counter = 0
        self.to_the_right = True

        # Rotation control attributes
        self.angle_setpoint = 0.0
        self.counter_change_direction = 40
        self.prev_diff_angle = 0.0
        self.rotation_counter = 0

        # Path tracking
        self.iter_path = 0
        self.path_done = Path()

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        Combines both translation and rotation PID control.
        """
        # Update path for visualization
        self.iter_path += 1
        if self.iter_path % 3 == 0:
            position = np.array([self.true_position()[0],
                                 self.true_position()[1]])
            angle = self.true_angle()
            pose = Pose(position=position, orientation=angle)
            self.path_done.append(pose)

        # --- Translation Control ---
        self.translation_counter += 1
        if self.translation_counter % self.counter_change_setpoint == 0:
            if self.to_the_right:
                self.position_setpoint = np.array([self.distance, 0.0])
            else:
                self.position_setpoint = np.array([-self.distance, 0.0])
            self.to_the_right = not self.to_the_right
            print("Switching translation setpoint.")

        diff_position = (self.position_setpoint -
                         np.asarray(self.true_position()))
        deriv_diff_position = diff_position - self.prev_diff_position

        # PD for translation
        Kp_translation = 1.6
        Kd_translation = 11.0
        forward = (Kp_translation * float(diff_position[0]) +
                   Kd_translation * float(deriv_diff_position[0]))

        forward = clamp(forward, -1.0, 1.0)
        self.prev_diff_position = diff_position

        # --- Rotation Control ---
        self.rotation_counter += 1
        if self.rotation_counter % self.counter_change_direction == 0:
            self.angle_setpoint = normalize_angle(self.angle_setpoint + np.pi / 2)
            print("Switching rotation setpoint.")

        diff_angle = normalize_angle(self.angle_setpoint - self.true_angle())
        deriv_diff_angle = normalize_angle(diff_angle - self.prev_diff_angle)

        # PD for rotation
        Kp_rotation = 9.0
        Kd_rotation = 0.6
        rotation = Kp_rotation * diff_angle + Kd_rotation * deriv_diff_angle
        rotation = clamp(rotation, -1.0, 1.0)
        self.prev_diff_angle = diff_angle

        # --- Combine Controls ---
        command = {"forward": forward,
                   "rotation": rotation}

        print(f"Forward: {forward}, Rotation: {rotation}")
        return command

    def draw_bottom_layer(self):
        self.draw_setpoint()
        self.draw_path(path=self.path_done, color=(255, 0, 255))
        self.draw_antedirection()
        self.draw_direction()

    def draw_setpoint(self):
        half_width = self._half_size_array[0]
        half_height = self._half_size_array[1]
        pt1 = self.position_setpoint + np.array([half_width, 0])
        pt2 = self.position_setpoint + np.array([half_width, 2 * half_height])


    def draw_path(self, path: Path(), color: Tuple[int, int, int]):
        length = path.length()
        pt2 = None
        for ind_pt in range(length):
            pose = path.get(ind_pt)
            pt1 = pose.position + self._half_size_array
            if ind_pt > 0:
                arcade.draw_line(float(pt2[0]),
                                 float(pt2[1]),
                                 float(pt1[0]),
                                 float(pt1[1]), color)
            pt2 = pt1

    def draw_antedirection(self):
        pt1 = np.array([self.true_position()[0], self.true_position()[1]])
        pt1 = pt1 + self._half_size_array
        pt2 = pt1 + 150 * np.array([math.cos(self.true_angle() + np.pi / 2),
                                    math.sin(self.true_angle() + np.pi / 2)])
        color = (255, 64, 0)
        arcade.draw_line(float(pt2[0]),
                         float(pt2[1]),
                         float(pt1[0]),
                         float(pt1[1]),
                         color)

    def draw_direction(self):
        pt1 = np.array([self.true_position()[0], self.true_position()[1]])
        pt1 = pt1 + self._half_size_array
        pt2 = pt1 + 250 * np.array([math.cos(self.true_angle()),
                                    math.sin(self.true_angle())])
        color = (255, 64, 0)
        arcade.draw_line(float(pt2[0]),
                         float(pt2[1]),
                         float(pt1[0]),
                         float(pt1[1]),
                         color)


class MyMap(MapAbstract):
    def __init__(self):
        super().__init__()

        # PARAMETERS MAP
        self._size_area = (600, 600)

        # POSITIONS OF THE DRONES
        self._number_drones = 1
        self._drones_pos = [((-50, 50), 0)]  # Initial position
        self._drones: List[DroneAbstract] = []

    def construct_playground(self, drone_type: Type[DroneAbstract]):
        playground = ClosedPlayground(size=self._size_area)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)

        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        return playground


def main():
    my_map = MyMap()
    my_playground = my_map.construct_playground(drone_type=Test_PID)

    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                use_keyboard=False,
                use_mouse_measure=True,
                enable_visu_noises=False)

    gui.run()


if __name__ == '__main__':
    main()
