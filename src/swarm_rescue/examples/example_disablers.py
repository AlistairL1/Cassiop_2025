"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import sys
from pathlib import Path
from typing import List, Type



# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg_overlay.reporting.result_path_creator import ResultPathCreator
from spg_overlay.reporting.team_info import TeamInfo
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.rescue_center import RescueCenter
from spg_overlay.entities.sensor_disablers import (NoGpsZone, NoComZone,
                                                   KillZone)
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.utils.misc_data import MiscData


class MyDroneDisablers(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        msg_data = (self.identifier,
                    (self.measured_gps_position(),
                     self.measured_compass_angle()))
        return msg_data

    def control(self):
        """
        We only send a command to do nothing
        """
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}
        return command


class MyMapDisablers(MapAbstract):

    def __init__(self):
        super().__init__()

        # PARAMETERS MAP
        self._size_area = (600, 600)

        self._rescue_center = RescueCenter(size=(100, 100))
        self._rescue_center_pos = ((0, 100), 0)

        self._no_com_zone = NoComZone(size=(150, 150))
        self._no_com_zone_pos = ((-150, 0), 0)

        self._no_gps_zone = NoGpsZone(size=(150, 150))
        self._no_gps_zone_pos = ((150, 0), 0)

        self._kill_zone = KillZone(size=(150, 150))
        self._kill_zone_pos = ((0, -200), 0)

        self._wounded_persons_pos = [(200, 0), (-200, 0),
                                     (200, -200), (-200, -200)]
        self._number_wounded_persons = len(self._wounded_persons_pos)
        self._wounded_persons: List[WoundedPerson] = []

        self._number_drones = 1
        self._drones_pos = [((0, 0), 0)]
        self._drones = []

    def construct_playground(self, drone_type: Type[DroneAbstract]):
        playground = ClosedPlayground(size=self._size_area)

        playground.add(self._rescue_center, self._rescue_center_pos)

        # DISABLER ZONES
        playground.add(self._no_com_zone, self._no_com_zone_pos)
        playground.add(self._no_gps_zone, self._no_gps_zone_pos)
        playground.add(self._kill_zone, self._kill_zone_pos)

        # POSITIONS OF THE WOUNDED PERSONS
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = (self._wounded_persons_pos[i], 0)
            playground.add(wounded_person, pos)

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
    my_map = MyMapDisablers()
    my_playground = my_map.construct_playground(drone_type=MyDroneDisablers)

    # Set this value to True to generate a video of the mission
    video_capture_enabled = False
    if video_capture_enabled:
        team_info = TeamInfo()
        rpc = ResultPathCreator(team_info)
        filename_video_capture = rpc.path + "/example_disablers.avi"
    else:
        filename_video_capture = None

    # enable_visu_noises : to enable the visualization. It will show also a
    # demonstration of the integration of odometer values, by drawing the
    # estimated path in red. The green circle shows the position of drone
    # according to the gps sensor and the compass.
    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                print_messages=True,
                use_keyboard=True,
                enable_visu_noises=True,
                filename_video_capture=filename_video_capture
                )
    gui.run()


if __name__ == '__main__':
    main()
