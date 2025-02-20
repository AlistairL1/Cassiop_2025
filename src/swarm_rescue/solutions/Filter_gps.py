from solutions.Filter import Filter

"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import sys
from pathlib import Path
from typing import Type

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg_overlay.reporting.result_path_creator import ResultPathCreator
from spg_overlay.reporting.team_info import TeamInfo
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.sensor_disablers import NoGpsZone, KillZone
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.utils.misc_data import MiscData


class MyDroneGpsDisabler(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Create an instance of Filter for the GPS smoothing
        self.gps_filter = Filter()
        self.alpha = 0.25
        self.list_gps = []

    def get_measured_gps_position(self):
        """
        Method to retrieve the measured GPS position.
        """
        self.list_gps.append(self.measured_gps_position())

        # Ensure the list contains a maximum of 1000 elements
        if len(self.list_gps) > 1000:
            self.list_gps.pop(0)  # Remove the oldest GPS position

        return self.measured_gps_position(), self.list_gps

    def define_message_for_all(self):
        """
        Include GPS filtering in the communication message
        """
        # Raw GPS position
        raw_gps_position = self.measured_gps_position()

        # Smooth the GPS position using the low-pass filter
        filtered_gps_position = (
            self.gps_filter.low_pass(raw_gps_position[0], self.alpha), #Position en X
            self.gps_filter.low_pass(raw_gps_position[1], self.alpha), #Position en Y
        )

        print(f"Position GPS Brute: {raw_gps_position}")
        print(f"Position GPS Filtre: {filtered_gps_position}")




        print(f"Erreur GPS: {self.gps_filter.error(filtered_gps_position, self.true_position())}")

        # Use the smoothed position in the message
        msg_data = (self.identifier,
                    (filtered_gps_position,
                     self.measured_compass_angle(),
                     raw_gps_position))

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


class MyMapGpsDisabler(MapAbstract):

    def __init__(self):
        super().__init__()

        # PARAMETERS MAP
        self._size_area = (800, 600)

        self._no_gps_zone = NoGpsZone(size=(300, 400))
        self._no_gps_zone_pos = ((200, 0), 0)

        self._kill_zone = KillZone(size=(150, 400))
        self._kill_zone_pos = ((-300, 0), 0)

        self._number_drones = 1
        self._drones_pos = [((-100, 0), 0)]
        self._drones = []

    def construct_playground(self, drone_type: Type[DroneAbstract]):
        playground = ClosedPlayground(size=self._size_area)

        # DISABLER ZONES
        #playground.add(self._no_gps_zone, self._no_gps_zone_pos)
        #playground.add(self._kill_zone, self._kill_zone_pos)

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
    my_map = MyMapGpsDisabler()
    my_playground = my_map.construct_playground(drone_type=MyDroneGpsDisabler)
    video_capture_enabled = False
    if video_capture_enabled:
        team_info = TeamInfo()
        rpc = ResultPathCreator(team_info)
        filename_video_capture = rpc.path + "/example_gps_disabler.avi"
    else:
        filename_video_capture = None

    # enable_visu_noises : to enable the visualization. It will show also a
    # demonstration of the integration of odometer values, by drawing the
    # estimated path in red. The green circle shows the position of drone
    # according to the gps sensor and the compass.
    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                print_messages=True,
                draw_gps=True,
                use_keyboard=True,
                enable_visu_noises=True,
                filename_video_capture=filename_video_capture
                )
    gui.run()


if __name__ == '__main__':
    main()
