import sys
from pathlib import Path
from typing import Type
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from spg.playground import Playground

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from maps.walls_medium_02 import add_walls, add_boxes
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.rescue_center import RescueCenter
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.utils.misc_data import MiscData


class MyDroneLidar(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.lidar_data = []
        self.lidar = self.init_lidar_sensor()

    def init_lidar_sensor(self):
        """
        Initialize the LIDAR sensor.
        """
        # Assuming there's a method to initialize the LIDAR sensor
        # Replace this with actual initialization code
        return LidarSensor()

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        We only send a command to do nothing
        """
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}
        return command

    def update_lidar_data(self):
        """
        Update the LIDAR data.
        """
        if self.lidar:
            self.lidar_data = self.lidar.get_sensor_values()


class MyMapLidar(MapAbstract):

    def __init__(self):
        super().__init__()

        # PARAMETERS MAP
        self._size_area = (1113, 750)

        self._rescue_center = RescueCenter(size=(210, 90))
        self._rescue_center_pos = ((440, 315), 0)

        self._number_drones = 1
        self._drones_pos = [((-50, 0), 0)]
        self._drones = []

    def construct_playground(self, drone_type: Type[DroneAbstract]) \
            -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        playground.add(self._rescue_center, self._rescue_center_pos)

        add_walls(playground)
        add_boxes(playground)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data,
                               display_lidar_graph=True)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        return playground


def update_fft_plot(frame, drone, line):
    """
    Update the FFT plot with new LIDAR data.
    """
    drone.update_lidar_data()
    if drone.lidar_data:
        # Compute FFT
        fft_values = np.fft.fft(drone.lidar_data)
        fft_freq = np.fft.fftfreq(len(drone.lidar_data))

        # Update line data
        line.set_data(fft_freq, np.abs(fft_values))
    return line,


def main():
    my_map = MyMapLidar()
    my_playground = my_map.construct_playground(drone_type=MyDroneLidar)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # enable_visu_noises : to enable the visualization. It will show also a
    # demonstration of the integration of odometer values, by drawing the
    # estimated path in red. The green circle shows the position of drone
    # according to the gps sensor and the compass
    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                draw_lidar_rays=True,
                use_keyboard=True,
                enable_visu_noises=True,
                )

    # Set up the FFT plot
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1000)
    ax.set_xlabel('Frequency')
    ax.set_ylabel('Amplitude')
    ax.set_title('Real-time FFT of LIDAR Data')

    # Get the drone instance
    drone = my_map._drones[0]

    # Create animation
    ani = FuncAnimation(fig, update_fft_plot, fargs=(drone, line), interval=100)

    # Run the GUI and the FFT plot
    plt.show(block=False)
    gui.run()


if __name__ == '__main__':
    main()