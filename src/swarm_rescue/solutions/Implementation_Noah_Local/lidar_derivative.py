import sys
from pathlib import Path
from typing import Type
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt, find_peaks

from spg.playground import Playground

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

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
        self.lidar_angles = []

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
        Update the LIDAR data and angles.
        """
        self.lidar_data = self.lidar_values()
        self.lidar_angles = self.lidar_rays_angles()  # Assuming there's a method to get angles


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


def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def lowpass_filter(data, cutoff, fs, order=2):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y


def update_plot(frame, drone, line1, line2, max_line, min_line):
    """
    Update the plot with new LIDAR data.
    """
    drone.update_lidar_data()
    if drone.lidar_data.size > 0:
        # Apply lowpass filter
        cutoff = 10.0  # Desired cutoff frequency of the filter, Hz
        fs = 100.0  # Sample rate, Hz
        filtered_data = lowpass_filter(drone.lidar_data, cutoff, fs)

        # Compute the derivative of the LIDAR signal
        derivative_data = np.gradient(drone.lidar_data)

        # Find peaks (maxima) and valleys (minima) in the derivative data
        peaks, _ = find_peaks(derivative_data)
        valleys, _ = find_peaks(-derivative_data)

        # Update line data for LIDAR plot
        line1.set_data(drone.lidar_angles, drone.lidar_data)

        # Update line data for derivative plot
        line2.set_data(drone.lidar_angles, derivative_data)

        # Update line data for maxima and minima
        max_line.set_data(drone.lidar_angles[peaks], drone.lidar_data[peaks])
        min_line.set_data(drone.lidar_angles[valleys], drone.lidar_data[valleys])
    return line1, line2, max_line, min_line


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

    # Set up the plot
    fig, ax = plt.subplots(figsize=(10, 8))

    # LIDAR plot
    line1, = ax.plot([], [], lw=2, label='LIDAR Data')
    # Derivative plot
    line2, = ax.plot([], [], lw=2, label='Derivative Data')
    # Maxima and minima
    max_line, = ax.plot([], [], 'ro', label='Maxima')
    min_line, = ax.plot([], [], 'bo', label='Minima')

    ax.set_xlim(-np.pi, np.pi)  # Adjusted to show the angle interval
    ax.set_ylim(-100, 400)  # Adjust as needed
    ax.set_xlabel('Angle [rad]')
    ax.set_ylabel('Value')
    ax.set_title('Real-time LIDAR and Derivative Data with Peaks')
    ax.legend()
    ax.grid()

    # Get the drone instance
    drone = my_map._drones[0]

    # Create animation
    ani = FuncAnimation(fig, update_plot, fargs=(drone, line1, line2, max_line, min_line), interval=100)

    # Run the GUI and the plot
    plt.show(block=False)
    gui.run()


if __name__ == '__main__':
    main()