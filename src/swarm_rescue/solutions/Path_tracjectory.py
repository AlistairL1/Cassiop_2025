import sys
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
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

class Path_tracjectory(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.obstacle = []
        # liste des obstacles dejà enregistré, verifier si dans la zone de la grid, il y a une postion d'obstacle contenu dans la liste

    def calculate_gradient(Z, X, Y):
        # Calculate gradient using numpy's gradient function
        grad_y, grad_x = np.gradient(Z)

        # Calculate gradient magnitude
        grad_magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)

        # Check if there's a meaningful field
        if np.any(grad_magnitude > 0.001):  # Small threshold to account for numerical precision
            # Find the minimum non-zero gradient
            # Create a masked array to ignore zero or near-zero gradients
            masked_magnitude = np.ma.masked_where(grad_magnitude < 0.001, grad_magnitude)

            # Find the minimum gradient
            min_idx = np.ma.argmin(masked_magnitude)
            min_i, min_j = np.unravel_index(min_idx, grad_magnitude.shape)
            min_grad_point = (X[min_i, min_j], Y[min_i, min_j])

            return grad_x, grad_y, min_grad_point, False
        else:
            # No meaningful field case
            return grad_x, grad_y, None, True

    def obstacle_gaussian(self, obstacle_position,variance_x, variance_y, r_obstacle,amplitude, X, Y):

        # obstacle_position is the center of the obstacle, check which object it is after LIDAR calculation
        # Create potential field using multivariate gaussian
        Z_obstacle = np.zeros_like(X, dtype=float)

        # Create multivariate gaussian potential field
        for i in range(len(x)):
            for j in range(len(y)):
                # Distance from point to obstacle center
                dx = X[i, j] - obstacle_position[0]
                dy = Y[i, j] - obstacle_position[1]

                # Inside the obstacle: constant maximum repulsion
                d_obstacle = np.sqrt(dx ** 2 + dy ** 2)
                if d_obstacle <= r_obstacle:
                    Z_obstacle[i, j] = - amplitude
                else:
                    # Multivariate Gaussian function: A * exp(-((x-μ)²/(2σx²) + (y-μ)²/(2σy²)))
                    Z_obstacle[i, j] = - amplitude * np.exp(-(dx ** 2 / (2 * variance_x) + dy ** 2 / (2 * variance_y))) * (r_obstacle / d_obstacle)
        return Z_obstacle
        
    def moving_grid(self, gps_position):
        grid_size = 50  # Define the size of the grid
        half_size = grid_size // 2

        # Create a grid centered around the gps_position
        x = np.arange(gps_position[0] - half_size, gps_position[0] + half_size, 1)
        y = np.arange(gps_position[1] - half_size, gps_position[1] + half_size, 1)
        X, Y = np.meshgrid(x, y)

        return X, Y

    def obstacle_position_lidar(self, gps_position, compass_angle, lidar_detection):
        # Extract the distance to the obstacle from the lidar detection
        distance_to_obstacle = lidar_detection[0]

        # Calculate the angle of the obstacle relative to the north
        obstacle_angle = compass_angle + lidar_detection[1]

        # Convert the angle to radians
        obstacle_angle_rad = np.deg2rad(obstacle_angle)

        # Calculate the obstacle position using trigonometry
        obstacle_x = gps_position[0] + distance_to_obstacle * np.cos(obstacle_angle_rad)
        obstacle_y = gps_position[1] + distance_to_obstacle * np.sin(obstacle_angle_rad)

        # Return the obstacle position
        return obstacle_x, obstacle_y       #retourne un tuple
        
    def is_obstacle_recorded(self, obstacle_position):
        
        for recorded_obstacle in self.obstacle:
            if np.allclose(recorded_obstacle, obstacle_position, atol=1e-2):
                return True
        return False