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

    def calculate_gradient(Z, X, Y):
        """
        Calculate the gradient of the potential field and find minimum gradient point.

        Args:
            Z: The potential field values
            X, Y: The meshgrid coordinates

        Returns:
            grad_x, grad_y: The gradient components
            min_grad_point: Coordinates of minimum gradient magnitude point
            no_field: Boolean indicating if there's no meaningful gradient field
        """
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

    def add_obstacle(self):
        # Define grid
        x = np.arange(0, 50, 1)
        y = np.arange(0, 50, 1)
        X, Y = np.meshgrid(x, y)

        # Define obstacle parameters
        obstacle = (25, 25)  # Center of the obstacle
        r_obstacle = 3  # Obstacle radius - increased for better visualization

        # Create smoother potential field for the obstacle only
        Z_obstacle = np.zeros_like(X, dtype=float)

        for i in range(len(x)):
            for j in range(len(y)):
                d_obstacle = np.sqrt((obstacle[0] - X[i, j]) ** 2 + (obstacle[1] - Y[i, j]) ** 2)

                # Create a stronger repulsion within the obstacle radius
                if d_obstacle <= r_obstacle:
                    Z_obstacle[i, j] = -200  # Maximum repulsion inside obstacle
                else:
                    # Smooth decay outside obstacle boundary using radius as scaling factor
                    Z_obstacle[i, j] = -200 * r_obstacle / (d_obstacle ** 2)


