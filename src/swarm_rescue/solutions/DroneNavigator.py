import sys
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from typing import Type
from solutions.Path_tracjectory import Path_tracjectory

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


class DroneNavigator(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.path_trajectory = Path_tracjectory()
        self.visited_positions = set()  # Keep track of visited positions
        self.exploration_step_size = 10  # Step size for exploration

    def decide_next_move(self, gps_position, compass_angle, lidar_detection, variance_x, variance_y, r_obstacle, amplitude):
        """
        Decide the next move for the drone based on the potential field gradient or exploration.

        Args:
            gps_position (tuple): Current GPS position of the drone (x, y).
            compass_angle (float): Current compass angle of the drone in degrees.
            lidar_detection (tuple): LIDAR detection data (distance, angle).
            variance_x (float): Variance in the x-direction for the Gaussian field.
            variance_y (float): Variance in the y-direction for the Gaussian field.
            r_obstacle (float): Radius of the obstacle.
            amplitude (float): Amplitude of the Gaussian field.

        Returns:
            tuple: The next position (x, y) for the drone to move to.
        """
        # Generate the moving grid centered around the drone's GPS position
        X, Y = self.path_trajectory.moving_grid(gps_position)

        # Calculate the obstacle position using LIDAR data
        obstacle_position = self.path_trajectory.obstacle_position_lidar(gps_position, compass_angle, lidar_detection)

        # Check if the obstacle is already recorded
        if not self.path_trajectory.is_obstacle_recorded(obstacle_position):
            self.path_trajectory.obstacle.append(obstacle_position)

        # Generate the Gaussian potential field for all obstacles
        Z_total = np.zeros_like(X, dtype=float)
        for recorded_obstacle in self.path_trajectory.obstacle:
            Z_obstacle = self.path_trajectory.obstacle_gaussian(
                recorded_obstacle, variance_x, variance_y, r_obstacle, amplitude, X, Y
            )
            Z_total += Z_obstacle

        # Calculate the gradient of the total potential field
        grad_x, grad_y, min_grad_point, no_field = self.path_trajectory.calculate_gradient(Z_total, X, Y)

        # Decide the next move
        if no_field or min_grad_point is None:
            # No meaningful gradient field, explore the map
            return self.explore_map(gps_position)
        else:
            # Move towards the minimum gradient point
            next_position = min_grad_point
            self.visited_positions.add(next_position)
            return next_position

    def explore_map(self, gps_position):
        """
        Explore the map by moving to unvisited positions in a systematic manner.

        Args:
            gps_position (tuple): Current GPS position of the drone (x, y).

        Returns:
            tuple: The next position (x, y) for the drone to move to.
        """
        # Generate candidate positions for exploration
        candidate_positions = [
            (gps_position[0] + self.exploration_step_size, gps_position[1]),
            (gps_position[0] - self.exploration_step_size, gps_position[1]),
            (gps_position[0], gps_position[1] + self.exploration_step_size),
            (gps_position[0], gps_position[1] - self.exploration_step_size),
        ]

        # Filter out positions that have already been visited or are near obstacles
        unvisited_positions = [
            pos for pos in candidate_positions
            if pos not in self.visited_positions and not self.is_near_obstacle(pos)
        ]

        if unvisited_positions:
            # Choose the first unvisited position
            next_position = unvisited_positions[0]
        else:
            # If all nearby positions are visited or blocked, stay in place
            next_position = gps_position

        # Mark the chosen position as visited
        self.visited_positions.add(next_position)
        return next_position

    def is_near_obstacle(self, position):
        """
        Check if a position is near any recorded obstacle.

        Args:
            position (tuple): The position to check.

        Returns:
            bool: True if the position is near an obstacle, False otherwise.
        """
        for obstacle in self.path_trajectory.obstacle:
            distance = np.sqrt((position[0] - obstacle[0])**2 + (position[1] - obstacle[1])**2)
            if distance < self.exploration_step_size:  # Threshold for being "near" an obstacle
                return True
        return False