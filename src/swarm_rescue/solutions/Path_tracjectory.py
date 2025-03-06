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

    def add_goal(X, Y, s, r, loc):
        delx = np.zeros_like(X)
        dely = np.zeros_like(Y)
        for i in range(len(x)):
            for j in range(len(y)):

                d = np.sqrt((loc[0] - X[i][j]) ** 2 + (loc[1] - Y[i][j]) ** 2)
                # print(f"{i} and {j}")
                theta = np.arctan2(loc[1] - Y[i][j], loc[0] - X[i][j])
                if d < r:
                    delx[i][j] = 0
                    dely[i][j] = 0
                elif d > r + s:
                    delx[i][j] = 50 * s * np.cos(theta)
                    dely[i][j] = 50 * s * np.sin(theta)
                else:
                    delx[i][j] = 50 * (d - r) * np.cos(theta)
                    dely[i][j] = 50 * (d - r) * np.sin(theta)
        return delx, dely

    x = np.arange(-0, 50, 1)
    y = np.arange(-0, 50, 1)
    goal = random.sample(range(0, 50), 2)
    s = 7
    r = 2
    seek_points = np.array([[0, 0]])
    X, Y = np.meshgrid(x, y)
    delx, dely = add_goal(X, Y, s, r, goal)


    def add_obstacle(X, Y, delx, dely, goal):
        #power of the field
        s = 7
        # generating obstacle with random sizes
        r = 1

        # generating random location of the obstacle
        obstacle = random.sample(range(0, 50), 2)
        for i in range(len(x)):
            for j in range(len(y)):

                d_goal = np.sqrt((goal[0] - X[i][j]) ** 2 + ((goal[1] - Y[i][j])) ** 2)
                d_obstacle = np.sqrt((obstacle[0] - X[i][j]) ** 2 + (obstacle[1] - Y[i][j]) ** 2)
                # print(f"{i} and {j}")
                theta_goal = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
                theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0] - X[i][j])
                if d_obstacle < r:
                    delx[i][j] = -1 * np.sign(np.cos(theta_obstacle)) * 5 + 0
                    dely[i][j] = -1 * np.sign(np.cos(theta_obstacle)) * 5 + 0
                elif d_obstacle > r + s:
                    delx[i][j] += 0 - (50 * s * np.cos(theta_goal))
                    dely[i][j] += 0 - (50 * s * np.sin(theta_goal))
                elif d_obstacle < r + s:
                    delx[i][j] += -150 * (s + r - d_obstacle) * np.cos(theta_obstacle)
                    dely[i][j] += -150 * (s + r - d_obstacle) * np.sin(theta_obstacle)
                if d_goal < r + s:
                    if delx[i][j] != 0:
                        delx[i][j] += (50 * (d_goal - r) * np.cos(theta_goal))
                        dely[i][j] += (50 * (d_goal - r) * np.sin(theta_goal))
                    else:

                        delx[i][j] = (50 * (d_goal - r) * np.cos(theta_goal))
                        dely[i][j] = (50 * (d_goal - r) * np.sin(theta_goal))

                if d_goal > r + s:
                    if delx[i][j] != 0:
                        delx[i][j] += 50 * s * np.cos(theta_goal)
                        dely[i][j] += 50 * s * np.sin(theta_goal)
                    else:

                        delx[i][j] = 50 * s * np.cos(theta_goal)
                        dely[i][j] = 50 * s * np.sin(theta_goal)
                if d_goal < r:
                    delx[i][j] = 0
                    dely[i][j] = 0

        return delx, dely, obstacle, r


