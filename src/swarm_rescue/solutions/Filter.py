import numpy as np
from filterpy.kalman import KalmanFilter


class Filter:
    def __init__(self):
        self.previous = None
        self.window_size = 0
        self.window = []


    def low_pass(self, current_value, alpha):
        self.previous = None

        if self.previous is None:
            # Initialize the filter with the first value
            self.previous = current_value
            return current_value

        filtered_value = alpha * current_value + (1 - alpha) * self.previous
        self.previous = filtered_value
        return filtered_value

    def moving_mean(self, value):
        self.window_size = 10
        self.window.append(value)
        # If the window exceeds the allowed size, remove the oldest value
        if len(self.window) > self.window_size:
            self.window.pop(0)
        return self.window

    def error(self, true, noise):
        return np.linalg.norm(true - noise)


