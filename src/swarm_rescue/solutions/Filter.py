import numpy as np


class Filter:
    def __init__(self):
        self.previous = None

    def low_pass(self, current_value, alpha):
        self.previous = None
        if self.previous is None:
            # Initialize the filter with the first value
            self.previous = current_value
            return current_value

        filtered_value = alpha * current_value + (1 - alpha) * self.previous
        self.previous = filtered_value
        return filtered_value

