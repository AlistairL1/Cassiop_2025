import unittest
import numpy as np
from solutions.my_drone_eval import MyDroneEval

class TestMyDroneEval(unittest.TestCase):
    def setUp(self):
        self.drone = MyDroneEval()

    def test_control_towards_target(self):
        # Mock the measured_gps_position method to return a position far from the target
        self.drone.measured_gps_position = lambda: np.array([0.0, 0.0])
        # Mock the measured_compass_angle method to return an angle of 0 radians
        self.drone.measured_compass_angle = lambda: 0.0

        command = self.drone.control()

        # Check if the drone is moving forward
        self.assertGreater(command["forward"], 0.0)
        # Check if the drone is rotating towards the target
        self.assertNotEqual(command["rotation"], 0.0)

    def test_control_target_reached(self):
        # Mock the measured_gps_position method to return a position close to the target
        self.drone.measured_gps_position = lambda: self.drone.goal
        # Mock the measured_compass_angle method to return an angle of 0 radians
        self.drone.measured_compass_angle = lambda: 0.0

        command = self.drone.control()

        # Check if the drone stops moving
        self.assertEqual(command["forward"], 0.0)
        self.assertEqual(command["rotation"], 0.0)

if __name__ == '__main__':
    unittest.main()