from spg_overlay.entities.drone_motionless import DroneMotionless
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.return_area import ReturnArea
from typing import Type

class EmptyMap(MapAbstract):
    def __init__(self):
        super().__init__()
        self._size_area = (800, 800)
        self._number_drones = 1
        self._drones_pos = [((0, 0), 0)]
        self._drones = []
        self.goal_position = (100, 100)  # Example goal position

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> ClosedPlayground:
        playground = ClosedPlayground(size=self._size_area)

        # POSITIONS OF THE DRONES
        for i in range(self._number_drones):
            drone = drone_type(identifier=i)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        # Add ReturnArea
        return_area = ReturnArea(size=(10, 10))
        playground.add(return_area, (self.goal_position, 0))

        return playground


# if __name__ == '__main__':
#     my_map = EmptyMap()
#     my_playground = my_map.construct_playground(drone_type=DroneMotionless)
#
#     gui = GuiSR(playground=my_playground,
#                 the_map=my_map,
#                 use_mouse_measure=True,
#                 )
#     gui.run()