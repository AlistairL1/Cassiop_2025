import sys
from pathlib import Path

# Insert the parent directory of the current file's directory into sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))

from solutions.implementations_Alistair.point_AB.empty_map import EmptyMap
from solutions.implementations_Alistair.point_AB.my_drone_eval import MyDroneEval
from spg_overlay.gui_map.gui_sr import GuiSR

def main():
    my_map = EmptyMap()
    my_playground = my_map.construct_playground(drone_type=MyDroneEval)

    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                use_mouse_measure=True,
                )
    gui.run()

if __name__ == '__main__':
    main()

