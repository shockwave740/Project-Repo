import py_trees
import BT_functions
import planner
from arm_actions import ArmController
from controller import Supervisor

class BT:
    def __init__(self, blackboard, robot: Supervisor, timestep: int):
        self.blackboard = blackboard
        self.map_path = planner.map_path()
        self.robot = robot
        self.arm_controller = ArmController(robot, timestep)

    def move_to(self, name, goal):
        return py_trees.composites.Sequence(
            name=name,
            memory=True,
            children=[
                BT_functions.PathPlan(self.blackboard, goal=goal),
                BT_functions.MoveTo(self.blackboard),
            ],
        )

    def initialize_map(self):
        return py_trees.composites.Selector(
            name="InitialMap",
            memory=False,
            children=[
                BT_functions.Does_map_exist(self.blackboard),
                BT_functions.map_kitchen(self.blackboard, self.map_path),
            ],
        )

    def pick_and_place_sequence(self):
        return py_trees.composites.Sequence(
            name="PickAndPlaceMission",
            memory=True,
            children=[
                self.move_to("WP1_GotoPick", (0.50, 0.63)),
                BT_functions.RotateToHeading(self.blackboard, self.robot, 0.18),
                BT_functions.GrabObject(self.arm_controller),
                self.move_to("WP2_GotoDrop", (0.65, -1)),
                BT_functions.RotateToHeading(self.blackboard, self.robot, -3),
                BT_functions.DropObject(self.arm_controller),
                self.move_to("WP3_GotoEnd", (0.763, 0.329)),
                BT_functions.RotateToHeading(self.blackboard, self.robot, -0.58),
                py_trees.behaviours.Success(name="MISSION_COMPLETE"),
            ],
        )

    def build_tree(self):
        root = py_trees.composites.Sequence(
            name="Main",
            memory=True,
            children=[
                self.initialize_map(),
                self.pick_and_place_sequence(),
            ],
        )
        return py_trees.trees.BehaviourTree(root)
