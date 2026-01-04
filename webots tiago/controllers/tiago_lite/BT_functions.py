import py_trees
import numpy as np
from scipy import signal
import os
from py_trees.common import Status
from build_navigator import Navigation
from build_map import Map
import planner
from arm_actions import ArmController

class RotateToHeading(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, robot, target_heading, name="RotateToHeading"):
        super().__init__(name)
        self.blackboard = blackboard
        self.robot = robot
        self.target_heading = target_heading
        self.tolerance = 0.01
        self.MAX_SPEED = 2.0
        self.P_GAIN = 5.0
        self.leftMotor = self.robot.getDevice("wheel_left_joint")
        self.rightMotor = self.robot.getDevice("wheel_right_joint")

    def get_heading(self):
        return self.blackboard.read("theta")

    def update(self):
        current_heading = self.get_heading()
        diff = self.target_heading - current_heading

        if abs(diff) < self.tolerance:
            self.blackboard.write("leftSpeed", 0.0)
            self.blackboard.write("rightSpeed", 0.0)
            return Status.SUCCESS

        speed = diff * self.P_GAIN
        speed = max(min(speed, self.MAX_SPEED), -self.MAX_SPEED)

        self.blackboard.write("leftSpeed", -speed)
        self.blackboard.write("rightSpeed", speed)

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status != Status.SUCCESS:
            self.leftMotor.setVelocity(0)
            self.rightMotor.setVelocity(0)

class MoveTo(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, name="MoveTo"):
        super().__init__(name)
        self.blackboard = blackboard
        self.nav = None

    def initialise(self):
        path = self.blackboard.read("path")
        if not path:
            self.nav = None
            return
        self.nav = Navigation(self.blackboard)

    def update(self):
        if self.nav is None:
            return Status.FAILURE

        self.nav.Navigate()
        return Status.SUCCESS if self.nav.done() else Status.RUNNING

    def terminate(self, new_status):
        if new_status != Status.RUNNING:
            self.nav = None
            self.blackboard.write("leftSpeed", 0.0)
            self.blackboard.write("rightSpeed", 0.0)

class PathPlan(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, goal, name="PathPlan"):
        super().__init__(name)
        self.blackboard = blackboard
        self.goal = goal
        self.plan = None
        self.planned = False

    def initialise(self):
        if self.planned and self.plan is not None:
            return
        self.plan = None
        self.planned = False

    def update(self):
        if self.planned:
            return Status.SUCCESS

        xw = self.blackboard.read("xw")
        yw = self.blackboard.read("yw")
        collisionMap = self.blackboard.read("collision_map")

        if xw is None or yw is None or collisionMap is None:
            return Status.RUNNING

        graph = planner.build_graph(collisionMap)
        self.plan = planner.get_correct_path(collisionMap, graph, (xw, yw), self.goal)

        if not self.plan:
            return Status.FAILURE

        self.blackboard.write("path", self.plan)
        self.blackboard.write("path_index", 0)
        self.planned = True
        return Status.SUCCESS

class map_kitchen(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, waypoints, name="map_kitchen"):
        super().__init__(name)
        self.blackboard = blackboard
        self.waypoints = waypoints
        self.nav = None
        self.completed = False

    def initialise(self):
        if self.completed:
            return
        if not self.waypoints:
            self.completed = True
            return
        self.blackboard.write("path", self.waypoints)
        self.blackboard.write("path_index", 0)
        self.nav = Navigation(self.blackboard)

    def update(self):
        if self.completed:
            return Status.SUCCESS
        if self.nav is None:
            return Status.FAILURE

        self.nav.Navigate()
        if self.nav.done():
            self.completed = True
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.SUCCESS and self.completed:
            occ = self.blackboard.read("map")
            if occ is None:
                return

            kernel = np.ones((31, 31))
            cmap = signal.convolve2d(occ, kernel, mode="same")
            cspace = cmap > 0.9

            self.blackboard.write("collision_map", cspace)
            self.blackboard.write("collision_map_ready", True)
            np.save("cspace.npy", cspace)

        if new_status != Status.RUNNING:
            self.blackboard.write("leftSpeed", 0.0)
            self.blackboard.write("rightSpeed", 0.0)

class Does_map_exist(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, name="Does_map_exist"):
        super().__init__(name)
        self.blackboard = blackboard

    def update(self):
        try:
            cspaceMap = np.load("cspace.npy")
            self.blackboard.write("collision_map", cspaceMap)
            return Status.SUCCESS
        except (FileNotFoundError, IOError):
            return Status.FAILURE

class ExecuteArmMission(py_trees.behaviour.Behaviour):
    def __init__(self, blackboard, arm_controller, name="ExecuteArmMission"):
        super().__init__(name)
        self.blackboard = blackboard
        self.arm_controller = arm_controller

    def update(self):
        return Status.SUCCESS if self.arm_controller.execute_coded_mission() else Status.FAILURE

class GrabObject(py_trees.behaviour.Behaviour):
    def __init__(self, arm_controller, name="GrabObject"):
        super().__init__(name)
        self.arm_controller = arm_controller

    def update(self):
        return Status.SUCCESS if self.arm_controller.grab_object() else Status.FAILURE

class DropObject(py_trees.behaviour.Behaviour):
    def __init__(self, arm_controller, name="DropObject"):
        super().__init__(name)
        self.arm_controller = arm_controller

    def update(self):
        return Status.SUCCESS if self.arm_controller.drop_object() else Status.FAILURE
