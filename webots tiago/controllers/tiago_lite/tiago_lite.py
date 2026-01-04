#tiago_lite.py

import numpy as np
import math
from controller import Supervisor
from scipy import signal
from matplotlib import pyplot as plt
import py_trees

from Blackboardd import Blackboard
import planner
from build_navigator import Navigation
from build_map import Map
from BT import BT
from arm_actions import ARM_JOINTS

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
display = robot.getDevice("display")
lidar = robot.getDevice("Hokuyo URG-04LX-UG01")

leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
rightMotor.setVelocity(0.0)
leftMotor.setVelocity(0.0)

gps.enable(timestep)
compass.enable(timestep)
lidar.enable(timestep)

blackboard = Blackboard()

map_instance = Map(blackboard)

bt_instance = BT(blackboard, robot, timestep)
tree = bt_instance.build_tree()

arm_controller_instance = bt_instance.arm_controller

print("CONTROLLER: Moving arm to initial setup position...")

arm_controller_instance.control_arm(ARM_JOINTS)
arm_controller_instance.wait_blocking(50)

#updates the BB
def update_blackboardd(blackboard, gps, compass, lidar):
    gps_values = gps.getValues()
    compass_values = compass.getValues()

    blackboard.write("compassVals", compass_values)
    blackboard.write("gpsVals", gps_values)
    blackboard.write("xw", gps_values[0])
    blackboard.write("yw", gps_values[1])
    blackboard.write("theta", math.atan2(compass_values[0], compass_values[1]))
    blackboard.write("lidar_image", lidar.getRangeImage())
    blackboard.write("lidar_fov", lidar.getFov())


#starts main while loop
while robot.step(timestep) != -1:
    update_blackboardd(blackboard, gps, compass, lidar)

    map_instance.map_fun()
    tree.tick()

    leftSpeed = blackboard.read("leftSpeed") or 0.0
    rightSpeed = blackboard.read("rightSpeed") or 0.0

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    updates = blackboard.read("map_updates") or []
    for px, py, color in updates:
        display.setColor(color)
        display.drawPixel(px, py)


print(map_instance)

kernel = np.ones((31, 31))

occ_map = blackboard.read("map")
cmap = signal.convolve2d(occ_map, kernel, mode="same")
cspace = cmap > 0.9
np.save("cspace", cspace)

#at end of run prints collision map
plt.imshow(cspace)
plt.show()
