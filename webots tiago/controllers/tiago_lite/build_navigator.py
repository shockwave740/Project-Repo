import numpy as np

class Navigation:
    def __init__(self, blackboard):
        self.blackboard = blackboard
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self.xw = 0.0
        self.yw = 0.0
        self.theta = 0.0
        self.rho = 0.0
        self.alpha = 0.0
        self.path = None
        self.path_index = 0
        self.wp = None
        self.P1 = 4
        self.P2 = 4
        self.VMAX = 10.0
        self.WP_PROXIMITY = 0.45

    def latch_path(self):
        if self.path is not None:
            return
        self.path = self.blackboard.read("path") or []
        self.path_index = int(self.blackboard.read("path_index") or 0)

    def read_Navigator_fun(self):
        self.xw = self.blackboard.read("xw") or 0.0
        self.yw = self.blackboard.read("yw") or 0.0
        self.theta = self.blackboard.read("theta") or 0.0
        self.latch_path()
        self.wp = None if not self.path or self.path_index >= len(self.path) else self.path[self.path_index]

    def write_Navigation_fun(self):
        self.blackboard.write("leftSpeed", float(self.leftSpeed))
        self.blackboard.write("rightSpeed", float(self.rightSpeed))
        self.blackboard.write("path_index", int(self.path_index))

    def velocity_control(self):
        self.rho = np.sqrt((self.xw - self.wp[0]) ** 2 + (self.yw - self.wp[1]) ** 2)
        self.alpha = np.arctan2(self.wp[1] - self.yw, self.wp[0] - self.xw) - self.theta
        if self.alpha > np.pi:
            self.alpha -= 2 * np.pi
        elif self.alpha < -np.pi:
            self.alpha += 2 * np.pi
        if abs(self.alpha) > np.pi / 2:
            self.leftSpeed = -self.alpha * self.P1
            self.rightSpeed = self.alpha * self.P1
        else:
            forward_speed = self.rho * self.P2
            turn_speed = -self.alpha * self.P1
            self.leftSpeed = forward_speed + turn_speed
            self.rightSpeed = forward_speed - turn_speed
        self.leftSpeed = max(min(self.leftSpeed, self.VMAX), -self.VMAX)
        self.rightSpeed = max(min(self.rightSpeed, self.VMAX), -self.VMAX)

    def waypoint_control(self):
        if self.rho < self.WP_PROXIMITY:
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.wp = None
                self.leftSpeed = 0.0
                self.rightSpeed = 0.0
            else:
                self.wp = self.path[self.path_index]

    def Navigate(self):
        self.read_Navigator_fun()
        if self.wp is None:
            self.leftSpeed = 0.0
            self.rightSpeed = 0.0
            self.write_Navigation_fun()
            return
        self.velocity_control()
        self.waypoint_control()
        self.write_Navigation_fun()

    def done(self):
        return self.wp is None

    def reset(self):
        self.path = None
        self.path_index = 0
        self.wp = None
