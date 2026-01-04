class Blackboard:
    def __init__(self):
        self.blackboard = {
            "xw": 0.0,
            "yw": 0.0,
            "theta": 0.0,
            "robot": 0.0,
            "gps": 0.0,
            "compass": 0.0,
            "lidar": 0.0,
            "leftMotor": 0.0,
            "rightMotor": 0.0,
            "timestep": 0.0,
            "compassVals": 0.0,
            "gpsVals": 0.0,
            "rho": 0.0,
            "alpha": 0.0,
            "path_index": 0,
            "map": None,
        }

    def read(self, key):
        return self.blackboard.get(key, None)

    def write(self, key, value):
        self.blackboard[key] = value
