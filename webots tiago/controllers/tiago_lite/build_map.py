import numpy as np
import math
import location_conversion as convert

MAP_W = 300
MAP_H = 300
ARENA_CENTER_X = -0.5
ARENA_CENTER_Y = -1.5
ARENA_SIZE_M = 6.0
X_MIN = ARENA_CENTER_X - ARENA_SIZE_M / 2.0
X_MAX = ARENA_CENTER_X + ARENA_SIZE_M / 2.0
Y_MIN = ARENA_CENTER_Y - ARENA_SIZE_M / 2.0
Y_MAX = ARENA_CENTER_Y + ARENA_SIZE_M / 2.0

class Map:
    def __init__(self, blackboard):
        self.blackboard = blackboard
        self.map = np.zeros((MAP_H, MAP_W), dtype=np.float32)
        self.prev_px = None
        self.prev_py = None
        self.blackboard.write("map_updates", [])

    def read_map(self):
        self.xw = self.blackboard.read("xw")
        self.yw = self.blackboard.read("yw")
        self.theta = self.blackboard.read("theta")
        self.lidar_image = self.blackboard.read("lidar_image")
        self.lidar_fov = self.blackboard.read("lidar_fov")

    def ranges_fun(self):
        if self.lidar_image is None:
            return None
        return np.asarray(self.lidar_image, dtype=np.float32)

    def angles_fun(self, n):
        if self.lidar_fov is None:
            return None
        return np.linspace(self.lidar_fov / 2.0, -self.lidar_fov / 2.0, n)

    def mapping(self):
        ranges = self.ranges_fun()
        if ranges is None:
            return

        n = len(ranges)
        angles = self.angles_fun(n)
        if angles is None:
            return

        ranges = np.where(np.isfinite(ranges), ranges, 100.0)

        r_T_s = np.array([[1, 0, 0.202], [0, 1, 0.0], [0, 0, 1]], dtype=np.float32)
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        w_T_r = np.array([[c, -s, self.xw], [s, c, self.yw], [0, 0, 1]], dtype=np.float32)
        w_T_s = w_T_r @ r_T_s

        X_i = np.vstack([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.ones(n, dtype=np.float32),
        ])

        D = w_T_s @ X_i
        xs, ys = D[0, :], D[1, :]

        px, py = convert.world2map(self.xw, self.yw)
        updates = []

        if self.prev_px is not None and 0 <= px < MAP_W and 0 <= py < MAP_H:
            updates.append((px, py, 0xFF0000))

        self.prev_px = px
        self.prev_py = py

        for i in range(80, min(587, n)):
            if ranges[i] <= 0.0 or ranges[i] >= 99.0:
                continue

            pxi, pyi = convert.world2map(xs[i], ys[i])
            if not (0 <= pxi < MAP_W and 0 <= pyi < MAP_H):
                continue

            self.map[pyi, pxi] = min(1.0, self.map[pyi, pxi] + 0.01)
            v = int(self.map[pyi, pxi] * 255)
            color = (v << 16) + (v << 8) + v
            updates.append((pxi, pyi, color))

        self.blackboard.write("map_updates", updates)

    def map_fun(self):
        self.read_map()
        self.mapping()
        self.blackboard.write("map", self.map)
