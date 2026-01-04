MAP_W = 300
MAP_H = 300
ARENA_CENTER_X = -0.5
ARENA_CENTER_Y = -1.5
ARENA_SIZE_M = 6.0
X_MIN = ARENA_CENTER_X - ARENA_SIZE_M / 2.0
X_MAX = ARENA_CENTER_X + ARENA_SIZE_M / 2.0
Y_MIN = ARENA_CENTER_Y - ARENA_SIZE_M / 2.0
Y_MAX = ARENA_CENTER_Y + ARENA_SIZE_M / 2.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def world2map(xw, yw):
    nx = (xw - X_MIN) / (X_MAX - X_MIN)
    ny = (Y_MAX - yw) / (Y_MAX - Y_MIN)
    px = int(nx * (MAP_W - 1) + 0.5)
    py = int(ny * (MAP_H - 1) + 0.5)
    return clamp(px, 0, MAP_W - 1), clamp(py, 0, MAP_H - 1)

def map2world(px, py):
    nx = px / (MAP_W - 1)
    ny = py / (MAP_H - 1)
    xw = X_MIN + nx * (X_MAX - X_MIN)
    yw = Y_MAX - ny * (Y_MAX - Y_MIN)
    return xw, yw
