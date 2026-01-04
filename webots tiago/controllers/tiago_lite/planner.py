from A_star_alg import A_star
import numpy as np
import math
import location_conversion as convert

def get_correct_path(collision_map, graph, start, goal):
    astar = A_star(collision_map, graph)
    start_px, start_py = convert.world2map(*start)
    goal_px, goal_py = convert.world2map(*goal)
    first_node = (start_py, start_px)
    end_node = (goal_py, goal_px)
    path = astar.search(first_node, end_node)
    if not path:
        print("Goal not found.")
        return None
    map_path = [convert.map2world(c, r) for (r, c) in path]
    drop = 15
    map_path = map_path[min(drop, len(map_path)) :]
    return map_path

def build_graph(collision_map):
    rows, cols = collision_map.shape
    node_graph = {}
    directions = [
        (-1, 0, 1),
        (1, 0, 1),
        (0, -1, 1),
        (0, 1, 1),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]
    for x in range(rows):
        for y in range(cols):
            neighbors = []
            for dx, dy, cost in directions:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < rows and 0 <= ny < cols:
                    neighbors.append((cost, (nx, ny)))
            node_graph[(x, y)] = neighbors
    return node_graph

def map_path():
    return [
        (0.8, 0.0),
        (0.8, -0.8),
        (0.75, -1.6),
        (0.65, -2.3),
        (0.3, -2.9),
        (-0.4, -3.05),
        (-1.2, -3.05),
        (-1.6, -2.6),
        (-1.65, -1.8),
        (-1.65, -1.0),
        (-1.65, -0.3),
        (-1.3, 0.3),
        (-0.7, 0.55),
        (0.1, 0.45),
        (0.6, 0.2),
        (0.8, 0.0),
        (0.6, 0.2),
        (0.1, 0.45),
        (-0.7, 0.55),
        (-1.3, 0.3),
        (-1.65, -0.3),
        (-1.65, -1.0),
        (-1.65, -1.8),
        (-1.6, -2.6),
        (-1.2, -3.05),
        (-0.4, -3.05),
        (0.3, -2.9),
        (0.65, -2.3),
        (0.75, -1.6),
        (0.8, -0.8),
        (0.8, 0.0),
        (0.0, 0.0),
    ]

path_index = 0
