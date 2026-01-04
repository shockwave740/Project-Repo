import numpy as np
import math
from heapq import heappush, heappop
from collections import defaultdict


class A_star:
    def __init__(self, collision_map, graph: dict):
        self.collision_map = collision_map
        self.graph = graph

    def getNeighbors(self, coord):
        neighbors = self.graph.get(coord, [])
        safe_neighbors = []
        for neighbor in neighbors:
            x = neighbor[1][0]
            y = neighbor[1][1]
            if self.collision_map[x][y] == False:
                safe_neighbors.append((neighbor[0], (x, y)))
        return safe_neighbors

    def path_trace(self, prev_nodes: dict, node: tuple, start: tuple):
        p = node
        path = []

        while p != start:
            if p not in prev_nodes:
                return None
            path.append(p)
            p = prev_nodes.get(p)
        path.append(start)

        path.reverse()

        return path

    def search(self, start: tuple, goal: tuple):
        self.distances = defaultdict(lambda: float("inf"))
        self.distances[start] = 0
        heap = []
        prev_nodes = {}
        visited = set()
        start_heuristic = np.sqrt((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2)

        heappush(heap, (start_heuristic, start))
        print(f"Starting at {start}")
        while heap:
            current_node = heappop(heap)

            # splitting needed coordinate out
            current_xy = current_node[1]
            current_cost = current_node[0]

            curr_heuristic = np.sqrt(
                (goal[0] - current_xy[0]) ** 2 + (goal[1] - current_xy[1]) ** 2
            )
            if current_cost > self.distances[current_xy] + curr_heuristic:
                # protects against wrong path from heap
                continue

            if current_xy in visited:
                continue
            visited.add(current_xy)

            if current_xy == goal:
                print(f"Goal found: {current_xy}")
                path = self.path_trace(prev_nodes, current_xy, start)
                print(path)
                return path

            current_neighbors = self.getNeighbors(current_xy)

            for neighbor in current_neighbors:
                # splitting into coordinate xy and cost
                neigh_xy = neighbor[1]
                neigh_cost = neighbor[0]

                new_distance = self.distances[current_xy] + neigh_cost
                heuristic = np.sqrt(
                    (goal[0] - neigh_xy[0]) ** 2 + (goal[1] - neigh_xy[1]) ** 2
                )
                if new_distance < self.distances[neigh_xy]:
                    self.distances[neigh_xy] = new_distance
                    priority = new_distance + heuristic
                    prev_nodes[neigh_xy] = current_xy
                    heappush(heap, (priority, neigh_xy))

        print("Goal not found.")


if __name__ == "__main__":
    # Generating graphs, start and goal with distances for a-star
    graph = {}
    rows = 20
    cols = 30

    directions = [
        (-1, 0, 1),  # up
        (1, 0, 1),  # down
        (0, -1, 1),  # left
        (0, 1, 1),  # right
        (-1, -1, math.sqrt(2)),  # diagonals
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]

    for x in range(rows):
        for y in range(cols):
            neighbors = []
            for dx, dy, cost in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols:
                    neighbors.append(
                        (cost, (nx, ny))
                    )  # (cost, coordinates) so that heap can sort by cost
            graph[(x, y)] = neighbors

    collision_map = np.random.rand(rows, cols) < 0.1

    start = (np.random.randint(0, rows), np.random.randint(0, cols))

    goal = (np.random.randint(0, rows), np.random.randint(0, cols))

    # Running a_star
    a_star = A_star(collision_map, graph)
    result = a_star.search(start, goal)
