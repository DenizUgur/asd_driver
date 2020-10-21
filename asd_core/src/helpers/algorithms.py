import heapq
import numpy as np
import math

from numpy.lib.function_base import select
from sdpath.rdp import rdp


class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        results = [
            (x - 1, y + 1),
            (x, y + 1),
            (x + 1, y + 1),
            (x + 1, y),
            (x + 1, y - 1),
            (x, y - 1),
            (x - 1, y - 1),
            (x - 1, y),
        ]
        # results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0:
            pass
            # results.reverse()  # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node, cost_so_far, i):
        return self.weights.get(to_node, 1)
        Co = cost_so_far.get(from_node, 1)
        Ci = cost_so_far.get(to_node, 1)
        weight = 1
        hd = self.weights.get(to_node, 1) - self.weights.get(from_node, 1)
        if i < 8:
            if i % 2 == 0:
                a = math.atan(hd / math.sqrt(2))
                x = math.sqrt(2 + hd ** 2) * ((Co + Ci) / 2 + a * weight)
            else:
                a = math.atan(hd)
                x = math.sqrt(1 + hd ** 2) * ((Co + Ci) / 2 + a * weight)
        else:
            n = list(super().neighbors(from_node))
            C1 = cost_so_far.get(n[i - 8], 1)
            if i == 15:
                C2 = cost_so_far.get(n[0], 1)
            else:
                C2 = cost_so_far.get(n[i - 7], 1)

            a = math.atan(hd / math.sqrt(5))
            x = math.sqrt(5 + hd ** 2) * ((Co + C1 + C2 + Ci) / 4 + a * weight)
        if x < 0:
            exit(34)
        return x


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for i, next in enumerate(graph.neighbors(current)):
            new_cost = cost_so_far[current] + graph.cost(current, next, cost_so_far, i)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)  # optional
    path.reverse()  # optional
    return np.array(path)


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for i, next in enumerate(graph.neighbors(current)):
            new_cost = cost_so_far[current] + graph.cost(current, next, cost_so_far, i)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def custom_search(dem, start, end, c1, c2):
    sx, sy = start
    ex, ey = end

    paths = lambda x, y: [
        [[x - 5, y], [x - 10, y], [x - 15, y]],
        [[x - 5, y + 4], [x - 10, y + 6], [x - 15, y + 8]],
        [[x - 5, y - 4], [x - 10, y - 6], [x - 15, y - 8]],
        [[x + 5, y], [x + 10, y], [x + 15, y]],
        [[x + 5, y + 4], [x + 10, y + 6], [x + 15, y + 8]],
        [[x + 5, y - 4], [x + 10, y - 6], [x + 15, y - 8]],
        [[x - 5, y + 4], [x - 7, y + 9], [x - 9, y + 14]],
        [[x + 5, y + 4], [x + 7, y + 9], [x + 9, y + 14]],
        [[x, y + 4], [x, y + 9], [x, y + 14]],
        [[x - 5, y - 4], [x - 7, y - 9], [x - 9, y - 14]],
        [[x + 5, y - 4], [x + 7, y - 9], [x + 9, y - 14]],
        [[x, y - 4], [x, y - 9], [x, y - 14]],
    ]
    full_path = [[sx, sy]]
    while math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2) > 10:
        cp = paths(sx, sy)

        values = []
        for i, path in enumerate(cp):
            path = np.array(path)

            cum = 0
            skip = False
            for point in path:
                try:
                    cum += dem.weights.get((point[0], point[1]))
                except TypeError:
                    skip = True
                    break
            if skip:
                continue

            dist = math.sqrt((ey - path[-1][1]) ** 2 + (ex - path[-1][0]) ** 2)

            values.append([i, cum * c1 + dist * c2])

        if len(values) == 0:
            break
        selected = cp[sorted(values, key=lambda x: x[1])[0][0]]
        full_path.extend(selected)
        sx, sy = selected[-1][0], selected[-1][1]
        # print(math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2))

    full_path.append([ex, ey])
    return np.array(full_path)

