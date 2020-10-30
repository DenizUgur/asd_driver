import heapq
import numpy as np
import math
from timeit import default_timer as dt
from sdpath.acc import CostAccumulator


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

    full_path = [[sx, sy]]
    while math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2) > 10:
        _, selected = custom_search_select(dem, (sx, sy), (ex, ey), c1, c2)
        if selected is None:
            break
        full_path.extend(selected)
        sx, sy = selected[-1][0], selected[-1][1]
        # print(math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2))

    full_path.append([ex, ey])
    return np.array(full_path)


def custom_search_select(dem, start, end, c1, c2, target=False):
    sx, sy = start
    ex, ey = end
    paths = lambda x, y, s: [
        [[x - 5 * s, y], [x - 10 * s, y], [x - 15 * s, y]],
        [[x - 5 * s, y + 4 * s], [x - 10 * s, y + 6 * s], [x - 15 * s, y + 8 * s]],
        [[x - 5 * s, y - 4 * s], [x - 10 * s, y - 6 * s], [x - 15 * s, y - 8 * s]],
        [[x + 5 * s, y], [x + 10 * s, y], [x + 15 * s, y]],
        [[x + 5 * s, y + 4 * s], [x + 10 * s, y + 6 * s], [x + 15 * s, y + 8 * s]],
        [[x + 5 * s, y - 4 * s], [x + 10 * s, y - 6 * s], [x + 15 * s, y - 8 * s]],
        [[x - 5 * s, y + 4 * s], [x - 7 * s, y + 9 * s], [x - 9 * s, y + 15 * s]],
        [[x + 5 * s, y + 4 * s], [x + 7 * s, y + 9 * s], [x + 9 * s, y + 15 * s]],
        [[x, y + 4 * s], [x, y + 9 * s], [x, y + 15 * s]],
        [[x - 5 * s, y - 4 * s], [x - 7 * s, y - 9 * s], [x - 9 * s, y - 15 * s]],
        [[x + 5 * s, y - 4 * s], [x + 7 * s, y - 9 * s], [x + 9 * s, y - 15 * s]],
        [[x, y - 4 * s], [x, y - 9 * s], [x, y - 15 * s]],
    ]
    cpR = paths(sx, sy, 0.05)
    cp = paths(dem.shape[0] // 2, dem.shape[1] // 2, 1)
    values = []
    for i, path in enumerate(cp):
        path = np.array(path)

        cum = 0
        skip = False
        for point in path:
            try:
                cum += dem[point[1], point[0]]
            except TypeError:
                skip = True
                break
        if skip:
            continue

        referance_point = cpR[i][-1]
        dist = math.sqrt(
            (ey - referance_point[1]) ** 2 + (ex - referance_point[0]) ** 2
        )

        values.append([i, cum * c1 + dist * c2])

    if len(values) == 0:
        return None

    if target:
        return cpR[sorted(values, key=lambda x: x[1])[0][0]][-1]
    else:
        return cp[sorted(values, key=lambda x: x[1])[0][0]]


def custom_search2(dem, current, target, c1=18.0, c2=2.0):
    all_times = []

    cost_acc = CostAccumulator()
    cost_acc.process_dem(dem[10:20, 10:20])
    cost_acc.set_points((0, 0))
    cost_acc.calculate()

    sx, sy = current
    ex, ey = target

    full_path = [[sx, sy]]
    full_start = dt()

    dist = math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2)
    while dist > 15.0:
        dist = math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2)
        print(dist)
        start = dt()
        cost_acc.process_dem(dem[sy - 30 : sy + 30, sx - 30 : sx + 30])
        _dem = cost_acc.calculate()
        # _dem = -1 * _dem + np.nanmax(_dem)
        _dem = scale(_dem, out_range=(0, 1))
        end = dt()

        all_times.append(end - start)
        selected = custom_search2_sel(
            _dem, (sx - 30, sy - 30), _dem.shape, target, (sx, sy), c1, c2
        )

        sx, sy = int(selected[0]), int(selected[1])

        full_path.append((sx, sy))

    full_end = dt()
    full_path.append([ex, ey])
    return (
        np.array(full_path),
        [
            full_end - full_start,
            max(all_times),
            min(all_times),
            np.mean(np.array(all_times)),
        ],
    )


def custom_search3(dem, current, target, c1=0.8, c2=2.0):
    all_times = []

    sx, sy = current
    ex, ey = target

    full_path = [[sx, sy]]
    full_start = dt()

    while math.sqrt((sy - ey) ** 2 + (sx - ex) ** 2) > 20:
        start = dt()
        _dem = dem[sy - 40 : sy + 40, sx - 40 : sx + 40]
        _dem = scale(_dem, out_range=(0, 1))
        end = dt()
        all_times.append(end - start)
        selected = custom_search2_sel(
            _dem, (sx - 40, sy - 40), _dem.shape, target, (sx, sy), c1, c2
        )

        sx, sy = int(selected[0]), int(selected[1])

        full_path.append((sx, sy))

    full_end = dt()
    full_path.append([ex, ey])
    return (
        np.array(full_path),
        [
            full_end - full_start,
            max(all_times),
            min(all_times),
            np.mean(np.array(all_times)),
        ],
    )


def perimiter(h, w, px, py, R=1.0, step=0.1):
    angles = np.linspace(0, 2 * math.pi, retstep=step)[0]
    return (
        np.vstack((R * 20 * np.cos(angles) + px, R * 20 * np.sin(angles) + py)).T,
        np.vstack(
            (R * 20 * np.cos(angles) + (h // 2), R * 20 * np.sin(angles) + (w // 2))
        ).T.astype(int),
    )


def scale(x, out_range=(0, 100)):
    domain = np.nanmin(x), np.nanmax(x)
    y = (x - (domain[1] + domain[0]) / 2) / (domain[1] - domain[0])
    return y * (out_range[1] - out_range[0]) + (out_range[1] + out_range[0]) / 2


def custom_search2_sel(costmap, extent, dim, target, current, c1=18.0, c2=2.0):
    XMI, YMI = extent
    h, w = dim
    x, y = target
    px, py = current

    distance_map = np.fromfunction(
        lambda iy, ix: (((XMI + ix * 1) - x) ** 2 + ((YMI + iy * 1) - y) ** 2) ** 0.5,
        (h, w),
        dtype=float,
    )
    distance_map = scale(distance_map, out_range=(0, 1))

    values = []
    fusedmap = c1 * costmap + c2 * distance_map

    R = 0.6
    _, perI = perimiter(h, w, px, py, R=R * 0.5)
    per, perM = perimiter(h, w, px, py, R=R)
    _, perO = perimiter(h, w, px, py, R=R * 1.5)

    values = []
    for p, (pri, prm, pro) in zip(per, zip(perI, perM, perO)):
        val = (
            fusedmap[pri[1], pri[0]]
            + fusedmap[prm[1], prm[0]]
            + fusedmap[pro[1], pro[0]]
        )
        if np.isnan(val):
            continue
        values.append([p, val])

    target = sorted(values, key=lambda k: k[1])[0][0]
    return target
