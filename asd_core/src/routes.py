import numpy as np
from matplotlib import pyplot as plt
from alive_progress import alive_bar
from helpers.algorithms import (
    a_star_search,
    GridWithWeights,
    reconstruct_path,
    dijkstra_search,
    custom_search,
)
from timeit import default_timer as dt
import multiprocessing as mp


def get_el(elev, path):
    res = []
    for x, y in path:
        res.append(elev[y, x])
    return np.array(res).reshape(-1, 1)


def f(val, enable, start, end, c1=2.5, c2=12):
    bound = 10
    #! Elevation
    _t = np.load("data/{}_terrain.npy".format(val))

    costmap = GridWithWeights(*_t.shape)
    costmap.walls = []
    for xi in range(_t.shape[0]):
        for yi in range(_t.shape[0]):
            if xi < bound or xi > (_t.shape[0] - bound):
                costmap.walls.append((xi, yi))
            elif yi < bound or yi > (_t.shape[0] - bound):
                costmap.walls.append((xi, yi))

    datafile = open("data/{}_data.csv".format(val), "r")
    lines = datafile.readlines()
    fixed_lines = []

    for line in lines:
        fixed_lines.append(line.split(",")[:3])

    costmap.weights = {
        (int(float(x)), int(float(y))): float(z) for (x, y, z) in fixed_lines
    }

    if enable == 0:
        sdt = dt()
        path, _ = a_star_search(costmap, start, end)
        print("{} - a_star - elev - {:.4f} s".format(val, dt() - sdt))
        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_a_star_t".format(val), path)

        # plt.contourf(_t, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 1:
        sdt = dt()
        path, _ = dijkstra_search(costmap, start, end)
        print("{} - dijkstra - elev - {:.4f} s".format(val, dt() - sdt))
        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_dijkstra_t".format(val), path)

        # plt.contourf(_t, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 2:
        sdt = dt()
        path = custom_search(costmap, start, end, 20.5, 55.5)
        print("{} - custom - elev - {:.4f} s".format(val, dt() - sdt))
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_custom_t".format(val), path)

        # plt.contourf(_t, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    #! Slope
    _f = np.load("data/{}_fused.npy".format(val))

    costmap = GridWithWeights(*_f.shape)
    costmap.walls = []
    for xi in range(_f.shape[0]):
        for yi in range(_f.shape[0]):
            if xi < bound or xi > (_f.shape[0] - bound):
                costmap.walls.append((xi, yi))
            elif yi < bound or yi > (_f.shape[0] - bound):
                costmap.walls.append((xi, yi))

    datafile = open("data/{}_data.csv".format(val), "r")
    lines = datafile.readlines()
    fixed_lines = []

    for line in lines:
        fixed_lines.append([line.split(",")[0], line.split(",")[1], line.split(",")[3]])

    costmap.weights = {
        (int(float(x)), int(float(y))): float(z) for (x, y, z) in fixed_lines
    }

    if enable == 3:
        sdt = dt()
        path, _ = a_star_search(costmap, start, end)
        print("{} - a_star - fused - {:.4f} s".format(val, dt() - sdt))
        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_a_star_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 4:
        sdt = dt()
        path, _ = dijkstra_search(costmap, start, end)
        print("{} - dijkstra - fused - {:.4f} s".format(val, dt() - sdt))
        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_dijkstra_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 5:
        sdt = dt()
        path = custom_search(costmap, start, end, c1, c2)
        print("{} - custom - fused - {:.4f} s".format(val, dt() - sdt))
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_custom_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()


if __name__ == "__main__":
    mp.set_start_method("forkserver")

    processes = []
    for i in range(6):
        processes.append(mp.Process(target=f, args=("first", i, (25, 50), (260, 180))))
        processes.append(mp.Process(target=f, args=("second", i, (26, 240), (246, 50))))
        processes.append(mp.Process(target=f, args=("third", i, (230, 253), (69, 230), 3.5)))

    for p in processes:
        p.start()

    for p in processes:
        p.join()
