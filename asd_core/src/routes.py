import numpy as np
from matplotlib import pyplot as plt
from alive_progress import alive_bar
from helpers.algorithms import (
    a_star_search,
    GridWithWeights,
    reconstruct_path,
    dijkstra_search,
    custom_search2,
    custom_search3,
)
from timeit import default_timer as dt
import multiprocessing as mp
import json


manager = mp.Manager()
data = manager.dict()


def get_el(elev, path):
    res = []
    for x, y in path:
        res.append(elev[y, x])
    return np.array(res).reshape(-1, 1)


class JSONEncoderWithDictProxy(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, mp.managers.DictProxy):
            return dict(o)
        return json.JSONEncoder.default(self, o)


def f(val, enable, start, end, _c1=17.0, _c2=2.0):
    bound = 10
    #! Elevation
    _t = np.load("data/{}_terrain.npy".format(val))

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
        fixed_lines.append(
            [line.split(",")[0], line.split(",")[1], line.split(",")[-1]]
        )

    costmap.weights = {
        (int(float(x)), int(float(y))): float(z) for (x, y, z) in fixed_lines
    }

    if enable == 0:
        sdt = dt()
        path, _ = a_star_search(costmap, start, end)
        print("{} - a_star - fused - {:.4f} s".format(val, dt() - sdt))
        data[val]["a_star"]["fused"] = dt() - sdt

        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_a_star_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 1:
        sdt = dt()
        path, _ = dijkstra_search(costmap, start, end)
        print("{} - dijkstra - fused - {:.4f} s".format(val, dt() - sdt))
        data[val]["dijkstra"]["fused"] = dt() - sdt

        path = reconstruct_path(path, start, end)
        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_dijkstra_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()

    if enable == 2:
        path, timings = custom_search2(_t, start, end, c1=_c1, c2=_c2)
        print("{} - custom - fused - {:.4f} s".format(val, timings[0]))
        data[val]["custom"]["fused"] = {
            "full": timings[0],
            "max": timings[1],
            "min": timings[2],
            "mean": timings[3],
        }

        path = np.hstack((path, get_el(_t, path)))
        np.save("data/paths/{}_custom_f".format(val), path)

        # plt.contourf(_f, levels=100)
        # plt.plot(path[:, 0], path[:, 1], "y-")
        # plt.show()


if __name__ == "__main__":
    data["first"] = manager.dict()
    data["second"] = manager.dict()
    data["third"] = manager.dict()

    data["first"]["a_star"] = manager.dict()
    data["second"]["a_star"] = manager.dict()
    data["third"]["a_star"] = manager.dict()
    data["first"]["dijkstra"] = manager.dict()
    data["second"]["dijkstra"] = manager.dict()
    data["third"]["dijkstra"] = manager.dict()
    data["first"]["custom"] = manager.dict()
    data["second"]["custom"] = manager.dict()
    data["third"]["custom"] = manager.dict()

    with mp.Pool(4) as pool:
        for i in range(3):
            # pool.apply_async(f, ("first", i, (230, 930), (900, 425), 0.45, 0.55))
            # pool.apply_async(f, ("second", i, (287, 308), (875, 675), 0.4, 0.6))
            pool.apply_async(f, ("third", i, (490, 798), (561, 339), 0.30, 0.70))

        pool.close()
        pool.join()

    # processes = []
    # for i in range(3):
    #     f("first", i, (230, 930), (900, 425), 0.45, 0.55)
    #     f("second", i, (287, 308), (875, 675), 0.4, 0.6)
    #     f("third", i, (490, 798), (561, 339), 0.30, 0.70)
        # processes.append(
        #     mp.Process(target=f, args=("first", i, (100, 725), (900, 425), 0.45, 0.55))
        # )
        # processes.append(
        #     mp.Process(target=f, args=("second", i, (170, 170), (910, 700), 0.40, 0.60))
        # )
        # processes.append(
        #     mp.Process(target=f, args=("third", i, (215, 300), (890, 700), 0.3, 0.7))
        # )

    # for p in processes:
    #     p.start()

    # for p in processes:
    #     p.join()

    with open("data/data.paths.json", "w") as file:
        json.dump(data, file, cls=JSONEncoderWithDictProxy)
