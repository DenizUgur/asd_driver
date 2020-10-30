from os import close
import numpy as np
import math
import sys
from sdpath.acc import CostAccumulator
from timeit import default_timer as dt
import json


def scale(x, out_range=(0, 100)):
    domain = np.nanmin(x), np.nanmax(x)
    y = (x - (domain[1] + domain[0]) / 2) / (domain[1] - domain[0])
    return y * (out_range[1] - out_range[0]) + (out_range[1] + out_range[0]) / 2


if __name__ == "__main__":

    dist = lambda x, y: math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)

    data = {}

    # * Warm up CA
    cost_acc = CostAccumulator()
    cost_acc.process_dem(np.ones((10, 10)))
    cost_acc.set_points((5, 5))
    cost_acc.calculate()

    for val in ["third"]:
        if sys.argv[-1] != "--no-maps":
            _t = np.load("data/{}_terrain.npy".format(val))

            start = dt()
            cost_acc.process_dem(_t)
            cost_acc.set_points((0, 0))
            _f = cost_acc.calculate()
            _f = -1 * _f + np.nanmax(_f)
            end = dt()
            np.save("data/{}_fused".format(val), _f)

            data[val] = {}
            data[val]["fused"] = {}
            data[val]["fused"]["process_time"] = end - start
            print(end - start)

            x_s = np.linspace(0, _t.shape[0] - 1, _t.shape[0])
            y_s = np.linspace(0, _t.shape[0] - 1, _t.shape[0])

            x_s, y_s = np.meshgrid(x_s, y_s)

            x_f = x_s.flatten().reshape(-1, 1)
            y_f = y_s.flatten().reshape(-1, 1)
            _t_f = _t.flatten().reshape(-1, 1)
            _f_f = _f.flatten().reshape(-1, 1)

            _t = scale(_t)
            _f = scale(_f)
            _t_f_s = _t.flatten().reshape(-1, 1)
            _f_f_s = _f.flatten().reshape(-1, 1)

            _z = np.hstack((x_f, y_f, _t_f, _f_f, _t_f_s, _f_f_s))
            np.savetxt("data/{}_data.csv".format(val), _z, delimiter=",", fmt="%f")

            data[val]["matlab_surface_file"] = "data/{}_data.csv".format(val)

        if sys.argv[-1] != "--no-paths":
            for ty in ["a_star_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

            for ty in ["custom_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

            for ty in ["dijkstra_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

    # with open("data/data.json", "w") as data_output:
    #     json.dump(data, data_output)

