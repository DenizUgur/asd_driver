import numpy as np
import math
import sys

if __name__ == "__main__":

    dist = lambda x, y: math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)

    for val in ["first", "second", "third"]:
        _t = np.load("data/{}_terrain.npy".format(val))
        _f = np.load("data/{}_fused.npy".format(val))

        ind = np.where(~np.isnan(_t))[0]
        first, last = ind[0], ind[-1]
        _t[:first] = _t[first]
        _t[last + 1 :] = _t[last]

        ind = np.where(~np.isnan(_f))[0]
        first, last = ind[0], ind[-1]
        _f[:first] = _f[first]
        _f[last + 1 :] = _f[last]

        x_s = np.linspace(0, _t.shape[0] - 1, _t.shape[0])
        y_s = np.linspace(0, _t.shape[0] - 1, _t.shape[0])

        x_s, y_s = np.meshgrid(x_s, y_s)

        x_f = x_s.flatten().reshape(-1, 1)
        y_f = y_s.flatten().reshape(-1, 1)
        _t_f = _t.flatten().reshape(-1, 1)
        _f_f = _f.flatten().reshape(-1, 1)

        _z = np.hstack((x_f, y_f, _t_f, _f_f))
        np.savetxt("data/{}_data.csv".format(val), _z, delimiter=",", fmt="%f")

        if sys.argv[-1] != "--no-paths":
            for ty in ["a_star_t", "a_star_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

            for ty in ["custom_t", "custom_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

            for ty in ["dijkstra_t", "dijkstra_f"]:
                data = np.load("data/paths/{}_{}.npy".format(val, ty))
                np.savetxt(
                    "data/paths/{}_{}.csv".format(val, ty),
                    data,
                    delimiter=",",
                    fmt="%f",
                )

