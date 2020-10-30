import numpy as np
import signal
from alive_progress import alive_bar
from timeit import default_timer as dt
from helpers.algorithms import custom_search2, custom_search3


def handler(signum, frame):
    raise Exception("end of time")


def get_el(elev, path):
    res = []
    for x, y in path:
        res.append(elev[y, x])
    return np.array(res).reshape(-1, 1)


if __name__ == "__main__":
    AB = {
        "first": {"A": (100, 725), "B": (900, 425)},
        "second": {"A": (90, 90), "B": (910, 700)},
        "third": {"A": (230, 190), "B": (890, 700)},
    }

    c1_range = list(np.linspace(0.1, 50.0, retstep=0.1)[0])
    c2_range = list(np.linspace(0.1, 50.0, retstep=0.1)[0])

    # * Define timeout
    signal.signal(signal.SIGALRM, handler)

    for val in ["first", "second", "third"]:
        _t = np.load("data/{}_terrain.npy".format(val))
        _f = np.load("data/{}_fused.npy".format(val))

        for ty in ["_t", "_f"]:
            referance = np.load("data/paths/{}_dijkstra{}.npy".format(val, ty))

            results = []

            with alive_bar(len(c1_range) * len(c2_range)) as bar:
                for c1_ in c1_range:
                    for c2_ in c2_range:
                        start = dt()
                        try:
                            signal.setitimer(signal.ITIMER_REAL, 5.5)
                            if ty == "_t":
                                path, _ = custom_search3(
                                    _t, AB[val]["A"], AB[val]["B"], c1=c1_, c2=c2_
                                )
                                path = np.hstack((path, get_el(_t, path)))
                            else:
                                path, _ = custom_search3(
                                    _f, AB[val]["A"], AB[val]["B"], c1=c1_, c2=c2_
                                )
                                path = np.hstack((path, get_el(_t, path)))

                            print(path, len(path))
                            print(referance, len(referance))
                            exit(1)

                            SSE = np.sum(np.power(referance - path, 2))
                            results.append([val, ty, SSE, c1_, c2_])
                        except Exception as w:
                            pass
                        bar("{} :: {} :: {:.2f}".format(c1_, c2_, dt() - start))

            #* Print results
            best = lambda file, ty: sorted([a for a in results if file in a and ty in a], key=lambda a: a[2])
            print(best(val, ty))
