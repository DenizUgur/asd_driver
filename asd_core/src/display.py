import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    val = "first"
    ty = "a_star_t"

    _t = np.load("data/{}_terrain.npy".format(val))
    path = np.load("data/paths/{}_{}.npy".format(val, ty))

    X = np.arange(0, 300, 1)
    Y = np.arange(0, 300, 1)
    X, Y = np.meshgrid(X, Y)

    if False:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.contourf(X, Y, _t, levels=100)
        ax.plot(path[:, 0], path[:, 1], path[:, 2] + 10)
    else:
        plt.contourf(_t, levels=100)
        plt.plot(path[:, 0], path[:, 1], 'y-')
    plt.show()