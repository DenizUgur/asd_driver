import numpy as np
from matplotlib import pyplot as plt

if __name__ == "__main__":
    fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(
        nrows=2, ncols=3, figsize=(30, 30)
    )

    ax1.contourf(np.load("data/first_terrain.npy"), levels=100, cmap="binary")
    ax2.contourf(np.load("data/second_terrain.npy"), levels=100, cmap="binary")
    ax3.contourf(np.load("data/third_terrain.npy"), levels=100, cmap="binary")
    ax4.contourf(np.load("data/first_fused.npy"), levels=100, cmap="binary")
    ax5.contourf(np.load("data/second_fused.npy"), levels=100, cmap="binary")
    ax6.contourf(np.load("data/third_fused.npy"), levels=100, cmap="binary")

    path = np.load("data/paths/first_a_star_f.npy")
    ax4.plot(path[:, 0], path[:, 1])
    # path = np.load("data/paths/first_dijkstra_f.npy")
    # ax4.plot(path[:, 0], path[:, 1], "--")
    path = np.load("data/paths/first_custom_f.npy")
    ax4.plot(path[:, 0], path[:, 1])

    path = np.load("data/paths/second_a_star_f.npy")
    ax5.plot(path[:, 0], path[:, 1])
    # path = np.load("data/paths/second_dijkstra_f.npy")
    # ax5.plot(path[:, 0], path[:, 1], "--")
    path = np.load("data/paths/second_custom_f.npy")
    ax5.plot(path[:, 0], path[:, 1])

    path = np.load("data/paths/third_a_star_f.npy")
    ax6.plot(path[:, 0], path[:, 1])
    path = np.load("data/paths/third_dijkstra_f.npy")
    ax6.plot(path[:, 0], path[:, 1], "--")
    path = np.load("data/paths/third_custom_f.npy")
    ax6.plot(path[:, 0], path[:, 1])

    fig.tight_layout()
    plt.show()
