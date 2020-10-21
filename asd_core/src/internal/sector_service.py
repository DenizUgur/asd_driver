import numpy as np
import math
import time
import rospy
import yaml
import cv2
from struct import pack, unpack

from grid_map_msgs.srv import GetGridMap
from grid_map_msgs.msg import GridMap

from sdpath.acc import CostAccumulator


class SectorService:
    def __init__(self):
        self.config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)
        self.terrain = None
        self.uncertainty = None
        self.colormap = None
        self.luminance = None
        self.sandmap = None

        self.pose = None
        self.cost_acc = CostAccumulator()
        assert self.config["local_size"] / self.config["resolution"] % 2 == 0

        self.get_submap = rospy.ServiceProxy(
            "/elevation_mapping/get_submap", GetGridMap
        )

    def scale(self, x, out_range=(0, 100)):
        domain = np.nanmin(x), np.nanmax(x)
        y = (x - (domain[1] + domain[0]) / 2) / (domain[1] - domain[0])
        return y * (out_range[1] - out_range[0]) + (out_range[1] + out_range[0]) / 2

    def update_terrain(self):
        payload = self.get_submap(
            "odom",
            self.pose.position.x,
            self.pose.position.y,
            self.config["local_size"],
            self.config["local_size"],
            [],
        ).map

        raw = payload.data[0]
        self.terrain = np.array(raw.data, dtype=float)
        self.terrain.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        self.terrain = self.scale(np.rot90(self.terrain, k=2))

        raw = payload.data[1]
        upper = np.array(raw.data, dtype=float)
        upper.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        upper = np.rot90(upper, k=2)

        raw = payload.data[2]
        lower = np.array(raw.data, dtype=float)
        lower.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        lower = np.rot90(lower, k=2)
        self.uncertainty = self.scale(upper - lower)

        raw = payload.data[3]

        red, green, blue = [], [], []
        for e in np.array(raw.data):
            if np.isnan(e):
                red.append(255)
                green.append(255)
                blue.append(255)
                continue

            b = pack("f", e)
            c = unpack("I", b)[0]
            red.append((c >> 16) & 0x0000FF)
            green.append((c >> 8) & 0x0000FF)
            blue.append(c & 0x0000FF)

        self.colormap = np.dstack(
            (
                np.array(red, dtype=int),
                np.array(green, dtype=int),
                np.array(blue, dtype=int),
            )
        )
        self.colormap.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size, 3)
        self.colormap = np.rot90(self.colormap, k=2)

        luminance = []
        for e in np.array(raw.data):
            if np.isnan(e):
                luminance.append(100)
                continue

            b = pack("f", e)
            c = unpack("I", b)[0]
            r = (c >> 16) & 0x0000FF
            g = (c >> 8) & 0x0000FF
            b = (c) & 0x0000FF

            vR = r / 255
            vG = g / 255
            vB = b / 255

            lR = vR / 12.92 if vR <= 0.04045 else ((vR + 0.055) / 1.055) ** 2.4
            lG = vG / 12.92 if vG <= 0.04045 else ((vG + 0.055) / 1.055) ** 2.4
            lB = vB / 12.92 if vB <= 0.04045 else ((vB + 0.055) / 1.055) ** 2.4

            Y = 0.2126 * lR + 0.7152 * lG + 0.0722 * lB
            L_star = Y * 903.3 if Y <= 0.008856 else (Y ** (1 / 3)) * 116 - 16

            luminance.append(L_star)

        self.luminance = np.array(luminance, dtype=float)
        self.luminance.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        self.luminance = self.scale(np.rot90(self.luminance, k=2))

        self.sandmap = cv2.inRange(self.colormap, (0, 0, 0), (30, 30, 30))
        # self.sandmap = cv2.inRange(self.colormap, (193, 97, 12), (193, 170, 12))
        self.sandmap = self.sandmap.astype(float)
        self.sandmap = self.scale(self.sandmap)
        self.sandmap[np.isnan(self.terrain)] = np.nan

    def get_extent(self):
        c = (self.config["local_size"] / self.config["resolution"]) / 2 + 1
        x = self.pose.position.x
        y = self.pose.position.y
        xmin = x - (c * self.config["resolution"])
        ymin = y - (c * self.config["resolution"])
        return (
            xmin,
            xmin + self.config["local_size"],
            ymin,
            ymin + self.config["local_size"],
        )

    def update_pose(self, payload):
        self.pose = payload.pose

    def start_alpha_thread(self, loc):
        """
        Alpha Thread is for applying ray-tracing on the local terrain.
        """
        from matplotlib import pyplot as plt

        try:
            self.cost_acc.process_dem(
                self.terrain, extent=self.get_extent(),
            )
            self.cost_acc.set_points(loc)
            COST = self.cost_acc.calculate()
            COST[np.isnan(self.terrain)] = np.nan
            COST = self.scale(COST)

            fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(
                nrows=2, ncols=3, sharex=True, sharey=True, figsize=(20, 12)
            )

            umap = ax1.contourf(
                self.uncertainty,
                cmap="turbo",
                levels=range(
                    int(np.nanmin(self.uncertainty)),
                    int(np.nanmax(self.uncertainty)),
                    1,
                ),
            )
            dtm = ax2.contourf(
                self.terrain,
                cmap="terrain",
                levels=range(
                    int(np.nanmin(self.terrain)), int(np.nanmax(self.terrain)), 1,
                ),
            )
            cost = ax3.contourf(
                COST,
                cmap="jet_r",
                levels=range(int(np.nanmin(COST)), int(np.nanmax(COST)), 1),
            )
            luminance = ax4.contourf(
                self.luminance,
                cmap="hot",
                levels=range(
                    int(np.nanmin(self.luminance)), int(np.nanmax(self.luminance)), 1
                ),
            )
            sandmap = ax6.contourf(
                self.sandmap,
                cmap="summer",
                levels=range(
                    int(np.nanmin(self.sandmap)), int(np.nanmax(self.sandmap)), 1
                ),
            )

            qUR = 0.8
            qEC = -2
            qLU = -0.2
            qSP = 0.6

            self.fusedmap = self.scale(
                (
                    qUR * self.uncertainty
                    + qEC * COST
                    + qLU * self.luminance
                    + qSP * self.sandmap
                )
            )

            fusedmap = ax5.contourf(
                self.fusedmap,
                cmap="RdYlGn_r",
                levels=range(
                    int(np.nanmin(self.fusedmap)), int(np.nanmax(self.fusedmap)), 1,
                ),
            )
            plt.colorbar(fusedmap, ax=ax5)
            ax5.set_title("Fused Map")

            plt.colorbar(umap, ax=ax1)
            ax1.set_title("Uncertainty Range")

            plt.colorbar(dtm, ax=ax2)
            ax2.set_title("Terrain")

            plt.colorbar(cost, ax=ax3)
            ax3.set_title("Energy Consumption Model")

            plt.colorbar(luminance, ax=ax4)
            ax4.set_title("Luminance")

            plt.colorbar(sandmap, ax=ax6)
            ax6.set_title("Sand Probability")

            fig.tight_layout()
            plt.show()

            q_save = input("Save? [y/n] [n]: ")
            if q_save == "y":
                q_save_name = input("Name? ")
                np.save("data/{}_terrain".format(q_save_name), self.terrain)
                np.save("data/{}_fused".format(q_save_name), self.fusedmap)
            
            exit(1)

            #! Save all
            # fig.savefig("all_wo_bird.eps", format="eps", dpi=500)

            # ax5.clear()
            # ax5.imshow(self.colormap)
            # ax5.set_title("Bird's-eye View")
            # ax5.set_anchor("W")

            # fig.savefig("all_w_bird.eps", format="eps", dpi=500)

        except Exception as why:
            print(repr(why))
            return None