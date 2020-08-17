# This file will track sectors and will provide get/set interface for sectors.
import numpy as np
import math
import ray
from matplotlib import pyplot as plt
import time
import rospy
from grid_map_msgs.srv import GetGridMap


@ray.remote
class RayTracer:
    def __init__(self, w, h):
        self.map = np.empty((w, h))
        self.count = 0

    def main(self, dtm, region, viewer):
        # TODO: implementation
        
        start, end = region
        self.map[start[1] : end[1], start[0] : end[0]] = dtm

    def get_map(self):
        return self.map


class SectorService:
    def __init__(self):
        self.terrain = None
        self.pose = None
        self.local_size = 11.0  # meters
        self.resolution = 0.05  # meters

        self.get_submap = rospy.ServiceProxy(
            "/elevation_mapping/get_submap", GetGridMap
        )

        self.a_sector_size = 1.0  # meters
        self.b_sector_size = 4.0  # meters

        self.sector_count = int(self.local_size / self.a_sector_size)
        assert (
            self.sector_count % 2 != 0
        )  # Must be an odd number to create correct sectors

    def is_online(self):
        return True

    def is_update_required(self):
        # * Get the current local terrain
        self.update_terrain(
            self.get_submap(
                "odom",
                self.pose.position.x,
                self.pose.position.y,
                self.local_size,
                self.local_size,
                ["elevation"],
            ).map
        )
        # * Divide current local terrain into A sectors (size defined in __init__)
        # TODO: Optimization in the future
        return True

    def update_terrain(self, payload):
        raw = payload.data[0]
        self.terrain = np.array(raw.data, dtype=float)
        self.terrain.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        self.terrain = np.rot90(self.terrain, k=2) * 100

    def update_pose(self, payload):
        self.pose = payload.pose

    def spiral(self, l):
        n = len(l)
        for l2 in l:
            assert n == len(l2)
        clone_l = [subl[:] for subl in l]
        return self._spiral(clone_l)

    def _spiral(self, l, upsideDown=False):
        if len(l) == 0:
            return []
        if len(l) == 1:
            return l[0]
        ret = None
        if upsideDown:
            ret = l.pop(-1)
            ret.reverse()
            for subl in reversed(l):
                ret.append(subl.pop(0))
        else:
            ret = l.pop(0)
            for subl in l:
                ret.append(subl.pop(-1))
        ret.extend(self._spiral(l, not upsideDown))
        return ret

    def start_alpha_thread(self):
        """
        Alpha Thread is for identifying small sectors (A Sector) and applying ray-tracing on the local terrain.
        Ray-trace results will be stored in memory to be used later for stitching local area.
        """
        a_sectors = []
        sector_px = int((self.a_sector_size * 100) / (self.resolution * 100))
        sector_count = int(self.local_size / self.a_sector_size)
        shape = [sector_count * sector_px, sector_count * sector_px]
        RT = RayTracer.remote(*shape)

        for x in range(sector_count):
            a_sectors_line = []
            for y in range(sector_count):
                a_sectors_line.append(
                    [
                        self.terrain[
                            y * sector_px : (y + 1) * sector_px,
                            x * sector_px : (x + 1) * sector_px,
                        ],
                        (
                            (x * sector_px, y * sector_px),
                            ((x + 1) * sector_px, (y + 1) * sector_px),
                        ),
                        (
                            self.local_size / self.resolution / 2,
                            self.local_size / self.resolution / 2,
                            self.pose.position.z * 100,
                        ),
                    ]
                )
            a_sectors.append(a_sectors_line)

        a_sectors = self.spiral(a_sectors)
        a_sectors.reverse()

        [RT.main.remote(*args) for args in a_sectors]
        final = ray.get(RT.get_map.remote())

        #! Remove this after testing
        if int(np.nanmax(final) / 100) == 0:
            plt.contourf(final, cmap="terrain")
        else:
            plt.contourf(
                final,
                cmap="terrain",
                levels=range(
                    int(np.nanmin(final)),
                    int(np.nanmax(final)),
                    int(np.nanmax(final) / 100),
                ),
            )

        plt.plot(
            [self.local_size / self.resolution / 2],
            [self.local_size / self.resolution / 2],
            ".r",
            markersize="10",
        )
        plt.colorbar()

        #! For visualizing sectors
        # for c in a_sectors:
        #     currentAxis = plt.gca()
        #     currentAxis.add_patch(
        #         plt.Rectangle(
        #             (c[1][0][0], c[1][0][1]),
        #             sector_px,
        #             sector_px,
        #             alpha=0.4,
        #             facecolor="red",
        #         )
        #     )
        #     plt.pause(0.1)

        plt.show()

        exit(1)

        return {
            "traversable": 43,
            "ray_traced": 435,
            "unknown": 2,
        }

