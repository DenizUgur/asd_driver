# This file will track sectors and will provide get/set interface for sectors.
import numpy as np
import math
import time
import rospy
from grid_map_msgs.srv import GetGridMap


class RayTracer:
    def __init__(self, w, h):
        self.map = np.empty((w, h))
        self.status = 0
        self.errors = []

    def calc_line(self, pox, poy, vx=100, vy=100, vz=10.0):
        """
        Returns the path to point and expected heights
        """
        if abs(pox - vx) > abs(poy - vy):
            if pox > vx:
                xs = np.arange(vx, pox + 0, step=1)
            else:
                xs = np.flip(np.arange(pox, vx + 0, step=1))

            if poy > vy:
                ys = np.linspace(vy, poy + 0, num=len(xs))
            else:
                ys = np.flip(np.linspace(poy, vy + 0, num=len(xs)))
        else:
            if poy > vy:
                ys = np.arange(vy, poy + 0, step=1)
            else:
                ys = np.flip(np.arange(poy, vy + 0, step=1))

            if pox > vx:
                xs = np.linspace(vx, pox + 0, num=len(ys))
            else:
                xs = np.flip(np.linspace(pox, vx + 0, num=len(ys)))

        xs = xs.astype(int)
        ys = ys.astype(int)
        zs = np.linspace(vz, 0, num=max(len(xs), len(ys)))

        arr = np.column_stack((ys, xs, zs))
        fx, fy, _ = arr[0]
        if fx != 100 or fy != 100:
            arr = np.insert(arr, 0, [100, 100, vz], axis=0)

        return arr

    def main(self, dtm, region, viewer):
        start, end = region
        if np.count_nonzero(np.isnan(dtm)) == 0:
            self.map[start[1] : end[1], start[0] : end[0]] = dtm
        else:
            # Find origin of current sector
            ox, oy = start[0], end[1]
            vx, vy, vz = viewer

            # Start filling nan values
            while np.count_nonzero(np.isnan(dtm)) != 0:
                for py, px in np.argwhere(np.isnan(dtm)):
                    pox = ox + px
                    poy = oy - py
                    now = dtm[py, px]

                    # We might already fixed this one
                    if not np.isnan(now):
                        continue

                    # First find the line to point from viewer
                    # TODO: This causes delay. Must start from other side of sector and implement multithreading later.
                    ray = self.calc_line(pox, poy, vz=vz)

                    try:
                        higher = False
                        on_ray = None
                        on_ray_point = None
                        delta_h = 999
                        acutal_line = []
                        for step in ray:
                            sy, sx, sz = int(step[0]), int(step[1]), step[2]
                            curr = self.map[sy - 1][sx - 1]
                            acutal_line.append(curr)

                            if not np.isnan(curr):
                                if curr > sz:
                                    higher = True
                                    break
                                else:
                                    if abs(sz - curr) < delta_h:
                                        on_ray = curr
                                        on_ray_point = (sy, sx)

                        if higher:
                            dtm[py, px] = acutal_line[-1]
                            # TODO: This should be a mirror
                            # if abs(poy - vy) > abs(pox - vx):
                            #     # y>x
                            #     if poy - vy > 0:
                            #         # up
                            #         edge_y = len(dtm)
                            #         edge_x = edge_y * pox / poy
                            #     else:
                            #         # down
                            #         edge_y = 0
                            #         edge_x = edge_y * pox / poy
                            # else:
                            #     # x>y
                            #     if pox - vx > 0:
                            #         # right
                            #         edge_x = len(dtm)
                            #         edge_y = edge_x * poy / pox
                            #     else:
                            #         # left
                            #         edge_x = 0
                            #         edge_y = edge_x * poy / pox

                            # rest = self.calc_line(edge_x, edge_y, pox, poy)
                            # for i, r in enumerate(rest):
                            #     ry, rx, _ = int(r[0]), int(r[1]), r[2]
                            #     if np.isnan(self.map[ry, rx]):
                            #         self.map[ry, rx] = acutal_line[-(i % len(acutal_line))]
                            #     else:
                            #         break

                        else:
                            dh = vz - on_ray
                            y, x = on_ray_point
                            dist = math.sqrt((y - vy) ** 2 + (x - vx) ** 2)
                            ratio = dh / dist
                            dist = math.sqrt((y - poy) ** 2 + (x - pox) ** 2)
                            dh = ratio * dist
                            dtm[py, px] = on_ray - dh

                        self.map[start[1] : end[1], start[0] : end[0]] = dtm
                        self.status = 1
                    except Exception as why:
                        self.errors.append({"time": time.localtime, "msg": why})
                        self.map[start[1] : end[1], start[0] : end[0]] = 0
                        self.status = -1
                        break

    def get_map(self):
        return self.map

    def get_status(self):
        return self.status

    def get_errors(self):
        tmp = self.errors
        self.errors = []
        return tmp


class SectorService:
    def __init__(self):
        self.terrain = None
        self.pose = None
        self.local_size = 7.0  # meters
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
        self.sector_px = int((self.a_sector_size * 100) / (self.resolution * 100))
        self.sector_count = int(self.local_size / self.a_sector_size)
        shape = [self.sector_count * self.sector_px, self.sector_count * self.sector_px]
        RT = RayTracer(*shape)

        for x in range(self.sector_count):
            a_sectors_line = []
            for y in range(self.sector_count):
                a_sectors_line.append(
                    [
                        np.copy(
                            self.terrain[
                                y * self.sector_px : (y + 1) * self.sector_px,
                                x * self.sector_px : (x + 1) * self.sector_px,
                            ]
                        ),
                        (
                            (x * self.sector_px, y * self.sector_px),
                            ((x + 1) * self.sector_px, (y + 1) * self.sector_px),
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

        try:
            [RT.main(*args) for args in a_sectors]
            final = RT.get_map()
            status, errors = RT.get_status(), RT.get_errors()
            del RT
        except Exception as why:
            print(why)
            return None, None, None, None

        return status, errors, final, self.terrain

