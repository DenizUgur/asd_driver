# This file will track sectors and will provide get/set interface for sectors.
import numpy as np
import math
import time
import rospy
from grid_map_msgs.srv import GetGridMap
import multiprocessing as mp
import queue


class RayTracer:
    NUMBER_OF_PROCESSES = 4

    def __init__(self):
        mp.set_start_method("forkserver")
        self.width = None
        self.height = None
        self.map = None
        self.task_queue = None
        self.done_queue = None
        self.workers = None
        self.start_workers()

    def get_map(self):
        m = np.min(self.map)
        return self.map + abs(m if m < 0 else 0)

    def set_map(self, init_map):
        self.map = init_map
        self.width, self.height = self.map.shape

    def start(self):
        nans = np.argwhere(np.isnan(self.map))
        quarters = []
        quarters.append(
            np.logical_and(nans[:, 0] >= self.height / 2, nans[:, 1] >= self.width / 2)
        )
        quarters.append(
            np.logical_and(nans[:, 0] >= self.height / 2, nans[:, 1] < self.width / 2)
        )
        quarters.append(
            np.logical_and(nans[:, 0] < self.height / 2, nans[:, 1] >= self.width / 2)
        )
        quarters.append(
            np.logical_and(nans[:, 0] < self.height / 2, nans[:, 1] < self.width / 2)
        )

        # Start jobs
        for q in quarters:
            self.task_queue.put(
                (np.copy(self.map), list(nans[q]), (self.width, self.height))
            )

        # Wait jobs
        for _ in range(len(quarters)):
            mask = np.isnan(self.map)
            self.map[mask] = self.done_queue.get(block=True)[mask]

        self.map[np.isnan(self.map)] = np.nanmedian(self.map)

    def start_workers(self):
        self.task_queue = mp.Queue()
        self.done_queue = mp.Queue()

        rospy.loginfo("Starting workers")

        self.workers = [
            mp.Process(
                target=self.worker, args=(self.task_queue, self.done_queue, None)
            )
            for _ in range(self.NUMBER_OF_PROCESSES)
        ]

        for p in self.workers:
            p.start()

        rospy.loginfo("Workers ready")

    def stop_workers(self):
        for p in self.workers:
            p.terminate()

    def worker(self, task_queue, done_queue, args):
        while 1:
            try:
                item = task_queue.get(block=True, timeout=0.05)
            except queue.Empty:
                continue

            if item == "STOP":
                break

            dtm, nans, (width, height) = item
            vx, vy = (width - 1) / 2, (height - 1) / 2
            nans.sort(
                key=lambda coord: math.sqrt((vy - coord[0]) ** 2 + (vx - coord[1]) ** 2)
            )

            for py, px in nans:
                if not np.isnan(dtm[py, px]):
                    continue

                try:
                    # * Extract the line containing viewer and this point
                    ray = self.calc_line(px, py, vx=vx, vy=vy)

                    # * Find the border point
                    if abs(py - vy) > abs(px - vx):
                        by = height - 1 if py - vy > 0 else 0
                        if not py is None and py > 0:
                            bx = by * px / py
                        else:
                            bx = 0
                        bx = (
                            bx
                            if ray[0][1] - ray[len(ray) - 1][1] > 0
                            else width - 1 - bx
                        )
                    elif abs(py - vy) == abs(px - vx):
                        by = height - 1 if py - vy > 0 else 0
                        bx = width - 1 if px - vx > 0 else 0
                    else:
                        bx = width - 1 if px - vx > 0 else 0
                        if not px is None and px > 0:
                            by = bx * py / px
                        else:
                            by = 0
                        by = (
                            by
                            if ray[0][0] - ray[len(ray) - 1][0] > 0
                            else height - 1 - by
                        )

                    rest = self.calc_line(bx, by, vx=px, vy=py)
                    ray = np.array(
                        [[y, x, dtm[y, x]] for y, x in np.concatenate((ray, rest[1:]))]
                    )

                    is_rest_nan = (
                        lambda arr: np.count_nonzero(~np.isnan(arr[:, 2])) == 0
                    )
                    for i, (ry, rx, _) in enumerate(ray):
                        if not np.isnan(dtm[math.floor(ry), math.floor(rx)]):
                            continue

                        # * Check if we need to place mirror or wood
                        if is_rest_nan(ray[i:]):
                            for i, (rpy, rpx, _) in enumerate(ray[i:]):
                                dtm[math.floor(ry), math.floor(rx)] = ray[:i][
                                    -(i % len(ray[:i]))
                                ]
                        else:
                            start_h = ray[i - 1]
                            end_i = np.argwhere(~np.isnan(ray[i:][:, 2]))[0]
                            end_h = ray[i:][end_i][0]
                            patch_distance = math.sqrt(
                                (end_h[0] - start_h[0]) ** 2
                                + (end_h[1] - start_h[1]) ** 2
                            )
                            ratio = (end_h[2] - start_h[2]) / patch_distance
                            step = patch_distance / end_i

                            for pi, (rpy, rpx, _) in enumerate(ray[i : i + end_i[0]]):
                                height = (pi * step[0]) * ratio
                                dtm[math.floor(rpy), math.floor(rpx)] = height

                except Exception:
                    dtm[py, px] = 0

            done_queue.put(dtm)

    def calc_line(self, pox, poy, vx, vy):
        """
        Returns the path to point and expected heights
        """
        if abs(pox - vx) > abs(poy - vy):
            if pox > vx:
                xs = np.arange(vx, pox, step=1)
            else:
                xs = np.flip(np.arange(pox, vx, step=1))

            if poy > vy:
                ys = np.linspace(vy, poy, num=len(xs))
            else:
                ys = np.flip(np.linspace(poy, vy, num=len(xs)))
        else:
            if poy > vy:
                ys = np.arange(vy, poy, step=1)
            else:
                ys = np.flip(np.arange(poy, vy, step=1))

            if pox > vx:
                xs = np.linspace(vx, pox, num=len(ys))
            else:
                xs = np.flip(np.linspace(pox, vx, num=len(ys)))

        xs = xs.astype(int)
        ys = ys.astype(int)

        arr = np.column_stack((ys, xs))
        fy, fx = arr[0]
        if fx != vx or fy != vy:
            arr = np.insert(arr, 0, [vy, vx], axis=0)

        fy, fx = arr[len(arr) - 1]
        if fx != pox or fy != poy:
            arr = np.insert(arr, len(arr), [poy, pox], axis=0)

        return arr


class SectorService:
    def __init__(self):
        self.terrain = None
        self.pose = None
        self.local_size = 7.0  # meters
        self.resolution = 0.05  # meters
        assert self.local_size / self.resolution % 2 == 0

        self.RT = RayTracer()
        self.get_submap = rospy.ServiceProxy(
            "/elevation_mapping/get_submap", GetGridMap
        )

    def shutdown(self):
        self.RT.stop_workers()

    def update_terrain(self):
        payload = self.get_submap(
            "odom",
            self.pose.position.x,
            self.pose.position.y,
            self.local_size,
            self.local_size,
            ["elevation"],
        ).map
        raw = payload.data[0]
        self.terrain = np.array(raw.data, dtype=float)
        self.terrain.shape = (raw.layout.dim[0].size, raw.layout.dim[1].size)
        self.terrain = np.rot90(self.terrain, k=2) * 100

    def get_extent(self):
        c = (self.local_size / self.resolution) / 2 + 1
        x = self.pose.position.x
        y = self.pose.position.y
        xmin = x - (c * self.resolution)
        ymin = y - (c * self.resolution)
        return (xmin, xmin + self.local_size, ymin, ymin + self.local_size)

    def update_pose(self, payload):
        self.pose = payload.pose

    def start_alpha_thread(self):
        """
        Alpha Thread is for applying ray-tracing on the local terrain.
        """
        try:
            self.RT.set_map(self.terrain)
            self.RT.start()
            final = self.RT.get_map()
        except Exception as why:
            print(repr(why))
            return None

        return final
