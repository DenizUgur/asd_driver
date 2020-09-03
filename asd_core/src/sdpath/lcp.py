# pylint: disable=no-member relative-beyond-top-level
import rpy2.robjects as ro
import rpy2.robjects.numpy2ri
from rpy2.robjects.packages import importr
from rpy2.robjects.packages import STAP
from .rdp import rdp

import numpy as np
import os
from threading import Thread

rpy2.robjects.numpy2ri.activate()


class LCP:
    TIMEOUT = 10

    def __init__(self, dem=None):
        self.raster = importr("raster")
        self.sp = importr("sp")

        script_dir = os.path.dirname(__file__)
        movecost_path = os.path.join(script_dir, "movecost.r")

        with open(movecost_path, "r") as f:
            buf = f.read()
        self.movecost = STAP(buf, "movecost")

        if dem is not None:
            self.process_dem(dem)

    def check_point(self, x, y):
        minx, maxx, miny, maxy = self.extent
        self.valid_target = False

        if y > maxy:
            if x > maxx:
                return maxx, maxy
            elif x < minx:
                return minx, maxy
            else:
                return x, maxy
        elif y < miny:
            if x > maxx:
                return maxx, miny
            elif x < minx:
                return minx, miny
            else:
                return x, miny
        else:
            if x > maxx:
                return maxx, y
            elif x < minx:
                return minx, y
            else:
                self.valid_target = True
                return x, y

    def set_points(self, origin, destination):
        """
        Set origin and destination as (x, y) tuples
        """
        sx, sy = origin
        self.ex, self.ey = destination

        ro.r("origin_r <- cbind({}, {})".format(sx, sy))
        ro.r(
            "destination_r <-  cbind({}, {})".format(
                *self.check_point(self.ex, self.ey)
            )
        )

    def r_connection(self, raster=False, dem=None):
        if raster:
            self.nr, self.nc = dem.shape
            dem = ro.r.matrix(dem, nrow=self.nr, ncol=self.nc)
            self.DEM_R = self.raster.raster(dem)
        else:
            self.result = self.movecost.movecost(
                ro.r.dem, ro.r.origin_r, ro.r.destination_r, resolution=self.resolution,
            )

    def process_dem(self, dem, resolution=1.0, extent=None):
        self.resolution = resolution
        t = Thread(target=self.r_connection, args=[True, dem])
        t.start()
        t.join(self.TIMEOUT)

        if self.DEM_R is None:
            raise Exception("Timed out converting to RasterLayer.")

        if extent is None:
            self.extent = (0, self.nc, 0, self.nr)
        else:
            self.extent = extent

        ro.r.assign("dem", self.DEM_R)
        ro.r("dem_extent <- extent({}, {}, {}, {})".format(*self.extent))
        ro.r("dem <- setExtent(dem, dem_extent)")
        ro.r("names(dem) <- 'v'")
        ro.r("dem <- na.omit(dem)")

        self.DEM_R = None

    def calculate(self):
        """
        Calculate the LCP and return results
        """
        if ro.r.origin_r is None:
            raise Exception("no points have been set!")

        t = Thread(target=self.r_connection)
        t.start()
        t.join(self.TIMEOUT)

        if self.result is None:
            raise Exception("Timed out calculating a route.")

        # Gather results
        ro.r.assign("result", self.result)
        ro.reval("LCP_DF <- as.data.frame(coordinates(result$LCPs)[1])")
        ro.reval("LCP_LENGTH_DF <- as.data.frame(result$LCPs$length[1])")
        ro.reval("COST_ACC_DF <- as.data.frame(result$accumulated.cost.raster)")
        ro.reval("COST_DF <- as.data.frame(result$cost.raster)")
        ro.reval("ENERGY_COST_DF <- as.data.frame(result$dest.loc.w.cost$cost[1])")

        LCP = ro.r.LCP_DF.astype([("x", "<f8"), ("y", "<f8")]).view("<f8")
        if not self.valid_target:
            LCP = np.append(LCP, [self.ex, self.ey])

        COST = np.array(ro.r.COST_DF, dtype=float).reshape(self.nr, self.nc)
        COST = np.flip(COST, axis=0)

        self.result = None
        return rdp(LCP.reshape(-1, 2), 0.15), COST
