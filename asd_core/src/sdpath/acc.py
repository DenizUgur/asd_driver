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


class CostAccumulator:
    TIMEOUT = 60

    def __init__(self, dem=None):
        rpy2.robjects.r["options"](warn=-1)
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

    def set_points(self, origin):
        """
        Set origin as (x, y) tuple
        """
        sx, sy = origin
        ro.r("origin_r <- cbind({}, {})".format(sx, sy))

    def r_connection(self, raster=False, dem=None):
        if raster:
            self.nr, self.nc = dem.shape
            dem = ro.r.matrix(dem, nrow=self.nr, ncol=self.nc)
            self.DEM_R = self.raster.raster(dem)
        else:
            self.result = self.movecost.movecost(ro.r.dem, ro.r.origin_r)

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
        Calculate the COST_ACC and return it
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
        ro.reval("COST_DF <- as.data.frame(result$cost.raster)")

        COST = np.array(ro.r.COST_DF, dtype=float).reshape(self.nr, self.nc)

        self.result = None
        return COST
