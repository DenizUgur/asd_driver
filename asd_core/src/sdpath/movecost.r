options(warn = -1)
movecost <- function (dtm, origin, destin, time="s", resolution=0.05) {

  cost_function <- function(x){ 
    ifelse(x[adj] > 0, 
      0.5 * exp(-0.3 * abs(x[adj])),
      0.5 * exp(-0.1 * abs(x[adj])))
  }

  altDiff <- function(x){ x[2] - x[1] }
  hd <- gdistance::transition(dtm, altDiff, 8, symm=FALSE)
  slope <- gdistance::geoCorrection(hd)

  adj <- raster::adjacent(dtm, cells=1:ncell(dtm), pairs=TRUE, directions=8)
  slope[adj] <- cost_function(slope)
  Conductance <- gdistance::geoCorrection(slope)

  accum_final <- gdistance::accCost(Conductance, sp::coordinates(origin))
  accum_final <- raster::mask(accum_final, dtm)

  sPath <- gdistance::shortestPath(Conductance, sp::coordinates(origin), sp::coordinates(destin), output="SpatialLines")
  sPath$length <- rgeos::gLength(sPath, byid=TRUE)
  destin$cost <- raster::extract(accum_final, destin)

  results <- list("accumulated.cost.raster"=accum_final,
                  "cost.raster"=raster(Conductance)@data@values,
                  "LCPs"=sPath,
                  "dest.loc.w.cost"=destin)
}