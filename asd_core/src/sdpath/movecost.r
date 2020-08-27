options(warn = -1)
movecost <- function (dtm, origin, destin, time="s", resolution=0.05) {

  cost_function <- function(x){ 
    ifelse(x[adj] > 0, 
      0.57 * exp(-56.0 * abs(x[adj])),
      1.40 * exp(-11.0 * abs(x[adj])))
  }

  # TODO: Cost Function
  cost_function <- function(x){ 
    6.0 * exp(-0.05 * abs(x[adj] + 0.05))
  }

  # Rise/Run to degrees
  # (atan(abs(x[adj]))*180/pi) turns rise/run into degrees
  #* x[adj] is rise/run in meters and its correct as raster calculated resolution automatically

  altDiff <- function(x){ x[2] - x[1] }
  hd <- gdistance::transition(dtm, altDiff, 8, symm=FALSE)
  slope <- gdistance::geoCorrection(hd)

  adj <- raster::adjacent(dtm, cells=1:ncell(dtm), pairs=TRUE, directions=8)
  speed <- slope
  speed[adj] <- cost_function(slope)
  #turn the walking speed from kmh to ms (0.278=1000/3600)
  speed <- speed * 0.278
  Conductance <- gdistance::geoCorrection(speed)

  accum_final <- gdistance::accCost(Conductance, sp::coordinates(origin))

  # accum_final is in seconds
  if (time=="h") {
    accum_final <- accum_final / 3600
  } else if (time=="m") {
    accum_final <- accum_final / 60
  } else {
    # accum_final <- accum_final / (3600*24)
  }

  accum_final <- raster::mask(accum_final, dtm)

  sPath <- gdistance::shortestPath(Conductance, sp::coordinates(origin), sp::coordinates(destin), output="SpatialLines")
  sPath$length <- rgeos::gLength(sPath, byid=TRUE)
  destin$cost <- raster::extract(accum_final, destin)

  results <- list("accumulated.cost.raster"=accum_final,
                  "cost.raster"=raster(Conductance)@data@values,
                  "LCPs"=sPath,
                  "dest.loc.w.cost"=destin)
}