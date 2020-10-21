movecost <- function (dtm, origin, time="s", resolution=0.05) {

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

  results <- list("cost.raster"=raster(Conductance)@data@values)
}