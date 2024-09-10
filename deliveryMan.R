##############################################################################################################


walterDM=function(roads, car, packages) {
  nextMove=0
  closest
  if (car$load==0) {
    # gets the first transition of the route to the 'cheapest' package
    nextMove = findCheapestPackage(roads, car, packages)[1]
  }
  else {
    # gets the first transition of the cheapest rout
    nextMove = runAstar(roads, list(car$x, car$y), list(car$load[3], car$load[4]))[1]
  }
  car$nextMove = nextMove
  return (car)
}
#' This function loops through all packages that have not been delivered and runs A* on them
#' It returns the path to the cheapest package
#' @param roads See help documentation for the runDeliveryMan function
#' @param car See help documentation for the runDeliveryMan function
#' @param packages See help documentation for the runDeliveryMan function
#' @return A list of transitions that illustrates the route to the 'cheapest' package. A transition is described by 2,4,5,6,8 to indicate the direction
findCheapestPackage=function(roads, car, packages){
  # initialise
    # cheapestPackage is the number of the package that is the shortest
  cheapestPackage = -1
    # shortestPath represents the path to the cheapestPackage
  #' TODO: find a proper way to represent paths
  shortestPath = Inf
  # loop trough all packages
  for(package in seq_along(packages)){
    # if package is delivered, skip
    if(package[[5]] == 2) next
    # if the car is holding a package, throw error (but should not happen)
    if(package[[5]] == 1){
      stop("Car is holding a package, but it is looking for the closest package. 'findCheapestPackage' is called while it should not have been called")
    }
    
    # run A* with the current package as the goal and the current location of the car as the start
    newPath = runAstar(roads, list(car$x, car$y), list(package[[1]], package[[2]]))
    # compare this distance/path with the current shortest path
    #' TODO: find way to represent and compare paths
    if(newPath < shortestPath){
      # If shorter, update shortest path and package.
      shortestPath = newPath
      cheapestPackage = package
    }
  }
  return(shortestPath)
}

# This function runs through all possible routes and returns the cheapest.
#' used for inspiration: https://www.algorithms-and-technologies.com/a_star/r
#' @param roads See help documentation for the runDeliveryMan function
#' @param start list with x and y coordinate of the start
#' @param goal list with x and y coordinate of the goal
#' @return A list of transitions that illustrate the rout to the destination. A transition is described by 2,4,5,6,8 to indicate the direction
#' @export
runAstar=function(roads, start, goal){
  # initialisation:
  distances = rep(Inf, nrow(roads))
  distances[start] = 0
  
  # TODO: find manhattan distance between start and goal
  mdistance = 0
  
  costs = rep(Inf, nrow(roads))
  costs[start] = mdistance
  
  visited = rep(FALSE, nrow(roads))
  
  # While there are still nodes left to visit:
  repeat{
    #' find cheapest transition
    cheapest_trans = Inf
    cheapest_trans_index = -1
    for(i in seq_along(costs)){ #' seq_along is like getting the length of costs.
      #' go through nodes that has not been visited
      if(costs[i] < cheapest_trans && !visited[i]){
        cheapest_trans = costs[i]
        cheapest_trans_index = i
      }
    }
    if(cheapest_trans_index == -1){
      #node not found
      return (-1)
    } else if(cheapest_trans_index == goal){
      #' goal node found
      #' TODO: find out how to return the correct path -> start from the goal node
      #' and find the adjacent node with the lowest cost and add that to the list.
      #' from this node, find the 'cheapest' neigbour again until you are back at the start.
      return (list(5,5,5,5,5,5,5))
    }
  }
  return (list(5,5,5,5,5))
}

#' function to retrieve the manhattan distance between two points
#' @param start A list of 2 elements, the first is the x coordinate, the second is the y coordinate
#' @param goal A list of 2 elements, the first is the x coordinate, the second is the y coordinate
manhattanDistance=function(start, goal){
  return(abs(start[[1]] - goal[[1]]) + abs(start[[2]] - goal[[2]]))
}

#################################################################################################################

