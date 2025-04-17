myFunction=function(roads, car, packages) {
  nextMove=0
  if (car$load==0) {
    cheapestPackage = findPathToCheapestPackage(roads, car, packages)
    nextMove = pathToNextMove(cheapestPackage$path)
  }
  else {
    packageDestination = c(packages[car$load, 3],packages[car$load, 4])
    nextPath = a_star(roads, c(car$x, car$y), packageDestination)$path
    nextMove = pathToNextMove(nextPath)
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
findPathToCheapestPackage=function(roads, car, packages){
  shortestPath = list(distance = Inf, path = c())
  for(package in seq_len(nrow(packages))){
    if(packages[package, 5] == 2){
      next
    }
    if(packages[package, 5] == 1){
      stop("Car is holding a package, but it is looking for the closest package. 'findCheapestPackage' is called while it should not have been called")
    }
    
    newPath = a_star(roads, c(car$x, car$y), c(packages[package,1], packages[package,2]))
    if(newPath$distance < shortestPath$distance){
      shortestPath = newPath
    }
  }
  return(shortestPath)
}

#' Finds the shortest distance and path between two nodes using the A-star (A*) algorithm
#' @param roads two matrices which represent the horizontal and vertical roads. The value of a cell represents the cost of moving to that cell.
#' @param start start coordinates e.g. of the car
#' @param goal goal coordinates e.g. of a package
#' @return A list containing the shortest distance and the path to the goal node
a_star = function(roads, start, goal) {
  nrow_grid= nrow(roads$hroads) + 1
  ncol_grid= ncol(roads$hroads) 
  
  start_index = node_index(start[1], start[2], ncol_grid)
  goal_index = node_index(goal[1], goal[2], ncol_grid)
  
  # Distances from the start node to all other nodes
  distances = rep(Inf, nrow_grid * ncol_grid)
  distances[start_index] = 0
  
  # Priorities with which to visit the nodes
  node_priorities = rep(Inf, nrow_grid * ncol_grid)
  node_priorities[start_index] = manhattanDistance(start, goal)
  
  # This keeps track of the parent of each node for path reconstruction
  parent = rep(NA, nrow_grid * ncol_grid)
  
  visited_nodes = rep(FALSE, nrow_grid * ncol_grid)
  
  repeat {
    inspected_node_priority = Inf
    inspected_node_index = -1
    for (i in seq_along(node_priorities)) {
      if (node_priorities[i] < inspected_node_priority && !visited_nodes[i]) {
        inspected_node_priority = node_priorities[i]
        inspected_node_index = i
      }
    }
    
    if (inspected_node_index == -1) {
      return(list(distance = -1, path = NULL))
    } 
    
    current_coords = index_to_coords(inspected_node_index, ncol_grid, nrow_grid)
    current_row = current_coords[1]
    current_col = current_coords[2]
    
    if (inspected_node_index == goal_index) {
      # reconstruct path
      path = c(goal)
      while (!is.na(parent[inspected_node_index])) {
        inspected_node_index = parent[inspected_node_index]
        node_coords = index_to_coords(inspected_node_index, ncol_grid, nrow_grid)
        path = append(node_coords, path)
      }
      
      return(list(distance = distances[goal_index], path = path))
    }
    
    # Right neighbor
    if (current_row < nrow_grid) {
      right_index = node_index(current_row + 1, current_col, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$hroads[current_row , current_col]
      if (new_dist < distances[right_index]) {
        distances[right_index] = new_dist
        node_priorities[right_index] = new_dist + manhattanDistance(c(current_row + 1, current_col),goal)
        parent[right_index] = inspected_node_index
      }
    }
    
    # Down neighbor
    if (current_col > 1) {
      down_index = node_index(current_row , current_col - 1, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$vroads[current_row , current_col - 1]
      if (new_dist < distances[down_index]) {
        distances[down_index] = new_dist
        node_priorities[down_index] = new_dist + manhattanDistance(c(current_row , current_col - 1),goal)
        parent[down_index] = inspected_node_index
      }
    }
    
    # Left neighbor
    if (current_row > 1) {
      left_index = node_index(current_row - 1, current_col , ncol_grid)
      new_dist = distances[inspected_node_index] + roads$hroads[current_row - 1, current_col ]
      if (new_dist < distances[left_index]) {
        distances[left_index] = new_dist
        node_priorities[left_index] = new_dist + manhattanDistance(c(current_row - 1, current_col),goal)
        parent[left_index] = inspected_node_index
      }
    }
    
    # Up neighbor
    if (current_col < ncol_grid) {
      up_index = node_index(current_row , current_col + 1, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$vroads[current_row , current_col]
      if (new_dist < distances[up_index]) {
        distances[up_index] = new_dist
        node_priorities[up_index] = new_dist + manhattanDistance(c(current_row , current_col + 1),goal)
        parent[up_index] = inspected_node_index
      }
    }
    
    visited_nodes[inspected_node_index] = TRUE
  }
}

####### Helper Astar functions #######

# Convert coordinates to indices in a 1D representation of the grid
node_index = function(row, col, ncol_grid) {
  return((row - 1) * ncol_grid + col)
}

# Reverse: Convert node index back to (row, col) coordinates
index_to_coords = function(index, ncol_grid, nrow_grid) {
  row = ((index - 1) %/% ncol_grid) + 1
  col = ((index - 1) %% ncol_grid) + 1
  return(c(row, col))
}

#' function to retrieve the manhattan distance between two points
#' @param start A list of 2 elements, the first is the x coordinate, the second is the y coordinate
#' @param goal A list of 2 elements, the first is the x coordinate, the second is the y coordinate
manhattanDistance=function(start, goal){
  return(abs(start[[1]] - goal[[1]]) + abs(start[[2]] - goal[[2]]))
}

pathToNextMove=function(path){
  # handle if on same node
  if(length(path) <= 2){
    return(5)
  }
  
  changeX = path[1] - path[3]
  changeY = path[2] - path[4]
  
  if (changeX == 0 && changeY == 0) {
    return(5)
  } else if (changeX == -1 && changeY == 0) {
    return(6)
  } else if (changeX == 1 && changeY == 0) {
    return(4)
  } else if (changeX == 0 && changeY == -1) {
    return(8)
  } else if (changeX == 0 && changeY == 1) {
    return(2)
  } else {
    stop("invalid transition. changeX: " ++ changeX ++ ", ChangeY: " ++ changeY)
  }
}
