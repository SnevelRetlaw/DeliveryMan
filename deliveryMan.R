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

#' Finds the shortest distance and path between two nodes using the A-star (A*) algorithm
#' @param roads two matrices which represent the horizontal and vertical roads. The value of a cell represents the cost of moving to that cell.
#' @param start start coordinates e.g. of the car
#' @param goal goal coordinates e.g. of a package
#' @return A list containing the shortest distance and the path to the goal node
a_star = function(roads, start, goal) {
  nrow_grid= nrow(roads$hroads) + 1
  ncol_grid= ncol(roads$hroads) 
  
  # Convert the start and goal coordinates to node indices
  start_index = node_index(start[1], start[2], ncol_grid)
  goal_index = node_index(goal[1], goal[2], ncol_grid)
  
  # This contains the distances from the start node to all other nodes, initialized with a distance of "Infinity"
  distances = rep(Inf, nrow_grid * ncol_grid)
  
  # The distance from the start node to itself is 0
  distances[start_index] = 0
  
  # This contains the priorities with which to visit the nodes, calculated using the heuristic.
  node_priorities = rep(Inf, nrow_grid * ncol_grid)
  
  # Start node has a priority equal to straight-line distance to the goal.
  node_priorities[start_index] = manhattanDistance(start, goal)
  
  # This keeps track of the parent of each node for path reconstruction
  parent = rep(NA, nrow_grid * ncol_grid)
  
  # This contains whether a node was already visited
  visited_nodes = rep(FALSE, nrow_grid * ncol_grid)
  
  # While there are nodes left to visit...
  repeat {
    # Find the node with the currently lowest priority...
    inspected_node_priority = Inf
    inspected_node_index = -1
    for (i in seq_along(node_priorities)) {
      # Going through all nodes that haven't been visited yet
      if (node_priorities[i] < inspected_node_priority && !visited_nodes[i]) {
        inspected_node_priority = node_priorities[i]
        inspected_node_index = i
      }
    }
    
    if (inspected_node_index == -1) {
      # There was no node not yet visited --> No path found
      return(list(distance = -1, path = NULL))
    } 
    
    # Get the coordinates of the current node
    current_coords = index_to_coords(inspected_node_index, ncol_grid, nrow_grid)
    current_row = current_coords[1]
    current_col = current_coords[2]
    
    if (inspected_node_index == goal_index) {
      # Goal node found, reconstruct the path
      cat("Goal node found!\n")
      
      # Reconstruct the path by backtracking from the goal node to the start node
      path = c(goal_index)
      while (!is.na(parent[inspected_node_index])) {
        inspected_node_index = parent[inspected_node_index]
        path = append(inspected_node_index, path)
      }
      
      return(list(distance = distances[goal_index], path = path))
    }
    
    # Visit the node with the lowest priority
    cat(
      "Visiting node",
      inspected_node_index,
      "with currently lowest priority of",
      inspected_node_priority,
      "\n"
    )
    #(3,1)-h(1,3)-(3,2)-h(2,3)-(3,3)
    #  |            |            |
    #v(1,2)        v(2,2)       v(3,2)
    #  |            |            |
    #(2,1)-h(1,2)-(2,2)-h(2,2)-(2,3)
    #  |            |            |
    #v(1,1)        v(2,1)       v(3,1)
    #  |            |            |
    #(1,1)-h(1,1)-(1,2)-h(2,1)-(1,3)
    
    # Check all neighboring nodes
    # Right neighbor
    if (current_col < ncol_grid) {
      right_index = node_index(current_row, current_col + 1, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$hroads[current_col , current_row]
      if (new_dist < distances[right_index]) {
        distances[right_index] = new_dist
        node_priorities[right_index] = new_dist + manhattanDistance(c(current_row, current_col + 1),goal)
        parent[right_index] = inspected_node_index
      }
    }
    
    # Down neighbor
    if (current_row > 1) {
      down_index = node_index(current_row - 1, current_col, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$vroads[current_col, current_row - 1]
      if (new_dist < distances[down_index]) {
        distances[down_index] = new_dist
        node_priorities[down_index] = new_dist + manhattanDistance(c(current_row - 1, current_col),goal)
        parent[down_index] = inspected_node_index
      }
    }
    
    # Left neighbor
    if (current_col > 1) {
      left_index = node_index(current_row, current_col - 1, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$hroads[current_col - 1, current_row]
      if (new_dist < distances[left_index]) {
        distances[left_index] = new_dist
        node_priorities[left_index] = new_dist + manhattanDistance(c(current_row, current_col - 1),goal)
        parent[left_index] = inspected_node_index
      }
    }
    
    # Up neighbor
    if (current_row < nrow_grid) {
      up_index = node_index(current_row + 1, current_col, ncol_grid)
      new_dist = distances[inspected_node_index] + roads$vroads[current_col, current_row]
      if (new_dist < distances[up_index]) {
        distances[up_index] = new_dist
        node_priorities[up_index] = new_dist + manhattanDistance(c(current_row + 1, current_col),goal)
        parent[up_index] = inspected_node_index
      }
    }
    
    # Mark the current node as visited after checking all its neighbors
    visited_nodes[inspected_node_index] = TRUE
    cat("Visited nodes:", visited_nodes, "\n")
    cat("Currently lowest distances:", distances, "\n")
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
######################################


#################################################################################################################
# testing
#testDM(myFunction = walterDM, verbose=2,returnVec=TRUE,n=1,seed=21,timeLimit=250)
