# from Queue import PriorityQueue
import copy 
import math

import heapq # for A* algorithm


class AStarPlanner:

  # Initialize any variables
  def __init__(self, safe_distance=1):
    # Save the safe_distance
    self.safe_distance = safe_distance
  
  # Find a plan through the map from drone_position to goal_position
  # Mapdata         : List of lists where each row in the occupancy grid is a list. map_data[0][0] will give you the first cell
  # Drone Position  : List with two integers. drone_position[0] will give you the x data
  # Goal Position   : List with two integers. goal_position[0] will give you the x data
  def plan(self, map_data, drone_position, goal_position):
    # Validate the data
    self.validate_data(map_data, drone_position, goal_position)

    # Expand the obstacles by the safety factor
    map_data = self.expand_obstacles(map_data, self.safe_distance)

    # Create both a frontier and a list to track which nodes are already processed
    # Init start node
    curr_pos = tuple(drone_position)
    # print(curr_pos, goal_position)

    # Objects in nodelists are [est_cost, position]
    came_from = {}
    cost_so_far = {}
    came_from[curr_pos] = None
    cost_so_far[curr_pos] = 0
    frontier = [(cost_so_far[curr_pos], curr_pos)]

    # heapq implements a minheap
    heapq.heapify(frontier)

    # While the frontier is not empty
    while frontier:
      # Get the node with the lowest cost
      current = heapq.heappop(frontier)

      # Check if the current node is the goal
      if curr_pos == tuple(goal_position):
        break

      # For each neighbor of the current node
      curr_pos = current[1]
      for next_pos in self.get_neighbors(curr_pos, map_data):
        next_pos = tuple(next_pos)
        # Compute the cost to traverse to that node
        new_cost = cost_so_far[curr_pos] + self.get_distance(curr_pos, next_pos)
        if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
          cost_so_far[next_pos] = new_cost

        # Compute the estimated cost to goal (heuristic)
          priority = new_cost + self.get_distance(goal_position, next_pos)

        # Add the neighbors to the frontier with the new cost
          heapq.heappush(frontier, (priority, next_pos))
          came_from[next_pos] = curr_pos


    # Return the trajectory
    return self.build_path(drone_position, goal_position, came_from)

  # Generate trajectory path
  def build_path(self, node_start, node_end, came_from):
    current = tuple(node_end)
    path = []

    while current != tuple(node_start):
      path.insert(0, list(current))
      # Check path exists
      if current not in came_from:
        return []
      current = came_from[current]
    path.insert(0, node_start)

    return path

  # Return the distance between two nodes
  # Assume node_start and node_end are lists with two integers (x,y)
  def get_distance(self, node_start, node_end):
    x1, y1 = node_start
    x2, y2 = node_end
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

  # Get the children nodes for the current node
  def get_neighbors(self, node_in, map_in):
    # A list of neighbors
    neighbors = []
    # Get the current position
    pos = node_in
    # For all adjacent values
    for x_dim in range(-1, 2):
      for y_dim in range(-1, 2):
        # If the values are not 0, 0 (which is itself)
        if not (x_dim == 0 and y_dim == 0):
          # If the values are inside the map
          if (0 <= pos[0] + x_dim < map_in.shape[0]) and(0 <= pos[1] + y_dim < map_in.shape[1]):
            # If that value is an open square on the map
            if map_in[pos[0] + x_dim][pos[1] + y_dim] == 0:
              # Create a neighbor with the new position
              n = [pos[0] + x_dim, pos[1] + y_dim]
              # Save the neighbors
              neighbors.append(n)
    return neighbors
    
  # Validate the incoming data
  def validate_data(self, map_data, drone_position, goal_position):
    # Confirm that the map has two dimensions
    assert(len(map_data.shape) == 2)
    # Confirm that the drone and goal position are two points
    assert(len(drone_position) == 2)
    assert(len(goal_position) == 2)
    # Confirm that all positions are integers
    for x in drone_position + goal_position:
      assert(type(x) == int)
    # Confirm the both the start and end goal lie inside the map
    assert(0 <= drone_position[0] < map_data.shape[0])
    assert(0 <= drone_position[1] < map_data.shape[1])
    assert(0 <= goal_position[0] < map_data.shape[0])
    assert(0 <= goal_position[1] < map_data.shape[1])
    # Confirm that the drone_position and the goal position are not the same
    assert(not (drone_position[0] == goal_position[0] and drone_position[1] == goal_position[1]))
    # Confirm that the goal and drone position are free space
    assert(map_data[drone_position[0], drone_position[1]] == 0)
    assert(map_data[goal_position[0], goal_position[1]] == 0)

  # Expand the obstacles by distance so you do not hit one
  def expand_obstacles(self, map_data, distance):
    new_map = copy.deepcopy(map_data)
    # For each element in the map
    for i in range(map_data.shape[0]):
      for j in range(map_data.shape[1]):
        # if this is an obstacle
        if map_data[i, j] != 0:
          # Expand the obstacle by 1 in all directions
          for x_dim in range(-1, 2):
            for y_dim in range(-1, 2):
              if (0 <= i + x_dim < new_map.shape[0]) and(0 <= j + y_dim < new_map.shape[1]):
                new_map[i + x_dim, j + y_dim] = map_data[i, j]
    return new_map


