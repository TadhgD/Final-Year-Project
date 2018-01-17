from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#Distance callback
class CreateDistanceCallback(object):
  #Create callback to calculate distances between points.


  def __init__(self):
    #Array of distances between points.
    #Example 1
    self.matrix = [ #A 3D Array representing a graph of nodes A to D, with various path costs for vehicles 0 or 1.
    [[0,0],   [5,10], [3,6],  [10,2]],
    [[5,10],  [0,0],  [2,4],  [7,14]],
    [[3,6],   [2,4],  [0,0],  [1,2]],
    [[10,2], [7,14], [1,2],  [0,0]]
    ]
    #Example 2
    """self.matrix = [ #A 3D Array representing a graph of nodes A to D, with various path costs for vehicles 0 or 1.
    [[0,0],   [10,5], [6,3],  [20,10]],
    [[10,5],  [0,0],  [4,2],  [14,7]],
    [[6,3],   [4,2],  [0,0],  [2,1]],
    [[20,10], [14,7], [2,1],  [0,0]]
    ]"""
    #Example 3
    """self.matrix = [ #A 3D Array representing a graph of nodes A to D, with various path costs for vehicles 0 or 1.
    [[0,0],   [5,10], [5,1],  [10,2]],
    [[5,10],  [0,0],  [2,4],  [7,14]],
    [[5,1],   [2,4],  [0,0],  [5,1]],
    [[10,2], [7,14], [5,1],  [0,0]]
    ]"""



  #The distance function for salaired drivers.
  def salary_distance(self, from_node, to_node):
    return int(self.matrix[from_node][to_node][0]) #Converts all distances to integers,
    #because the routing solver only works with integers.

    #Could possibly calculate much faster using pandas library?

  #The distance function for special drivers.
  def special_distance(self, from_node, to_node):
    return int(self.matrix[from_node][to_node][1])

def main():

  #Locations
  loc_names = ["A","B","C","D"]
  tsp_size = len(loc_names)
  num_vehicles = 2 #Number of vehicles specified here.
  #Nodes are indexed from 0 to tsp_size - 1. The depot is the starting node of the route.
  depot = 0

  #Create routing model
  if tsp_size > 0:
    routing = pywrapcp.RoutingModel(tsp_size, num_vehicles, depot)
    #Define search parameters for how the route will be found.
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters() #Can modify search strategy here.

    #Create the distance callback, which takes two arguments (the from and to node indices)
    #and returns the distance between these nodes.
    dist_between_nodes = CreateDistanceCallback()
    salary_dist_callback = dist_between_nodes.salary_distance
    special_dist_callback = dist_between_nodes.special_distance
    routing.SetArcCostEvaluatorOfVehicle(salary_dist_callback, 0) #Distance callback for vehicle 0 passed to solver.
    routing.SetArcCostEvaluatorOfVehicle(special_dist_callback, 1) #Distance callback for vehicle 1 passed to solver.

    #Solve, returns a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment: #If a solution was found.
      #Solution cost.
      print("Total distance: ", str(assignment.ObjectiveValue()), " units\n")
      
      for vehicle_nbr in range(num_vehicles):
        index = routing.Start(vehicle_nbr) #Sets index to be the index of the starting node of the route of the vehicle given by vehicle_nbr.
        route = ''
        while not routing.IsEnd(index): #While there are still nodes left on this path.
          #Convert variable indices to node indices in the displayed route.
          route += str(loc_names[routing.IndexToNode(index)]) + ' -> '
          index = assignment.Value(routing.NextVar(index)) #Sets index to be the next node on the path
        route += str(loc_names[routing.IndexToNode(index)]) #Adds the final node to the route.
        print("Vehicle number: ", vehicle_nbr)
        print("Route:\n\n", route)
    else:
      print("No solution found.")
  else:
    print("Specify an instance greater than 0.")

if __name__ == '__main__':
  main()