from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#Distance callback
class CreateDistanceCallback(object):
  #Create callback to calculate distances between points.

  def __init__(self, locations):
    #Array of distances between points.
    self.locations = locations

  #The distance function for salaired drivers.
  def salary_distance(self, from_node, to_node):
    return int(self.locations[from_node][to_node][0]) #Converts all distances to integers,
    #because the routing solver only works with integers.

    #Could possibly calculate much faster using pandas library?

  #The distance function for special drivers.
  def special_distance(self, from_node, to_node):
    return int(self.locations[from_node][to_node][1])

#Demand callback
class CreateDemandCallback(object):
  #Create callback to get demands at each location.

  def __init__(self, demands):
    self.demands = demands

  def demand(self, from_node, to_node):
    return self.demands[from_node]

def main():

  data = create_data_array()
  locations = data[0]
  demands = data[1]
  num_locations = len(locations)
  num_vehicles = 2 #Number of vehicles specified here.
  #Nodes are indexed from 0 to tsp_size - 1. The depot is the starting node of the route.
  depot = 0

  #Create routing model
  if num_locations > 0:
    routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
    #Define search parameters for how the route will be found.
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters() #Can modify search strategy here.

    #Create the distance callback, which takes two arguments (the from and to node indices)
    #and returns the distance between these nodes.
    dist_between_nodes = CreateDistanceCallback(locations)
    salary_dist_callback = dist_between_nodes.salary_distance
    special_dist_callback = dist_between_nodes.special_distance
    routing.SetArcCostEvaluatorOfVehicle(salary_dist_callback, 0) #Distance callback for vehicle 0 passed to solver.
    routing.SetArcCostEvaluatorOfVehicle(special_dist_callback, 1) #Distance callback for vehicle 1 passed to solver.

    #Create the demand callback
    demands_at_nodes = CreateDemandCallback(demands)
    demands_callback = demands_at_nodes.demand

    #Add Dimension for demand to routing model.
    slack_max = 0
    vehicle_capacity = 100
    fix_start_cumul_to_zero = True
    demand = "Demand"
    routing.AddDimension(demands_callback, slack_max, vehicle_capacity, fix_start_cumul_to_zero, demand)

    #Solve, returns a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment: #If a solution was found.
      #Solution cost.
      print("Total distance: ", str(assignment.ObjectiveValue()), " units\n")
      
      for vehicle_nbr in range(num_vehicles):
        index = routing.Start(vehicle_nbr) #Sets index to be the index of the starting node of the route of the vehicle given by vehicle_nbr.
        index_next = assignment.Value(routing.NextVar(index))
        route = ''
        route_length = 0
        route_demand = 0
        while not routing.IsEnd(index_next): #While there are still nodes left on this path.
          #Convert variable indices to node indices in the displayed route.
          node_index = routing.IndexToNode(index)
          node_index_next = routing.IndexToNode(index_next)
          route += str(node_index) + " -> "
          #Add route distance.
          route_length += locations[node_index][node_index_next][vehicle_nbr]
          #Add route demand.
          route_demand += demands[node_index_next]
          #Advance to the next node.
          index = index_next 
          index_next = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        node_index_next = routing.IndexToNode(index_next)
        route += str(node_index) + " -> " + str(node_index_next)

        route_length += locations[node_index][node_index_next][vehicle_nbr]
        print("Vehicle number: ", vehicle_nbr)
        print("Vehicle demand: ", route_demand)
        print("Distance travelled: ", route_length)
        print("Route:\n\n", route)
        

    else:
      print("No solution found.")

  else:
    print("Specify an instance greater than 0.")

def create_data_array():
  locations = [ #A 3D Array representing a graph of nodes A to D, with various path costs for vehicles 0 or 1.
    [[0,0],   [5,10], [3,6],  [10,2]],
    [[5,10],  [0,0],  [2,4],  [7,14]],
    [[3,6],   [2,4],  [0,0],  [1,2]],
    [[10,2], [7,14], [1,2],  [0,0]]
  ]

  demands = [0, 100, 20, 60]
    #self.demands = [100, 100, 100, 100, 100]
    #self.demands = [50, 50, 50, 50, 50]
    #self.demands = [20, 20, 20, 20, 20]
    #self.demands = [101, 1, 1, 1, 1]

  data = [locations, demands]
  return data

if __name__ == '__main__':
  main()
