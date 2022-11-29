# OR-tools is a module from Google, which includes a solver for the vehicle routing problem
#https://developers.google.com/optimization/introduction/python
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model(times, demands):
    """Stores the data for the problem."""
    data = {}
    data["time_matrix"] = times.tolist()
    data["num_vehicles"] = 500
    data["vehicle_capacities"] = [25 for i in range(data["num_vehicles"])]
    data["depot"] = 0
    data["demands"] = demands["DEMAND"].fillna(0).values.tolist()
    # data["time_windows"] = [(0, 600) for i in range(demands.shape[0])]
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    total_load = 0
    total_routes = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            time_var = time_dimension.CumulVar(index)
            plan_output += "{0} Time({1},{2}) -> ".format(
                node_index, solution.Min(time_var),
                solution.Max(time_var))

            index = solution.Value(routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += "{0} Time({1},{2})\n".format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var))
        route_time = solution.Min(time_var)
        plan_output += "Time of the route: {}min\n".format(
            route_time)
        plan_output += "Load of the route: {}\n".format(route_load)
        if route_load > 0:
            print(plan_output)
        total_time += route_time
        total_load += route_load
        if route_load>0:
            total_routes += 1
    print("Total time of all routes: {}m".format(total_time))
    print("Total load of all routes: {}".format(total_load))
    print("Total number of routes: {}".format(total_routes))


def get_solution(times, demands):
        """Entry point of the program."""
        # Instantiate the data problem.
        data = create_data_model(times, demands)
    
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]),
                                               data["num_vehicles"], data["depot"])
    
        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)
    
    
        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to time matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["time_matrix"][from_node][to_node]
    
        transit_callback_index = routing.RegisterTransitCallback(time_callback)
        
        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return data["demands"][from_node]
    
        demand_callback_index = routing.RegisterUnaryTransitCallback(
            demand_callback)
    
    
        dimension_name = "Capacity"
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            data["vehicle_capacities"],  # vehicle maximum capacities
            True,  # start cumul to zero
            dimension_name)
    
        # Add time constraint.
        dimension_name = "Time"
        routing.AddDimension(
            transit_callback_index,
            0,  # maximum wait time at store
            540,  # vehicle maximum time
            False,  # start cumul to zero
            dimension_name)
        #distance_dimension = routing.GetDimensionOrDie(dimension_name)
        #distance_dimension.SetGlobalSpanCostCoefficient(200000)
        time_dimension = routing.GetDimensionOrDie("Time")
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(data["time_windows"]):
            if location_idx == data["depot"]:
                continue
            index = manager.NodeToIndex(location_idx)
            #print(time_window[0])
            #print(time_window[1])
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        depot_idx = data["depot"]
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                data["time_windows"][depot_idx][0],
                data["time_windows"][depot_idx][1])
    
        # Instantiate route start and end times to produce feasible times.
        for i in range(data["num_vehicles"]):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i)))
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i)))
    
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT)
        search_parameters.use_full_propagation = False
        search_parameters.log_search = True
        search_parameters.solution_limit = 80
        search_parameters.lns_time_limit.seconds = 5
    
        # Solve the problem.
        print("Solving...")
        solution = routing.SolveWithParameters(search_parameters)
        print(routing.status())
    
        # Print solution on console.
        # if solution:
        #     print_solution(data, manager, routing, solution)
        # else:
        #     print("No solution found!")
        
        return data, manager, routing, solution


#### #### #### ####

# Example code for iterating through the solution data to get journey times

import numpy as np

route_times = np.array([])
unloading_times = np.array([])
time_dimension = routing.GetDimensionOrDie("Time")
for vehicle_id in range(data["num_vehicles"]):
    cumul_unloading_time = 0
    index = routing.Start(vehicle_id)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        if node_index > 0:
            cumul_unloading_time += unloading_time(data, node_index, vehicle_id)
        index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
    time_var = time_dimension.CumulVar(index)
    route_time = solution.Min(time_var)
    route_times = np.append(route_times, [route_time])
    unloading_times = np.append(unloading_times, [cumul_unloading_time])