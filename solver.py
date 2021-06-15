#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
# from collections import namedtuple
from tsp_simanne import tsp
# import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(points):
    """Stores the data for the problem."""

    data = {}
    # Locations in block units
    data['locations'] = points # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    # coord_0, coord_1 = locations[node_0], locations[node_1]

    # return math.hypot((coord_0[1] - coord_1[1]),(coord_0[0] - coord_1[0]))
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (
                    math.hypot((from_node[0] - to_node[0]),
                                (from_node[1] - to_node[1])))
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Objective: {}m\n'.format(route_distance)


def main(points,time):
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(points)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]*1000

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = time
    search_parameters.log_search = True
    # search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # search_parameters.first_solution_strategy = (
    #     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    # if solution:
    #     print_solution(manager, routing, solution)
        
    return manager, routing, solution

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append([float(parts[0]), float(parts[1])])
    
    # sa = tsp(points)
    # sa.anneal()
    # sa.batch_anneal()
    if nodeCount != 33810:
        if nodeCount == 100:
            time = 120
        else:
            time = 60
        man, rou, sol = main(points,time)
        min_dist = sol.ObjectiveValue()/1000
        route_distance = 0
        plan_output = 'Route:\n'
        index = rou.Start(0)
        index1 = [index]
        while not rou.IsEnd(index):
            plan_output += ' {} ->'.format(man.IndexToNode(index))
            previous_index = index
            index = sol.Value(rou.NextVar(index))
            index1.append(index)
            route_distance += rou.GetArcCostForVehicle(previous_index, index, 0)
        man.IndexToNode(index)
        index1 = index1[0:nodeCount]
    else:
        sa = tsp(points)
        sa.anneal()
        min_dist = sa.best_dist
        index1 = sa.best_route
    
    
    # prepare the solution in the specified output format
    output_data = str(min_dist) + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, index1))

    return output_data


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')

