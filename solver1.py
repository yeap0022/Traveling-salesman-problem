#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from tsp_simanne import tsp

# Point = namedtuple("Point", ['x', 'y'])

# def length(point1, point2):
#     return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


file = open("./data/tsp_33810_1.txt")
input_data = file.read()
# print(input_data)
file.close()
# parse the input
lines = input_data.split('\n')

nodeCount = int(lines[0])

points = []
for i in range(1, nodeCount+1):
    line = lines[i]
    parts = line.split()
    points.append([i, float(parts[0]), float(parts[1])])

del lines

sa = tsp(points)
route, dist = sa.greedy_tsp()
# sa.anneal()
# sa.batch_anneal()
# sa.visualize_routes()


# prepare the solution in the specified output format
output_data = str(sa.best_dist) + ' ' + str(0) + '\n'
output_data += ' '.join(map(str, sa.best_route))



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

