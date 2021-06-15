# -*- coding: utf-8 -*-
"""
Created on Sun Jul 19 13:47:55 2020

@author: USER
"""
import math
import random
import visualize_tsp
import matplotlib.pyplot as plt

class tsp(object):
    def __init__(self, coor, temp=-1, alpha=-1, stop_temp=-1, stop_iter=-1):
        self.coor = coor
        self.N = len(coor)
        self.temp = math.sqrt(self.N) if temp == -1 else temp
        # self.temp = 20 if temp == -1 else temp
        self.stop_temp = 1e-8 if stop_temp == -1 else stop_temp
        self.temp_store = self.temp  # save inital T to reset if batch annealing is used
        self.alpha = 0.998 if alpha == -1 else alpha
        self.stop_iter = 100000 if stop_iter == -1 else stop_iter
        self.iter = 1

        self.nodes = [i for i in range(self.N)]

        self.best_route = None
        self.best_dist = float("Inf")
        self.dist_list = []

    def greedy_tsp(self):

        cur_node = random.choice(self.nodes)  # start from a random node
        route = [cur_node]

        free_nodes = set(self.nodes)
        free_nodes.remove(cur_node)
        while free_nodes:
            next_node = min(free_nodes, key=lambda x: self.euc_dist(cur_node, x))  # nearest neighbour
            # next_node = min(free_nodes, key=lambda x: self.distances[cur_node][x])  # nearest neighbour
            free_nodes.remove(next_node)
            route.append(next_node)
            cur_node = next_node

        dist = self.total_dist(route)
        if dist < self.best_dist:  # If best found so far, update best fitness
            self.best_dist = dist
            self.best_route = route
        self.dist_list.append(dist)
        
        return route, dist

    def euc_dist(self, node_0, node_1):

        coord_0, coord_1 = self.coor[node_0], self.coor[node_1]

        return math.hypot((coord_0[1] - coord_1[1]),(coord_0[0] - coord_1[0]))


    def total_dist(self, route):

        cur_dist = 0
        for i in range(self.N):
            cur_dist += self.euc_dist(route[i % self.N], route[(i + 1) % self.N])
            # cur_fit += self.distances[solution[i % self.N]][solution[(i + 1) % self.N]]
        return cur_dist

    def prob(self, cand_dist):

        return math.exp(-abs(cand_dist - self.cur_dist) / self.temp)

    def accept(self, cand):

        cand_dist = self.total_dist(cand)
        if cand_dist < self.cur_dist:
            self.cur_dist, self.cur_route = cand_dist, cand
            if cand_dist < self.best_dist:
                self.best_dist, self.best_route = cand_dist, cand
        else:
            if random.random() < self.prob(cand_dist):
                self.cur_dist, self.cur_route = cand_dist, cand
                
    # def three_opt(self, broad=False):
    #     """In the broad sense, 3-opt means choosing any three edges ab, cd
    #     and ef and chopping them, and then reconnecting (such that the
    #     result is still a complete tour). There are eight ways of doing
    #     it. One is the identity, 3 are 2-opt moves (because either ab, cd,
    #     or ef is reconnected), and 4 are 3-opt moves (in the narrower
    #     sense)."""
    #     p = self.cur_route
    #     n = len(p)
    #     # choose 3 unique edges defined by their first node
    #     a, c, e = random.sample(range(n+1), 3)
    #     # without loss of generality, sort
    #     a, c, e = sorted([a, c, e])
    #     b, d, f = a+1, c+1, e+1
    
    #     if broad == True:
    #         which = random.randint(0, 7) # allow any of the 8
    #     else:
    #         which = random.choice([3, 4, 5, 6]) # allow only strict 3-opt
    
    #     # in the following slices, the nodes abcdef are referred to by
    #     # name. x:y:-1 means step backwards. anything like c+1 or d-1
    #     # refers to c or d, but to include the item itself, we use the +1
    #     # or -1 in the slice
    #     if which == 0:
    #         sol = p[:a+1] + p[b:c+1]    + p[d:e+1]    + p[f:] # identity
    #     elif which == 1:
    #         sol = p[:a+1] + p[b:c+1]    + p[e:d-1:-1] + p[f:] # 2-opt
    #     elif which == 2:
    #         sol = p[:a+1] + p[c:b-1:-1] + p[d:e+1]    + p[f:] # 2-opt
    #     elif which == 3:
    #         sol = p[:a+1] + p[c:b-1:-1] + p[e:d-1:-1] + p[f:] # 3-opt
    #     elif which == 4:
    #         sol = p[:a+1] + p[d:e+1]    + p[b:c+1]    + p[f:] # 3-opt
    #     elif which == 5:
    #         sol = p[:a+1] + p[d:e+1]    + p[c:b-1:-1] + p[f:] # 3-opt
    #     elif which == 6:
    #         sol = p[:a+1] + p[e:d-1:-1] + p[b:c+1]    + p[f:] # 3-opt
    #     elif which == 7:
    #         sol = p[:a+1] + p[e:d-1:-1] + p[c:b-1:-1] + p[f:] # 2-opt
    
    #     return sol

    def anneal(self):

        # Initialize with the greedy solution.
        self.cur_route, self.cur_dist = self.greedy_tsp()

        while self.temp >= self.stop_temp and self.iter < self.stop_iter:
            cand = list(self.cur_route)
            l = random.randint(2, self.N - 1)
            i = random.randint(0, self.N - l)
            cand[i : (i + l)] = reversed(cand[i : (i + l)])
            # cand = self.three_opt()
            self.accept(cand)
            self.temp *= self.alpha
            self.iter += 1

            self.dist_list.append(self.cur_dist)
        
        print("Best fitness obtained: ", self.best_dist)
        improvement = 100 * (self.dist_list[0] - self.best_dist) / (self.dist_list[0])
        print(f"Improvement over greedy heuristic: {improvement : .2f}%")

    # def anneal2(self):

    #     while self.temp >= self.stop_temp and self.iter < self.stop_iter:
    #         # cand = list(self.cur_route)
    #         # l = random.randint(2, self.N - 1)
    #         # i = random.randint(0, self.N - l)
    #         # cand[i : (i + l)] = reversed(cand[i : (i + l)])
    #         cand = self.three_opt()
    #         self.accept(cand)
    #         self.temp *= self.alpha
    #         self.iter += 1

    #         self.dist_list.append(self.cur_dist)
        
    #     print("Best fitness obtained: ", self.best_dist)
    #     improvement = 100 * (self.dist_list[0] - self.best_dist) / (self.dist_list[0])
    #     print(f"Improvement over greedy heuristic: {improvement : .2f}%")

    def batch_anneal(self, times=10):
 
        for i in range(1, times + 1):
            print(f"Iteration {i}/{times} -------------------------------")
            self.temp = self.temp_store
            self.iter = 1
            # self.cur_route, self.cur_dist = self.greedy_tsp()
            self.anneal()
            
    def visualize_routes(self):
        """
        Visualize the TSP route with matplotlib.
        """
        visualize_tsp.plotTSP([self.best_route], self.coor)

    def plot_learning(self):
        """
        Plot the fitness through iterations.
        """
        plt.plot([i for i in range(len(self.dist_list))], self.dist_list)
        plt.ylabel("Fitness")
        plt.xlabel("Iteration")
        plt.show()


