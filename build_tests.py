""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""
from rrt_star_builder import RRT_star

__author__ = "Xenia Ioannidou"

import numpy as np
from graph import *
from node import *
import sys

class Tests():

    def read_file(self, fname):
        content_array = []
        with open(fname) as f:
            # Content_list is the list that contains the read lines
            for line in f:
                content_array.append(line)
        return content_array


    def test_rrt_star(self, fname):
        iters = 2500
        neigh_rad = 2
        step_size = []
        obstacles = []
        dict = {}
        world, init, goal = [], [], []
        rate = 5
        content_array = self.read_file(fname)
        # print("content array: ", content_array)

        for line in range(len(content_array)):
            array = content_array[line].split("-")

            if array[0] == 'w':
                world = [int(a) for a in array[1:]]

            if array[0] == 'i':
                start_node_number = int(array[1])
                init = Node(start_node_number)

            if array[0] == 'g':
                goal_node_name = int(array[1])
                goal = Node(goal_node_name)

            if array[0] == 'e':
                for i in array[1:]:
                    list = (i.split(":"))
                    dict.update({list[0]: list[1]})

            if array[0] == 'step':
                step_size = [int(a) for a in array[1:]]
            if array[0] == 'iter':
                iters = int(array[1])
            if array[0] == 'nr':
                neigh_rad = float(array[1])
            if array[0] == 'gl':
                rate = int(array[1])
            if array[0] == 'o':
                obstacles = [int(a) for a in array[1:]]

        print("Start RRT* path planning")

        network = Graph()
        network.create_weighed_graph(dict)
        # network.plot_graph(network.G)

        for step_i in step_size:

            rrt_star = RRT_star(world, dict, init, goal, obstacles, iters, neigh_rad, step_i, rate)
            path, cost = rrt_star.planning(network)
        #     path, cost = rrt_star.planning()
            print("End of path planning with cost: ", cost)
            for i in range(len(path)):
                print((path[i].node_number), " - ", path[i].parent)


    def create_graph_network(self):
        graph = Graph()
        graph.add_one_node(1)
        print("Graph has #nodes: ", graph.G.number_of_nodes())
        graph.add_node_list([1,2,3,4,5,6,7,8,9])
        print("Graph has #nodes: ", graph.G.number_of_nodes())
        # graph.add_edge(1,2)
        edgelist = [(1, 2),(1,3),(2,4),(2,5),(4,8),(5,8),(3,6),(3,7),(6,9),(7,9)]
        graph.add_edge_list(edgelist)
        # graph.add_path_graph()
        # graph.draw_graph()
        # print("Graph has #nodes: ", graph.G.number_of_nodes())
        # graph.point_graph()
        # print("Graph has #nodes: ", graph.G.number_of_nodes())
        graph.draw_graph()


