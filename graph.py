""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""

__author__ = "Xenia Ioannidou"

import numpy as np
import random, math, copy
import networkx as nx
import matplotlib.pyplot as plt
import networkx.drawing


class Graph():
    """
	Graph class for RRT*
	"""

    def __init__(self):
        self.G = nx.Graph()

    def create_weighed_graph(self, edges_dict):
        print("Start building network with edges: ", edges_dict)
        for key in edges_dict:
            # print("key = ", key, " with value = ", edges_dict[key])
            key_split = str(key).replace("(","").replace(")","").split(",")
            self.G.add_edge(key_split[0], key_split[1], weight=edges_dict[key])


    def plot_graph(self, G):
        pos = nx.spring_layout(G)  # positions for all nodes

        # nodes
        nx.draw_networkx_nodes(G, pos, node_size=300)

        # edges
        nx.draw_networkx_edges(G, pos, edgelist=G.edges(data=True),
                               width=2)
        # labels
        labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
        nx.draw_networkx_labels(G, pos, font_size=20, font_family='sans-serif')

        plt.axis('off')
        plt.show()


    def add_one_node(self, node):
        self.G.add_node(node)

    def add_node_list(self, my_list):
        self.G.add_nodes_from(my_list)

    def add_path_graph(self, num=10):
        """
        G contains the nodes of H as nodes of G ,
        so now G has only the num of nodes that H has

        :num: how many nodes has the new graph
        """
        H = nx.path_graph(num)
        self.G.add_nodes_from(H)

    def point_graph(self, num=10):
        """
        G points to a new graph, so has the #nodes as previously plus one

        :num: how many nodes has the new graph that G points to
        """

        H = nx.path_graph(num)
        self.G.add_node(H)

    def add_edge(self, start, end):
        e = (start, end)
        self.G.add_edge(*e)

    def add_edge_list(self,edgelist):
        for i in range(len(edgelist)):
            self.G.add_edge(*edgelist[i])

    def draw_graph(self):
        nx.draw(self.G, with_labels=True, font_weight='bold')
        plt.show()
