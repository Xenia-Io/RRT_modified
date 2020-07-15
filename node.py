""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""

__author__ = "Xenia Ioannidou"


class Node:
    """
    RRT* Node
    """
    def __init__(self, node_number):
        self.node_number = node_number
        self.cost = 0.0
        self.visited = False
        self.parent = None
        self.edges_of_node = None

    # def __init__(self, x, y):
    #     self.x = x
    #     self.y = y
    #     self.cost = 0.0
    #     self.parent = None