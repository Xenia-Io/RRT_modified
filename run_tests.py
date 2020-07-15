""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""

__author__ = "Xenia Ioannidou"


import numpy as np
from build_tests import *


def main():

    tester = Tests()
    # tester.create_graph_network()
    tester.test_rrt_star("input.txt")

if __name__ == "__main__":
    main()