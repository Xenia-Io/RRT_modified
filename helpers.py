""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""
import math

__author__ = "Xenia Ioannidou"


def writing_to_file(nodes, costs):
    with open('rrt_st.txt', 'a') as fl:
        w = ''
        print("Start writing in file. Length of costs: ", len(costs))
        for i in range(len(costs)):
            w += str(nodes[i]) + '\t' + str(costs[i]) + '\n'
        fl.write(w)


def find_eucl_norm(x, y, point):
    dx = x - point.x
    dy = y - point.y
    res = dx*dx + dy*dy
    eucl_norm = math.sqrt(res)

    return eucl_norm


def get_path_cost(node, path):
    # if the given node is the initial just return 0
    if node.parent is None:
        return 0
    else:
        # Start from the given node
        cost = 0
        last_indx = path.index(node)
        ## check if its parent is not None == the current node is not initial node
        while path[last_indx].parent is not None:
            current_node = path[last_indx]
            current_parent = path[path[last_indx].parent]
            dx = current_node.x - current_parent.x
            dy = current_node.y - current_parent.y
            cost += math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            ## The new node is the parent of the current node
            last_indx = current_node.parent
        return cost