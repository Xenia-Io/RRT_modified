""" Implementation of RRT* path finding algorithm

    For the Degree Project at KTH Royal Institute
    of Technology - May 2020
"""
from helpers import writing_to_file
from node import Node

__author__ = "Xenia Ioannidou"

import random, math, copy
from graph import *


class Answer_Info():
    """
	Class that holds the information that is needed
	so as to perform graph search
	"""

    def __init__(self, parent, weight_so_far, depth):
        self.parent = parent
        self.weight_so_far = weight_so_far
        self.depth = depth


class RRT_star():

    def __init__(self, world, edges, start_point, target_point, obstacles, max_iter, radius, exp_dist, goal_rate):
        self.world = world
        self.edges = edges
        self.start_point = start_point
        self.target_point = target_point
        self.goal_rate = goal_rate
        self.max_iter = max_iter
        self.neighborhood_radius = radius
        self.expanding_distance = exp_dist
        self.obstacles = obstacles
        self.answer_map = {}
        self.nodes_info = {}

        print("Building RRT* is done.")
        print("World is a list of machines: ", self.world)
        print("With edges: ", self.edges, " and obstacles: ", self.obstacles)


    def planning(self, network):
        """
        RRT* path planning

        :return: a node list, which is the path for the goal
        """
        goal_found = False
        goal_path_cost = 1e11
        nodes = []
        costs = []
        self.start_point.parent = -1
        self.final_path = [self.start_point]
        self.path_edges = {}
        self.path_graph = Graph()
        str_final_path = ''
        # Create first edge and then start looping
        self.create_first_edge(network)

        # Run until our path has as many nodes as the number of max_iter
        iter = 0
        for i in range(self.max_iter):
            iter += 1
            # Get random node
            z_rand = self.get_biased_randpoint()

            # Finding the nearest node from the path to the goal given the random point
            print("z_rand is: ", z_rand)
            selected_node, selected_max_weight, path = self.get_nnode_from_path(network, z_rand)

            # Apply steer function
            new_node = self.steering(selected_node, path, z_rand)
            print("STEERING resulted new node: ", new_node.node_number)

            # Check if the new node collide with an obstacle
            if self.collision_found(new_node):
                # If they collide just go and find a new random node until it is valid
                print("Collision detected, so retry ..")
                continue

            # Append the node to the list if no collision was detected
            # and if it has a direct edge with the last node in the final path
            if str(new_node.node_number) in network.G.neighbors(str(self.final_path[-1].node_number)):
                print("New node is connected directly with the last node in the final path. ")
                self.final_path.append(new_node)
            else:
                print("New node is not connected directly with the last node in the final path.")
                continue

            print("debug 111 new_node_: ", new_node.node_number)
            # Calculate if the new node is equal to the goal point
            if new_node.node_number == self.target_point.node_number:
                print("GOAL FOUND !!!")
                for i in range(len(self.final_path)):
                    str_final_path += str(self.final_path[i].node_number) + ' '

                goal_found = True
                # Compute the path cost
                temp_goal_path_cost = self.get_path_cost(new_node, self.final_path, network)

                if temp_goal_path_cost <= goal_path_cost:
                    goal_path_cost = temp_goal_path_cost
                    if not temp_goal_path_cost in costs:
                        costs.append(temp_goal_path_cost)
                        nodes.append(len(self.final_path))

                print("Goal found with cost: ", goal_path_cost)
                break
                # continue

        print("Final_path is: ", str_final_path, " and it was found after ", iter, " iterations")
        print("Planning is over, len of costs: ", len(costs), " and len of nodes: ", len(nodes))
        if not goal_found:
            return [], goal_path_cost

        return self.final_path, goal_path_cost


    def create_first_edge(self, network):
        first_weight = 0
        second_node = None
        self.path_graph.G.add_node(str(self.start_point.node_number))
        list_neighbors = network.G.neighbors(str(self.start_point.node_number))

        for i in list_neighbors:
            weight_i = int(network.G.edges[(str(self.start_point.node_number), i)]['weight'])
            if weight_i > first_weight:
                first_weight = weight_i
                second_node = Node(i)

        self.path_graph.G.add_node(str(second_node.node_number))
        self.path_graph.G.add_edge(str(self.start_point.node_number), str(second_node.node_number), weight=first_weight)
        second_node.parent = int(self.start_point.node_number)
        self.final_path.append(second_node)
        # self.path_graph.plot_graph(self.path_graph.G)


    def get_path_cost(self, selected_node, path, network):

        path_list = []
        for i in range(len(path)):
            path_list.append(path[i].node_number)

        if selected_node.parent == -1:
            return 0
        else:
            last_indx = len(path_list) - 1
            cost = 0

            for i in range(len(path_list)-1):
                current_node = path_list[last_indx]
                current_parent = path_list[last_indx-1]
                cost += int(network.G.edges[(str(current_node), str(current_parent))]['weight'])
                last_indx = last_indx - 1

            return cost



    def get_biased_randpoint(self):
        z_rand = 0
        temp_random = random.randint(0, 100)
        if temp_random > self.goal_rate or self.goal_rate == 0:
            z_rand = random.randint(int(self.world[0]), int(self.world[-1]))
            print(z_rand)
            for i in range(len(self.final_path)):
                if z_rand == int(self.final_path[i].node_number):
                    print("DEBUGGGG !!!!!!!!!!!!!........")
                    return self.get_biased_randpoint()
            print("z_rand ISSSS 0_: ", z_rand)
            return z_rand
        else:
            print("In else : ", type(self.target_point.node_number), self.target_point.node_number)
            z_rand = int(self.target_point.node_number)
            print("now z_rand ISSSS 00: ", z_rand)
            return z_rand



    def get_nearest_neighbor(self, network, z_rand):
        """
        Find the nearest neighbor that is the closest to the random point.
        The closest node here is the node with the highest weight,
        because the higher the weight the faster a 'package' travels
        from one node to another. So weights represent speed and
        'closeness' is getting increased when the speed is getting higher.

        :param network: the whole network topology
        :param z_rand: is an object of <node.Node.node_number>
        :return: the nearest index and its distance
        """

        selected_weight = 0
        nearest_node = Node(0)
        dlist = network.G.edges([str(z_rand)])
        print("z_rand: ", z_rand, " with edges: ", dlist)

        # Get the highest weight
        for i in dlist:
            if (int(network.G.get_edge_data(*i)['weight'])) > selected_weight:
                selected_weight = (int(network.G.get_edge_data(*i)['weight']))
                # print(i, (int(graph.G.get_edge_data(*i)['weight'])))
                node_1 , node_2 = i
                nearest_node = Node(node_2)

        print("Results: ", selected_weight, nearest_node.node_number)
        return nearest_node, selected_weight


    def collision_found(self, new_node):
        print("Start COLLISION CHECK..")
        print(type(new_node),new_node.node_number, type(self.obstacles[0]))
        if new_node.node_number in self.obstacles:
            return True
        return False


    def apply_BFS_modified(self, source_node, network, targets):

        print("*********** START apply_BFS_modified *********** ", targets)
        # Clean the answer map
        if len(self.answer_map) != 0:
            self.answer_map.clear()

        source_node = int(source_node)
        bfs = []
        bfs.append(source_node)
        self.answer_map[source_node] = Answer_Info(-1, 0, 0)
        maxDepth = 1000
        self.answers_found = []
        self.answers_found_set = set(self.answers_found)
        self.unique_list_answers_found = (list(self.answers_found_set))

        while bfs:
            answer_info = self.answer_map[bfs[-1]]
            this_node = bfs[-1]
            print("start while with bfs: ", bfs, " and this_node: ", this_node)
            bfs.pop()
            print("now bfs is: ", bfs)

            if answer_info.depth >= maxDepth:
                print(answer_info.depth, " , maxDepth: ", maxDepth)
                print("END OF BFD SEARCH ...", self.unique_list_answers_found)
                return

            # Get this_node's neighbors and check the edges
            neighbors_list = network.G.edges([str(this_node)])

            for edge_i in neighbors_list:
                node_from, node_to = edge_i
                current_weight = int(network.G.get_edge_data(*edge_i)['weight'])

                if str(node_to) in targets:
                    self.unique_list_answers_found.append(int(node_to))
                    maxDepth = answer_info.depth + 1

                if int(node_to) in self.answer_map.keys():
                    item = self.answer_map[int(node_to)]

                    if (answer_info.depth + 1) < item.depth or \
                            ((answer_info.depth + 1) == item.depth and \
                            (answer_info.weight_so_far + current_weight) > item.weight_so_far):
                        item.depth = answer_info.depth + 1
                        item.weight_so_far = answer_info.weight_so_far + current_weight
                else:
                    bfs.append(int(node_to))
                    self.answer_map[int(node_to)] = Answer_Info(int(this_node), \
                                                                answer_info.weight_so_far + current_weight, \
                                                                answer_info.depth + 1)


    def get_nnode_from_path(self, network, this_node):
        """
        Find the nearest node from the path to the goal given the current
        node. The nearest node is the node from the path that has the highest
        cost for the minimum depth, which means that a 'package' travels very
        fast to get there from this node.

        :param network: the whole network topology
        :param this_node: the current node

        :return: the nearest node alongside with the cost for the path
        """
        print("*** Get nearest node from the path ***")
        targets = []

        # Find all candidates nearest nodes from the final path
        for path_node_i in self.final_path:
            targets.append(path_node_i.node_number)
            # print("PATH node i: ", path_node_i.node_number)
            # print("this node: ", this_node)

        # Choose nearest node from the path
        self.apply_BFS_modified(this_node, network, targets)
        selected_node, selected_max_weight = self.find_correct_answer()
        path = self.get_path(selected_node, this_node)
        print("Selected path: ", path)

        # Convert path from string to list of integers
        path_as_list = path.split(",")
        for i in range(len(path_as_list)):
            path_as_list[i] = int(path_as_list[i])

        return selected_node, selected_max_weight, path_as_list


    def find_correct_answer(self):
        selected_node = 0
        selected_max_weight = 0
        for i in self.unique_list_answers_found:
            print("i: ", i)
            ai = self.answer_map[i]
            print("ai: ", ai.weight_so_far)
            if ai.weight_so_far > selected_max_weight:
                selected_node = i
                selected_max_weight = ai.weight_so_far

        print("correct answer: ", selected_node, selected_max_weight)
        return selected_node, selected_max_weight


    def get_path(self, selected_node, this_node):

        if selected_node == 0:
            return "No path found"
        else:
            curr = selected_node
            answer = str(curr)

            while True:
                curr = self.answer_map[curr].parent
                if curr == this_node:
                    answer = str(curr) + "," + answer
                    break
                answer = str(curr) + "," + answer

            return answer


    def steering(self, selected_node, path, z_rand):
        """
        This function drives the system from z_rand close to the nearest node found

        :param selected_node: The node of the final path that is the nearest
        :param z_rand: The node that we want to steer to the nearest node
        :param path: The path that has to be followed to get to the nearest node

        :return: The new node that is closer to the direction of the nearest node
        """

        if len(path) == 2:
            # No need to update the position of z_rand in the graph
            new_node = Node(int(z_rand))
            new_node.parent = int(selected_node)
            print("steering's new node 1: ", new_node.node_number)
            return new_node
        else:
            new_node = None
            final_path_list = []
            for i in range(len(self.final_path)):
                final_path_list.append(int(self.final_path[i].node_number))

            for j in range(len(path)):
                print("xenia 1: ", int(path[len(path) - 1 - j]))
                if int(path[len(path) - 1 - j]) in final_path_list:
                    continue
                else:
                    print("j: ", j , " and len of path: ", len(path))
                    new_node_num = path[len(path) - 1 - j]
                    new_node = Node(new_node_num)
                    new_node.parent = path[len(path) - j]
                    return new_node

            if new_node == None:
                # No need to update the position of z_rand in the graph
                new_node = Node(int(z_rand))
                new_node.parent = int(selected_node)
                print("steering's new node 1: ", new_node.node_number)
                return new_node


    def validate_edges(self, xenia):
        run_again = False

        # Select the nearest node from the final path
        for key, value in xenia.items():
            # Create a new list
            new_value = copy.deepcopy(value)
            print("my value: ", (value), " copied list: ", new_value, " with key: ", key)
            # Validate edges and remove edges that do not exist
            for val_i in range(len(value) - 1):
                edge_str_1 = '(' + str(value[val_i]) + ',' + str(value[val_i + 1]) + ')'
                edge_str_2 = '(' + str(value[val_i + 1]) + ',' + str(value[val_i]) + ')'

                print(edge_str_1)
                if edge_str_1 in self.edges or edge_str_2 in self.edges:
                    print("this edge exists: ", edge_str_2, " OR ", edge_str_1)
                else:
                    print("DELETING: ", new_value[val_i+1])
                    del new_value[val_i+1]
                    # self.nnodes_candidates[key] = new_value
                    xenia[key] = new_value
                    print("new xenia: ", xenia[key])
                    run_again = True
                    break

            # Assign the cleaned list in the dictionary
            # self.nnodes_candidates[key] = new_value

        return run_again


