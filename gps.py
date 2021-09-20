#!/usr/bin/env python3
'''
CPSC 415 -- Homework #2
Jeremy Buechler, University of Mary Washington, fall 2021
'''

from atlas import Atlas
import numpy as np
import logging
import sys

class Node:
    def __init__(self, n, p, c, tc):
        self.node_num = n
        self.parent = p
        self.cost = c
        self.total_cost = tc

    def get_node_num(self):
        return self.node_num

    def get_parent(self):
        return self.parent

    def get_cost(self):
        return self.cost

    def get_total_cost(self):
        return self.total_cost

def find_best_path(atlas):
    '''Finds the best path from src to dest, based on costs from atlas.
    Returns a tuple of two elements. The first is a list of city numbers,
    starting with 0 and ending with atlas.num_cities-1, that gives the
    optimal path between those two cities. The second is the total cost
    of that path.'''

    node_start = Node(0, None, 0, 0)
    frontier = [node_start]
    explored = []
    goal_state = atlas.get_num_cities() - 1
    cost_final = 0
    node_goal = None

    # while frontier is not empty
    while frontier.__len__() != 0:

        # determine node on frontier with lowest path cost
        lowest_cost_node = frontier[0]
        for node in frontier:
            if node.get_total_cost() < lowest_cost_node.get_total_cost():
                lowest_cost_node = node

        # if goal state, end search
        if lowest_cost_node.get_node_num() == goal_state:
            cost_final = lowest_cost_node.get_total_cost()
            node_goal = lowest_cost_node
            break

        # this node will be explored, remove from queue and add to explored + optimal path
        frontier.remove(lowest_cost_node)
        explored.append(lowest_cost_node)

        # determine what paths exist from lowest cost node to any other node
        for a_node in range(atlas.get_num_cities()):
            cost = atlas.get_road_dist(lowest_cost_node.get_node_num(), a_node)

            # if a path exists
            if cost > 0 and np.isinf(cost) == False:
                child_node = Node(a_node, lowest_cost_node, cost, lowest_cost_node.get_total_cost() + cost)

                member_of_frontier = False
                already_explored = False

                # test if child is on frontier
                if frontier.__len__() != 0:
                    for node in frontier:
                        if child_node.get_node_num() == node.get_node_num():
                            member_of_frontier = True

                # test if child has been explored previously
                for node in explored:
                    if child_node.get_node_num() == node.get_node_num():
                        already_explored = True

                # if neither is true, add child node to frontier
                if member_of_frontier == False and already_explored == False:
                    frontier.append(child_node)

                # else if child is in frontier queue with a higher path cost, replace frontier node with child
                elif member_of_frontier:
                    for node in frontier:
                        if child_node.get_node_num() == node.get_node_num():
                            if child_node.get_total_cost() < node.get_total_cost():
                                frontier.remove(node)
                                frontier.append(child_node)

    # return optimal path tuple
    node = node_goal
    optimal_path = [node.get_node_num()]
    while node.get_parent() is not None:
        optimal_path.insert(0, node.get_parent().get_node_num())
        node = node.get_parent()

    return_tuple = (optimal_path, cost_final)
    return return_tuple

if __name__ == '__main__':

    if len(sys.argv) not in [2,3]:
        print("Usage: gps.py numCities|atlasFile [debugLevel].")
        sys.exit(1)

    if len(sys.argv) > 2:
        if sys.argv[2] not in ['DEBUG','INFO','WARNING','ERROR']:
            print('Debug level must be one of: DEBUG, INFO, WARNING, ERROR.')
            sys.exit(2)
        logging.getLogger().setLevel(sys.argv[2])
    else:
        logging.getLogger().setLevel('INFO')

    try:
        num_cities = int(sys.argv[1])
        logging.info('Building random atlas with {} cities...'.format(
            num_cities))
        usa = Atlas(num_cities)
        logging.info('...built.')
    except:
        logging.info('Loading atlas from file {}...'.format(sys.argv[1]))
        usa = Atlas.from_filename(sys.argv[1])
        logging.info('...loaded.')

    path, cost = find_best_path(usa)
    print('Best path from {} to {} costs {}: {}.'.format(0,
        usa.get_num_cities()-1, cost, path))
    print('You expanded {} nodes: {}'.format(len(usa._nodes_expanded),
        usa._nodes_expanded))

