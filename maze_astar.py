import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index

    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)

    def goal_test(self, current_state):
        if current_state == self.goal_state:
            return True
        return False

    def get_cost(self, current_state, next_state):
        return 1

    def get_successors(self, state):
        direction_rows = [0, 0, -1, 1]
        direction_columns = [-1, 1, 0, 0]
        state_row = state[0]
        state_column = state[1]
        successors = []

        for i in range(4):
            new_row = state_row + direction_rows[i]
            new_column = state_column + direction_columns[i]
            if self.map[new_row, new_column] != 0.0:
                successors.append((state[0] + direction_rows[i], state[1] + direction_columns[i]))

        return successors

    # heuristics function
    def heuristics(self, state):
        return abs(state[0] - self.goal_state[0]) + abs(state[1] - self.goal_state[1])

    # priority of node 
    def priority(self, node):
        return node.cost_from_start + self.heuristics(node.state)

    # solve it
    def solve(self):
        if self.goal_test(self.start_state):
            return

        self.visited.append(self.start_state)
        first_node = Node(self.start_state, 0, None)
        count = 0
        priority_queue = [(self.priority(first_node), count, first_node)]

        while priority_queue:
            best_node = heappop(priority_queue)[2]

            successors = self.get_successors(best_node.state)

            for successor in successors:
                if successor in self.visited:
                    continue
                self.visited.append(successor)

                next_node = Node(successor, best_node.cost_from_start + self.get_cost(best_node.state, successor), best_node)
                if self.goal_test(successor):
                    self.draw(next_node)
                    return
                count += 1
                heappush(priority_queue, (self.priority(next_node), count, next_node))

    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    