import numpy as np
import matplotlib.pyplot as plt
from map import Map
import heapq

class AStarPlanner:
    def __init__(self, mapfile, start, goal):
        self.map = Map(mapfile, start, goal) # initialize the Map object
        self.start = start
        self.goal = goal

        # the open list is the current frontier, 
        # which is a priority queue with (cost, state) tuples, ipmlemented as a heap
        self.open = [(0, start)] 
        heapq.heapify(self.open)
        self.closed = [] 

        self.parents = {} # dictionary of parents for each state
        self.cost_so_far = {start: 0} # dictionary of costs to reach each state
        self.planned_path = [] # final planned path

    # search the map for a path from start to goal
    def search(self):
        while self.open:
            curr = heapq.heappop(self.open) # pop the state with the lowest f-cost

            # search all 8 neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == dy == 0: # skip the current state
                        continue
                    neighbor = (curr[1][0] + dx, curr[1][1] + dy)
                    if not self.map.checkState(neighbor): # skip if the neighbor is not valid
                        continue
                    if neighbor == self.goal: # goal reached
                        self.parents[neighbor] = curr[1]
                        return True
                    
                    new_cost_g = self.cost_so_far[curr[1]] + self.euclidean_dist(curr[1], neighbor) # compute new g-cost
                    # skip if the neighbor is already in the open list with a lower g-cost
                    if neighbor in self.cost_so_far and new_cost_g >= self.cost_so_far[neighbor]: 
                        continue
                    self.cost_so_far[neighbor] = new_cost_g

                    new_cost_h = self.heuristic_dist(neighbor, self.goal)
                    new_cost_f = new_cost_g + new_cost_h
                    self.parents[neighbor] = curr[1] # update the parent of the neighbor
                    heapq.heappush(self.open, (new_cost_f, neighbor)) # add the neighbor to the open list
            self.closed.append(curr[1]) # add the current state to the closed list
        return False
    
    # plan a path from start to goal
    def plan(self):
        # first search for a path from start to goal
        if self.search():
            # then trace the path from goal to start
            curr = self.goal
            while curr != self.start:
                self.planned_path.append(curr)
                curr = self.parents[curr]
            self.planned_path.append(self.start)
            # reverse the list for the path from start to goal
            self.planned_path.reverse()
            print("Path found!")
            return True
        
        else:
            print('No path found!')
            return False

    # calculate the euclidean distance between two states                                
    def euclidean_dist(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    # calculate the heuristic distance (Manhattan distance) between two states
    def heuristic_dist(self, a, b):
        return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])
    
    
    def plotMap(self):
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        plt.show()


    def plotPath(self, path):
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        self.map.plotPath(ax, path)
        plt.show()
    
if __name__ == '__main__':
    mapfile = 'map2.txt'
    start = (0, 0)
    goal = (13, 14)
    planner = AStarPlanner(mapfile, start, goal)
    # planner.plotMap()
    planner.plan()
    planner.plotPath(planner.planned_path)