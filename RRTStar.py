from map import Map
from RRTTree import RRTTree
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utils import *

class RRTStarPlanner(object):
    def __init__(self, mapfile, start, goal, maxiter=200, stepsize=1.5):
        self.map = Map(mapfile, start, goal) # initialize the Map object
        self.start = start
        self.goal = goal
        self.tree = RRTTree()
        self.tree.costs[start] = 0
        self.startID = self.tree.addVertex(start)

        self.maxiter = maxiter
        self.stepsize = stepsize
        self.planned_path = [] # final planned path

    def plan(self):
        '''
        Plan the path from start to goal by sampling random states and extending the tree towards them.
        '''

        goalReached = False
        for i in range(self.maxiter):
            # sample a random state
            randState = self.sampleState()
            # find the nearest vertex
            nearestID, nearestVertex, nearestDist = self.tree.getNearestVertex(randState)
            # expand the tree towards the random state
            newID, newState = self.extend(nearestID, nearestVertex, nearestDist, randState)

            # reconnect the tree if the new state present shorter path
            if newID != -1:
                kNNIDs, kNNVertices, kNNDists = self.tree.getKNN(newState, 8)
                for j in range(len(kNNVertices)):
                    # First check if path between new state and kNNVertices[j] is valid




                    if self.tree.costs[kNNVertices[j]] + kNNDists[j] < self.tree.costs[newState]:
                        self.tree.addEdge(kNNVertices[j], newState)
                        self.tree.costs[newState] = self.tree.costs[kNNVertices[j]] + kNNDists[j]
                        self.tree.parents[newState] = kNNVertices[j]
                    elif self.tree.costs[kNNVertices[j]] > self.tree.costs[newState] + kNNDists[j]:
                        self.tree.addEdge(newState, kNNVertices[j])
                        self.tree.costs[kNNVertices[j]] = self.tree.costs[newState] + kNNDists[j]
                        self.tree.parents[kNNVertices[j]] = newState


            # check if the goal is reached
            if self.goal == newState:
                goalReached = True
                print('Goal reached!')
                print(i, len(self.tree.vertices))
                # break
        
        if goalReached:
            # trace the path from goal to start
            curr = self.goal
            self.planned_path.append(curr)
            while curr != self.start:
                curr = self.tree.parents[curr]
                self.planned_path.append(curr)
            self.planned_path.reverse()
            return True

    def sampleState(self):
        '''
        Sample a random state within the map. 
        Repeat sampling until we find a valid state that's not currently in the RRT tree.
        '''
        while True:
            rand_x = np.random.randint(self.map.xlim[0], self.map.xlim[1]+1)
            rand_y = np.random.randint(self.map.ylim[0], self.map.ylim[1]+1)
            if (rand_x, rand_y) in self.tree.vertices:
                continue
            if self.map.checkState((rand_x, rand_y)):
                return (rand_x, rand_y)
    

    def extend(self, nearestID, nearestVertex, nearestDist, randState):
        '''
        Extend the tree from the nearest vertex towards the random state.
        @param nearestID: the ID of the nearest vertex
        @param nearestVertex: the nearest vertex
        @param nearestDist: the distance between the nearest vertex and the random state
        @param randState: the random state
        '''
        if nearestDist < self.stepsize:
            newState = randState
        else:
            # if randState is further away than 1 unit, compute the new state in that direction
            direction = [randState[0] - nearestVertex[0], randState[1] - nearestVertex[1]]
            direction = np.clip(direction, -1, 1)
            newState = list(nearestVertex) + direction
            newState = tuple(newState)
        
            # check if the new state is valid
            if not self.map.checkState(newState):
                return -1, None
        
        # add the new state to the tree
        newID = self.tree.addVertex(newState)
        self.tree.addEdge(nearestVertex, newState)
        return newID, newState


    def plotMap(self):
        '''
        Plot the map with start and goal states.
        '''
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        plt.show()


    def plotPath(self, path, plotTree=True):
        '''
        Plot the map with planned path. 
        @param path: the planned path
        @param plotTree: if True, plot the RRT tree with all vertices and edges
        '''
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        if plotTree:
            self.plotTree(ax)
        self.map.plotPath(ax, path)
        plt.show()

    
    def plotTree(self, ax):
        '''
        Plot the RRT tree with all vertices and edges.
        @param ax: the matplotlib axis object
        '''
        for v in self.tree.vertices:
            patch = Rectangle((v[0]-0.5, v[1]-0.5), 1, 1, color='paleturquoise')
            ax.add_patch(patch)
            for u in self.tree.edges[v]:
                ax.plot([v[0], u[0]], [v[1], u[1]], 'peachpuff')

        

if __name__ == '__main__':
    mapfile = 'map2.txt'
    start = (0, 0)
    goal = (13, 14)
    planner = RRTStarPlanner(mapfile, start, goal, maxiter=2000, stepsize=1)
    # planner.plotMap()
    planner.plan()
    print(planner.planned_path)
    planner.plotPath(planner.planned_path)
    # planner.plotTree()