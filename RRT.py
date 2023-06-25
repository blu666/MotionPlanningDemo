from map import Map
from RRTTree import RRTTree
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
from utils import *

class RRTPlanner(object):
    def __init__(self, mapfile, start, goal, maxiter=200, stepsize=1.5):
        self.map = Map(mapfile, start, goal) # initialize the Map object
        self.start = start
        self.goal = goal
        self.tree = RRTTree()
        self.startID = self.tree.addVertex(start)
        # self.goalID = self.tree.addVertex(goal)

        self.maxiter = maxiter
        self.stepsize = stepsize
        self.planned_path = [] # final planned path

    def plan(self):
        goalReached = False
        for i in range(self.maxiter):
            # sample a random state
            randState = self.sampleState()
            # find the nearest vertex
            nearestID, nearestVertex, nearestDist = self.tree.getNearestVertex(randState)
            # expand the tree towards the random state
            newID, newState = self.extend(nearestID, nearestVertex, nearestDist, randState)
            # check if the goal is reached
            if self.goal == newState:
                goalReached = True
                print('Goal reached!')
                print(i, len(self.tree.vertices))
                break
        
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
        while True:
            rand_x = np.random.randint(self.map.xlim[0], self.map.xlim[1]+1)
            rand_y = np.random.randint(self.map.ylim[0], self.map.ylim[1]+1)
            if (rand_x, rand_y) in self.tree.vertices:
                continue
            if self.map.checkState((rand_x, rand_y)):
                return (rand_x, rand_y)
    

    def extend(self, nearestID, nearestVertex, nearestDist, randState):
        if nearestDist < self.stepsize:
            newState = randState
            # unitVec = (randState[0] - nearestVertex[0], randState[1] - nearestVertex[1])
            # unitVec = unitVec / np.linalg.norm(unitVec)
        else:
            # compute the unit vector towards the random state
            # unitVec = (randState[0] - nearestVertex[0], randState[1] - nearestVertex[1])
            # unitVec = unitVec / np.linalg.norm(unitVec)

            # compute the new state by extending a step towards the random state, 
            # and use ceiling to make sure the new state is on the grid
            # newState = (nearestVertex[0] + int(self.stepsize*unitVec[0]), nearestVertex[1] + int(self.stepsize*unitVec[1]))
            direction = [randState[0] - nearestVertex[0], randState[1] - nearestVertex[1]]
            direction = np.clip(direction, -1, 1)
            newState = list(nearestVertex) + direction

            
            # direction[0] = max(direction[0], 1)
            # direction[1] = max(direction[1], 1)
            newState = tuple(newState)
        
        # check if the new state is valid
        # print(nearestVertex, unitVec, newState)
        # for i in range(1, self.stepsize+1):
            # interState = (int(nearestVertex[0] + i*unitVec[0]), int(nearestVertex[1] + i*unitVec[1]))
            if not self.map.checkState(newState):
                print(newState)
                return -1, None
        
        # if self.map.checkState(newState):
        # add the new state to the tree
        newID = self.tree.addVertex(newState)
        self.tree.addEdge(nearestVertex, newState)
        return newID, newState


    def plotMap(self):
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        plt.show()


    def plotPath(self, path):
        fig, ax = plt.subplots()
        self.map.plotMap(ax)
        self.plotTree(ax)
        self.map.plotPath(ax, path)
        plt.show()

    
    def plotTree(self, ax):
        for v in self.tree.vertices:
            patch = Rectangle((v[0]-0.5, v[1]-0.5), 1, 1, color='paleturquoise')
            ax.add_patch(patch)
            for u in self.tree.edges[v]:
                ax.plot([v[0], u[0]], [v[1], u[1]], 'peachpuff')

        

if __name__ == '__main__':
    mapfile = 'map2.txt'
    start = (0, 0)
    goal = (13, 14)
    planner = RRTPlanner(mapfile, start, goal, maxiter=800, stepsize=1)
    # planner.plotMap()
    planner.plan()
    print(planner.planned_path)
    planner.plotPath(planner.planned_path)
    # planner.plotTree()