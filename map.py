import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.patches import Rectangle

class Map:
    def __init__(self, file, start, goal):

        self.grid = np.loadtxt(file, dtype=np.int8)
        self.xlim = [0, self.grid.shape[1]-1]
        self.ylim = [0, self.grid.shape[0]-1]
        self.start = start
        self.goal = goal

        # check if the start and goal states are valid
        if self.checkState(self.start) == False:
            raise Exception('Start state is not valid!')
        if self.checkState(self.goal) == False:
            raise Exception('Goal state is not valid!')
        
        # fig, ax = plt.subplots()
        # self.plotMap(ax)
        # plt.show()
        print('Map initialized.')
    
    # check if a state is valid (i.e. not out of bounds and not an obstacle)
    def checkState(self, state):
        if state[0] < self.xlim[0] or state[0] > self.xlim[1] or state[1] < self.ylim[0] or state[1] > self.ylim[1]:
            return False
        elif self.grid[state[1], state[0]] == 1:
            return False
        else:
            return True
    
    # plot the map with the start and goal states
    def plotMap(self, ax):
        cmap = colors.ListedColormap(['white', 'dimgray'])
        
        ax.imshow(self.grid, cmap=cmap, interpolation='nearest')
        ax.grid(which='minor', axis='both', linestyle='-', color='k', linewidth=1)
        ax.set_xticks(np.arange(self.xlim[0], self.xlim[1]+1, 1))
        ax.set_xticks(np.arange(self.xlim[0]-0.5, self.xlim[1]+1, 1), minor=True)
        ax.set_yticks(np.arange(self.ylim[0], self.ylim[1]+1, 1))
        ax.set_yticks(np.arange(self.ylim[0]-0.5, self.ylim[1]+1, 1), minor=True)

        start_patch = Rectangle((self.start[0]-0.5, self.start[1]-0.5), 1, 1, color='r')
        goal_patch = Rectangle((self.goal[0]-0.5, self.goal[1]-0.5), 1, 1, color='g')
        ax.add_patch(start_patch)
        ax.add_patch(goal_patch)


    # plot the planned path
    def plotPath(self, ax, path):
        lastState = self.start
        for state in path[1:-1]:
            patch = Rectangle((state[0]-0.5, state[1]-0.5), 1, 1, color='c')
            ax.add_patch(patch)
            ax.plot([lastState[0], state[0]], [lastState[1], state[1]], 'lightcoral')
            lastState = state
        state = self.goal
        ax.plot([lastState[0], state[0]], [lastState[1], state[1]], 'lightcoral')
