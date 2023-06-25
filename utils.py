import numpy as np

def computeEuclideanDist(state1, state2):
        '''
        Compute the Euclidean distance between two states.
        @param state1: the first state
        @param state2: the second state
        '''
        return np.sqrt((state1[0]-state2[0])**2 + (state1[1]-state2[1])**2)