import numpy as np
import matplotlib.pyplot as plt
from utils import *

class RRTTree(object):

    def __init__(self):
        self.vertices = []
        self.edges = dict()
        self.parents = dict()

    
    def getNearestVertex(self, state):
        '''
        Return the nearest vertex to the given state (state ID, state Vertex, distance).
        @param state: a query state
        '''
        dists = []
        for v in self.vertices:
            dists.append(computeEuclideanDist(state, v))
        
        min_dist = min(dists)
        min_id = dists.index(min_dist)
        return min_id, self.vertices[min_id], min_dist
        # return self.vertices[min_id], min_dist
    

    def getKNN(self, state, k):
        '''
        Return the k nearest vertices to the given state (state IDs, state Vertices, distances).
        @param state: a query state
        @param k: the number of nearest neighbors
        '''
        dists = []

        for v in self.vertices:
            dists.append(computeEuclideanDist(state, v))
        
        knnIDs = np.argpartition(dists, k)
        knnDists = [dists[i] for i in knnIDs]
        knnVertices = [self.vertices[i] for i in knnIDs]
        return knnIDs, knnVertices, knnDists
        # return knnVertices, knnDists
    

    def addVertex(self, state):
        '''
        Add a vertex with the given state.
        @param state: the state of the vertex
        '''
        self.vertices.append(state)
        self.edges[state] = []
        return len(self.vertices) - 1

    
    def addEdge(self, state1, state2):
        '''
        Add an edge between two vertices with the given states.
        @param state1: the state of the first vertex
        @param state2: the state of the second vertex
        '''
        self.edges[state1].append(state2)
        # self.edges[state2].append(state1)
        self.parents[state2] = state1