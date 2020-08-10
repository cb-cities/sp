from __future__ import print_function

from sys import argv
from ctypes import *
from numpy.ctypeslib import ndpointer
import pandas as pd
import numpy as np

import os 
absolute_path = os.path.dirname(os.path.abspath(__file__))

libsp = cdll.LoadLibrary(absolute_path+"/build/liblsp.dylib")
libsp.distance.restype = c_double

class ShortestPath(Structure):
    @property
    def origin(self):
        return libsp.origin(byref(self))

    def distance(self, destination):
        return libsp.distance(byref(self), destination)

    def parent(self, destination):
        return libsp.parent(byref(self), destination)

    def route(self, destination):
        if destination != self.origin:
            parent = self.parent(destination)
            yield from self.route(parent)
            yield parent, destination
    def clear(self):
        return libsp.clear(byref(self))

libsp.dijkstra.restype = POINTER(ShortestPath)

class Graph(Structure):
    def dijkstra(self, origin, destination):
        return libsp.dijkstra(byref(self), origin, destination).contents
    def update_edge(self, origin, destination, weight):
        return libsp.update_edge(byref(self), origin, destination, weight)
    def writegraph(self, filename):
        return libsp.writegraph(byref(self), filename)

libsp.simplegraph.restype = POINTER(Graph)
libsp.readgraph.restype = POINTER(Graph)
libsp.creategraph.restype = POINTER(Graph)
libsp.creategraph.argtypes = [
        ndpointer(c_int32, flags='C_CONTIGUOUS'),
        ndpointer(c_int32, flags='C_CONTIGUOUS'),
        ndpointer(c_double, flags='C_CONTIGUOUS'),
        c_int, c_int, c_bool]

def simplegraph(directed=True):
    return libsp.simplegraph(directed).contents

def readgraph(filename, directed=True):
    return libsp.readgraph(filename, directed).contents

def from_dataframe(edges=None, start_node_col=None, end_node_col=None, weight_col=None, directed=True):
    # print(np.max(edges[[start_node_col, end_node_col]].values))
    return libsp.creategraph(
        edges[start_node_col].values.astype(np.int32),
        edges[end_node_col].values.astype(np.int32),
        edges[weight_col].values,
        edges.shape[0], ### the number of edges is computed from the edges dataframe, not an input to the python interface
        np.max(edges[[start_node_col, end_node_col]].values), ### number of vertices is the maximum of the start or end node id in the edges dataframe
        directed).contents


def test():
    g = simplegraph()
    #g = readgraph(b"../sf.mtx")
    res = g.update_edge(1, 3, c_double(0.5))
    sp = g.dijkstra(1, -1)

    print("origin:", sp.origin)
    g.writegraph(b"test.mtx")
    for destination in [2, 3]:
        print(destination, sp.distance(destination))

        print( " -> ".join("%s"%vertex[1] for vertex in sp.route(destination)) )
    sp.clear()

def test_df():
    print("test 1")
    df = pd.DataFrame({'start':[0,1,2,3,4,5,6,7], 'end':[1,2,3,4,5,6,7,0], 'wgh':[0.1,0.5,1.9,1.1,1.2,1.5,1.6,1.9]})
    g = from_dataframe(df, 'start', 'end', 'wgh')

    origin, destin = 1, 5
    sp = g.dijkstra(origin, destin)

    print("origin is {}, destination is {}".format(sp.origin, destin))
    print("path is {} --> ".format(sp.origin), " -> ".join("%s"%vertex[1] for vertex in sp.route(destin)) )
    print("distance is ", sp.distance(destin))
    sp.clear()

    print("\ntest 2")
    df = pd.DataFrame({'start':[0,2,4,10,12], 'end':[1,3,5,12,0], 'wgh':[0.1,0.5,1.9,1.1,1.2]})
    # df = pd.DataFrame({'start':[0,1,2,3,4], 'end':[1,2,3,4,0], 'wgh':[0.1,0.5,1.9,1.1,1.2]})
    g = from_dataframe(df, 'start', 'end', 'wgh')

    origin, destin = 10,1 #0,4
    sp = g.dijkstra(origin, destin)

    print("origin is {}, destination is {}".format(sp.origin, destin))
    print("path is {} --> ".format(sp.origin), " -> ".join("%s"%vertex[1] for vertex in sp.route(destin)) )
    print("distance is ", sp.distance(destin))
    sp.clear()

if __name__ == '__main__':
    test_df()
