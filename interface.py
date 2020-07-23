from __future__ import print_function

from sys import argv
from ctypes import *

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

def simplegraph(directed=True):
    return libsp.simplegraph(directed).contents


def readgraph(filename, directed=True):
    return libsp.readgraph(filename, directed).contents


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

if __name__ == '__main__':
    test()
