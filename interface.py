from __future__ import print_function

from sys import argv
from ctypes import *

libsp = cdll.LoadLibrary("./liblsp.so")
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

libsp.shortestpath.restype = POINTER(ShortestPath)

class Graph(Structure):
    def dijkstra(self, origin):
        return libsp.shortestpath(byref(self), origin).contents

libsp.simplegraph.restype = POINTER(Graph)
libsp.readgraph.restype = POINTER(Graph)

def simplegraph(directed=True):
    return libsp.simplegraph(directed).contents


def readgraph(filename, directed=True):
    return libsp.readgraph(filename, directed).contents


def test():
    g = simplegraph()
    g = readgraph(b"../sf.mtx")
    sp = g.dijkstra(1)

    print("origin:", sp.origin)

    for destination in [2,3]:
        print(destination, sp.distance(destination))

        print( " -> ".join("%s"%vertex[1] for vertex in sp.route(destination)) )

if __name__ == '__main__':
    test()
