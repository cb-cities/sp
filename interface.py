from __future__ import print_function

from sys import argv
from ctypes import *

libsp = cdll.LoadLibrary("./liblsp.so")

class Graph(Structure):
    def dijkstra(self, origin, destination):
        sp = ShortestPath()
        if not isinstance(destination, list):
            destination = [destination]
        ndest = len(destination)
        libsp.shortestpath(byref(sp), byref(self), origin,
                (c_int*ndest)(*destination),
                len(destination))

        dests = [sp.destination[i] for i in range(ndest)]
        dists = [sp.distance[i] for i in range(ndest)]
        return zip(dests, dists)


libsp.simplegraph.restype = POINTER(Graph)

def simplegraph(directed=True):
    return libsp.simplegraph(directed).contents

libsp.readgraph.restype = POINTER(Graph)

def readgraph(filename, directed=True):
    return libsp.readgraph(filename, directed).contents


class ShortestPath(Structure):
    _fields_ = [("ndestination", c_int), 
                ("destination", POINTER(c_int)),
                ("distance", POINTER(c_double))]

    pass


def test():
    g = simplegraph()
    sp = g.dijkstra(1, [2,3])

    for thing in sp:
        print(thing[0], thing[1])

if __name__ == '__main__':
    test()
