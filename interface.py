from __future__ import print_function

from sys import argv
from ctypes import *

libsp = cdll.LoadLibrary("./liblsp.so")

class Graph(Structure):
    def dijkstra(self, origin, destination):
        sp = ShortestPath()
        libsp.shortestpath(byref(sp), byref(self), origin, destination)
        return sp


libsp.simplegraph.restype = POINTER(Graph)

def simplegraph(directed=True):
    return libsp.simplegraph(directed).contents

libsp.readgraph.restype = POINTER(Graph)

def readgraph(filename, directed=True):
    return libsp.readgraph(filename, directed).contents


class ShortestPath(Structure):
    _fields_ = [("destination", c_int),
                ("distance", c_double)]

    pass


def test():
    g = simplegraph()
    sp = g.dijkstra(1, 3)

    print(sp.destination)
    print(sp.distance)

if __name__ == '__main__':
    test()
