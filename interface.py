from __future__ import print_function

from sys import argv
from ctypes import *

py_main = cdll.LoadLibrary("./liblsp.so").py_main

class ShortestPath(Structure):
    _fields_ = [("destination", c_int),
                ("distance", c_double)]

sp = ShortestPath()

filename = "../sf.mtx"
py_main(byref(sp), filename, 1, 3)

print(sp.destination)
print(sp.distance)
