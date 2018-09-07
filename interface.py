from __future__ import print_function

from sys import argv
from ctypes import *

sp = cdll.LoadLibrary("./liblsp.so")
py_main = sp.py_main

class ShortestPath(Structure):
    _fields_ = [("destination", c_int),
                ("distance", c_double)]

py_main.restype = POINTER(ShortestPath)

filename = "../sf.mtx"
ans = sp.py_main(filename, 1, 3)
print(ans)
print(ans.contents)

print(ans.contents.destination)
print(ans.contents.distance)
