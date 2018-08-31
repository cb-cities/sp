# Compile GPU
```
nvcc -arch=sm_60 dijkstra.cc -o dikjstra -lcuda
```
# Compile CPU
```
g++ dijkstra-omp.cc -o dikjstra -lomp
```

# Dijkstra Binary Heap Queue

`g++ dijkstra-heap.cc -o heap`

```
5 dist 21.2
4 dist 20.7
6 dist 11.5
3 dist 9.1
1 dist 0
2 dist 7.5
```
