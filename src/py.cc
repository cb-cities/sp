#include <memory>
#include "graph.h"

extern "C" {

  struct ShortestPath_py {
    int ndestination; 
    int* destination;
    double* distance;
    int* parent;
  };

  Graph* simplegraph(bool directed) {
    Graph* graph = new Graph(directed);
    graph->generate_simple_graph();
    return graph;
  }

  Graph* readgraph(char* filename, bool directed) {
    Graph* graph = new Graph(directed);
    graph->read_graph_matrix_market(filename);
    return graph;
  }

  ShortestPath* shortestpath(Graph* graph, int origin) { //, int* destination, int ndest) {
    ShortestPath* sp = new ShortestPath(
      graph->dijkstra_priority_queue(origin) //, std::vector<int>(destination, destination+ndest));
        );
    return sp;

    /*
    ret->destination = new int[ndest];
    ret->distance = new double[ndest];
    ret->ndestination = ndest;

    for (unsigned i = 0; i < ndest; ++i) {
      ret->destination[i] = destination[i];
      ret->distance[i] = sp.distances[destination[i]];
      ret->parent[i] = sp.parent[destination[i]];
    }

    return 0;
    */
  }

  double distance(ShortestPath *sp, int destination) {
    return sp->distances[destination];
  }
}
