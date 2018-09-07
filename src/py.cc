#include <memory>
#include "graph.h"

extern "C" {

  struct ShortestPath_py {
    int destination;
    double distance;
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

  int shortestpath(ShortestPath_py* ret, Graph* graph, int origin, int destination) {
    const auto sp = graph->dijkstra_priority_queue(origin, destination);
    ret->destination = destination;
    ret->distance = sp.distances[destination];
    return 0;
  }
}
