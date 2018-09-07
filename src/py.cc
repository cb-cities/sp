#include <memory>
#include "graph.h"

extern "C" {

  struct ShortestPath_py {
    int destination;
    double distance;
  };


  int py_main(ShortestPath_py* ret, const char* filename, int origin, int destination) {
    std::cout << "filename: " << filename << std::endl;

    const bool directed = true;
    auto graph = std::make_unique<Graph>(directed);
    //graph->read_graph_matrix_market(filename);
    graph->generate_simple_graph();
    const auto sp = graph->dijkstra_priority_queue(origin, destination);

    ret->destination = destination;
    ret->distance = sp.distances[destination];

    return 0;
  }
}
