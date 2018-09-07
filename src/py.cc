#include <memory>
#include "graph.h"

extern "C" {

  struct ShortestPath_py {
    int destination;
    double distance;
  };


  ShortestPath_py* py_main(const char* filename, int origin, int destination) {
    std::cout << "filename: " << filename << std::endl;

    const bool directed = true;
    auto graph = std::make_unique<Graph>(directed);
    //graph->read_graph_matrix_market(filename);
    graph->generate_simple_graph();
    const auto sp = graph->dijkstra_priority_queue(origin, destination);

    ShortestPath_py *ret = (ShortestPath_py*)malloc(sizeof(ShortestPath_py));
    ret->destination = destination;
    ret->distance = sp.distances[destination];

    return ret;
  }
}
