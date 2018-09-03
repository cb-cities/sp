#include "graph.h"
#include "omp.h"
#include <memory>

int main(int argc, char** argv) {
  auto graph = std::make_shared<Graph>();
  if (argc > 1) {
    // Read MatrixMarket file
    const std::string filename = argv[1];
    graph->read_graph_matrix_market(filename);
  } else {
    // Generate a simple graph
    graph->generate_simple_graph();
  }
  const auto dist = graph->dijkstra(1, -1);
  for (const std::pair<Graph::vertex_t, Graph::weight_t>& p : dist)
    std::cout << p.first << " dist " << p.second << std::endl;
}
