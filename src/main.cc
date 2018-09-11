#include <memory>

#include "graph.h"

int main(int argc, char** argv) {
  const bool directed = true;
  auto graph = std::make_unique<Graph>(directed);
  if (argc > 1) {
    // Read MatrixMarket file
    const std::string filename = argv[1];
    graph->read_graph_matrix_market(filename);
  } else {
    // Generate a simple graph
    graph->generate_simple_graph();
  }

  // const auto distances = graph->dijkstra_priority_queue(1, -1);
  const auto sp = graph->dijkstra_priority_queue(1, 3);
  std::cout << "Dijkstra PriorityQueue\n";
  /*
  unsigned i = 0;
  for (const auto& distance : sp.distances) {
    std::cout << i << "\t" << distance << "\n";
    ++i;
  }
  */
}
