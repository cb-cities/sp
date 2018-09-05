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

  /*
  const auto dist = graph->dijkstra_heap_queue(1, -1);

  std::cout << "Dijkstra BinaryHeapQueue\n";
  for (const std::pair<Graph::vertex_t, Graph::weight_t>& p : dist)
    std::cout << p.first << " dist " << p.second << std::endl;
  */

  const auto distances = graph->dijkstra_priority_queue(1, -1);
  std::cout << "Dijkstra PriorityQueue\n";
  /*
  unsigned i = 0;
  for (const auto& distance : distances) {
    std::cout << i << "\t" << distance << "\n";
    ++i;
  }
  */
}
