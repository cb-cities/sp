#include "graph.h"
#include <memory>

int main(int argc, char** argv) {
  auto graph = std::make_shared<Graph>();
  const bool directed = true;
  if (argc > 1) {
    // Read MatrixMarket file
    const std::string filename = argv[1];
    graph->read_graph_matrix_market(filename, directed);
  } else {
    // Generate a simple graph
    graph->generate_simple_graph(directed);
  }

  /*
  const auto dist = graph->dijkstra(1, -1);
  std::cout << "Dijkstra BinaryHeapQueue\n";
  for (const std::pair<Graph::vertex_t, Graph::weight_t>& p : dist)
    std::cout << p.first << " dist " << p.second << std::endl;
  */
  std::cout << "Dijkstra PriorityQueue\n";
  graph->dijkstra_priority_queue(1, 3);
}
