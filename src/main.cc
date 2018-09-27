#include <chrono>
#include <ctime>
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
  auto start = std::chrono::system_clock::now();
  const auto sp = graph->dijkstra_priority_queue(1, -1);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds.count() << "s\n";

  // graph->write_graph_matrix_market("test.mtx");
  /*
  unsigned i = 0;
  for (const auto& distance : sp.distances) {
    std::cout << i << "\t" << distance << "\n";
    ++i;
  }
  */

  // Get path
  /*
  path.emplace_back(destination);
  // Iterate until source has been reached
  while (destination != source) {
    destination = parent.at(destination);
    path.emplace_back(destination);
  }
  // Reverse to arrange from source to destination
  std::reverse(path.begin(), path.end());
  */
}
