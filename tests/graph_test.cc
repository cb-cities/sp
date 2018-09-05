#include <memory>

#include "catch.hpp"

#include "graph.h"

// Check Graph class
TEST_CASE("Graph is checked", "[graph][sp]") {
  auto graph = std::make_shared<Graph>();
  const bool directed = true;
  graph->generate_simple_graph(directed);
  graph->dijkstra_priority_queue(1, 3);
}
