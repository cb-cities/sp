#include <memory>

#include "catch.hpp"

#include "graph.h"

// Check Graph class
TEST_CASE("Graph is checked", "[graph][sp]") {
  // Tolerance
  const double Tolerance = 1.E-7;

  SECTION("Directed graph") {
    const bool directed = true;
    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Create a simple example graph
    graph->generate_simple_graph();

    // Run Dijkstra Priority Queue
    const auto distances = graph->dijkstra_priority_queue(1, 3);

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(7.2).epsilon(Tolerance));
  }

  SECTION("Undirected graph") {
    const bool directed = false;
    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Create a simple example graph
    graph->generate_simple_graph();

    // Run Dijkstra Priority Queue
    const auto distances = graph->dijkstra_priority_queue(1, 3);

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(5.6).epsilon(Tolerance));
  }
}
