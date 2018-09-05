#include <memory>

#include "catch.hpp"

#include "graph.h"

// Check Graph class
TEST_CASE("Graph is checked", "[graph][od]") {
  // Tolerance
  const double Tolerance = 1.E-7;

  SECTION("Test SSSP in directed graph") {
    // Set graph properties
    const bool directed = true;

    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Create a simple example graph
    graph->generate_simple_graph();

    // Run Dijkstra Priority Queue
    Graph::vertex_t source = 1;
    Graph::vertex_t destination = 3;
    auto distances = graph->dijkstra_priority_queue(source, destination);

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(7.2).epsilon(Tolerance));

    // Check remove edge
    SECTION("Check remove edge") {
      // Remove edge (3, 1)
      graph->remove_edge(3, 1);
      // Run Dijkstra Priority Queue
      distances = graph->dijkstra_priority_queue(source, destination);
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(7.2).epsilon(Tolerance));

      // Remove edge (2, 4)
      graph->remove_edge(2, 4);
      // Run Dijkstra Priority Queue
      distances = graph->dijkstra_priority_queue(source, destination);
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));
    }
  }

  SECTION("Test SSSP in undirected graph") {
    const bool directed = false;
    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Create a simple example graph
    graph->generate_simple_graph();

    // Run Dijkstra Priority Queue
    Graph::vertex_t source = 1;
    Graph::vertex_t destination = 3;
    auto distances = graph->dijkstra_priority_queue(source, destination);

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(5.6).epsilon(Tolerance));

    // Check remove edge
    SECTION("Check remove edge") {
      // Remove edge (3, 1)
      graph->remove_edge(3, 1);
      // Run Dijkstra Priority Queue
      distances = graph->dijkstra_priority_queue(source, destination);
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));
    }
  }
}
