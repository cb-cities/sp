#include <chrono>
#include <memory>

#include "catch.hpp"

#include "graph.h"

// Check Graph class
TEST_CASE("Graph class and shortest-path is checked", "[graph][sp][od]") {
  // Tolerance
  const double Tolerance = 1.E-7;

  // Test directed graph
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
    auto sp = graph->dijkstra_priority_queue(source, destination);
    auto distances = sp.distances;

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(7.2).epsilon(Tolerance));

    // Check update edge
    SECTION("Check update edge") {
      // Update edge (1, 3) to weight 3.7
      graph->update_edge(1, 3, 3.7);
      // Run Dijkstra Priority Queue
      sp = graph->dijkstra_priority_queue(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(3.7).epsilon(Tolerance));
      // Run Dijkstra Fibonacci heap
      auto fh_sp = graph->dijkstra_fibonacci_heap(source, destination);
      // Get distances
      auto fh_distances = fh_sp.distances;
      // Check shortest path
      REQUIRE(fh_distances.at(3) == Approx(3.7).epsilon(Tolerance));
    }

    // Check remove edge
    SECTION("Check remove edge") {
      // Remove edge (3, 1)
      graph->remove_edge(3, 1);
      // Run Dijkstra Priority Queue
      sp = graph->dijkstra_priority_queue(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(7.2).epsilon(Tolerance));

      // Remove edge (2, 4)
      graph->remove_edge(2, 4);
      // Run Dijkstra Priority Queue
      sp = graph->dijkstra_priority_queue(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));

      // Run Dijkstra Fibonacci heap
      sp = graph->dijkstra_fibonacci_heap(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));
    }

    // Write graph
    SECTION("Check write graph") {
      REQUIRE(graph->write_graph_matrix_market("test-graph-directed.mtx") ==
              true);
    }
  }

  // Test undirected graph
  SECTION("Test SSSP in undirected graph") {
    const bool directed = false;
    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Create a simple example graph
    graph->generate_simple_graph();

    // Run Dijkstra Priority Queue
    Graph::vertex_t source = 1;
    Graph::vertex_t destination = 3;
    auto sp = graph->dijkstra_priority_queue(source, destination);
    // Get distances
    auto distances = sp.distances;

    // Check distances
    REQUIRE(distances.size() == graph->nvertices());
    // Check shortest path
    REQUIRE(distances.at(3) == Approx(5.6).epsilon(Tolerance));

    // Check remove edge
    SECTION("Check remove edge") {
      // Remove edge (3, 1)
      graph->remove_edge(3, 1);
      // Run Dijkstra Priority Queue
      sp = graph->dijkstra_priority_queue(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));

      // Run Dijkstra Fibonacci Heap
      sp = graph->dijkstra_fibonacci_heap(source, destination);
      // Get distances
      distances = sp.distances;
      // Check shortest path
      REQUIRE(distances.at(3) == Approx(9.1).epsilon(Tolerance));
    }
  }

  SECTION("Test SSSP in directed graph from file") {
    // Set graph properties
    const bool directed = true;

    // Create graph object
    auto graph = std::make_unique<Graph>(directed);
    // Read MatrixMarket file
    const std::string filename = "../network.mtx";
    // Read file
    REQUIRE(graph->read_graph_matrix_market(filename) == true);

    // Run Dijkstra Priority Queue
    Graph::vertex_t source = 1020;
    Graph::vertex_t destination = 20;

    SECTION("Dijkstra Priority Queue") {
      auto start = std::chrono::system_clock::now();
      const auto sp = graph->dijkstra_priority_queue(source, destination);
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      std::cout << "PQ: " << elapsed_seconds.count() << "s\n";

      // Get distances
      auto distances = sp.distances;

      // Check distances
      REQUIRE(distances.size() == graph->nvertices());
      // Check shortest path
      REQUIRE(distances.at(20) ==
              Approx(12409.660000000002).epsilon(Tolerance));
    }

    SECTION("Dijkstra Fibonacci Heap") {
      auto start = std::chrono::system_clock::now();
      auto sp = graph->dijkstra_fibonacci_heap(source, destination);
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      std::cout << "PQ: " << elapsed_seconds.count() << "s\n";
      // Get distances
      auto distances = sp.distances;

      // Check distances
      REQUIRE(distances.size() == graph->nvertices());
      // Check shortest path
      REQUIRE(distances.at(20) ==
              Approx(12409.660000000002).epsilon(Tolerance));
    }

    SECTION("Test non-existant file") {
      // Create graph object
      auto graph = std::make_unique<Graph>(directed);
      // Read MatrixMarket file
      const std::string filename = "nofile.mtx";
      // Read file should fail
      REQUIRE(graph->read_graph_matrix_market(filename) == false);
    }
  }
}
