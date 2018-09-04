#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <vector>

class Graph {
 public:
  using vertex_t = int;     // Vertex id type
  using weight_t = double;  // Weight type, that can be added with +
  using gpair = std::pair<vertex_t, weight_t>;
  // Edge
  // {{v1, v2}, weight}
  using Edge = std::pair<std::pair<vertex_t, vertex_t>, weight_t>;

  // Assign number of vertices
  void assign_nvertices(unsigned size) { this->nvertices_ = size; }

  //! Add edge
  void add_edge(vertex_t vertex1, vertex_t vertex2, weight_t weight,
                bool directed = false);

  //! Remove edge
  void remove_edge(const std::shared_ptr<Edge>& edge);

  //! Generate a simple graph
  void generate_simple_graph(bool directed = false);

  //! Read MatrixMarket graph file format
  void read_graph_matrix_market(const std::string& filename,
                                bool directed = false);

  //! Compute the shortest path using binary heap queue dijkstra
  std::unordered_map<vertex_t, weight_t> dijkstra(vertex_t source,
                                                  vertex_t dest);

  //! Compute the shortest path using priority queue
  void dijkstra_priority_queue(vertex_t src, vertex_t dest);

  // Get path from source to j using parent array
  std::vector<vertex_t> get_path(
      const std::unordered_map<vertex_t, vertex_t>& parent, vertex_t vertex) {
    std::vector<vertex_t> path;
    while (vertex != -1) {
      vertex = parent.at(vertex);
      if (vertex != -1) path.emplace_back(vertex);
    }
    return path;
  }

  // Print shortest path distances for destinations
  void print_distances(vertex_t destination,
                       const std::vector<weight_t>& distance,
                       const std::unordered_map<vertex_t, vertex_t>& parent) {
    int src = 0;
    if (destination != -1) {
      std::cout << "Destination: " << destination
                << " distance: " << distance[destination];
      std::cout << " path: ";
      auto path = get_path(parent, destination);
      for (const auto& item : path) std::cout << item << "\t";
      std::cout << "\n";
    }
  }

 private:
  unsigned nvertices_;
  // Edges and weights
  std::vector<std::shared_ptr<Edge>> edges_;
  // dijkstra's algorithm with a heap priority queue
  // adjacency list with iteration over each edge
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;
};

#endif  // _GRAPH_H_
