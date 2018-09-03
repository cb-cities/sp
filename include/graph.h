#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <vector>

class Graph {
 public:
  using vertex_t = int;     // Vertex id type
  using weight_t = double;  // Weight type, that can be added with +
  // Edge
  // {{v1, v2}, weight}
  using Edge = std::pair<std::pair<vertex_t, vertex_t>, weight_t>;

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

 private:
  // Edges and weights
  std::vector<std::shared_ptr<Edge>> edges_;
  // dijkstra's algorithm with a heap priority queue
  // adjacency list with iteration over each edge
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;
};

#endif  // _GRAPH_H_
