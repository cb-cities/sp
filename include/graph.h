#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <tuple>
#include <vector>

#include "unordered_map_tuple_hash.h"

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
      const std::unordered_map<vertex_t, vertex_t>& parent,
      vertex_t destination);

 private:
  unsigned nvertices_;
  // Edges
  std::unordered_map<std::tuple<vertex_t, vertex_t>, std::shared_ptr<Edge>>
      edges_;
  // adjacency list with iteration over each edge
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;
};

#endif  // _GRAPH_H_
