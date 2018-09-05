#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

class Graph {
 public:
  using vertex_t = int;     // Vertex id type
  using weight_t = double;  // Weight type, that can be added with +
  using gpair = std::pair<vertex_t, weight_t>;
  //! Edge {{v1, v2}, weight}
  using Edge = std::pair<std::pair<vertex_t, vertex_t>, weight_t>;

  //! Constructor
  explicit Graph(bool directed) : directed_{directed} {};

  //! Assign number of vertices
  void assign_nvertices(unsigned size) { this->nvertices_ = size; }

  //! Number of vertices
  unsigned nvertices() const { return nvertices_; }

  //! Add edge
  void add_edge(vertex_t vertex1, vertex_t vertex2, weight_t weight);

  //! Remove edge
  void remove_edge(vertex_t vertex1, vertex_t vertex2);

  //! Generate a simple graph
  void generate_simple_graph();

  //! Read MatrixMarket graph file format
  void read_graph_matrix_market(const std::string& filename);

  //! Compute the shortest path using priority queue
  std::vector<weight_t> dijkstra_priority_queue(vertex_t src, vertex_t dest);

  // Get path from source to j using parent array
  std::vector<vertex_t> get_path(
      const std::unordered_map<vertex_t, vertex_t>& parent,
      vertex_t destination);

 private:
  // Directed / undirected
  bool directed_{false};
  // Number of graph vertices
  unsigned nvertices_{std::numeric_limits<unsigned>::max()};
  // Edges
  std::map<std::tuple<vertex_t, vertex_t>, std::shared_ptr<Edge>> edges_;
  // adjacency list with iteration over each edge
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;
};

#endif  // _GRAPH_H_
