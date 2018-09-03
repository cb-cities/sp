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

  //! Make edge
  std::shared_ptr<Edge> make_edge(vertex_t vertex1, vertex_t vertex2,
                                  weight_t weight);
  //! Add edge
  void add_edge(vertex_t vertex1, vertex_t vertex2, weight_t weight);

  void remove_edge(const std::shared_ptr<Edge>& edge);

  //! Read MatrixMarket graph file format
  void read_graph_matrix_market(const std::string& filename);
  //! Generate a simple graph
  void generate_simple_graph();
  //! Run shortest path
  void dijkstra(vertex_t source, vertex_t dest);

 private:
  // dijkstra's algorithm with a heap priority queue
  // adjacency list with iteration over each edge
  std::vector<std::shared_ptr<Edge>> edges_;
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;
};

#endif  // _GRAPH_H_
