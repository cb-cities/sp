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
#include <utility>
#include <vector>

#include "tsl/robin_map.h"

//! \brief ShortestPath struct to return source, distance and parents
struct ShortestPath {
  //! Vertex id type
  using vertex_t = int;
  //! Weight type, that can be added with +
  using weight_t = double;

  //! Get path from source to j using parent array
  //! \param[in] parent Map of vertex to its parent id
  //! \param[in] destination Destination vertex id to get path
  //! \param[in] source Source vertex id to get path (SSSP set as -1)
  //! \retval path Path from source to destination
  std::vector<vertex_t> get_path(vertex_t source, vertex_t destination) {
    // Create an empty path
    std::vector<vertex_t> path;
    // Iterate until source has been reached
    while (destination != source) {
      destination = parent.at(destination);
      if (destination != source)
        path.emplace_back(destination);
    }
    // Reverse to arrange from source to destination
    std::reverse(path.begin(), path.end());
    return path;
  }

  //! Source
  vertex_t source;
  //! Destination
  // std::vector<vertex_t> destinations;
  //! Distances
  std::vector<weight_t> distances;
  //! Parent array to store shortest path tree
  std::vector<vertex_t> parent;
};

//! \brief Graph class to store vertices and edge and compute shortest path
//! \details Graph class has Priority Queue Dijkstra algorithm for SSSP
class Graph {
public:
  //! Vertex id type
  using vertex_t = int;
  //! Weight type, that can be added with +
  using weight_t = double;
  //! Edge {{v1, v2}, weight}
  using Edge = std::pair<std::pair<vertex_t, vertex_t>, weight_t>;

  //! Construct directed / undirected graph
  //! \param[in] directed Defines if the graph is directed or not
  explicit Graph(bool directed) : directed_{directed} {};

  //! Return number of vertices
  unsigned nvertices() const { return nvertices_; }

  //! Add edge to graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  //! \param[in] weight Weight of edge connecting vertex 1 and 2
  void add_edge(vertex_t vertex1, vertex_t vertex2, weight_t weight);

  //! Update edge of a graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  //! \param[in] weight Weight of edge connecting vertex 1 and 2
  void update_edge(vertex_t vertex1, vertex_t vertex2, weight_t weight);

  //! Remove edge from graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  void remove_edge(vertex_t vertex1, vertex_t vertex2);

  //! Generate a simple graph
  void generate_simple_graph();

  //! Create a graph based on edges from Pandas DataFrame
  //! \param[in] edge_starts name of the dataframe column containing edge start
  //! nodes \param[in] edge_ends name of the dataframe column containing edge
  //! end nodes \param[in] edge_wghs name of the dataframe column containing
  //! edge weights \param[in] nedges numbers of edges (number of rows in the
  //! edges dataframe) \param[in] nvertices numbers of vertices (maximum of
  //! start and end node ids in the edges dataframe)
  void generate_graph(int *edge_starts, int *edge_ends, double *edge_wghs,
                      int nedges, int nvertices);

  //! Read MatrixMarket graph file format
  //! \param[in] filename Name of input MatrixMarket file
  //! \retval status File read status
  bool read_graph_matrix_market(const std::string &filename);

  //! Write MatrixMarket graph file format
  //! \param[in] filename Name of output MatrixMarket file
  bool write_graph_matrix_market(const std::string &filename);

  //! Compute the shortest path using priority queue
  //! \param[in] source ID of source vertex1
  //! \param[in] destination ID of destination vertex (default is -1 for SSSP)
  //! \retval sp Shortest path and distances
  ShortestPath dijkstra_priority_queue(vertex_t source,
                                       vertex_t destination = -1);

private:
  //! Assign number of vertices
  //! \param[in] nvertices Number of vertices in graph
  void assign_nvertices(unsigned nvertices) { this->nvertices_ = nvertices; }

  // Directed / undirected
  bool directed_{false};
  // Number of graph vertices
  unsigned nvertices_{std::numeric_limits<unsigned>::max()};
  // Edges
  std::map<std::tuple<vertex_t, vertex_t>, std::shared_ptr<Edge>> edges_;
  // adjacency list with iteration over each edge
  tsl::robin_map<vertex_t, std::vector<std::shared_ptr<Edge>>> vertex_edges_;
};

#endif // _GRAPH_H_
