#include <limits>
#include <memory>

#include "graph.h"

extern "C" {

//! Create a simple toy graph
//! \param[in] directed Set if graph is directed or not
//! \retval graph Pointer to a graph object
Graph* simplegraph(bool directed) {
  Graph* graph = new Graph(directed);
  graph->generate_simple_graph();
  return graph;
}

//! Create a graph based on MatrixMarket file
//! \param[in] filename Filename of input graph
//! \param[in] directed Set if graph is directed or not
//! \retval graph Pointer to a graph object
Graph* readgraph(char* filename, bool directed) {
  Graph* graph = new Graph(directed);
  graph->read_graph_matrix_market(filename);
  return graph;
}

//! Write graph as MatrixMarket file
//! \param[in] filename Graph filename
//! \param[in] graph Graph object
bool writegraph(Graph* graph, char* filename) {
  return graph->write_graph_matrix_market(filename);
}

//! Find the shortest path using Dijkstra priority queue
//! \param[in] graph Graph object to find the shortest path
//! \param[in] origin Origin of the shortest path
//! \param[in] destination Destination of the shortest path
ShortestPath* dijkstra(Graph* graph, int origin, int destination) {
  return new ShortestPath(graph->dijkstra_priority_queue(origin, destination));
}

//! Update edge weights
//! \param[in] graph Graph object
//! \param[in] vertex1 Vertex1 of the edge
//! \param[in] vertex2 Vertex2 of the edge
//! \param[in] weight Weight of the edge
void update_edge(Graph* graph, int vertex1, int vertex2, double weight) {
  graph->update_edge(vertex1, vertex2, weight);
}

//! return origin of shortest path
//! \param[in] sp Shortest path object
int origin(ShortestPath* sp) { return sp->source; }

//! return the shortest path cost
//! \param[in] sp Shortest path object
//! \param[in] destination Destination of the shortest path
double distance(ShortestPath* sp, int destination) {
  return (sp->distances.size() == 0 ? std::numeric_limits<double>::max()
                                    : sp->distances[destination]);
}

//! Delete Shortest Path
void clear(ShortestPath* sp) { delete sp; }

//! return the parent of a vertex in shortest path tree
//! \param[in] sp Shortest path object
//! \param[in] destination Destination of the shortest path
int parent(ShortestPath* sp, int destination) {
  return sp->parent.at(destination);
}
}
