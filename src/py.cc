#include "graph.h"
#include <memory>

extern "C" {

Graph* simplegraph(bool directed) {
  Graph* graph = new Graph(directed);
  graph->generate_simple_graph();
  return graph;
}

Graph* readgraph(char* filename, bool directed) {
  Graph* graph = new Graph(directed);
  std::cout << "filename: " << filename << std::endl;
  graph->read_graph_matrix_market(filename);
  return graph;
}

ShortestPath* shortestpath(Graph* graph, int origin) {
  return new ShortestPath(graph->dijkstra_priority_queue(origin));
}

void update_edge(Graph* graph, int vertex1, int vertex2, double weight) {
  graph->update_edge(vertex1, vertex2, weight);
}

int origin(ShortestPath* sp) { return sp->source; }

double distance(ShortestPath* sp, int destination) {
  return sp->distances[destination];
}

int parent(ShortestPath* sp, int destination) {
  return sp->parent.at(destination);
}
}
