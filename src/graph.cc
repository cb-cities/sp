#include "graph.h"

//! Add edge
inline void Graph::add_edge(Graph::vertex_t vertex1, Graph::vertex_t vertex2,
                            Graph::weight_t weight, bool directed) {

  if (!directed)
    if (vertex1 > vertex2) std::swap(vertex1, vertex2);

  // Create an edge
  auto edge = std::make_shared<Graph::Edge>(
      std::make_pair(std::make_pair(vertex1, vertex2), weight));
  edges_.emplace_back(edge);

  // Vertex 1
  auto vertex1_edges = vertex_edges_[vertex1];
  vertex1_edges.emplace_back(edge);
  vertex_edges_[vertex1] =
      std::vector<std::shared_ptr<Graph::Edge>>(vertex1_edges);

  if (!directed) {
    // Vertex 2
    auto vertex2_edges = vertex_edges_[vertex2];
    vertex2_edges.emplace_back(edge);
    vertex_edges_[vertex2] =
        std::vector<std::shared_ptr<Graph::Edge>>(vertex2_edges);
  }
}

void Graph::remove_edge(const std::shared_ptr<Graph::Edge>& edge) {
  edges_.erase(remove(edges_.begin(), edges_.end(), edge), edges_.end());
  auto v1edge = vertex_edges_[edge->first.first];
  auto v2edge = vertex_edges_[edge->first.second];
  v1edge.erase(remove(v1edge.begin(), v1edge.end(), edge), v1edge.end());
  v2edge.erase(remove(v2edge.begin(), v2edge.end(), edge), v2edge.end());
  vertex_edges_[edge->first.first] = v1edge;
  vertex_edges_[edge->first.second] = v2edge;
}

//! Read MatrixMarket graph file format
void Graph::read_graph_matrix_market(const std::string& filename,
                                     bool directed) {
  std::fstream file;
  file.open(filename.c_str(), std::ios::in);
  try {
    if (file.is_open() && file.good()) {
      // Line
      std::string line;
      bool header = true;
      double ignore;
      while (std::getline(file, line)) {
        std::istringstream istream(line);
        int v1, v2;
        double weight;
        // ignore comment lines (# or !) or blank lines
        if ((line.find('#') == std::string::npos) &&
            (line.find('%') == std::string::npos) && (line != "")) {
          if (header) {
            // Ignore header
            while (istream.good()) istream >> ignore;
            header = false;
          }
          while (istream.good()) {
            // Read vertices edges and weights
            istream >> v1 >> v2 >> weight;
            this->add_edge(v1, v2, weight, directed);
          }
        }
      }
    }
  } catch (std::exception& exception) {
    std::cout << "Read matrix market file: " << exception.what() << "\n";
  }
  std::cout << "Graph size: " << this->edges_.size() << "\n";
}

void Graph::generate_simple_graph(bool directed) {
  // set up a simple graph
  this->add_edge(1, 2, 7.5, directed);
  this->add_edge(1, 3, 9.1, directed);
  this->add_edge(1, 6, 14.3, directed);
  this->add_edge(2, 1, 0.9, directed);
  this->add_edge(2, 3, 5.9, directed);
  this->add_edge(2, 4, 15.5, directed);
  this->add_edge(3, 4, 11.6, directed);
  this->add_edge(3, 6, 2.4, directed);
  this->add_edge(4, 5, 6.2, directed);
  this->add_edge(5, 6, 9.7, directed);
}

// Compute shortest path using binary heap queue dijkstra
std::unordered_map<Graph::vertex_t, Graph::weight_t> Graph::dijkstra(
    Graph::vertex_t source, Graph::vertex_t dest) {
  std::unordered_map<Graph::vertex_t, Graph::vertex_t> prev;
  // parallel map and priority queue
  // {vertex, distance}
  std::unordered_map<Graph::vertex_t, Graph::weight_t> dist;
  std::vector<std::pair<Graph::vertex_t, Graph::weight_t>> dist_pq;
  std::unordered_map<Graph::vertex_t, Graph::weight_t> shortest_path;
  dist[source] = 0;
  dist_pq.push_back({source, 0});
  // already a heap with size 1

  const auto parent = [](size_t i) -> size_t {
    return i == 0 ? i : (i - 1) / 2;
  };
  const auto lchild = [](size_t i) { return 2 * i + 1; };
  const auto rchild = [](size_t i) { return 2 * i + 2; };

  const auto min_comp =
      [](const std::pair<Graph::vertex_t, Graph::weight_t>& d1,
         const std::pair<Graph::vertex_t, Graph::weight_t>& d2) {
        // by default it makes a max heap
        // so we reverse the operation
        return d1.second > d2.second;
      };

  const auto bubble =
      [&](std::vector<std::pair<Graph::vertex_t, Graph::weight_t>>& heap,
          size_t i) {
        size_t pi, lc, rc;
        // bubble up if it's compared wrongly with its parent
        while (true) {
          // condition testing
          pi = parent(i);
          if (pi == i || min_comp(heap[i], heap[pi])) {
            break;
          }
          swap(heap[i], heap[pi]);
          i = pi;
        }

        // bubble down if it's compared wrongly with either child
        while (true) {
          lc = lchild(i);
          rc = rchild(i);
          if (lc < heap.size()) {
            // left child exists
            if (min_comp(heap[lc], heap[i])) {
              // left child is fine
              if (rc < heap.size()) {
                // (left and) right children exist
                if (min_comp(heap[rc], heap[i])) {
                  // right child is fine
                  break;
                } else {
                  // right child is wrongly placed
                  swap(heap[i], heap[rc]);
                  i = rc;
                }
              } else {
                break;
              }
            } else {
              // left child is wrongly placed
              swap(heap[i], heap[lc]);
              i = lc;
            }
          } else {
            break;
          }
        }
        return i;
      };

  while (!dist_pq.empty()) {
    std::pop_heap(dist_pq.begin(), dist_pq.end(), min_comp);
    // u is current vertex
    Graph::vertex_t u = dist_pq.back().first;
    Graph::weight_t udist = dist_pq.back().second;
    dist_pq.pop_back();

    // break if destination is found
    if (dest == u) break;

    // neighbour edge e
    for (const std::shared_ptr<Graph::Edge>& edge : vertex_edges_[u]) {
      // *std::shared_ptr<Graph::Edge> is {{v1, v2}, weight}
      // neighbour
      const Graph::vertex_t neighbour =
          edge->first.first == u ? edge->first.second : edge->first.first;
      Graph::weight_t altdist = udist + edge->second;
      // if not checked or better path
      if (dist.find(neighbour) == dist.end() || altdist < dist[neighbour]) {
        prev[neighbour] = u;
        dist[neighbour] = altdist;
        if (dest == neighbour && dest != -1) shortest_path[neighbour] = altdist;
        // handle parallel priority queue
        auto pqit =
            find_if(dist_pq.begin(), dist_pq.end(),
                    [neighbour](std::pair<Graph::vertex_t, Graph::weight_t> p) {
                      return p.first == neighbour;
                    });

        if (pqit != dist_pq.end()) {
          pqit->second = altdist;
          // bubble/down
          bubble(dist_pq, pqit - dist_pq.begin());

        } else {
          dist_pq.push_back({neighbour, altdist});
          std::push_heap(dist_pq.begin(), dist_pq.end(), min_comp);
        }
      }
    }
  }
  return (dest != -1 ? shortest_path : dist);
}
