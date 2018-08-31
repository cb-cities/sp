#include <algorithm>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

using vertex_t = int;     // some sort of id
using weight_t = double;  // some type that can be added with +
// {{v1, v2}, weight}
using Edge = std::pair<std::pair<vertex_t, vertex_t>, weight_t>;

int main(int argc, char** argv) {

  // dijkstra's algorithm with a heap priority queue
  // adjacency list with iteration over each edge
  std::vector<std::shared_ptr<Edge>> edges;
  std::unordered_map<vertex_t, std::vector<std::shared_ptr<Edge>>> vertex_edges;

  const auto add_edge = [&](std::shared_ptr<Edge> edge) -> void {
    edges.push_back(edge);
    std::vector<std::shared_ptr<Edge>>& v1edge = vertex_edges[edge->first.first];
    std::vector<std::shared_ptr<Edge>>& v2edge = vertex_edges[edge->first.second];
    v1edge.push_back(edge);
    v2edge.push_back(edge);
  };

  const auto remove_edge = [&](std::shared_ptr<Edge> edge) -> void {
    edges.erase(remove(edges.begin(), edges.end(), edge), edges.end());
    std::vector<std::shared_ptr<Edge>>& v1edge = vertex_edges[edge->first.first];
    std::vector<std::shared_ptr<Edge>>& v2edge = vertex_edges[edge->first.second];
    v1edge.erase(remove(v1edge.begin(), v1edge.end(), edge), v1edge.end());
    v2edge.erase(remove(v2edge.begin(), v2edge.end(), edge), v2edge.end());
  };

  const auto make_edge = [](vertex_t vertex1, vertex_t vertex2,
                            weight_t weight) -> std::shared_ptr<Edge> {
    if (vertex1 > vertex2) std::swap(vertex1, vertex2);
    return std::make_shared<Edge>(
        std::make_pair(std::make_pair(vertex1, vertex2), weight));
  };

  // set up a simple graph
  add_edge(make_edge(1, 2, 7.5));
  add_edge(make_edge(1, 3, 9.1));
  add_edge(make_edge(1, 6, 14.3));
  add_edge(make_edge(2, 3, 10.9));
  add_edge(make_edge(2, 4, 15.5));
  add_edge(make_edge(3, 4, 11.6));
  add_edge(make_edge(3, 6, 2.4));
  add_edge(make_edge(4, 5, 6.2));
  add_edge(make_edge(5, 6, 9.7));

  // source vertex
  const vertex_t source = 1;
  const vertex_t dest = -1;
  // const vertex_t dest = 5;

  std::unordered_map<vertex_t, vertex_t> prev;
  // parallel map and priority queue
  // {vertex, distance}
  std::unordered_map<vertex_t, weight_t> dist;
  std::vector<std::pair<vertex_t, weight_t>> dist_pq;
  dist[source] = 0;
  dist_pq.push_back({source, 0});
  // already a heap with size 1

  const auto parent = [](size_t i) -> size_t {
    return i == 0 ? i : (i - 1) / 2;
  };
  const auto lchild = [](size_t i) { return 2 * i + 1; };
  const auto rchild = [](size_t i) { return 2 * i + 2; };

  const auto min_comp = [](const std::pair<vertex_t, weight_t>& d1,
                           const std::pair<vertex_t, weight_t>& d2) {
    // by default it makes a max heap
    // so we reverse the operation
    return d1.second > d2.second;
  };

  const auto bubble = [&](std::vector<std::pair<vertex_t, weight_t>>& heap,
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
    // u is current vertex
    pop_heap(dist_pq.begin(), dist_pq.end(), min_comp);
    vertex_t u = dist_pq.back().first;
    weight_t udist = dist_pq.back().second;
    dist_pq.pop_back();

    // BREAK HERE IF DESTINATION FOUND
    if (dest == u) break;

    // neighbor edge e
    for (const std::shared_ptr<Edge>& edge : vertex_edges[u]) {
      // *std::shared_ptr<Edge> is {{v1, v2}, weight}
      // neighbor
      const vertex_t neighbor =
          edge->first.first == u ? edge->first.second : edge->first.first;
      weight_t altdist = udist + edge->second;
      // if not checked or better path
      if (dist.find(neighbor) == dist.end() || altdist < dist[neighbor]) {
        prev[neighbor] = u;
        dist[neighbor] = altdist;

        // handle parallel priority queue
        auto pqit = find_if(dist_pq.begin(), dist_pq.end(),
                            [&neighbor](std::pair<vertex_t, weight_t> p) {
                              return p.first == neighbor;
                            });

        if (pqit != dist_pq.end()) {
          pqit->second = altdist;
          // bubble/down
          bubble(dist_pq, pqit - dist_pq.begin());

        } else {
          dist_pq.push_back({neighbor, altdist});
          push_heap(dist_pq.begin(), dist_pq.end(), min_comp);
        }
      }
    }
  }

  // for each node n:
  // if algorithm doesn't stop after finding path to destination
  //     dist[n] contains the distance to it
  // prev[n] contains the node before it
  // building a path requires calling prev[n] iteratively
  // and pushing the result onto a stack

  for (const std::pair<vertex_t, weight_t>& p : dist) {
    std::cout << p.first << " dist " << p.second << std::endl;
  }
}
