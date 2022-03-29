/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace common {

template <typename T = int, typename Hash = std::hash<T>>
class Dijkstra {
  struct DijkstraVertex {
    T index;
    T parent;  // same as index if not assigned
    std::vector<std::pair<T, double>>
        edges;  // might repeat, users' reasonability
    double dist = std::numeric_limits<double>::infinity();

    DijkstraVertex() = default;
    DijkstraVertex(const T _index) : index(_index), parent(_index) {}
  };

 public:
  Dijkstra() = default;

  void AddEdge(const T& from, const T& to, const double weight);
  bool FindPath(const T& source, const T& target, std::vector<T>* path);

 private:
  std::unordered_map<T, DijkstraVertex, Hash> graph_;
};

template <typename T, typename Hash>
void Dijkstra<T, Hash>::AddEdge(const T& from, const T& to,
                                const double weight) {
  auto it_from = graph_.find(from);

  if (it_from == graph_.end()) {
    DijkstraVertex vertex(from);
    vertex.edges.emplace_back(std::pair<T, double>(to, weight));
    graph_[from] = vertex;
  } else {
    it_from->second.edges.emplace_back(std::pair<T, double>(to, weight));
  }

  if (graph_.find(to) == graph_.end()) {
    graph_[to] = DijkstraVertex(to);
  }
}

template <typename T, typename Hash>
bool Dijkstra<T, Hash>::FindPath(const T& source, const T& target,
                                 std::vector<T>* path) {
  typedef typename std::pair<T, double> Edge;
  typedef typename std::vector<std::pair<T, double>> Edges;

  if (graph_.find(source) == graph_.end() ||
      graph_.find(target) == graph_.end()) {
    fprintf(stderr, "source vertex or target vertex is not in the graph\n");
    return false;
  } else if (source == target) {
    fprintf(stderr, "source and target is same\n");
    return false;
  }

  auto Compare = [](const Edge& e1, const Edge& e2) -> bool {
    return e1.second > e2.second;
  };
  std::priority_queue<Edge, Edges, decltype(Compare)> unvisited_set(Compare);
  unvisited_set.push(Edge(source, 0));

  graph_[source].dist = 0;

  while (!unvisited_set.empty()) {
    T current_index = unvisited_set.top().first;
    if (current_index == target) break;
    double current_dist = unvisited_set.top().second;
    unvisited_set.pop();

    const auto& vertex = graph_[current_index];
    for (const auto& edge : vertex.edges) {
      auto& next_vertex = graph_[edge.first];
      if (current_dist + edge.second < next_vertex.dist) {
        next_vertex.dist = current_dist + edge.second;
        unvisited_set.push(Edge(next_vertex.index, next_vertex.dist));
        next_vertex.parent = current_index;
      }
    }
  }

  if (graph_[target].dist == std::numeric_limits<double>::max()) {
    fprintf(stderr, "cannot find a path\n");
    return false;
  }

  path->emplace_back(target);
  while (true) {
    const auto& current_vertex = graph_[path->back()];
    if (current_vertex.index != current_vertex.parent) {
      path->emplace_back(current_vertex.parent);
    } else {
      break;
    }
  }

  std::reverse(path->begin(), path->end());
  return true;
}
}  // namespace common
