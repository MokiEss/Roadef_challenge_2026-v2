#pragma once
#include "Graph.h"
#include <unordered_set>
#include <vector>
#include <limits>

// ─────────────────────────────────────────────────────────────────
// K-Shortest Simple Paths — Yen's algorithm
//
// Returns all simple (loop-free) paths from src to dst, grouped by
// distinct distance level.  For each level ALL paths of that cost
// are returned.  Stops after k distinct levels (or all paths exhausted).
// ─────────────────────────────────────────────────────────────────

using Path = std::vector<int>;   // sequence of node IDs

struct KShortestResult {
    // paths_by_level[i] = all paths of the i-th distinct distance level
    std::vector<std::vector<Path>> paths_by_level;
    std::vector<long long>         distances;      // one per level
};

KShortestResult k_shortest_paths(const Graph& g, int src, int dst, int k,
                                  const std::unordered_set<int>& disabled);
