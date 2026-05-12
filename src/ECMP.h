#pragma once
#include "Graph.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <limits>

// ─────────────────────────────────────────────────────────────────
// ECMP algorithms: Dijkstra and flow computation
// ─────────────────────────────────────────────────────────────────

static constexpr long long ECMP_INF = std::numeric_limits<long long>::max() / 2;

// Forward Dijkstra from src
std::vector<long long> dijkstra_forward(const Graph& g, int src,
    const std::unordered_set<int>& disabled);

// Backward Dijkstra from dst (reverse graph)
std::vector<long long> dijkstra_backward(const Graph& g, int dst,
    const std::unordered_set<int>& disabled);

// Full backward Dijkstra result: dist[], SP-DAG out-degree, topological order
struct BwdDijkstraResult {
    std::vector<long long> dist;        // dist[v] = distance v → dst
    std::vector<int>       sp_out_deg;  // SP-DAG fan-out per node
    std::vector<int>       topo_order;  // nodes sorted by dist descending
};

BwdDijkstraResult dijkstra_backward_full(const Graph& g, int dst,
    const std::unordered_set<int>& disabled);

BwdDijkstraResult dijkstra_backward_full(const Graph& g, int dst,
    const std::vector<bool>& disabled_vec);

// ECMP fraction vector: edge_fraction[e] = fraction of demand flow on e
struct ECMPResult {
    std::vector<double> edge_fraction;   // length = g.m
    bool reachable = true;
};

// Uncached: allocates one BwdDijkstraResult per destination
ECMPResult compute_ecmp(const Graph& g, const std::vector<int>& waypoints,
    const std::unordered_set<int>& disabled);

// Cached: reuses bwd_cache across calls (one Dijkstra per unique destination)
ECMPResult compute_ecmp_cached(const Graph& g, const std::vector<int>& waypoints,
    const std::unordered_set<int>& disabled,
    std::unordered_map<int, BwdDijkstraResult>& bwd_cache);

// Forward BFS on SP-DAG (priority-queue based)
bool ecmp_segment_bwd_only(const Graph& g, int s,
    const BwdDijkstraResult& bwd,
    const std::unordered_set<int>& disabled,
    std::vector<double>& frac);

// Forward BFS on SP-DAG (topological-order, O(V+E), thread-local scratch)
bool ecmp_segment_topo(const Graph& g, int s,
    const BwdDijkstraResult& bwd,
    const std::vector<bool>& disabled_vec,
    std::vector<double>& frac);

// Faster cached ECMP: topo BFS + vector<bool> disabled
ECMPResult compute_ecmp_topo_cached(const Graph& g, const std::vector<int>& waypoints,
    const std::vector<bool>& disabled_vec,
    std::unordered_map<int, BwdDijkstraResult>& bwd_cache);
