#pragma once
#include "main_header.h"

// ─────────────────────────────────────────────────────────────────
// Graph data structures
// ─────────────────────────────────────────────────────────────────

struct Node {
    int         id;
    std::string name;
};

struct Edge {
    int    id;
    int    from, to;
    double metric;
    double capacity;
};

struct Graph {
    int n = 0, m = 0;
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    std::vector<std::vector<int>> out_adj;   // [u] → list of edge indices
    std::vector<std::vector<int>> in_adj;    // [v] → list of edge indices

    std::unordered_map<int, int>       node_id_map;  // external_id → internal
    std::unordered_map<long long, int> edge_map;     // u*n+v → edge idx

    const Edge& edge(int idx)    const { return edges[idx]; }
    const Node& node_at(int idx) const { return nodes[idx]; }
    int out_degree(int u)        const { return (int)out_adj[u].size(); }

    // Returns edge index for (u,v) or -1 if not found
    int find_edge(int u, int v) const {
        auto it = edge_map.find(static_cast<long long>(u) * n + v);
        return (it != edge_map.end()) ? it->second : -1;
    }
};
