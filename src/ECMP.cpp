#include "ECMP.h"
#include <queue>
#include <algorithm>
#include <cassert>

using namespace std;

// ─────────────────────────────────────────────────────────────────
// Forward Dijkstra
// ─────────────────────────────────────────────────────────────────
vector<long long> dijkstra_forward(const Graph& g, int src,
    const unordered_set<int>& disabled)
{
    vector<long long> dist(g.n, ECMP_INF);
    dist[src] = 0;
    using PQ = pair<long long, int>;
    priority_queue<PQ, vector<PQ>, greater<PQ>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        for (int eidx : g.out_adj[u]) {
            if (disabled.count(eidx)) continue;
            const Edge& e = g.edges[eidx];
            long long nd = dist[u] + static_cast<long long>(e.metric);
            if (nd < dist[e.to]) { dist[e.to] = nd; pq.push({nd, e.to}); }
        }
    }
    return dist;
}

// ─────────────────────────────────────────────────────────────────
// Backward Dijkstra (reverse graph)
// ─────────────────────────────────────────────────────────────────
vector<long long> dijkstra_backward(const Graph& g, int dst,
    const unordered_set<int>& disabled)
{
    vector<long long> dist(g.n, ECMP_INF);
    dist[dst] = 0;
    using PQ = pair<long long, int>;
    priority_queue<PQ, vector<PQ>, greater<PQ>> pq;
    pq.push({0, dst});
    while (!pq.empty()) {
        auto [d, v] = pq.top(); pq.pop();
        if (d > dist[v]) continue;
        for (int eidx : g.in_adj[v]) {
            if (disabled.count(eidx)) continue;
            const Edge& e = g.edges[eidx];
            long long nd = dist[v] + static_cast<long long>(e.metric);
            if (nd < dist[e.from]) { dist[e.from] = nd; pq.push({nd, e.from}); }
        }
    }
    return dist;
}

// ── Helper: fill sp_out_deg and topo_order after Dijkstra ────────
template<typename IsDisabled>
static void fill_sp_meta(const Graph& g, BwdDijkstraResult& res, IsDisabled is_disabled)
{
    res.sp_out_deg.assign(g.n, 0);
    for (int eidx = 0; eidx < g.m; ++eidx) {
        if (is_disabled(eidx)) continue;
        const Edge& e = g.edges[eidx];
        if (res.dist[e.from] == ECMP_INF || res.dist[e.to] == ECMP_INF) continue;
        if (res.dist[e.from] == res.dist[e.to] + static_cast<long long>(e.metric))
            ++res.sp_out_deg[e.from];
    }
    res.topo_order.clear();
    res.topo_order.reserve(g.n);
    for (int v = 0; v < g.n; ++v)
        if (res.dist[v] < ECMP_INF)
            res.topo_order.push_back(v);
    sort(res.topo_order.begin(), res.topo_order.end(),
         [&res](int a, int b){ return res.dist[a] > res.dist[b]; });
}

// ─────────────────────────────────────────────────────────────────
// Full backward Dijkstra (unordered_set disabled)
// ─────────────────────────────────────────────────────────────────
BwdDijkstraResult dijkstra_backward_full(const Graph& g, int dst,
    const unordered_set<int>& disabled)
{
    BwdDijkstraResult res;
    res.dist = dijkstra_backward(g, dst, disabled);
    fill_sp_meta(g, res, [&](int e){ return disabled.count(e) != 0; });
    return res;
}

// ─────────────────────────────────────────────────────────────────
// Full backward Dijkstra (vector<bool> disabled — O(1) lookup)
// ─────────────────────────────────────────────────────────────────
BwdDijkstraResult dijkstra_backward_full(const Graph& g, int dst,
    const vector<bool>& disabled_vec)
{
    BwdDijkstraResult res;
    res.dist.assign(g.n, ECMP_INF);
    res.dist[dst] = 0;
    using PQ = pair<long long, int>;
    priority_queue<PQ, vector<PQ>, greater<PQ>> pq;
    pq.push({0, dst});
    while (!pq.empty()) {
        auto [d, v] = pq.top(); pq.pop();
        if (d > res.dist[v]) continue;
        for (int eidx : g.in_adj[v]) {
            if (disabled_vec[eidx]) continue;
            const Edge& e = g.edges[eidx];
            long long nd = res.dist[v] + static_cast<long long>(e.metric);
            if (nd < res.dist[e.from]) { res.dist[e.from] = nd; pq.push({nd, e.from}); }
        }
    }
    fill_sp_meta(g, res, [&](int e){ return disabled_vec[e]; });
    return res;
}

// ─────────────────────────────────────────────────────────────────
// ECMP segment: BFS on SP-DAG (priority-queue based)
// ─────────────────────────────────────────────────────────────────
bool ecmp_segment_bwd_only(const Graph& g, int s,
    const BwdDijkstraResult& bwd,
    const unordered_set<int>& disabled,
    vector<double>& frac)
{
    if (bwd.dist[s] == ECMP_INF) return false;

    vector<double> flow_at(g.n, 0.0);
    flow_at[s] = 1.0;

    vector<bool> pushed(g.n, false);
    using PQ = pair<long long, int>;
    priority_queue<PQ, vector<PQ>, greater<PQ>> pq;
    pq.push({-bwd.dist[s], s});
    pushed[s] = true;

    while (!pq.empty()) {
        auto [neg_d, v] = pq.top(); pq.pop();
        const int fan_out = bwd.sp_out_deg[v];
        if (fan_out == 0 || flow_at[v] == 0.0) continue;
        const double split = flow_at[v] / static_cast<double>(fan_out);
        for (int eidx : g.out_adj[v]) {
            if (disabled.count(eidx)) continue;
            const Edge& e = g.edges[eidx];
            if (bwd.dist[e.to] == ECMP_INF) continue;
            if (bwd.dist[v] != bwd.dist[e.to] + static_cast<long long>(e.metric)) continue;
            frac[eidx] += split;
            flow_at[e.to] += split;
            if (!pushed[e.to]) { pushed[e.to] = true; pq.push({-bwd.dist[e.to], e.to}); }
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
// ECMP segment: topological-order BFS — O(V+E), thread-local scratch
// ─────────────────────────────────────────────────────────────────
bool ecmp_segment_topo(const Graph& g, int s,
    const BwdDijkstraResult& bwd,
    const vector<bool>& disabled_vec,
    vector<double>& frac)
{
    if (bwd.dist[s] == ECMP_INF) return false;

    thread_local vector<double> flow_at;
    thread_local vector<int>    dirty;

    if ((int)flow_at.size() < g.n) flow_at.assign(g.n, 0.0);
    for (int v : dirty) flow_at[v] = 0.0;
    dirty.clear();

    flow_at[s] = 1.0;
    dirty.push_back(s);

    for (int v : bwd.topo_order) {
        if (flow_at[v] == 0.0) continue;
        const int fan_out = bwd.sp_out_deg[v];
        if (fan_out == 0) continue;
        const double split = flow_at[v] / static_cast<double>(fan_out);
        for (int eidx : g.out_adj[v]) {
            if (disabled_vec[eidx]) continue;
            const Edge& e = g.edges[eidx];
            if (bwd.dist[e.to] == ECMP_INF) continue;
            if (bwd.dist[v] != bwd.dist[e.to] + static_cast<long long>(e.metric)) continue;
            frac[eidx] += split;
            if (flow_at[e.to] == 0.0) dirty.push_back(e.to);
            flow_at[e.to] += split;
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
// Uncached ECMP
// ─────────────────────────────────────────────────────────────────
ECMPResult compute_ecmp(const Graph& g, const vector<int>& waypoints,
    const unordered_set<int>& disabled)
{
    assert(waypoints.size() >= 2);
    ECMPResult res;
    res.edge_fraction.assign(g.m, 0.0);
    for (int seg = 0; seg + 1 < (int)waypoints.size(); ++seg) {
        BwdDijkstraResult bwd = dijkstra_backward_full(g, waypoints[seg+1], disabled);
        if (!ecmp_segment_bwd_only(g, waypoints[seg], bwd, disabled, res.edge_fraction)) {
            res.reachable = false;
            fill(res.edge_fraction.begin(), res.edge_fraction.end(), 0.0);
            return res;
        }
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────
// Cached ECMP (unordered_set)
// ─────────────────────────────────────────────────────────────────
ECMPResult compute_ecmp_cached(const Graph& g, const vector<int>& waypoints,
    const unordered_set<int>& disabled,
    unordered_map<int, BwdDijkstraResult>& bwd_cache)
{
    assert(waypoints.size() >= 2);
    ECMPResult res;
    res.edge_fraction.assign(g.m, 0.0);
    for (int seg = 0; seg + 1 < (int)waypoints.size(); ++seg) {
        const int dst = waypoints[seg+1];
        auto bit = bwd_cache.find(dst);
        if (bit == bwd_cache.end())
            bit = bwd_cache.emplace(dst, dijkstra_backward_full(g, dst, disabled)).first;
        if (!ecmp_segment_bwd_only(g, waypoints[seg], bit->second, disabled, res.edge_fraction)) {
            res.reachable = false;
            fill(res.edge_fraction.begin(), res.edge_fraction.end(), 0.0);
            return res;
        }
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────
// Faster cached ECMP (vector<bool> + topo BFS)
// ─────────────────────────────────────────────────────────────────
ECMPResult compute_ecmp_topo_cached(const Graph& g, const vector<int>& waypoints,
    const vector<bool>& disabled_vec,
    unordered_map<int, BwdDijkstraResult>& bwd_cache)
{
    assert(waypoints.size() >= 2);
    ECMPResult res;
    res.edge_fraction.assign(g.m, 0.0);
    for (int seg = 0; seg + 1 < (int)waypoints.size(); ++seg) {
        const int dst = waypoints[seg+1];
        auto bit = bwd_cache.find(dst);
        if (bit == bwd_cache.end())
            bit = bwd_cache.emplace(dst, dijkstra_backward_full(g, dst, disabled_vec)).first;
        if (!ecmp_segment_topo(g, waypoints[seg], bit->second, disabled_vec, res.edge_fraction)) {
            res.reachable = false;
            fill(res.edge_fraction.begin(), res.edge_fraction.end(), 0.0);
            return res;
        }
    }
    return res;
}
