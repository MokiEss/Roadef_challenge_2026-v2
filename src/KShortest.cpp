#include "KShortest.h"
#include "ECMP.h"
#include <queue>
#include <set>
#include <algorithm>
#include <cassert>

using namespace std;

static constexpr long long KS_INF = numeric_limits<long long>::max() / 2;

// ── Dijkstra excluding both edges and nodes ───────────────────────
static vector<long long> dijkstra_with_ban(const Graph& g, int src,
    const unordered_set<int>& edge_ban,
    const unordered_set<int>& node_ban)
{
    vector<long long> dist(g.n, KS_INF);
    if (node_ban.count(src)) return dist;
    dist[src] = 0;
    using PQ = pair<long long, int>;
    priority_queue<PQ, vector<PQ>, greater<PQ>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        for (int eidx : g.out_adj[u]) {
            if (edge_ban.count(eidx)) continue;
            const Edge& e = g.edges[eidx];
            if (node_ban.count(e.to)) continue;
            long long nd = dist[u] + static_cast<long long>(e.metric);
            if (nd < dist[e.to]) { dist[e.to] = nd; pq.push({nd, e.to}); }
        }
    }
    return dist;
}

// ── Reconstruct one shortest path (lexicographically first) ──────
static Path reconstruct_path(const Graph& g, int src, int dst,
    const unordered_set<int>& edge_ban,
    const unordered_set<int>& node_ban)
{
    auto dist = dijkstra_with_ban(g, src, edge_ban, node_ban);
    if (dist[dst] == KS_INF) return {};

    Path rev;
    int cur = dst;
    while (cur != src) {
        rev.push_back(cur);
        int best = -1;
        for (int eidx : g.in_adj[cur]) {
            if (edge_ban.count(eidx)) continue;
            const Edge& e = g.edges[eidx];
            int u = e.from;
            if (node_ban.count(u)) continue;
            if (dist[u] == KS_INF) continue;
            long long cost = dist[u] + static_cast<long long>(e.metric);
            if (cost == dist[cur] && (best == -1 || u < best))
                best = u;
        }
        if (best == -1) return {};
        cur = best;
    }
    rev.push_back(src);
    reverse(rev.begin(), rev.end());
    return rev;
}

static long long path_cost(const Graph& g, const Path& p,
    const unordered_set<int>& edge_ban)
{
    if (p.size() < 2) return 0;
    long long cost = 0;
    for (int i = 0; i + 1 < (int)p.size(); ++i) {
        int u = p[i], v = p[i+1];
        long long best = KS_INF;
        for (int eidx : g.out_adj[u]) {
            if (edge_ban.count(eidx)) continue;
            if (g.edges[eidx].to == v)
                best = min(best, (long long)g.edges[eidx].metric);
        }
        if (best == KS_INF) return KS_INF;
        cost += best;
    }
    return cost;
}

// ─────────────────────────────────────────────────────────────────
// k_shortest_paths — Yen's algorithm with level grouping
// ─────────────────────────────────────────────────────────────────
KShortestResult k_shortest_paths(const Graph& g, int src, int dst, int k,
    const unordered_set<int>& disabled)
{
    KShortestResult result;
    if (k <= 0 || src == dst) return result;

    vector<pair<long long, Path>> accepted;

    using Candidate = pair<long long, Path>;
    auto cmp = [](const Candidate& a, const Candidate& b){ return a.first > b.first; };
    priority_queue<Candidate, vector<Candidate>, decltype(cmp)> heap(cmp);
    set<vector<int>> candidate_set;

    {
        unordered_set<int> empty;
        Path p = reconstruct_path(g, src, dst, disabled, empty);
        if (p.empty()) return result;
        long long c = path_cost(g, p, disabled);
        accepted.push_back({c, p});
    }

    int distinct_levels = 1;
    result.distances.push_back(accepted[0].first);
    result.paths_by_level.push_back({accepted[0].second});

    while (distinct_levels < k) {
        const auto& [prev_cost, prev_path] = accepted.back();

        for (int spur_idx = 0; spur_idx + 1 < (int)prev_path.size(); ++spur_idx) {
            int spur_node = prev_path[spur_idx];
            vector<int> root(prev_path.begin(), prev_path.begin() + spur_idx + 1);

            unordered_set<int> edge_ban = disabled;
            for (auto& [ac, ap] : accepted) {
                if ((int)ap.size() > spur_idx &&
                    vector<int>(ap.begin(), ap.begin() + spur_idx + 1) == root)
                {
                    int u = ap[spur_idx], v = ap[spur_idx + 1];
                    for (int eidx : g.out_adj[u])
                        if (g.edges[eidx].to == v) edge_ban.insert(eidx);
                }
            }

            unordered_set<int> node_ban;
            for (int i = 0; i < spur_idx; ++i) node_ban.insert(root[i]);

            Path spur = reconstruct_path(g, spur_node, dst, edge_ban, node_ban);
            if (spur.empty()) continue;

            Path full = root;
            full.pop_back();
            full.insert(full.end(), spur.begin(), spur.end());

            unordered_set<int> seen;
            bool simple = true;
            for (int n : full) {
                if (!seen.insert(n).second) { simple = false; break; }
            }
            if (!simple) continue;
            if (candidate_set.count(full)) continue;
            candidate_set.insert(full);

            long long c = path_cost(g, full, disabled);
            if (c < KS_INF) heap.push({c, move(full)});
        }

        if (heap.empty()) break;

        auto [best_cost, best_path] = heap.top();
        heap.pop();
        candidate_set.erase(best_path);
        accepted.push_back({best_cost, best_path});

        if (best_cost != result.distances.back()) {
            result.distances.push_back(best_cost);
            result.paths_by_level.push_back({best_path});
            ++distinct_levels;
        } else {
            result.paths_by_level.back().push_back(best_path);
        }
    }

    return result;
}
