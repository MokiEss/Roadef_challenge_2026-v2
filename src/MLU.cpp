#include "MLU.h"
#include <numeric>
#include <thread>
#include <atomic>
#include <algorithm>
#include <iomanip>

using namespace std;

// ─────────────────────────────────────────────────────────────────
// compute_mlu
// ─────────────────────────────────────────────────────────────────
MLUState compute_mlu(const NetworkContext& ctx, MLUMode mode) {
    const Graph& g  = ctx.graph;
    const auto&  D  = ctx.demands;
    const int    T  = ctx.num_slots;
    const int    E  = g.m;
    const int    ND = static_cast<int>(D.size());

    MLUState s;
    s.num_slots = T;
    s.num_edges = E;
    s.util.assign(T, vector<double>(E, 0.0));
    s.load.assign(T, vector<double>(E, 0.0));
    s.contrib.resize(T, vector<vector<pair<int,double>>>(E));
    s.mlu_per_slot.assign(T, 0.0);
    s.max_util_per_arc.assign(E, 0.0);

    vector<unordered_set<int>> disabled(T);
    for (int t = 0; t < T; ++t) disabled[t] = ctx.scenario.disabled_at(t);

    struct VecHash {
        size_t operator()(const vector<int>& v) const noexcept {
            size_t seed = v.size();
            for (int x : v) seed ^= size_t(x) + 0x9e3779b9u + (seed<<6) + (seed>>2);
            return seed;
        }
    };

    auto process_slot_sequential = [&](int t) {
        vector<bool> dis_vec(E, false);
        for (int eidx : disabled[t]) dis_vec[eidx] = true;
        unordered_map<int, BwdDijkstraResult> bwd_cache;
        unordered_map<vector<int>, ECMPResult, VecHash> ecmp_cache;

        for (int d = 0; d < ND; ++d) {
            const Demand& dem = D[d];
            const double  vol = dem.volume[t];
            if (vol == 0.0) continue;
            const vector<int>& wp = dem.waypoints_at(t);
            auto [it, ins] = ecmp_cache.emplace(wp, ECMPResult{});
            if (ins) it->second = compute_ecmp_topo_cached(g, wp, dis_vec, bwd_cache);
            const ECMPResult& ec = it->second;
            if (!ec.reachable) continue;
            for (int e = 0; e < E; ++e) {
                if (ec.edge_fraction[e] == 0.0) continue;
                double flow = ec.edge_fraction[e] * vol;
                s.load[t][e]  += flow;
                s.contrib[t][e].emplace_back(d, flow);
            }
        }
        for (int e = 0; e < E; ++e) {
            s.util[t][e]      = s.load[t][e] / g.edges[e].capacity;
            s.mlu_per_slot[t] = max(s.mlu_per_slot[t], s.util[t][e]);
        }
    };

    if (mode == MLUMode::Sequential) {
        for (int t = 0; t < T; ++t) process_slot_sequential(t);
    }
    else if (mode == MLUMode::Parallel) {
        const int N = static_cast<int>(thread::hardware_concurrency());
        atomic<int> next_slot{0};
        vector<jthread> threads;
        threads.reserve(N);
        for (int i = 0; i < N; ++i)
            threads.emplace_back([&]() {
                for (int t = next_slot.fetch_add(1, memory_order_relaxed);
                     t < T;
                     t = next_slot.fetch_add(1, memory_order_relaxed))
                    process_slot_sequential(t);
            });
    }
    else { // FullParallel
        const int N     = static_cast<int>(thread::hardware_concurrency());
        const int chunk = max(1, (ND + N - 1) / N);
        const int W     = min(N, (ND + chunk - 1) / chunk);

        struct PBuf { vector<double> load; vector<vector<pair<int,double>>> contrib; };
        vector<PBuf> partial(W * T);
        for (auto& pb : partial) { pb.load.assign(E,0.0); pb.contrib.resize(E); }

        {
            vector<jthread> threads;
            threads.reserve(W);
            for (int w = 0; w < W; ++w) {
                int d0 = w * chunk, d1 = min(d0 + chunk, ND);
                threads.emplace_back([&, w, d0, d1]() {
                    vector<bool> dis_vec(E, false);
                    unordered_map<int, BwdDijkstraResult> bwd_cache;
                    unordered_map<vector<int>, ECMPResult, VecHash> ecmp_cache;
                    for (int t = 0; t < T; ++t) {
                        fill(dis_vec.begin(), dis_vec.end(), false);
                        for (int eidx : disabled[t]) dis_vec[eidx] = true;
                        bwd_cache.clear(); ecmp_cache.clear();
                        PBuf& pb = partial[w*T + t];
                        for (int d = d0; d < d1; ++d) {
                            const Demand& dem = D[d];
                            double vol = dem.volume[t];
                            if (vol == 0.0) continue;
                            const vector<int>& wp = dem.waypoints_at(t);
                            auto [it, ins] = ecmp_cache.emplace(wp, ECMPResult{});
                            if (ins) it->second = compute_ecmp_topo_cached(g, wp, dis_vec, bwd_cache);
                            const ECMPResult& ec = it->second;
                            if (!ec.reachable) continue;
                            for (int e = 0; e < E; ++e) {
                                if (ec.edge_fraction[e] == 0.0) continue;
                                double flow = ec.edge_fraction[e] * vol;
                                pb.load[e] += flow;
                                pb.contrib[e].emplace_back(d, flow);
                            }
                        }
                    }
                });
            }
        }
        for (int t = 0; t < T; ++t) {
            for (int w = 0; w < W; ++w) {
                PBuf& pb = partial[w*T+t];
                for (int e = 0; e < E; ++e) {
                    s.load[t][e] += pb.load[e];
                    for (auto& kv : pb.contrib[e]) s.contrib[t][e].push_back(kv);
                }
            }
            for (int e = 0; e < E; ++e) {
                s.util[t][e]      = s.load[t][e] / g.edges[e].capacity;
                s.mlu_per_slot[t] = max(s.mlu_per_slot[t], s.util[t][e]);
            }
        }
    }

    for (int e = 0; e < E; ++e) {
        for (int t = 0; t < T; ++t)
            s.max_util_per_arc[e] = max(s.max_util_per_arc[e], s.util[t][e]);
        s.global_mlu = max(s.global_mlu, s.max_util_per_arc[e]);
    }
    return s;
}

// ─────────────────────────────────────────────────────────────────
// get_congested_arcs
// ─────────────────────────────────────────────────────────────────
vector<CongestedArc> get_congested_arcs(const MLUState& s, int time_slot) {
    vector<CongestedArc> result;
    const int T = s.num_slots, E = s.num_edges;
    int t0 = (time_slot < 0) ? 0 : time_slot;
    int t1 = (time_slot < 0) ? T-1 : time_slot;
    for (int t = t0; t <= t1; ++t) {
        double sum = 0.0;
        for (int e = 0; e < E; ++e) sum += s.util[t][e];
        double mean = sum / static_cast<double>(E);
        for (int e = 0; e < E; ++e)
            if (s.util[t][e] >= mean)
                result.push_back({t, e, s.util[t][e], mean});
    }
    sort(result.begin(), result.end(),
         [](const CongestedArc& a, const CongestedArc& b){ return a.utilisation > b.utilisation; });
    return result;
}

vector<ArcContributor> get_arc_contributors(const NetworkContext& ctx,
    const MLUState& s, int edge_id, int time_slot)
{
    const double cap = ctx.graph.edges[edge_id].capacity;
    const double arc_load = s.load_at(time_slot, edge_id);
    vector<ArcContributor> result;
    for (auto& [d, flow] : s.contributors_at(time_slot, edge_id))
        result.push_back({d, ctx.demands[d].src, ctx.demands[d].dst,
                          flow, flow/cap, (arc_load>0.0)?flow/arc_load:0.0});
    sort(result.begin(), result.end(),
         [](const ArcContributor& a, const ArcContributor& b){ return a.flow > b.flow; });
    return result;
}

unordered_map<int, vector<ArcContributor>> get_arcs_contributors(
    const NetworkContext& ctx, const MLUState& s,
    const vector<int>& edge_ids, int time_slot)
{
    unordered_map<int, vector<ArcContributor>> result;
    for (int eid : edge_ids) result[eid] = get_arc_contributors(ctx, s, eid, time_slot);
    return result;
}

// ─────────────────────────────────────────────────────────────────
// Pretty-print helpers
// ─────────────────────────────────────────────────────────────────
void print_mlu_summary(const NetworkContext& ctx, const MLUState& s) {
    cout << fixed << setprecision(4);
    for (int t = 0; t < s.num_slots; ++t) {
        cout << "\n--- Slot " << t << "  MLU=" << s.mlu_per_slot[t] << " ---\n";
        for (int e = 0; e < s.num_edges; ++e) {
            if (s.util[t][e] == 0.0) continue;
            const Edge& arc = ctx.graph.edges[e];
            cout << "  e" << setw(3) << e << "  v" << arc.from << "->v" << arc.to
                 << "  load=" << setw(9) << s.load[t][e]
                 << "  cap=" << arc.capacity << "  util=" << s.util[t][e] << "\n";
        }
    }
    cout << "\nGlobal MLU: " << s.global_mlu << "\n";
}

void print_congested_arcs(const NetworkContext& ctx, const vector<CongestedArc>& arcs) {
    cout << fixed << setprecision(4) << "\n--- Congested arcs (util >= mean) ---\n";
    for (const auto& ca : arcs) {
        const Edge& arc = ctx.graph.edges[ca.edge_id];
        cout << "  slot=" << ca.time_slot << "  e" << ca.edge_id
             << "  v" << arc.from << "->v" << arc.to
             << "  util=" << ca.utilisation << "  mean=" << ca.mean_util << "\n";
    }
}

void print_arcs_sorted_by_util(const NetworkContext& ctx, const MLUState& s) {
    cout << fixed << setprecision(4);
    for (int t = 0; t < s.num_slots; ++t) {
        double sum = 0.0;
        for (int e = 0; e < s.num_edges; ++e) sum += s.util[t][e];
        double mean = sum / static_cast<double>(s.num_edges);
        vector<int> order(s.num_edges);
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b){ return s.util[t][a] > s.util[t][b]; });
        cout << "\n--- Arc util  slot=" << t << "  MLU=" << s.mlu_per_slot[t]
             << "  mean=" << mean << " ---\n";
        for (int e : order) {
            if (s.util[t][e] == 0.0) break;
            const Edge& arc = ctx.graph.edges[e];
            cout << "  e" << setw(3) << e << "  v" << setw(2) << arc.from
                 << "->v" << setw(2) << arc.to << "  util=" << s.util[t][e]
                 << (s.util[t][e] >= mean ? "  *" : "") << "\n";
        }
    }
}

void print_arc_contributors(const NetworkContext& ctx,
    const vector<ArcContributor>& cs, int edge_id, int time_slot)
{
    const Edge& arc = ctx.graph.edges[edge_id];
    cout << fixed << setprecision(4) << "\n--- Contributors to e" << edge_id
         << " (v" << arc.from << "->v" << arc.to << ") slot=" << time_slot << " ---\n";
    for (const auto& c : cs)
        cout << "  d" << c.demand_id
             << "  " << ctx.graph.nodes[c.src].name << "->" << ctx.graph.nodes[c.dst].name
             << "  flow=" << c.flow
             << "  util_share=" << c.utilisation_share
             << "  arc%=" << c.pct_of_arc_load * 100.0 << "%\n";
}
