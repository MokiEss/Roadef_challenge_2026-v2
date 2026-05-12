#include "Debug.h"
#include "Solution.h"


#include <fstream>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <cstdlib>

using namespace std;
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────
// get_list_congest_arcs
// ─────────────────────────────────────────────────────────────────
vector<CongestedArc> get_list_congest_arcs(const NetworkContext& ctx,
    const MLUState& mlu, int time_slot)
{
    return get_congested_arcs(mlu, time_slot);
}

// ─────────────────────────────────────────────────────────────────
// get_mlu_of_all_arcs
// ─────────────────────────────────────────────────────────────────
vector<ArcUtil> get_mlu_of_all_arcs(const NetworkContext& ctx,
    const MLUState& mlu, ArcSortOrder sort_order, int time_slot)
{
    const int T = mlu.num_slots, E = mlu.num_edges;
    int t0 = (time_slot < 0) ? 0 : time_slot;
    int t1 = (time_slot < 0) ? T-1 : time_slot;

    vector<ArcUtil> result;
    result.reserve((t1-t0+1)*E);

    for (int t = t0; t <= t1; ++t)
        for (int e = 0; e < E; ++e) {
            const Edge& arc = ctx.graph.edges[e];
            result.push_back({t, e, arc.from, arc.to,
                               mlu.util[t][e], arc.capacity, mlu.load[t][e]});
        }

    if (sort_order == ArcSortOrder::DecreasingUtil)
        sort(result.begin(), result.end(),
             [](const ArcUtil& a, const ArcUtil& b){ return a.utilisation > b.utilisation; });
    else // BySlotThenDecreasing
        sort(result.begin(), result.end(),
             [](const ArcUtil& a, const ArcUtil& b){
                 if (a.time_slot != b.time_slot) return a.time_slot < b.time_slot;
                 return a.utilisation > b.utilisation;
             });
    return result;
}

// ─────────────────────────────────────────────────────────────────
// get_top_congested_arcs
// ─────────────────────────────────────────────────────────────────
vector<ArcUtil> get_top_congested_arcs(const NetworkContext& ctx,
    const MLUState& mlu, int N)
{
    auto all = get_mlu_of_all_arcs(ctx, mlu, ArcSortOrder::DecreasingUtil, -1);
    if ((int)all.size() > N) all.resize(N);
    return all;
}

// ─────────────────────────────────────────────────────────────────
// get_list_of_waypoints_per_list_of_demands
// ─────────────────────────────────────────────────────────────────
vector<vector<vector<int>>> get_list_of_waypoints_per_list_of_demands(
    const NetworkContext& ctx, const vector<int>& demand_ids)
{
    const int T = ctx.num_slots;
    vector<vector<vector<int>>> result(demand_ids.size());
    for (int i = 0; i < (int)demand_ids.size(); ++i) {
        int d_id = demand_ids[i];
        if (d_id < 0 || d_id >= (int)ctx.demands.size()) continue;
        result[i].resize(T);
        for (int t = 0; t < T; ++t)
            result[i][t] = ctx.demands[d_id].waypoints[t];
    }
    return result;
}

// ─────────────────────────────────────────────────────────────────
// get_list_of_shortest_paths_per_list_of_demands
// ─────────────────────────────────────────────────────────────────
vector<KShortestResult> get_list_of_shortest_paths_per_list_of_demands(
    const NetworkContext& ctx, const vector<int>& demand_ids,
    int k_paths, int time_slot)
{
    vector<KShortestResult> result(demand_ids.size());
    auto dis = ctx.scenario.disabled_at(time_slot);
    for (int i = 0; i < (int)demand_ids.size(); ++i) {
        int d_id = demand_ids[i];
        if (d_id < 0 || d_id >= (int)ctx.demands.size()) continue;
        const Demand& d = ctx.demands[d_id];
        result[i] = k_shortest_paths(ctx.graph, d.src, d.dst, k_paths, dis);
    }
    return result;
}

// ─────────────────────────────────────────────────────────────────
// Visualizer helpers
// ─────────────────────────────────────────────────────────────────
void write_vis_graph(const VisGraph& vg, const string& output_dir) {
    json j;
    j["title"] = vg.title;
    j["nodes"] = json::array();
    for (const auto& n : vg.nodes) {
        json jn;
        jn["id"] = n.id;
        jn["name"] = n.name;
        jn["highlighted"] = n.highlighted;
        jn["color"] = n.color;
        j["nodes"].push_back(jn);
    }
    j["edges"] = json::array();
    for (const auto& e : vg.edges) {
        json je;
        je["from"] = e.from;
        je["to"] = e.to;
        je["util"] = e.util;
        je["highlighted"] = e.highlighted;
        je["label"] = e.label;
        j["edges"].push_back(je);
    }
    string path = output_dir + "/graph_data.json";
    ofstream f(path);
    if (!f.is_open()) {
        cerr << "Cannot write " << path << "\n";
        return;
    }
    f << j.dump(2);
    cout << "Wrote visualizer data to " << path << "\n";
}

void launch_visualizer(const string& output_dir) {
    string cmd = "python \"" + output_dir + "/main.py\"";
    cout << "Launching visualizer: " << cmd << "\n";
    system(cmd.c_str());
}

// ─────────────────────────────────────────────────────────────────
// draw_list_of_waypoints_in_graph
// ─────────────────────────────────────────────────────────────────
void draw_list_of_waypoints_in_graph(const NetworkContext& ctx,
    const vector<int>& demand_ids, int time_slot, const string& output_dir)
{
    const Graph& g = ctx.graph;
    MLUState mlu = compute_mlu(ctx);

    // Identify highlighted edges (on waypoint paths)
    unordered_set<int> hl_edges;
    unordered_set<int> hl_nodes;
    vector<int> ids = demand_ids.empty() ?
        [&](){ vector<int> v(ctx.demands.size()); iota(v.begin(),v.end(),0); return v; }() :
        demand_ids;

    auto dis = ctx.scenario.disabled_at(time_slot);
    unordered_map<int, BwdDijkstraResult> bwd_cache;
    vector<bool> dis_vec(g.m, false);
    for (int eid : dis) dis_vec[eid] = true;

    for (int d_id : ids) {
        if (d_id < 0 || d_id >= (int)ctx.demands.size()) continue;
        const Demand& d = ctx.demands[d_id];
        hl_nodes.insert(d.src);
        hl_nodes.insert(d.dst);
        const auto& wp = d.waypoints[time_slot];
        for (int w : wp) hl_nodes.insert(w);
        // Trace ECMP paths
        for (int seg = 0; seg+1 < (int)wp.size(); ++seg) {
            int s = wp[seg], dst = wp[seg+1];
            auto bit = bwd_cache.find(dst);
            if (bit == bwd_cache.end())
                bit = bwd_cache.emplace(dst,
                    dijkstra_backward_full(g, dst, dis_vec)).first;
            // Walk from s to dst along SP-DAG
            // Collect edges on the DAG
            vector<double> frac(g.m, 0.0);
            ecmp_segment_topo(g, s, bit->second, dis_vec, frac);
            for (int e = 0; e < g.m; ++e)
                if (frac[e] > 1e-9) hl_edges.insert(e);
        }
    }

    VisGraph vg;
    vg.title = "Waypoints at slot " + to_string(time_slot);
    for (int n = 0; n < g.n; ++n) {
        VisNode vn;
        vn.id = n;
        vn.name = g.nodes[n].name;
        vn.highlighted = hl_nodes.count(n) > 0;
        vn.color = vn.highlighted ? "#ff6b6b" : "#97c2fc";
        vg.nodes.push_back(vn);
    }
    for (int e = 0; e < g.m; ++e) {
        VisEdge ve;
        ve.from = g.edges[e].from;
        ve.to   = g.edges[e].to;
        ve.util = (time_slot >= 0 && time_slot < mlu.num_slots) ?
                   mlu.util[time_slot][e] : 0.0;
        ve.highlighted = hl_edges.count(e) > 0;
        ve.label = to_string(int(ve.util * 100)) + "%";
        vg.edges.push_back(ve);
    }
    write_vis_graph(vg, output_dir);
    launch_visualizer(output_dir);
}

// ─────────────────────────────────────────────────────────────────
// draw_list_of_shortest_paths_of_demands
// ─────────────────────────────────────────────────────────────────
void draw_list_of_shortest_paths_of_demands(const NetworkContext& ctx,
    const vector<int>& demand_ids, int k_paths, int time_slot,
    const string& output_dir)
{
    const Graph& g = ctx.graph;
    MLUState mlu = compute_mlu(ctx);

    auto dis = ctx.scenario.disabled_at(time_slot);
    vector<int> ids = demand_ids.empty() ?
        [&](){ vector<int> v(ctx.demands.size()); iota(v.begin(),v.end(),0); return v; }() :
        demand_ids;

    unordered_set<int> hl_edges, hl_nodes;
    for (int d_id : ids) {
        if (d_id < 0 || d_id >= (int)ctx.demands.size()) continue;
        const Demand& d = ctx.demands[d_id];
        hl_nodes.insert(d.src); hl_nodes.insert(d.dst);
        KShortestResult ksr = k_shortest_paths(g, d.src, d.dst, k_paths, dis);
        for (auto& lv : ksr.paths_by_level)
            for (auto& p : lv) {
                for (int n : p) hl_nodes.insert(n);
                for (int i = 0; i+1 < (int)p.size(); ++i) {
                    int eid = g.find_edge(p[i], p[i+1]);
                    if (eid >= 0) hl_edges.insert(eid);
                }
            }
    }

    VisGraph vg;
    vg.title = "Shortest paths at slot " + to_string(time_slot);
    for (int n = 0; n < g.n; ++n) {
        VisNode vn;
        vn.id = n;
        vn.name = g.nodes[n].name;
        vn.highlighted = hl_nodes.count(n) > 0;
        vn.color = vn.highlighted ? "#ffd93d" : "#97c2fc";
        vg.nodes.push_back(vn);
    }
    for (int e = 0; e < g.m; ++e) {
        VisEdge ve;
        ve.from = g.edges[e].from;
        ve.to   = g.edges[e].to;
        ve.util = (time_slot >= 0 && time_slot < mlu.num_slots) ?
                   mlu.util[time_slot][e] : 0.0;
        ve.highlighted = hl_edges.count(e) > 0;
        ve.label = to_string(int(ve.util * 100)) + "%";
        vg.edges.push_back(ve);
    }
    write_vis_graph(vg, output_dir);
    launch_visualizer(output_dir);
}

// ─────────────────────────────────────────────────────────────────
// print_arc_util_table
// ─────────────────────────────────────────────────────────────────
void print_arc_util_table(const NetworkContext& ctx,
    const MLUState& mlu, int time_slot)
{
    const int E = mlu.num_edges;
    int t = (time_slot < 0) ? 0 : time_slot;
    double sum = 0.0;
    for (int e = 0; e < E; ++e) sum += mlu.util[t][e];
    double mean = sum / E;

    cout << "\n=== Arc Utilisation Table  slot=" << t
         << "  MLU=" << fixed << setprecision(4) << mlu.mlu_per_slot[t]
         << "  mean=" << mean << " ===\n";
    cout << setw(5) << "Edge" << setw(5) << "From" << setw(5) << "To"
         << setw(12) << "Util" << setw(12) << "Load"
         << setw(12) << "Cap" << "  Congest\n";
    cout << string(60, '-') << "\n";

    vector<int> order(E);
    iota(order.begin(), order.end(), 0);
    sort(order.begin(), order.end(),
         [&](int a, int b){ return mlu.util[t][a] > mlu.util[t][b]; });

    for (int e : order) {
        if (mlu.util[t][e] == 0.0) break;
        const Edge& arc = ctx.graph.edges[e];
        cout << setw(5) << e
             << setw(5) << arc.from << setw(5) << arc.to
             << setw(12) << fixed << setprecision(4) << mlu.util[t][e]
             << setw(12) << mlu.load[t][e]
             << setw(12) << arc.capacity
             << (mlu.util[t][e] >= mean ? "  ***" : "") << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────
// print_all_waypoints
// ─────────────────────────────────────────────────────────────────
void print_all_waypoints(const NetworkContext& ctx) {
    cout << "\n=== Waypoints ===\n";
    for (int d = 0; d < (int)ctx.demands.size(); ++d) {
        const Demand& dem = ctx.demands[d];
        cout << "D" << d << " (v" << dem.src << "->v" << dem.dst << "):\n";
        for (int t = 0; t < ctx.num_slots; ++t) {
            cout << "  t=" << t << " [";
            for (int i = 0; i < (int)dem.waypoints[t].size(); ++i) {
                if (i) cout << ",";
                cout << dem.waypoints[t][i];
            }
            cout << "]\n";
        }
    }
}

// ─────────────────────────────────────────────────────────────────
// print_budget_report
// ─────────────────────────────────────────────────────────────────
void print_budget_report(const NetworkContext& ctx) {
    cout << "\n=== Budget Feasibility Report ===\n";
    for (int t = 1; t < ctx.num_slots; ++t) {
        int dist = distance_step(ctx, t);
        double cap = ctx.scenario.budget_cap(t);
        bool ok = static_cast<double>(dist) <= cap;
        cout << "  t=" << t
             << "  dist=" << dist
             << "  budget=" << (isinf(cap) ? "inf" : to_string(int(cap)))
             << "  " << (ok ? "OK" : "VIOLATION") << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────
// verify_solution
// ─────────────────────────────────────────────────────────────────
bool verify_solution(const NetworkContext& ctx, const MLUState& mlu, bool verbose) {
    bool ok = true;

    // Budget feasibility
    if (!is_budget_feasible(ctx)) {
        if (verbose) {
            cerr << "[verify] Budget constraint violated!\n";
            print_budget_report(ctx);
        }
        ok = false;
    } else if (verbose) {
        cout << "[verify] Budget: OK\n";
    }

    // MLU sanity: no negative values
    for (int t = 0; t < mlu.num_slots; ++t)
        for (int e = 0; e < mlu.num_edges; ++e)
            if (mlu.util[t][e] < -1e-9) {
                if (verbose)
                    cerr << "[verify] Negative utilisation at t=" << t << " e=" << e << "\n";
                ok = false;
            }

    if (verbose && ok) cout << "[verify] Solution feasible.\n";
    return ok;
}

// ─────────────────────────────────────────────────────────────────
// get_lex_objective / print_lex_objective
// ─────────────────────────────────────────────────────────────────
vector<double> get_lex_objective(const MLUState& mlu) {
    vector<double> vals;
    vals.reserve(mlu.num_slots * mlu.num_edges);
    for (int t = 0; t < mlu.num_slots; ++t)
        for (int e = 0; e < mlu.num_edges; ++e)
            vals.push_back(mlu.util[t][e]);
    sort(vals.begin(), vals.end(), greater<double>());
    return vals;
}

void print_lex_objective(const MLUState& mlu, int top_n) {
    auto vals = get_lex_objective(mlu);
    cout << "\n=== Lex Objective (top " << top_n << " utilisation values) ===\n";
    for (int i = 0; i < min(top_n, (int)vals.size()); ++i)
        cout << "  [" << setw(3) << i+1 << "] " << fixed << setprecision(6) << vals[i] << "\n";
}
