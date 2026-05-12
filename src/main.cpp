#include "main_header.h"
#include "NetworkContext.h"
#include "MLU.h"
#include "WaypointEnum.h"
#include "KShortest.h"






// ---------------------------------------------------------------──
// main
// ---------------------------------------------------------------──
int main() {
    string global_path = "./dataset/setA/setA-";
    string instance_id = "1";
    if (instance_id.size() == 1) {
        instance_id = "0" + instance_id;
    }
    string net_path, tm_path, scenario_path;
    net_path      = global_path + instance_id + "-net.json";
    tm_path       = global_path + instance_id + "-tm.json";
    scenario_path = global_path + instance_id + "-scenario.json";

    // ── Load data ------------------------------------------------─
    cout << "Loading: " << net_path << "\n"
         << "         " << tm_path  << "\n"
         << "         " << scenario_path << "\n\n";

    NetworkContext ctx;
    try {
        ctx = build_context(net_path, tm_path, scenario_path);
    } catch (exception& ex) {
        cerr << "Failed to load inputs: " << ex.what() << "\n";
        return 1;
    }

    int T = ctx.num_slots;
    cout << "Graph   : " << ctx.graph.n << " nodes, " << ctx.graph.m << " edges\n"
         << "Demands : " << ctx.demands.size() << "\n"
         << "Slots   : " << T << "\n\n";

    // ════════════════════════════════════════════════════════════════
    // EXAMPLE 1 — Candidate waypoint solutions for a given demand
    //   Build the candidate pool for demand[1] at slot 0 and list
    //   every candidate waypoint sequence together with its segment count.
    // ════════════════════════════════════════════════════════════════
    {
        const int demand_idx = 1;
        const Demand& dem = ctx.demands[demand_idx];
        int k_paths = 5, max_waypoints = 3, pool_cap = 20, per_d_maxcands = 10;
        bool include_all_path_nodes = false;
        vector<int> monitored;
        auto dis = ctx.scenario.disabled_at(0);

        auto t0 = chrono::steady_clock::now();
        CandidateSet cands = build_candidate_set(
            ctx.graph, dem, 0, k_paths, monitored, dis,
            max_waypoints, pool_cap, include_all_path_nodes, per_d_maxcands);
        auto ms = chrono::duration_cast<chrono::milliseconds>(
                      chrono::steady_clock::now() - t0).count();

        cout << "--- Example 1: Candidate waypoints for demand " << dem.id
             << "  (src=" << dem.src << " → dst=" << dem.dst
             << ", slot=0)  [" << ms << " ms] ---\n";
        cout << "  Total candidates: " << cands.candidates.size() << "\n";
        for (size_t i = 0; i < cands.candidates.size(); ++i) {
            const auto& c = cands.candidates[i];
            cout << "  Candidate " << (i+1) << ": waypoints = {";
            for (size_t j = 0; j < c.waypoints.size(); ++j) {
                cout << c.waypoints[j];
                if (j + 1 < c.waypoints.size()) cout << ", ";
            }
            cout << "}  segments=" << c.num_segments << "\n";
        }
        cout << "\n";
    }

    // ════════════════════════════════════════════════════════════════
    // EXAMPLE 2 — Assigned waypoints per slot for a given demand
    //   After loading (or solving), print the waypoint plan currently
    //   stored in ctx for demand[0] across every time slot.
    // ════════════════════════════════════════════════════════════════
    {
        const int demand_idx = 0;
        const Demand& dem = ctx.demands[demand_idx];
        cout << "--- Example 2: Assigned waypoints per slot for demand "
             << dem.id << "  (src=" << dem.src << " → dst=" << dem.dst << ") ---\n";
        for (int t = 0; t < T; ++t) {
            const auto& wp = dem.waypoints_at(t);
            cout << "  slot " << t << ": {";
            for (size_t j = 0; j < wp.size(); ++j) {
                cout << wp[j];
                if (j + 1 < wp.size()) cout << ", ";
            }
            cout << "}\n";
        }
        cout << "\n";
    }

    // ════════════════════════════════════════════════════════════════
    // EXAMPLE 3 — Compute MLU and inspect global summary
    // ════════════════════════════════════════════════════════════════
    {
        cout << "--- Example 3: MLU computation ---\n";
        auto t0 = chrono::steady_clock::now();
        MLUState mlu = compute_mlu(ctx);
        auto ms = chrono::duration_cast<chrono::milliseconds>(
                      chrono::steady_clock::now() - t0).count();

        cout << "  Computed in " << ms << " ms\n";
        cout << "  Global MLU : " << mlu.global_mlu << "\n";
        cout << "  Mean util  : " << mlu.mean_utilisation() << "\n";
        for (int t = 0; t < T; ++t) {
            cout << "  slot " << t << ": MLU=" << mlu.mlu_per_slot[t]
                 << "  mean=" << mlu.mean_utilisation_at(t) << "\n";
        }
        cout << "\n";
    }

    // ════════════════════════════════════════════════════════════════
    // EXAMPLE 4 — Demands contributing to a given arc (edge 0, slot 0)
    //   Uses the MLUState contrib table to list every demand that sends
    //   flow on the specified edge at the specified time slot.
    // ════════════════════════════════════════════════════════════════
    {
        MLUState mlu = compute_mlu(ctx);
        const int target_edge = 0;
        const int target_slot = 0;
        const Edge& e = ctx.graph.edge(target_edge);

        cout << "--- Example 4: Demands on arc " << target_edge
             << " (" << e.from << "->" << e.to
             << ", cap=" << e.capacity << ")  at slot " << target_slot << " ---\n";

        auto contributors = get_arc_contributors(ctx, mlu, target_edge, target_slot);
        if (contributors.empty()) {
            cout << "  (no demand uses this arc at this slot)\n";
        } else {
            cout << "  " << contributors.size() << " contributing demand(s):\n";
            for (const auto& c : contributors) {
                cout << "  demand " << c.demand_id
                     << "  flow=" << c.flow
                     << "  util_share=" << c.utilisation_share
                     << "  pct_of_arc=" << c.pct_of_arc_load << "\n";
            }
        }
        cout << "\n";
    }

    // ════════════════════════════════════════════════════════════════
    // EXAMPLE 5 — K-shortest simple paths for a given demand
    //   Runs Yen's algorithm for demand[0] at slot 0 and lists
    //   every path grouped by distance level.
    // ════════════════════════════════════════════════════════════════
    {
        const int demand_idx = 0;
        const Demand& dem = ctx.demands[demand_idx];
        const int k = 5;
        auto dis_set = ctx.scenario.disabled_at(0);

        cout << "--- Example 5: " << k << "-shortest paths for demand "
             << dem.id << "  (src=" << dem.src << " -> dst=" << dem.dst << ") ---\n";

        KShortestResult ksr = k_shortest_paths(ctx.graph, dem.src, dem.dst, k, dis_set);
        if (ksr.paths_by_level.empty()) {
            cout << "  (no path found)\n";
        } else {
            for (size_t lvl = 0; lvl < ksr.paths_by_level.size(); ++lvl) {
                cout << "  Level " << lvl << "  dist=" << ksr.distances[lvl]
                     << "  paths=" << ksr.paths_by_level[lvl].size() << "\n";
                for (const auto& path : ksr.paths_by_level[lvl]) {
                    cout << "    [";
                    for (size_t j = 0; j < path.size(); ++j) {
                        cout << path[j];
                        if (j + 1 < path.size()) cout << " -> ";
                    }
                    cout << "]\n";
                }
            }
        }
        cout << "\n";
    }

    cout << "Done.\n";
    return 0;
}
