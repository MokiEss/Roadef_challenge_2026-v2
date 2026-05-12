#include "WaypointEnum.h"
#include "MLU.h"

#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <sstream>
#include <numeric>

using namespace std;

// ─────────────────────────────────────────────────────────────────
// FNV-1a hash utilities
// ─────────────────────────────────────────────────────────────────
static uint64_t fnv1a(const vector<double>& v, double zero_tol = 1e-9) {
    uint64_t h = 14695981039346656037ULL;
    for (double x : v) {
        uint64_t bits = 0;
        if (fabs(x) >= zero_tol) memcpy(&bits, &x, sizeof(bits));
        h ^= bits;
        h *= 1099511628211ULL;
    }
    return h;
}

static bool flow_vecs_equal(const vector<double>& a, const vector<double>& b,
                             double tol = 1e-9)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (fabs(a[i] - b[i]) > tol) return false;
    return true;
}

// ─────────────────────────────────────────────────────────────────
// Sparse segment ECMP fractions (per-slot shared cache)
// ─────────────────────────────────────────────────────────────────
using SparseSegFrac = vector<pair<int,float>>;

struct SlotCtx {
    vector<bool>                                        dis_vec;
    unordered_set<int>                                  dis_set;
    unordered_map<int, BwdDijkstraResult>               bwd_cache;
    unordered_map<long long, SparseSegFrac>             seg_cache;
    unordered_map<long long, KShortestResult>           ksp_cache;

    // Optional shared read-only caches (pre-built before parallel phase)
    const unordered_map<int, BwdDijkstraResult>*        shared_bwd = nullptr;
    const unordered_map<long long, KShortestResult>*    shared_ksp = nullptr;
    const unordered_map<long long, SparseSegFrac>*      shared_seg = nullptr;
};

// ─────────────────────────────────────────────────────────────────
// Fast candidate evaluation using SlotCtx caches
// ─────────────────────────────────────────────────────────────────
static bool eval_candidate_fast(WaypointCandidate& wc, const Graph& g,
    const vector<int>& monitored, SlotCtx& ctx)
{
    wc.num_segments = (int)wc.waypoints.size() - 1;
    wc.flow_on_M.assign(monitored.size(), 0.0);

    thread_local vector<double> tl_accum;
    if ((int)tl_accum.size() < g.m) tl_accum.assign(g.m, 0.0);
    fill(tl_accum.begin(), tl_accum.begin() + g.m, 0.0);

    for (int seg = 0; seg + 1 < (int)wc.waypoints.size(); ++seg) {
        const int s   = wc.waypoints[seg];
        const int dst = wc.waypoints[seg+1];
        const long long key = (long long)s * g.n + dst;

        const SparseSegFrac* sp = nullptr;
        {
            auto sit = ctx.seg_cache.find(key);
            if (sit != ctx.seg_cache.end()) {
                sp = &sit->second;
            } else if (ctx.shared_seg) {
                auto ssit = ctx.shared_seg->find(key);
                if (ssit != ctx.shared_seg->end()) sp = &ssit->second;
            }
        }

        if (!sp) {
            const BwdDijkstraResult* bwd_ptr = nullptr;
            {
                auto bit = ctx.bwd_cache.find(dst);
                if (bit != ctx.bwd_cache.end()) {
                    bwd_ptr = &bit->second;
                } else if (ctx.shared_bwd) {
                    auto sbit = ctx.shared_bwd->find(dst);
                    if (sbit != ctx.shared_bwd->end()) bwd_ptr = &sbit->second;
                }
                if (!bwd_ptr) {
                    auto ni = ctx.bwd_cache.emplace(dst,
                        dijkstra_backward_full(g, dst, ctx.dis_vec)).first;
                    bwd_ptr = &ni->second;
                }
            }

            thread_local vector<double> tl_sf;
            if ((int)tl_sf.size() < g.m) tl_sf.assign(g.m, 0.0);
            fill(tl_sf.begin(), tl_sf.begin() + g.m, 0.0);

            if (!ecmp_segment_topo(g, s, *bwd_ptr, ctx.dis_vec, tl_sf))
                return false;

            SparseSegFrac sparse;
            for (int e = 0; e < g.m; ++e)
                if (tl_sf[e] > 1e-15) sparse.push_back({e, (float)tl_sf[e]});
            auto ins = ctx.seg_cache.emplace(key, move(sparse));
            sp = &ins.first->second;
        }

        for (auto& [e, f] : *sp) tl_accum[e] += f;
    }

    for (size_t i = 0; i < monitored.size(); ++i)
        wc.flow_on_M[i] = tl_accum[monitored[i]];

    wc.sparse_fef.clear();
    for (int e = 0; e < g.m; ++e)
        if (tl_accum[e] > 1e-9)
            wc.sparse_fef.push_back({e, (float)tl_accum[e]});

    wc.flow_hash = fnv1a(wc.flow_on_M);
    return true;
}

// ─────────────────────────────────────────────────────────────────
// Uncached evaluation (public API)
// ─────────────────────────────────────────────────────────────────
bool evaluate_candidate(WaypointCandidate& wc, const Graph& g,
    const vector<int>& monitored, const unordered_set<int>& disabled)
{
    wc.num_segments = (int)wc.waypoints.size() - 1;
    wc.flow_on_M.assign(monitored.size(), 0.0);

    ECMPResult er = compute_ecmp(g, wc.waypoints, disabled);
    if (!er.reachable) return false;

    for (size_t i = 0; i < monitored.size(); ++i)
        wc.flow_on_M[i] = er.edge_fraction[monitored[i]];

    wc.sparse_fef.clear();
    for (int e = 0; e < (int)er.edge_fraction.size(); ++e)
        if (er.edge_fraction[e] > 1e-9)
            wc.sparse_fef.push_back({e, (float)er.edge_fraction[e]});

    wc.flow_hash = fnv1a(wc.flow_on_M);
    return true;
}

// ─────────────────────────────────────────────────────────────────
// Extract candidate waypoints from k-shortest paths:
// divergence/merge points + optionally all path nodes.
//
// Divergence point: node where some paths go one way and others go
//   another — it appears at different positions in different paths.
// Merge point: same, but paths rejoin here.
//
// Both are detected by checking if a node appears at different
// offsets across paths, OR appears in some but not all paths at
// the same position.
//
// Returns: ordered pool of candidate intermediate nodes.
// ─────────────────────────────────────────────────────────────────
static vector<int> extract_candidate_pool(const KShortestResult& ksr,
    int src, int dst, int pool_cap, bool include_all)
{
    // Collect all paths
    vector<const Path*> all_paths;
    for (auto& level : ksr.paths_by_level)
        for (auto& p : level) all_paths.push_back(&p);

    if (include_all) {
        // Simple: all intermediate nodes in first-appearance order
        vector<int> pool;
        unordered_set<int> seen;
        for (auto* pp : all_paths)
            for (int i = 1; i+1 < (int)pp->size(); ++i) {
                int v = (*pp)[i];
                if (seen.insert(v).second) {
                    pool.push_back(v);
                    if ((int)pool.size() == pool_cap) return pool;
                }
            }
        return pool;
    }

    // Advanced: divergence/merge point analysis
    // For each intermediate node: record all (path_idx, position_in_path) pairs
    unordered_map<int, vector<pair<int,int>>> node_occurrences;  // node -> [(path,pos)]
    for (int pi = 0; pi < (int)all_paths.size(); ++pi) {
        const Path& p = *all_paths[pi];
        for (int i = 1; i+1 < (int)p.size(); ++i)
            node_occurrences[p[i]].emplace_back(pi, i);
    }

    // A node is meaningful if:
    //  1. It appears in multiple paths at different positions (diverge/merge)
    //  2. It appears in some but not all paths (alternative route)
    const int np = (int)all_paths.size();
    unordered_set<int> meaningful;
    for (auto& [v, occ] : node_occurrences) {
        if (v == src || v == dst) continue;
        // Check if appears in different positions
        bool diff_pos = false;
        for (int i = 1; i < (int)occ.size() && !diff_pos; ++i)
            if (occ[i].second != occ[0].second) diff_pos = true;
        // Check if appears in a subset of paths
        bool subset = (int)occ.size() < np;
        if (diff_pos || subset) meaningful.insert(v);
    }

    // Collect in first-appearance order, capped
    vector<int> pool;
    unordered_set<int> in_pool;
    for (auto* pp : all_paths)
        for (int i = 1; i+1 < (int)pp->size(); ++i) {
            int v = (*pp)[i];
            if (meaningful.count(v) && in_pool.insert(v).second) {
                pool.push_back(v);
                if ((int)pool.size() == pool_cap) return pool;
            }
        }
    // If still have room, add nodes that appear in ALL paths at same pos
    for (auto* pp : all_paths)
        for (int i = 1; i+1 < (int)pp->size(); ++i) {
            int v = (*pp)[i];
            if (in_pool.insert(v).second) {
                pool.push_back(v);
                if ((int)pool.size() == pool_cap) return pool;
            }
        }
    return pool;
}

// ─────────────────────────────────────────────────────────────────
// Build candidate set using a pre-configured SlotCtx
// ─────────────────────────────────────────────────────────────────
static CandidateSet build_candidate_set_impl(const Graph& g,
    const Demand& demand, int t, int k,
    const vector<int>& monitored, SlotCtx& ctx,
    int max_waypoints, int pool_cap, bool include_all, int max_candidates = 10)
{
    CandidateSet cs;
    cs.demand_id = demand.id;
    cs.time_slot  = t;

    vector<WaypointCandidate> accepted;
    unordered_map<uint64_t, int> hash_map;
    hash_map.reserve(max_candidates + 1);

    // Seed: direct path {src, dst}
    {
        WaypointCandidate wc;
        wc.waypoints = {demand.src, demand.dst};
        if (eval_candidate_fast(wc, g, monitored, ctx)) {
            hash_map.emplace(wc.flow_hash, 0);
            accepted.push_back(move(wc));
        }
    }

    // Get k-shortest paths
    const long long ksp_key = (long long)demand.src * g.n + demand.dst;
    const KShortestResult* ksr_ptr = nullptr;
    {
        auto it = ctx.ksp_cache.find(ksp_key);
        if (it != ctx.ksp_cache.end()) {
            ksr_ptr = &it->second;
        } else if (ctx.shared_ksp) {
            auto sit = ctx.shared_ksp->find(ksp_key);
            if (sit != ctx.shared_ksp->end()) ksr_ptr = &sit->second;
        }
        if (!ksr_ptr) {
            auto ni = ctx.ksp_cache.emplace(ksp_key,
                k_shortest_paths(g, demand.src, demand.dst, k, ctx.dis_set)).first;
            ksr_ptr = &ni->second;
        }
    }

    // Build candidate pool from normal k-shortest paths
    vector<int> pool = extract_candidate_pool(*ksr_ptr, demand.src, demand.dst,
                                               pool_cap, include_all);

    // ── Arc-bypass pool augmentation ─────────────────────────────
    // Disable the top-K globally congested arcs ALL at once and compute
    // k-shortest paths, providing bypass routes that genuinely avoid ALL
    // co-bottlenecks.
    // Without this, bypass paths for arc-1698 demands go through arc-31
    // (the next bottleneck), just shifting load without improving b1.
    // Only run the bypass if the demand actually traverses at least one
    // of the disabled arcs (skip demands unrelated to the bottlenecks).
    {
        const int bypass_top_k = 5;
        unordered_set<int> bypass_pool_set(pool.begin(), pool.end());

        // Disable the top-K congested arcs unconditionally
        unordered_set<int> bypass_dis = ctx.dis_set;
        int bypass_added = 0;
        for (int mon_e = 0;
             mon_e < (int)monitored.size() && bypass_added < bypass_top_k;
             ++mon_e)
        {
            bypass_dis.insert(monitored[mon_e]);
            ++bypass_added;
        }

        if (bypass_added > 0) {
            // Only run bypass if demand actually uses one of the bottleneck arcs
            bool demand_uses_any = false;
            for (auto& lv2 : ksr_ptr->paths_by_level) {
                for (auto& p : lv2) {
                    for (int i = 0; i + 1 < (int)p.size() && !demand_uses_any; ++i) {
                        int eid = g.find_edge(p[i], p[i+1]);
                        if (eid >= 0 && bypass_dis.count(eid) && !ctx.dis_set.count(eid))
                            demand_uses_any = true;
                    }
                }
                if (demand_uses_any) break;
            }

            if (demand_uses_any) {
                KShortestResult bypass_ksr =
                    k_shortest_paths(g, demand.src, demand.dst, k, bypass_dis);
                // Use include_all=true: strict diverge/merge analysis is too
                // aggressive on the limited bypass graph.
                vector<int> bypass_pool =
                    extract_candidate_pool(bypass_ksr, demand.src, demand.dst,
                                           pool_cap, /*include_all=*/true);
                for (int v : bypass_pool) {
                    if (bypass_pool_set.insert(v).second)
                        pool.push_back(v);
                }
            }
        }
    }

    // Generate k-combinations of pool nodes as waypoint sequences
    const int pool_sz  = (int)pool.size();
    const int max_lvl  = min(max_waypoints, pool_sz);

    WaypointCandidate scratch;
    scratch.flow_on_M.resize(monitored.size(), 0.0);
    scratch.waypoints.reserve(max_waypoints + 2);

    for (int level = 1; level <= max_lvl; ++level) {
        // Inline combination generation
        vector<int> idx(level);
        iota(idx.begin(), idx.end(), 0);
        for (;;) {
            scratch.waypoints.clear();
            scratch.waypoints.push_back(demand.src);
            for (int i = 0; i < level; ++i) scratch.waypoints.push_back(pool[idx[i]]);
            scratch.waypoints.push_back(demand.dst);

            if (eval_candidate_fast(scratch, g, monitored, ctx)) {
                auto hit = hash_map.find(scratch.flow_hash);
                if (hit == hash_map.end()) {
                    if ((int)accepted.size() < max_candidates) {
                        hash_map.emplace(scratch.flow_hash, (int)accepted.size());
                        accepted.push_back(scratch);
                    }
                } else {
                    auto& ex = accepted[hit->second];
                    if (flow_vecs_equal(scratch.flow_on_M, ex.flow_on_M)) {
                        if (scratch.num_segments < ex.num_segments) ex = scratch;
                    } else {
                        if ((int)accepted.size() < max_candidates)
                            accepted.push_back(scratch);
                    }
                }
            }

            // Advance combination index
            int i = level - 1;
            while (i >= 0 && idx[i] == pool_sz - level + i) --i;
            if (i < 0) break;
            ++idx[i];
            for (int j = i+1; j < level; ++j) idx[j] = idx[j-1]+1;
        }
    }

    sort(accepted.begin(), accepted.end(),
         [](const WaypointCandidate& a, const WaypointCandidate& b){
             return a.num_segments < b.num_segments; });
    cs.candidates = move(accepted);

    // Pairwise distances
    const int nc = (int)cs.candidates.size();
    for (int i = 0; i < nc; ++i) {
        cs.candidates[i].dist_to_others.assign(nc, 0);
        for (int j = 0; j < nc; ++j) {
            if (i == j) continue;
            cs.candidates[i].dist_to_others[j] =
                distance_waypoints(cs.candidates[i].waypoints,
                                   cs.candidates[j].waypoints, g.n);
        }
    }
    return cs;
}

// ─────────────────────────────────────────────────────────────────
// Propagate: re-evaluate slot-0 waypoint sequences for another slot
// ─────────────────────────────────────────────────────────────────
static CandidateSet propagate_candidate_set(const CandidateSet& src_cs, int t,
    const Graph& g, const vector<int>& monitored, SlotCtx& ctx)
{
    CandidateSet cs;
    cs.demand_id = src_cs.demand_id;
    cs.time_slot  = t;

    vector<WaypointCandidate> accepted;
    unordered_map<uint64_t, int> hash_map;
    hash_map.reserve(src_cs.candidates.size());

    WaypointCandidate scratch;
    scratch.flow_on_M.resize(monitored.size(), 0.0);

    for (const auto& sc : src_cs.candidates) {
        scratch.waypoints = sc.waypoints;
        if (!eval_candidate_fast(scratch, g, monitored, ctx)) continue;
        auto hit = hash_map.find(scratch.flow_hash);
        if (hit == hash_map.end()) {
            hash_map.emplace(scratch.flow_hash, (int)accepted.size());
            accepted.push_back(scratch);
        } else {
            auto& ex = accepted[hit->second];
            if (flow_vecs_equal(scratch.flow_on_M, ex.flow_on_M)) {
                if (scratch.num_segments < ex.num_segments) ex = scratch;
            } else {
                accepted.push_back(scratch);
            }
        }
    }

    sort(accepted.begin(), accepted.end(),
         [](const WaypointCandidate& a, const WaypointCandidate& b){
             return a.num_segments < b.num_segments; });
    cs.candidates = move(accepted);

    const int nc = (int)cs.candidates.size();
    for (int i = 0; i < nc; ++i) {
        cs.candidates[i].dist_to_others.assign(nc, 0);
        for (int j = 0; j < nc; ++j) {
            if (i == j) continue;
            cs.candidates[i].dist_to_others[j] =
                distance_waypoints(cs.candidates[i].waypoints,
                                   cs.candidates[j].waypoints, g.n);
        }
    }
    return cs;
}

// ─────────────────────────────────────────────────────────────────
// Public: build_candidate_set (single demand, single slot)
// ─────────────────────────────────────────────────────────────────
CandidateSet build_candidate_set(const Graph& g, const Demand& demand, int t, int k,
    const vector<int>& monitored, const unordered_set<int>& disabled,
    int max_waypoints, int pool_cap, bool include_all, int max_candidates)
{
    SlotCtx ctx;
    ctx.dis_set = disabled;
    ctx.dis_vec.assign(g.m, false);
    for (int eid : disabled) ctx.dis_vec[eid] = true;
    return build_candidate_set_impl(g, demand, t, k, monitored, ctx,
                                    max_waypoints, pool_cap, include_all, max_candidates);
}

// ─────────────────────────────────────────────────────────────────
// build_all_candidates: orchestrate candidate building for all
// contributing demands across all time slots.
// ─────────────────────────────────────────────────────────────────
CandidateResult build_all_candidates(const NetworkContext& ctx,
    const MLUState& mlu, const CandidateParams& params)
{
    const int T = ctx.num_slots;
    const int E = ctx.graph.m;

    auto t_start = chrono::steady_clock::now();
    auto timed_out = [&]() -> bool {
        if (params.timeout_seconds <= 0.0) return false;
        auto now = chrono::steady_clock::now();
        return chrono::duration<double>(now - t_start).count() >= params.timeout_seconds;
    };

    // ── Collect monitored arcs per slot ──────────────────────────
    // A = arcs with util >= mean (this slot)
    // + optionally arcs appearing >= h times in k-shortest paths
    vector<vector<int>> monitored_per_slot(T);
    for (int t = 0; t < T; ++t) {
        double sum = 0.0;
        for (int e = 0; e < E; ++e) sum += mlu.util[t][e];
        const double mean = sum / static_cast<double>(E);

        // Count arc frequencies across k-shortest paths (if h > 0)
        unordered_map<int,int> arc_freq;
        if (params.arc_freq_threshold > 0) {
            auto dis = ctx.scenario.disabled_at(t);
            for (const auto& d : ctx.demands) {
                unordered_set<int> sdis = dis;
                KShortestResult ksr = k_shortest_paths(ctx.graph, d.src, d.dst,
                                                        params.k_paths, sdis);
                for (auto& lv : ksr.paths_by_level)
                    for (auto& p : lv)
                        for (int i = 0; i+1 < (int)p.size(); ++i) {
                            int eid = ctx.graph.find_edge(p[i], p[i+1]);
                            if (eid >= 0) ++arc_freq[eid];
                        }
            }
        }

        unordered_set<int> mon_set;
        for (int e = 0; e < E; ++e) {
            if (mlu.util[t][e] >= mean) mon_set.insert(e);
            if (params.arc_freq_threshold > 0) {
                auto it = arc_freq.find(e);
                if (it != arc_freq.end() && it->second >= params.arc_freq_threshold)
                    mon_set.insert(e);
            }
        }
        if (params.max_congested_arcs > 0) {
            // sort by util, keep top N
            vector<pair<double,int>> sorted_mon;
            for (int e : mon_set) sorted_mon.emplace_back(mlu.util[t][e], e);
            sort(sorted_mon.begin(), sorted_mon.end(),
                 [](auto& a, auto& b){ return a.first > b.first; });
            int lim = min((int)sorted_mon.size(), params.max_congested_arcs);
            for (int i = 0; i < lim; ++i) monitored_per_slot[t].push_back(sorted_mon[i].second);
        } else {
            // Always sort by util descending so the bypass targets the most congested arc first
            vector<pair<double,int>> sorted_mon;
            sorted_mon.reserve(mon_set.size());
            for (int e : mon_set) sorted_mon.emplace_back(mlu.util[t][e], e);
            sort(sorted_mon.begin(), sorted_mon.end(),
                 [](auto& a, auto& b){ return a.first > b.first; });
            for (auto& [u, e] : sorted_mon) monitored_per_slot[t].push_back(e);
        }
    }

    // ── Identify contributing demands ────────────────────────────
    // A demand contributes if it has non-zero flow on any monitored arc
    // in any slot.
    unordered_map<int,double> demand_flow_on_mon;  // demand_id -> total flow on monitored arcs
    for (int t = 0; t < T; ++t) {
        for (int e : monitored_per_slot[t]) {
            for (auto& [d_id, flow] : mlu.contrib[t][e])
                demand_flow_on_mon[d_id] += flow;
        }
    }

    // Sort / filter contributing demands
    vector<pair<double,int>> sorted_demands;
    for (auto& [d_id, flow] : demand_flow_on_mon)
        sorted_demands.emplace_back(flow, d_id);

    if (params.sort_mode == DemandSortMode::ByTotalFlow) {
        sort(sorted_demands.begin(), sorted_demands.end(),
             [](auto& a, auto& b){ return a.first > b.first; });
    }

    int max_d = (params.max_candidate_demands > 0) ?
                min((int)sorted_demands.size(), params.max_candidate_demands) :
                (int)sorted_demands.size();

    // ── Targeted enrichment ──────────────────────────────────────
    // For each top congested arc, guarantee its heaviest individual
    // contributors appear in the candidate set.  Demands that only
    // heavily load ONE congested arc (e.g. a demand that routes
    // almost all its traffic through the most congested edge) may
    // rank low in the global sort because they don't spread across
    // many monitored arcs — yet they are the most critical to reroute.
    // We insert up to (enrich_arcs * enrich_per_arc) such demands at
    // the FRONT of the list so they get the highest candidate tier.
    vector<int> cand_demand_ids;
    cand_demand_ids.reserve(max_d);
    {
        const int enrich_arcs     = 5;   // top congested arcs to consider
        const int enrich_per_arc  = 20;  // top contributors per arc to guarantee

        // Collect enrichment demand IDs in order: top arc first, top contributor first
        vector<int> enriched_order;
        unordered_set<int> enriched_seen;
        for (int t = 0; t < T; ++t) {
            int narcs = min(enrich_arcs, (int)monitored_per_slot[t].size());
            for (int ai = 0; ai < narcs; ++ai) {
                int e = monitored_per_slot[t][ai];
                // Sort contributors to this arc by flow descending
                vector<pair<double,int>> ac;
                ac.reserve(mlu.contrib[t][e].size());
                for (auto& [d_id, flow] : mlu.contrib[t][e])
                    ac.emplace_back(flow, d_id);
                sort(ac.begin(), ac.end(), [](auto& a, auto& b){ return a.first > b.first; });
                int nkeep = min(enrich_per_arc, (int)ac.size());
                for (int i = 0; i < nkeep; ++i) {
                    int d_id = ac[i].second;
                    if (enriched_seen.insert(d_id).second)
                        enriched_order.push_back(d_id);
                }
            }
        }

        // Build final list: enriched first, then global ranking (deduped), up to max_d
        unordered_set<int> added;
        // Enriched demands first (they rank highest for tiered candidates)
        for (int d_id : enriched_order) {
            if (demand_flow_on_mon.count(d_id) && added.insert(d_id).second) {
                cand_demand_ids.push_back(d_id);
                if ((int)cand_demand_ids.size() == max_d) break;
            }
        }
        // Fill remaining from global sort
        for (int i = 0; i < (int)sorted_demands.size() && (int)cand_demand_ids.size() < max_d; ++i) {
            int d_id = sorted_demands[i].second;
            if (added.insert(d_id).second)
                cand_demand_ids.push_back(d_id);
        }
    }  // end enrichment scope — cand_demand_ids is now populated

    CandidateResult result;
    result.demands_total = (int)cand_demand_ids.size();
    result.candidates.resize(cand_demand_ids.size());
    for (int i = 0; i < (int)cand_demand_ids.size(); ++i)
        result.candidates[i].demand_id = cand_demand_ids[i];

    const int ND = (int)cand_demand_ids.size();

    // ── Per-demand max_candidates (tiered or uniform) ─────────────
    // Demands are sorted descending by congestion contribution.
    // With tiers = {t0, t1, t2, ...} we split ND into len(tiers) equal
    // groups: group i gets tiers[i] candidates.
    // Partial last group is covered by the last tier value.
    vector<int> per_d_maxcands(ND, params.max_candidates);
    if (!params.max_candidates_tiers.empty() && ND > 0) {
        const int NT = (int)params.max_candidates_tiers.size();
        cout << "  Candidate tiers (" << NT << "): ";
        for (int ti = 0; ti < NT; ++ti) {
            int lo = (ti * ND) / NT;
            int hi = ((ti + 1) * ND) / NT;
            if (ti == NT - 1) hi = ND;   // last tier catches remainder
            for (int i = lo; i < hi; ++i)
                per_d_maxcands[i] = params.max_candidates_tiers[ti];
            cout << "[" << lo << "-" << hi-1 << "]="
                 << params.max_candidates_tiers[ti];
            if (ti < NT - 1) cout << "  ";
        }
        cout << "\n";
    }

    // ── Build candidates ──────────────────────────────────────────
    if (params.mode == CandidateBuildMode::Sequential) {
        for (int i = 0; i < ND && !timed_out(); ++i) {
            int d_id = cand_demand_ids[i];
            const Demand& dem = ctx.demands[d_id];
            DemandCandidates dc;
            dc.demand_id = d_id;
            dc.per_slot.resize(T);
            for (int t = 0; t < T; ++t) {
                auto dis = ctx.scenario.disabled_at(t);
                dc.per_slot[t] = build_candidate_set(ctx.graph, dem, t, params.k_paths,
                    monitored_per_slot[t], dis, params.max_waypoints,
                    params.pool_cap, params.include_all_path_nodes, per_d_maxcands[i]);
            }
            result.candidates[i] = move(dc);
            ++result.demands_done;
        }
    }
    else if (params.mode == CandidateBuildMode::SingleSlotPropagate) {
        // Phase 1: build t=0 candidates in parallel per demand
        const int N = (int)thread::hardware_concurrency();
        atomic<int> next_d{0};
        {
            vector<jthread> threads;
            threads.reserve(min(N, ND));
            for (int w = 0; w < min(N, ND); ++w) {
                threads.emplace_back([&]() {
                    for (int i = next_d.fetch_add(1, memory_order_relaxed);
                         i < ND && !timed_out();
                         i = next_d.fetch_add(1, memory_order_relaxed))
                    {
                        int d_id = cand_demand_ids[i];
                        const Demand& dem = ctx.demands[d_id];
                        auto dis = ctx.scenario.disabled_at(0);
                        result.candidates[i].per_slot.resize(T);
                        result.candidates[i].per_slot[0] =
                            build_candidate_set(ctx.graph, dem, 0, params.k_paths,
                                monitored_per_slot[0], dis, params.max_waypoints,
                                params.pool_cap, params.include_all_path_nodes, per_d_maxcands[i]);
                    }
                });
            }
        }

        // Phase 2: propagate to slots 1..T-1 in parallel per (demand, slot)
        if (!timed_out() && T > 1) {
            struct Task { int i; int t; };
            vector<Task> tasks;
            for (int i = 0; i < ND; ++i)
                for (int t = 1; t < T; ++t)
                    tasks.push_back({i, t});
            atomic<int> next_task{0};
            vector<jthread> threads;
            threads.reserve(min(N, (int)tasks.size()));
            for (int w = 0; w < min(N, (int)tasks.size()); ++w) {
                threads.emplace_back([&]() {
                    for (int ti = next_task.fetch_add(1, memory_order_relaxed);
                         ti < (int)tasks.size() && !timed_out();
                         ti = next_task.fetch_add(1, memory_order_relaxed))
                    {
                        auto [i, t] = tasks[ti];
                        auto dis = ctx.scenario.disabled_at(t);
                        SlotCtx sctx;
                        sctx.dis_set = dis;
                        sctx.dis_vec.assign(ctx.graph.m, false);
                        for (int eid : dis) sctx.dis_vec[eid] = true;
                        result.candidates[i].per_slot[t] =
                            propagate_candidate_set(
                                result.candidates[i].per_slot[0], t,
                                ctx.graph, monitored_per_slot[t], sctx);
                    }
                });
            }
        }
        result.demands_done = ND;
    }
    else if (params.mode == CandidateBuildMode::ParallelPerSlot) {
        const int N = (int)thread::hardware_concurrency();
        for (int i = 0; i < ND && !timed_out(); ++i) {
            int d_id = cand_demand_ids[i];
            const Demand& dem = ctx.demands[d_id];
            result.candidates[i].per_slot.resize(T);
            atomic<int> next_slot{0};
            vector<jthread> threads;
            threads.reserve(min(N, T));
            for (int w = 0; w < min(N, T); ++w) {
                threads.emplace_back([&]() {
                    for (int t = next_slot.fetch_add(1, memory_order_relaxed);
                         t < T;
                         t = next_slot.fetch_add(1, memory_order_relaxed))
                    {
                        auto dis = ctx.scenario.disabled_at(t);
                        result.candidates[i].per_slot[t] =
                            build_candidate_set(ctx.graph, dem, t, params.k_paths,
                                monitored_per_slot[t], dis, params.max_waypoints,
                                params.pool_cap, params.include_all_path_nodes, per_d_maxcands[i]);
                    }
                });
            }
            ++result.demands_done;
        }
    }
    else { // ParallelPerDemand
        const int N = (int)thread::hardware_concurrency();
        // (demand, slot) tasks
        struct Task { int i; int t; };
        vector<Task> tasks;
        for (int i = 0; i < ND; ++i)
            for (int t = 0; t < T; ++t)
                tasks.push_back({i, t});
        // Pre-resize
        for (int i = 0; i < ND; ++i) result.candidates[i].per_slot.resize(T);
        atomic<int> next_task{0};
        vector<jthread> threads;
        threads.reserve(min(N, (int)tasks.size()));
        for (int w = 0; w < min(N, (int)tasks.size()); ++w) {
            threads.emplace_back([&]() {
                for (int ti = next_task.fetch_add(1, memory_order_relaxed);
                     ti < (int)tasks.size() && !timed_out();
                     ti = next_task.fetch_add(1, memory_order_relaxed))
                {
                    auto [i, t] = tasks[ti];
                    int d_id = cand_demand_ids[i];
                    const Demand& dem = ctx.demands[d_id];
                    auto dis = ctx.scenario.disabled_at(t);
                    result.candidates[i].per_slot[t] =
                        build_candidate_set(ctx.graph, dem, t, params.k_paths,
                            monitored_per_slot[t], dis, params.max_waypoints,
                            params.pool_cap, params.include_all_path_nodes, per_d_maxcands[i]);
                }
            });
        }
        result.demands_done = ND;
    }

    if (timed_out()) {
        result.timed_out = true;
        result.stop_reason = "timeout after " +
            to_string(params.timeout_seconds) + "s";
    }

    // ── Completion sweep ─────────────────────────────────────────
    // If candidate generation timed out (e.g. Phase 2 of Propagate mode),
    // some demands may have slot-0 candidates but empty slot t>0.
    // Fill them by propagating slot-0 waypoints into the missing slots.
    // This is fast (no new k-shortest paths) and ensures MIP feasibility.
    if (T > 1) {
        for (int i = 0; i < ND; ++i) {
            auto& dc = result.candidates[i];
            if ((int)dc.per_slot.size() < T) continue;  // not yet resized
            if (dc.per_slot[0].candidates.empty()) continue;  // no base
            for (int t = 1; t < T; ++t) {
                if (!dc.per_slot[t].candidates.empty()) continue;
                auto dis = ctx.scenario.disabled_at(t);
                SlotCtx sctx;
                sctx.dis_set = dis;
                sctx.dis_vec.assign(ctx.graph.m, false);
                for (int eid : dis) sctx.dis_vec[eid] = true;
                dc.per_slot[t] = propagate_candidate_set(
                    dc.per_slot[0], t, ctx.graph, monitored_per_slot[t], sctx);
            }
        }
    }

    // Remove entries that were not fully built due to timeout
    result.candidates.erase(
        remove_if(result.candidates.begin(), result.candidates.end(),
                  [](const DemandCandidates& dc){ return dc.per_slot.empty(); }),
        result.candidates.end());

    return result;
}

// ─────────────────────────────────────────────────────────────────
// build_candidates_for_round — incremental candidate builder
// Used by solve_iterative_rounds.
// Builds candidates only for `new_demand_ids`; existing entries in
// `existing_cands` are left untouched.  New entries are appended.
// ─────────────────────────────────────────────────────────────────
int build_candidates_for_round(
    vector<DemandCandidates>&   existing_cands,
    const vector<int>&          new_demand_ids,
    const vector<int>&          monitored,
    const unordered_set<int>&   bypass_arcs,
    const NetworkContext&       ctx,
    const CandidateParams&      params,
    double                      deadline_secs)
{
    if (new_demand_ids.empty()) return 0;

    const int T  = ctx.num_slots;
    const int ND = (int)new_demand_ids.size();

    auto now_secs = []() -> double {
        using namespace chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    };
    auto timed_out = [&]() -> bool {
        return deadline_secs > 0.0 && now_secs() >= deadline_secs;
    };

    // Pre-compute per-slot disabled sets
    vector<unordered_set<int>> dis_per_slot(T);
    for (int t = 0; t < T; ++t) dis_per_slot[t] = ctx.scenario.disabled_at(t);

    // Merge scenario disabled + bypass_arcs for the bypass k-shortest call
    // (bypass_arcs are the Q congested arcs from this round + any previously
    // congested arcs, so bypass paths avoid ALL known bottlenecks)
    vector<unordered_set<int>> bypass_dis_per_slot(T);
    for (int t = 0; t < T; ++t) {
        bypass_dis_per_slot[t] = dis_per_slot[t];
        for (int e : bypass_arcs) bypass_dis_per_slot[t].insert(e);
    }

    // Reserve slots at end of existing_cands
    int start_idx = (int)existing_cands.size();
    existing_cands.resize(start_idx + ND);
    for (int i = 0; i < ND; ++i)
        existing_cands[start_idx + i].demand_id = new_demand_ids[i];

    // Phase 1: build slot-0 candidates in parallel per demand
    const int NTHREADS = max(1, (int)thread::hardware_concurrency());
    {
        atomic<int> next_d{0};
        vector<jthread> threads;
        threads.reserve(min(NTHREADS, ND));
        for (int w = 0; w < min(NTHREADS, ND); ++w) {
            threads.emplace_back([&]() {
                for (int i = next_d.fetch_add(1, memory_order_relaxed);
                     i < ND && !timed_out();
                     i = next_d.fetch_add(1, memory_order_relaxed))
                {
                    const int d_id = new_demand_ids[i];
                    const Demand& dem = ctx.demands[d_id];

                    // Build normal slot-0 candidate set
                    CandidateSet cs0 = build_candidate_set(
                        ctx.graph, dem, /*t=*/0, params.k_paths,
                        monitored, dis_per_slot[0],
                        params.max_waypoints, params.pool_cap,
                        params.include_all_path_nodes, params.max_candidates);

                    // ── Bypass augmentation ───────────────────────
                    // Run an extra k-shortest with all bypass_arcs disabled
                    // to generate candidates that avoid the round's bottleneck
                    // arcs simultaneously.
                    if (!bypass_arcs.empty() && !bypass_dis_per_slot[0].empty()) {
                        // Only run bypass if demand uses any bypass arc in its
                        // normal k-shortest paths
                        KShortestResult normal_ksr = k_shortest_paths(
                            ctx.graph, dem.src, dem.dst, params.k_paths, dis_per_slot[0]);

                        bool uses_bypass_arc = false;
                        for (auto& lv2 : normal_ksr.paths_by_level) {
                            for (auto& p : lv2) {
                                for (int j = 0; j+1 < (int)p.size() && !uses_bypass_arc; ++j) {
                                    int eid = ctx.graph.find_edge(p[j], p[j+1]);
                                    if (eid >= 0 && bypass_arcs.count(eid))
                                        uses_bypass_arc = true;
                                }
                            }
                            if (uses_bypass_arc) break;
                        }

                        if (uses_bypass_arc) {
                            KShortestResult bypass_ksr = k_shortest_paths(
                                ctx.graph, dem.src, dem.dst, params.k_paths,
                                bypass_dis_per_slot[0]);
                            vector<int> bypass_pool = extract_candidate_pool(
                                bypass_ksr, dem.src, dem.dst,
                                params.pool_cap, /*include_all=*/true);

                            // Collect existing pool nodes for dedup
                            unordered_set<int> pool_set;
                            // We don't have the pool directly, but we can extract
                            // unique intermediate nodes from cs0's waypoints
                            for (auto& wc : cs0.candidates)
                                for (int k2 = 1; k2+1 < (int)wc.waypoints.size(); ++k2)
                                    pool_set.insert(wc.waypoints[k2]);

                            // Build extra candidate set from bypass pool
                            if (!bypass_pool.empty()) {
                                // Merge bypass_pool into a fresh build
                                // by calling build_candidate_set with bypass disabled
                                CandidateSet cs_bypass = build_candidate_set(
                                    ctx.graph, dem, /*t=*/0, params.k_paths,
                                    monitored, bypass_dis_per_slot[0],
                                    params.max_waypoints, params.pool_cap,
                                    /*include_all=*/true, params.max_candidates);

                                // Append unique candidates from bypass
                                unordered_map<uint64_t, int> hash_map;
                                for (int k2 = 0; k2 < (int)cs0.candidates.size(); ++k2)
                                    hash_map[cs0.candidates[k2].flow_hash] = k2;

                                for (auto& wc : cs_bypass.candidates) {
                                    if (!hash_map.count(wc.flow_hash) &&
                                        (int)cs0.candidates.size() < params.max_candidates)
                                    {
                                        hash_map[wc.flow_hash] = (int)cs0.candidates.size();
                                        cs0.candidates.push_back(wc);
                                    }
                                }
                            }
                        }
                    }

                    existing_cands[start_idx + i].per_slot.resize(T);
                    existing_cands[start_idx + i].per_slot[0] = move(cs0);
                }
            });
        }
    }  // threads join here

    // Phase 2: propagate slot-0 → slots 1..T-1
    if (!timed_out() && T > 1) {
        struct Task { int i; int t; };
        vector<Task> tasks;
        for (int i = 0; i < ND; ++i)
            for (int t = 1; t < T; ++t)
                tasks.push_back({i, t});
        atomic<int> next_task{0};
        vector<jthread> threads;
        threads.reserve(min(NTHREADS, (int)tasks.size()));
        for (int w = 0; w < min(NTHREADS, (int)tasks.size()); ++w) {
            threads.emplace_back([&]() {
                for (int ti = next_task.fetch_add(1, memory_order_relaxed);
                     ti < (int)tasks.size() && !timed_out();
                     ti = next_task.fetch_add(1, memory_order_relaxed))
                {
                    auto [i, t] = tasks[ti];
                    auto& dc = existing_cands[start_idx + i];
                    if (dc.per_slot[0].candidates.empty()) continue;
                    SlotCtx sctx;
                    sctx.dis_set = dis_per_slot[t];
                    sctx.dis_vec.assign(ctx.graph.m, false);
                    for (int eid : dis_per_slot[t]) sctx.dis_vec[eid] = true;
                    dc.per_slot[t] = propagate_candidate_set(
                        dc.per_slot[0], t, ctx.graph, monitored, sctx);
                }
            });
        }
    }

    // Completion sweep: fill any slots still empty after timeout
    for (int i = 0; i < ND; ++i) {
        auto& dc = existing_cands[start_idx + i];
        if ((int)dc.per_slot.size() < T) continue;
        if (dc.per_slot[0].candidates.empty()) continue;
        for (int t = 1; t < T; ++t) {
            if (!dc.per_slot[t].candidates.empty()) continue;
            SlotCtx sctx;
            sctx.dis_set = dis_per_slot[t];
            sctx.dis_vec.assign(ctx.graph.m, false);
            for (int eid : dis_per_slot[t]) sctx.dis_vec[eid] = true;
            dc.per_slot[t] = propagate_candidate_set(
                dc.per_slot[0], t, ctx.graph, monitored, sctx);
        }
    }

    // Count successfully built
    int built = 0;
    for (int i = 0; i < ND; ++i) {
        auto& dc = existing_cands[start_idx + i];
        if (!dc.per_slot.empty() && !dc.per_slot[0].candidates.empty()) ++built;
    }

    // Remove empty entries (timeout before build)
    existing_cands.erase(
        remove_if(existing_cands.begin() + start_idx, existing_cands.end(),
                  [](const DemandCandidates& dc){ return dc.per_slot.empty(); }),
        existing_cands.end());

    return built;
}
