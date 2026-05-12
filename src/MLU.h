#pragma once
#include "NetworkContext.h"
#include "ECMP.h"

// ─────────────────────────────────────────────────────────────────
// MLU: Maximum Link Utilisation computation and analysis
// ─────────────────────────────────────────────────────────────────

struct ArcContributor {
    int    demand_id;
    int    src, dst;
    double flow;
    double utilisation_share;   // flow / arc capacity
    double pct_of_arc_load;     // flow / total arc load
};

struct CongestedArc {
    int    time_slot;
    int    edge_id;
    double utilisation;
    double mean_util;           // mean used as threshold
};

struct MLUState {
    int num_slots = 0;
    int num_edges = 0;

    std::vector<std::vector<double>> util;    // [t][e] = load/capacity
    std::vector<std::vector<double>> load;    // [t][e] = absolute flow
    // contrib[t][e] = {(demand_id, flow), …}
    std::vector<std::vector<std::vector<std::pair<int,double>>>> contrib;

    std::vector<double> mlu_per_slot;
    std::vector<double> max_util_per_arc;
    double              global_mlu = 0.0;

    // Lex objective breakdown: lex_b[0]=b1, lex_b[1]=b2, lex_b[2]=b3
    // Filled by solve(); empty if no MIP was run.
    std::vector<double> lex_b;
    // The mean utilisation used inside the MIP (baseline, pre-solve).
    // b_k = k-th largest |util - mip_mean|, NOT k-th largest raw util.
    double mip_mean = 0.0;

    double utilisation_at(int t, int e) const { return util[t][e]; }
    double load_at(int t, int e)        const { return load[t][e]; }
    const std::vector<std::pair<int,double>>&
        contributors_at(int t, int e)   const { return contrib[t][e]; }

    // Global mean utilisation across all arcs and slots
    double mean_utilisation() const {
        if (num_slots == 0 || num_edges == 0) return 0.0;
        double sum = 0.0;
        for (auto& row : util) for (double v : row) sum += v;
        return sum / static_cast<double>(num_slots * num_edges);
    }

    // Mean utilisation for a single slot
    double mean_utilisation_at(int t) const {
        if (num_edges == 0) return 0.0;
        double sum = 0.0;
        for (int e = 0; e < num_edges; ++e) sum += util[t][e];
        return sum / static_cast<double>(num_edges);
    }
};

enum class MLUMode { Sequential, Parallel, FullParallel };

MLUState compute_mlu(const NetworkContext& ctx, MLUMode mode = MLUMode::Sequential);

// ── Congested arc queries ──────────────────────────────────────────
// time_slot = -1 → all slots
std::vector<CongestedArc> get_congested_arcs(const MLUState& s, int time_slot = -1);

std::vector<ArcContributor> get_arc_contributors(const NetworkContext& ctx,
    const MLUState& s, int edge_id, int time_slot);

std::unordered_map<int, std::vector<ArcContributor>> get_arcs_contributors(
    const NetworkContext& ctx, const MLUState& s,
    const std::vector<int>& edge_ids, int time_slot);

// ── Pretty-print helpers ───────────────────────────────────────────
void print_mlu_summary(const NetworkContext& ctx, const MLUState& s);
void print_congested_arcs(const NetworkContext& ctx,
    const std::vector<CongestedArc>& arcs);
void print_arcs_sorted_by_util(const NetworkContext& ctx, const MLUState& s);
void print_arc_contributors(const NetworkContext& ctx,
    const std::vector<ArcContributor>& cs, int edge_id, int time_slot);
