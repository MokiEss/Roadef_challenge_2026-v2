#pragma once
#include "Graph.h"
#include "ECMP.h"
#include "KShortest.h"
#include "NetworkContext.h"
#include "MLU.h"

#include <cstdint>
#include <unordered_set>
#include <vector>
#include <atomic>

// ─────────────────────────────────────────────────────────────────
// WaypointCandidate: one waypoint sequence + its ECMP fingerprint
// ─────────────────────────────────────────────────────────────────

struct WaypointCandidate {
    std::vector<int>                    waypoints;    // {src, w1, …, dst}
    // Flow on monitored arcs (for redundancy checking)
    std::vector<double>                 flow_on_M;
    uint64_t                            flow_hash = 0;
    // Sparse ECMP fractions on ALL edges
    std::vector<std::pair<int,float>>   sparse_fef;
    int                                 num_segments = 0;
    // Pairwise budget-distance to other candidates in the same CandidateSet
    std::vector<int>                    dist_to_others;
};

struct CandidateSet {
    int demand_id = -1;
    int time_slot  = -1;
    std::vector<WaypointCandidate> candidates;
};

// ─────────────────────────────────────────────────────────────────
// DemandCandidates: candidate sets for all time slots of one demand
// ─────────────────────────────────────────────────────────────────
struct DemandCandidates {
    int demand_id;
    std::vector<CandidateSet> per_slot;   // one CandidateSet per time slot
};

// ─────────────────────────────────────────────────────────────────
// Candidate build parameters
// ─────────────────────────────────────────────────────────────────
enum class CandidateBuildMode {
    Sequential,
    ParallelPerSlot,
    ParallelPerDemand,
    SingleSlotPropagate,  // full build at t=0, re-evaluate other slots
    IterativeRounds,      // external round loop; use build_candidates_for_round()
};

enum class DemandSortMode {
    ByTotalFlow,
    ByCongestedArcAppearance,
};

struct CandidateParams {
    // Arc set A: monitored arcs (MLU >= mean)
    int  max_congested_arcs    = 0;   // 0 = all arcs >= mean
    int  k_paths               = 5;   // distinct distance levels for Yen
    int  max_candidate_demands = 0;   // 0 = all contributing demands
    int  max_waypoints         = 3;   // max intermediate waypoints (maxSeg - 1)
    int  pool_cap              = 20;  // max nodes in candidate pool per demand

    // Hyperparameter h: arcs appearing >= h times in k-shortest paths are
    // also added to A (in addition to MLU-based selection).
    int  arc_freq_threshold    = 0;   // 0 = disabled

    // Timeout in wall-clock seconds (0 = unlimited)
    double timeout_seconds = 240.0;  // 4 minutes

    // Whether to include ALL nodes from found shortest paths as waypoint
    // candidates (slower but more thorough for small instances).
    bool include_all_path_nodes = false;

    // Max distinct candidates kept per (demand, slot) pair.
    // Higher values explore more routing options at the cost of a larger MIP.
    int  max_candidates         = 10;

    // Tiered per-demand max_candidates.
    // Demands are sorted by congestion contribution (descending) then split
    // into len(tiers) equal groups; group[i] gets tiers[i] max candidates.
    // When non-empty, overrides max_candidates for each demand individually.
    // Example: {30, 15, 6} → top-third gets 30, mid-third 15, bottom-third 6.
    // Demands with zero flow on monitored arcs are never candidates.
    std::vector<int> max_candidates_tiers;

    CandidateBuildMode mode      = CandidateBuildMode::SingleSlotPropagate;
    DemandSortMode     sort_mode = DemandSortMode::ByTotalFlow;
};

struct CandidateResult {
    std::vector<DemandCandidates> candidates;
    bool        timed_out    = false;
    int         demands_done = 0;
    int         demands_total = 0;
    std::string stop_reason;
};

// ─────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────

// Build candidate set for a single demand at one slot.
CandidateSet build_candidate_set(
    const Graph&                    g,
    const Demand&                   demand,
    int                             t,
    int                             k,
    const std::vector<int>&         monitored,
    const std::unordered_set<int>&  disabled,
    int                             max_waypoints  = 3,
    int                             pool_cap       = 20,
    bool                            include_all    = false,
    int                             max_candidates = 10);

// Build all candidates for all candidate demands, with parallelism.
CandidateResult build_all_candidates(
    const NetworkContext&  ctx,
    const MLUState&        mlu,
    const CandidateParams& params = {});

// ─────────────────────────────────────────────────────────────────
// Iterative-rounds candidate building.
// Builds candidates for `new_demand_ids` only (demands already in
// `existing_cands` are NOT rebuilt — their entries are preserved).
// `monitored` is list A for this round (controls redundancy check).
// `bypass_arcs` are disabled when generating bypass k-shortest paths.
// New DemandCandidates are appended to `existing_cands`.
// Returns the number of new demands successfully built.
// ─────────────────────────────────────────────────────────────────
int build_candidates_for_round(
    std::vector<DemandCandidates>&   existing_cands,   // in/out
    const std::vector<int>&          new_demand_ids,
    const std::vector<int>&          monitored,         // list A
    const std::unordered_set<int>&   bypass_arcs,       // arcs to disable in bypass
    const NetworkContext&            ctx,
    const CandidateParams&           params,
    double                           deadline_secs);    // absolute epoch deadline

// Evaluate a candidate (compute ECMP fractions; no cache)
bool evaluate_candidate(
    WaypointCandidate&              wc,
    const Graph&                    g,
    const std::vector<int>&         monitored,
    const std::unordered_set<int>&  disabled);
