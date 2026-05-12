#pragma once
#include "NetworkContext.h"
#include "MLU.h"
#include "KShortest.h"
#include "WaypointEnum.h"

#include <optional>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────
// Sort modes for arc queries
// ─────────────────────────────────────────────────────────────────
enum class ArcSortOrder {
    DecreasingUtil,       // highest utilisation first (within each slot)
    BySlotThenDecreasing, // grouped by slot, then decreasing util within slot
};

// ─────────────────────────────────────────────────────────────────
// Arc-level queries
// ─────────────────────────────────────────────────────────────────

// Returns all congested arcs (util >= mean of that slot), sorted by util desc.
// time_slot = -1 → all slots combined.
std::vector<CongestedArc> get_list_congest_arcs(
    const NetworkContext& ctx,
    const MLUState& mlu,
    int time_slot = -1);

// Returns all arcs with their utilisation values.
// sort_order controls ordering; time_slot = -1 → all slots.
struct ArcUtil {
    int    time_slot;
    int    edge_id;
    int    from, to;
    double utilisation;
    double capacity;
    double load;
};
std::vector<ArcUtil> get_mlu_of_all_arcs(
    const NetworkContext& ctx,
    const MLUState& mlu,
    ArcSortOrder sort_order = ArcSortOrder::DecreasingUtil,
    int time_slot = -1);

// Returns the top-N most congested arcs (across all slots).
// N defaults to 10.
std::vector<ArcUtil> get_top_congested_arcs(
    const NetworkContext& ctx,
    const MLUState& mlu,
    int N = 10);

// ─────────────────────────────────────────────────────────────────
// Demand / waypoint queries
// ─────────────────────────────────────────────────────────────────

// Returns waypoint sequences for a list of demand IDs, all slots.
// Returns [demand_idx][slot] -> {waypoint node IDs}
std::vector<std::vector<std::vector<int>>> get_list_of_waypoints_per_list_of_demands(
    const NetworkContext& ctx,
    const std::vector<int>& demand_ids);

// Returns k-shortest paths for a list of demands (at slot t=0 by default).
// Returns [demand_idx] -> KShortestResult
std::vector<KShortestResult> get_list_of_shortest_paths_per_list_of_demands(
    const NetworkContext& ctx,
    const std::vector<int>& demand_ids,
    int k_paths = 5,
    int time_slot = 0);

// ─────────────────────────────────────────────────────────────────
// Visualizer integration
// ─────────────────────────────────────────────────────────────────
// Writes a graph_data.json file and launches the Python visualizer.

// Draw waypoints for a list of demands.
// demand_ids: which demands to visualise (empty = all)
// time_slot: which time slot to show
// output_dir: where to write graph_data.json (defaults to visualizer/python_visualizer/)
void draw_list_of_waypoints_in_graph(
    const NetworkContext& ctx,
    const std::vector<int>& demand_ids,
    int time_slot = 0,
    const std::string& output_dir = "visualizer/python_visualizer");

// Draw shortest paths for a list of demands.
void draw_list_of_shortest_paths_of_demands(
    const NetworkContext& ctx,
    const std::vector<int>& demand_ids,
    int k_paths = 3,
    int time_slot = 0,
    const std::string& output_dir = "visualizer/python_visualizer");

// ─────────────────────────────────────────────────────────────────
// Additional debugging utilities
// ─────────────────────────────────────────────────────────────────

// Print a formatted table of arc utilisation for a slot.
void print_arc_util_table(const NetworkContext& ctx,
    const MLUState& mlu, int time_slot = 0);

// Print waypoints for all demands at all slots.
void print_all_waypoints(const NetworkContext& ctx);

// Print budget feasibility report.
void print_budget_report(const NetworkContext& ctx);

// Verify solution: check flow conservation and budget constraints.
// Returns true if feasible.
bool verify_solution(const NetworkContext& ctx, const MLUState& mlu,
    bool verbose = true);

// Compute and print the "lex" objective value for a given solution:
// returns the sorted utilisation vector (descending).
std::vector<double> get_lex_objective(const MLUState& mlu);
void print_lex_objective(const MLUState& mlu, int top_n = 20);

// ─────────────────────────────────────────────────────────────────
// Visualizer JSON export helpers (can be used independently)
// ─────────────────────────────────────────────────────────────────
struct VisEdge {
    int from, to;
    double util;
    bool highlighted;
    std::string label;
};
struct VisNode {
    int id;
    std::string name;
    bool highlighted;
    std::string color;
};
struct VisGraph {
    std::vector<VisNode> nodes;
    std::vector<VisEdge> edges;
    std::string title;
};

// Write graph_data.json for the Python visualizer.
void write_vis_graph(const VisGraph& vg, const std::string& output_dir);
// Launch the Python visualizer (opens a browser).
void launch_visualizer(const std::string& output_dir = "visualizer/python_visualizer");
