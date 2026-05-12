#pragma once
#include "Graph.h"
#include "Demand.h"
#include "Scenario.h"

// ─────────────────────────────────────────────────────────────────
// NetworkContext: aggregates all input data (no algorithms)
// ─────────────────────────────────────────────────────────────────

struct NetworkContext {
    Graph               graph;
    std::vector<Demand> demands;
    int                 num_slots = 0;
    ScenarioConfig      scenario;
};

NetworkContext build_context(const std::string& net_path,
                             const std::string& tm_path,
                             const std::string& scenario_path);
