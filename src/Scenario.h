#pragma once
#include "main_header.h"

// ─────────────────────────────────────────────────────────────────
// Scenario configuration: link failures and budget per time slot
// ─────────────────────────────────────────────────────────────────

struct ScenarioConfig {
    int max_segments = 1;   // max waypoints including src+dst (maxSeg)

    // Sparse: only slots that actually have a budget appear here.
    std::unordered_map<int, double> budget_per_slot;

    // Sparse: only slots with at least one failure appear here.
    std::unordered_map<int, std::unordered_set<int>> disabled_per_slot;

    bool edge_available(int edge_id, int t) const {
        auto it = disabled_per_slot.find(t);
        if (it == disabled_per_slot.end()) return true;
        return it->second.count(edge_id) == 0;
    }

    double budget_cap(int t) const {
        auto it = budget_per_slot.find(t);
        if (it == budget_per_slot.end())
            return std::numeric_limits<double>::infinity();
        return it->second;
    }

    std::unordered_set<int> disabled_at(int t) const {
        auto it = disabled_per_slot.find(t);
        if (it == disabled_per_slot.end()) return {};
        return it->second;
    }
};
