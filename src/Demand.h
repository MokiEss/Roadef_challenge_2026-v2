#pragma once
#include "main_header.h"

// ─────────────────────────────────────────────────────────────────
// Demand data structure
// waypoints[t] = { src, w1, w2, ..., dst }
// Initialised to {src, dst}; solver fills intermediate waypoints.
// ─────────────────────────────────────────────────────────────────

using WaypointPlan = std::vector<std::vector<int>>;  // [slot] → node sequence

struct Demand {
    int                 id;
    int                 src, dst;
    std::vector<double> volume;   // volume[t], one entry per time slot
    WaypointPlan        waypoints;

    void init_waypoints(int num_slots) {
        waypoints.assign(num_slots, {src, dst});
    }

    const std::vector<int>& waypoints_at(int t) const {
        assert(t >= 0 && t < (int)waypoints.size());
        return waypoints[t];
    }
};
