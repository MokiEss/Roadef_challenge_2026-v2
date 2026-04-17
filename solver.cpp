//
// Created by uha on 14/04/2026.
//
#include "solver.h"

bool solver::isBetter(const vector<double> & sat1, const vector<double> & sat2, int & where, bool & equal) {
    equal = true ;

    int i = 0 ;
    while (i < sat1.size() && equal==true) {
        equal = (sat1[i] == sat2[i]) ;
        i++;
    }
    where = i-1;
    return sat1[i-1] < sat2[i-1] ;
}

void solver::route_solution(solution & sol, int t,Digraph::ArcMap<DemandArray> & dpa) {

    sol.sr.clear();
    sol.sr.run(sol.inst.demand_graph, sol.inst.dvms[t], sol.rs.getSrPathsAt(t), dpa);
}

int solver::getOneOfTheMostCongestedArcs(const solution& sol, int t) {

    // Step 1: collect all arcs with their saturation as roulette wheel weights.
    std::vector<std::pair<Arc, double>> arc_saturations;
    arc_saturations.reserve(sol.inst.network.arcNum());

    double total_saturation = 0.0;

    for (ArcIt arc(sol.inst.network); arc != nt::INVALID; ++arc) {
        const double cap = sol.inst.capacities[arc];
        if (cap <= 0.0) continue;

        const double sat = sol.sr.saturation(arc, cap);
        if (sat <= 0.0) continue;

        arc_saturations.emplace_back(arc, sat);
        total_saturation += sat;
    }

    if (arc_saturations.empty() || total_saturation <= 0.0) return -1;

    // Step 2: roulette wheel — spin in [0, total_saturation].
    // Higher saturation → wider slice → more likely to be picked.
    const double r = genRandomDouble(total_saturation);
    double cumulative = 0.0;

    for (const auto& [arc, sat] : arc_saturations) {
        cumulative += sat;
        if (r <= cumulative) {
            return sol.inst.network.id(arc);
        }
    }

    // Fallback due to floating-point rounding: return the last (most congested).
    return sol.inst.network.id(arc_saturations.back().first);
}

DemandArc solver::getOneOfTheMostContributingDemandsToArc(const solution& sol, int arc_id, int t,
    const Digraph::ArcMap<DemandArray> & dpa) {
    Arc arc = sol.inst.network.arcFromId(arc_id);
    if (arc == nt::INVALID) return nt::INVALID;

    // dpa[arc] contains all demands that route through this arc at time slot t
    const DemandArray& demands_on_arc = dpa[arc];

    if (demands_on_arc.size() == 0) return nt::INVALID;

    // Step 1: collect demand contributions weighted by their flow at time t
    std::vector<std::pair<DemandArc, double>> demand_contributions;
    double total_contribution = 0.0;

    for (DemandArc demand_arc : demands_on_arc) {
        double total_demand_flow = sol.inst.dvms[t][demand_arc];
        if (total_demand_flow <= 0.0) continue;

        const SrPathBit& path = sol.rs.getSrPath(t, demand_arc);
        double demand_contribution_on_arc = 0.0;

        // Iterate through the SR path segments to see how much flow uses our arc
        path.forEachSegment(
            sol.inst.network,
            [&](Node s, Node target_node) {
                if (s == target_node) return true;

                // For this segment, compute ECMP split ratios on each arc
                typename Digraph::template StaticArcMap<double> ratio_map(sol.inst.network);
                sol.sr.underlyingProtocol().computeRatios(s, target_node, ratio_map);

                // Check if our arc is part of this segment and with what split ratio
                if (ratio_map[arc] > 0.0) {
                    demand_contribution_on_arc += total_demand_flow * ratio_map[arc];
                }
                return true;
            },
            [&](Arc seg_arc) {
                // Explicit arc segment: full demand flow uses this arc
                if (seg_arc == arc) {
                    demand_contribution_on_arc += total_demand_flow;
                }
            });

        if (demand_contribution_on_arc > 0.0) {
            demand_contributions.emplace_back(demand_arc, demand_contribution_on_arc);
            total_contribution += demand_contribution_on_arc;
        }
    }

    if (demand_contributions.empty() || total_contribution <= 0.0) return nt::INVALID;

    // Step 2: roulette wheel selection: higher flow → wider slice → more likely picked
    const double r = genRandomDouble(total_contribution);
    double cumulative = 0.0;

    for (const auto& [demand_arc, contribution] : demand_contributions) {
        cumulative += contribution;
        if (r <= cumulative) {
            return demand_arc;
        }
    }

    // Fallback: return last (highest-contributing) demand due to floating-point rounding
    return demand_contributions.back().first;
}


double solver::update_sr_path_for_demand_arcs(solution& sol, const DemandArc& da, int t, neighbor & m) {
    SrPathBit& path = sol.rs.getSrPath(t, da);
    SrPathBit backup;
    int old_cost = total_cost ;
    int old_cost_demand = (t == 0) ?
                        dist(sol.rs.getSrPath(t + 1 , da), path)
                        : dist(sol.rs.getSrPath(t - 1, da), path);

    old_cost-=old_cost_demand ;
    backup.copyFrom(path);
    Node source = sol.inst.demand_graph.source(da);
    m.da = da ;
    int iter = 0;
    // Check if path has waypoints (segment count > 2 means at least 1 waypoint)
    bool has_waypoints = (path.segmentNum() > 2);
    double new_mlu = 0.0;
    double budget = (t == 0) ? scenario.budget[t+1] : scenario.budget[t] ;
    do {
        iter++;
        path.copyFrom(backup);
        // initialize move
        m.replace = false ; m.add = false ; m.remove = false ;m.index_wp_add=-1; m.index_wp_remove=-1;
        // initialize path
        if (has_waypoints) {
        double rand_val = genRandomDouble(1.0);

        if (rand_val < 0.2) {
            // Remove a random waypoint
            int num_waypoints = path.segmentNum() - 2;
            if (num_waypoints > 0) {

                int wp_index = genRandomInt(num_waypoints);
                sol.removeWayPointFromExistingPath(path[wp_index + 1].toNode(), path, da);

                m.remove = true ;
                m.index_wp_remove = wp_index ;
            }
        }
        else {
             // Add a waypoint
             if (rand_val < 0.5) {
                 double rand_add = genRandomDouble(1.0);
                 std::vector<Node> candidates;

                 if (rand_add < 0.5) {
                     // Use only 1-hop neighbors
                     candidates = sol.getNeighbors(source, false);

                 } else {
                     // Use both 1-hop and 2-hop neighbors
                     candidates = sol.getNeighbors(source, true);
                 }

                 if (!candidates.empty()) {

                     int idx = genRandomInt(candidates.size());
                     std::vector<Node> waypoints = {candidates[idx]};
                     m.add = true ;
                     m.index_wp_add = inst.network.id(candidates[idx]) ;
                     sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments);
                 }
             }
             else {
                 // replace one way point with another
                 // remove a random waypoint
                 m.replace = true ;
                 int num_waypoints = path.segmentNum() - 2;
                 if (num_waypoints > 0) {

                     int wp_index = genRandomInt(num_waypoints);
                     m.index_wp_remove = wp_index ;
                     sol.removeWayPointFromExistingPath(path[wp_index + 1].toNode(), path, da);
                 }
                 // add a new waypoint
                 double rand_add = genRandomDouble(1.0);
                 std::vector<Node> candidates;

                 if (rand_add < 0.7) {
                     // Use only 1-hop neighbors
                     candidates = sol.getNeighbors(source, false);

                 } else {
                     // Use both 1-hop and 2-hop neighbors
                     candidates = sol.getNeighbors(source, true);
                 }

                 if (!candidates.empty()) {

                     int idx = genRandomInt(candidates.size());
                     std::vector<Node> waypoints = {candidates[idx]};
                     m.index_wp_add = inst.network.id(candidates[idx]) ;
                     sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments);
                 }
             }
         }
    }
        else {
        // No waypoints yet, always add one
        double rand_add = genRandomDouble(1.0);
        std::vector<Node> candidates;
        if (rand_add < 0.5) {
            // Use only 1-hop neighbors
            candidates = sol.getNeighbors(source, false);
        } else {
            // Use both 1-hop and 2-hop neighbors
            candidates = sol.getNeighbors(source, true);
        }

        if (!candidates.empty()) {
            m.add = true ;
            int idx = genRandomInt(candidates.size());
            std::vector<Node> waypoints = {candidates[idx]};
            m.index_wp_add = inst.network.id(candidates[idx]) ;
            sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments);
        }
    }
        m.cost_move = (t == 0) ? dist(sol.rs.getSrPath(t + 1 , da), path) :
                    dist(sol.rs.getSrPath(t - 1, da), path);

    } while ( (old_cost + m.cost_move > budget && iter < 10));

    // Evaluate the new path and compute MLU

    Digraph::ArcMap<DemandArray> dpa(sol.inst.network);
    int most_congested_arc_id ;


    new_mlu = sol.computeMLU(sol.sr, t,  most_congested_arc_id,da,
        backup, path, dpa, true ) ;

    for (ArcIt arc(inst.network); arc != nt::INVALID; ++arc) {
        m.saturations.push_back(sol.sr.saturation(arc, inst.capacities[arc])) ;
    }
    std::sort(
    m.saturations.begin(),
    m.saturations.end(),
    [](const auto& a, const auto& b) {
        return a > b; // Sort descending by saturation
    }
    );
    double f = sol.computeMLU(sol.sr, t,  most_congested_arc_id,da,
        path, backup , dpa, true ) ;

    path = std::move(backup);

    return new_mlu;
}

void solver::apply_move_to_solution(solution & sol, const neighbor & m, int t) {
    SrPathBit& path = sol.rs.getSrPath(t, m.da);
    SrPathBit backup;
    backup.copyFrom(path);

    if (m.replace) {
        if (m.index_wp_remove!=-1) {
            sol.removeWayPointFromExistingPath(path[m.index_wp_remove + 1].toNode(), path, m.da);
        }
        // add a new waypoint
        if (m.index_wp_add !=-1) {
            std::vector<Node> waypoints = {inst.network.nodeFromId(m.index_wp_add)};
            sol.insertWaypointsIntoExistingPath(path, m.da, waypoints, sol.scenario.i_max_segments);
        }
    }
    else {
        if (m.add) {
            if (m.index_wp_add !=-1) {
                std::vector<Node> waypoints = {inst.network.nodeFromId(m.index_wp_add)};
                sol.insertWaypointsIntoExistingPath(path, m.da, waypoints, sol.scenario.i_max_segments);
            }
        }
        else if (m.remove) {
            if (m.index_wp_remove!=-1) {
                sol.removeWayPointFromExistingPath(path[m.index_wp_remove + 1].toNode(), path, m.da);
            }
        }
    }

    Digraph::ArcMap<DemandArray> dpa(sol.inst.network);
    int most_congested_arc_id ;
    sol.solution_mlu[t] = sol.computeMLU(sol.sr, t,  most_congested_arc_id,m.da,
        backup, path, dpa, true ) ;

}

void solver::optimize() {

    // 1- initial solution
    solution s(inst, use_ftxui, result_builder, scenario, sr );
    total_cost = s.newHeuristicRun();

    int nbNeighbors = 30;
    // 2- generate 10 neighbors of initial solution by applying moves on them in while loop and keep always the best of them to
    //  replace the initial solution by the neighbor if the neighbor is better.
    for (int t = 0 ; t < inst.i_num_time_slots ; t++) {
        InterventionGuard intervention_guard(s.inst.network, s.inst.metrics, s.scenario.interventions[t]);
        std::cout << "=== Time Slot " << t << std::endl;
        int iterations = 1000 ;

        while (iterations--) {
            vector<neighbor> moves(nbNeighbors) ;
            vector<double> mlu_neighbors(nbNeighbors) ;
            vector<DemandArc> demand_neighbors(nbNeighbors) ;
            Digraph::ArcMap<DemandArray> dpa(s.inst.network);
            route_solution(s, t,dpa);
            for (int i = 0 ; i < nbNeighbors ; i++) {
                int arc_id = getOneOfTheMostCongestedArcs(s, t);
                if (arc_id == -1) continue;
                demand_neighbors[i] = getOneOfTheMostContributingDemandsToArc(s, arc_id, t,dpa);
                if (demand_neighbors[i] == nt::INVALID) continue;
                mlu_neighbors[i] = update_sr_path_for_demand_arcs(s, demand_neighbors[i], t,moves[i]);
            }
            // find the best neighbor
            int best_neighbor_index = 0 ;
            int where = -1 ;
            for (int i = 1 ; i < nbNeighbors ; i++) {
                bool equal ;
                if (isBetter(moves[i].saturations, moves[best_neighbor_index].saturations, where,equal)) {
                    best_neighbor_index = i ;
                }
                else {
                    if (equal && moves[i].cost_move < moves[best_neighbor_index].cost_move) {
                        best_neighbor_index = i ;
                    }
                }
            }
            bool equal ;
            int old_cost_demand = (t == 0) ?
            dist(s.rs.getSrPath(t ,demand_neighbors[best_neighbor_index])
                    , s.rs.getSrPath(t+1, demand_neighbors[best_neighbor_index])) :
            dist(s.rs.getSrPath(t - 1,demand_neighbors[best_neighbor_index])
                    , s.rs.getSrPath(t, demand_neighbors[best_neighbor_index]));
            double budget = (t == 0) ? scenario.budget[t+1] : scenario.budget[t] ;
            if (isBetter(moves[best_neighbor_index].saturations,s.solution_saturations[t] , where,equal)
                && ( ((total_cost-old_cost_demand) + moves[best_neighbor_index].cost_move) <= budget)) {
                // Apply the best move to the solution
                total_cost -= old_cost_demand;
                s.solution_saturations[t] = moves[best_neighbor_index].saturations ;
                apply_move_to_solution(s, moves[best_neighbor_index],  t);
                total_cost+=moves[best_neighbor_index].cost_move ;
                cout << "MLU improved " << s.solution_saturations[t][where] << " on the "<<
                    where+1 <<" most saturated arc and total cost is " << total_cost <<  endl ;

            }



        }



    }


    cout << "total cost is " << total_cost << endl ;








    // ===== END OF LOOP =====
    // In solver::optimize() before calling simulateSegmentRouting

    if (!simulateSegmentRouting(s.inst, s.scenario, s.rs, s.result_builder)) {
        s.result_builder.setValid(false);
        s.result_builder.display(i_max_decimal_places);
        return ;
    }
    bool feasible = checkBudgetConstraint(s.rs, s.inst, s.scenario, result_builder._i_total_cost);


    std::sort(
    s.result_builder._sat_values.begin(),
    s.result_builder._sat_values.end(),
    [](const auto& a, const auto& b) {
        return a.third > b.third; // Sort descending by saturation
    }
    );



    // final step: check the solution with the checker
    if (!feasible) {
        cout << "solution not feasible" << endl ;
        s.result_builder.setValid(false);
        s.result_builder.display(12);
    }
    else {
        cout << "solution  feasible" << endl ;
        s.result_builder.setValid(true);
        s.result_builder.display(12);
    }

}


