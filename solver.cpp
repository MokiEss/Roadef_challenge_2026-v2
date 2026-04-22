//
// Created by uha on 14/04/2026.
//
#include "solver.h"

#include <fstream>
#include <filesystem>

// Exporte RS au format checker/visualizer: {"srpaths":[{"d":..,"t":..,"w":[..]}, ...]}
static nt::JSONDocument buildSrpathsJsonFromRs(Instance& inst, RoutingScheme& rs) {
    nt::JSONDocument doc;
    doc.SetObject();
    auto& alloc = doc.GetAllocator();

    nt::JSONValue srpaths(nt::JSONValueType::ARRAY_TYPE);

    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        for (int d = 0; d < inst.demand_graph.arcNum(); ++d) {
            DemandArc da = inst.demand_graph.arcFromId(d);
            if (da == nt::INVALID) continue;

            SrPathBit& path = rs.getSrPath(t, da);

            nt::JSONValue entry(nt::JSONValueType::OBJECT_TYPE);
            entry.AddMember("d", d, alloc);
            entry.AddMember("t", t, alloc);

            nt::JSONValue w(nt::JSONValueType::ARRAY_TYPE);

            // SR path nodes are: [source, wp1, wp2, ..., target]
            // Waypoints = internal nodes only
            if (path.segmentNum() >= 2) {
                for (int k = 1; k < path.segmentNum() - 1; ++k) {
                    if (!path[k].isNode()) continue; // defensive
                    Node wp = path[k].toNode();
                    w.PushBack(inst.network.id(wp), alloc);
                }
            }

            entry.AddMember("w", w, alloc);
            srpaths.PushBack(entry, alloc);
        }
    }

    doc.AddMember("srpaths", srpaths, alloc);
    return doc;
}

bool solver::isBetter(const vector<double> & sat1, const vector<double> & sat2, int & where, bool & equal) {
    if(sat1.size()!=sat2.size()) {
        throw std::invalid_argument("Saturation vectors must be of the same size");
    }
    equal = true ;
    int i = 0 ;
    while (i < sat1.size() && equal==true) {
        equal = (sat1[i] == sat2[i]) ;
        i++;
    }
    where = i-1;
    return sat1[i-1] < sat2[i-1] ;
}

int solver::route_solution(solution & sol, int t,Digraph::ArcMap<DemandArray> & dpa) {
    sol.sr.clear();
    sol.sr.run(sol.inst.demand_graph, sol.inst.dvms[t], sol.rs.getSrPathsAt(t), dpa);
    auto a = sol.sr.mostLoadedArc(sol.inst.capacities);
    return sol.inst.network.id(a.first) ;
}

int solver::getOneOfTheMostCongestedArcs(const solution& sol, int most_congested) {
    double total = 0.0;
    Arc chosen = nt::INVALID;
    for (ArcIt arc(sol.inst.network); arc != nt::INVALID; ++arc) {
        const double cap = sol.inst.capacities[arc];
        if (cap <= 0.0) continue;

        const double w = sol.sr.saturation(arc, cap);
        if (w <= 0.0 || !std::isfinite(w)) continue;

        total += w;

        // Replace current choice with probability w / total
        // u in [0,1)
        const double u = genRandomDouble(1.0);
        if (u < (w / total)) {
            chosen = arc;
        }
    }
    return (chosen == nt::INVALID) ? -1 : sol.inst.network.id(chosen);
}

DemandArc solver::getOneOfTheMostContributingDemandsToArc(
    const solution& sol, int arc_id, int t,
    const Digraph::ArcMap<DemandArray>& dpa) {

    Arc arc = sol.inst.network.arcFromId(arc_id);
    if (arc == nt::INVALID) return nt::INVALID;

    const DemandArray& demands_on_arc = dpa[arc];
    if (demands_on_arc.size() == 0) return nt::INVALID;

    std::vector<std::pair<DemandArc, double>> demand_contributions;
    demand_contributions.reserve(demands_on_arc.size());

    double total_contribution = 0.0;

    // Deduplicate demands coming from dpa[arc]
    std::unordered_set<int> seen_demand_ids;
    seen_demand_ids.reserve(static_cast<size_t>(demands_on_arc.size() * 2));

    for (DemandArc demand_arc : demands_on_arc) {
        const int da_id = sol.inst.demand_graph.id(demand_arc);
        if (!seen_demand_ids.insert(da_id).second) {
            continue; // duplicate entry in dpa[arc]
        }

        double total_demand_flow = sol.inst.dvms[t][demand_arc];
        if (total_demand_flow <= 0.0) continue;

        const SrPathBit& path = sol.rs.getSrPath(t, demand_arc);
        double demand_contribution_on_arc = 0.0;

        path.forEachSegment(
            sol.inst.network,
            [&](Node s, Node target_node) {
                if (s == target_node) return true;

                typename Digraph::template StaticArcMap<double> ratio_map(sol.inst.network);
                sol.sr.underlyingProtocol().computeRatios(s, target_node, ratio_map);

                if (ratio_map[arc] > 0.0) {
                    demand_contribution_on_arc += total_demand_flow * ratio_map[arc];
                }
                return true;
            },
            [&](Arc seg_arc) {
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

    const double r = genRandomDouble(total_contribution);
    double cumulative = 0.0;

    for (const auto& [demand_arc, contribution] : demand_contributions) {
        cumulative += contribution;
        if (r <= cumulative) return demand_arc;
    }

    return demand_contributions.back().first;
}

double solver::update_sr_path_for_demand_arcs(solution& sol, const DemandArc& da, int arc_id, int t, neighbor & m) {
    SrPathBit& path = sol.rs.getSrPath(t, da);
    SrPathBit backup;
    int old_cost = total_cost ;
    int old_cost_demand = (t == 0) ?
                        dist(sol.rs.getSrPath(t + 1 , da), path)
                        : dist(sol.rs.getSrPath(t - 1, da), path);

    old_cost-=old_cost_demand ;
    backup.copyFrom(path);
    Arc congested_arc = sol.inst.network.arcFromId(arc_id);
    if (congested_arc == nt::INVALID) {
        path.copyFrom(backup);
        m.valid = false ;
        return -1.0;
    }
    Node source = sol.inst.network.source(congested_arc);
    Node target = sol.inst.network.target(congested_arc);
    Node ds = sol.inst.demand_graph.source(da);
    Node dt = sol.inst.demand_graph.target(da);
    m.da = da ;
    int iter = 0;
    // Check if path has waypoints (segment count > 2 means at least 1 waypoint)
    // "t":0,"from":17,"to":0,"sat":0.729592816091
    bool has_waypoints = (path.segmentNum() > 2);
    double new_mlu = 0.0;
    double budget = (t == 0) ? scenario.budget[t+1] : scenario.budget[t] ;
    do {
        iter++;
        path.copyFrom(backup);
        // initialize move
        m.valid = false ; m.replace = false ; m.add = false ;
        m.remove = false ;m.index_wp_add=-1; m.index_wp_remove=-1;
        // initialize path
        if (has_waypoints) {
        double rand_val = genRandomDouble(1.0);
            // Remove a random waypoint
        int num_waypoints = path.segmentNum() - 2;
        if (num_waypoints > 0 && rand_val < 0.1) {
            int wp_index = genRandomInt(num_waypoints);
            if(sol.removeWayPointFromExistingPath(path[wp_index + 1].toNode(), path, da)) {
                // removal successful
                m.remove = true ;
                m.index_wp_remove = wp_index ;
            }
        }
        else {
             // Add a waypoint
             if (rand_val < 0.7) {
                 double rand_add = genRandomDouble(1.0);
                 std::vector<Node> candidates;

                 if (rand_add < 0.1) {
                     // Use only 1-hop neighbors
                     //candidates = sol.getNeighbors(source, target,ds, dt, false);
                     candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, false, 8);

                 } else {
                     // Use both 1-hop and 2-hop neighbors
                    // candidates = sol.getNeighbors(source, target,ds,dt, true);
                     candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, true, 8);
                 }

                 if (!candidates.empty()) {
                     int idx = genRandomInt(candidates.size());
                     std::vector<Node> waypoints = {candidates[idx]};
                     if(sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments)) {
                         // insertion successful
                         m.add = true ;
                         m.index_wp_add = sol.inst.network.id(candidates[idx]) ;
                     }
                 }
             }
            // replace one way point with another
             else {
                 // remove a random waypoint
                 int num_waypoints = path.segmentNum() - 2;
                 if (num_waypoints > 0) {
                     int wp_index = genRandomInt(num_waypoints);
                     m.replace = sol.removeWayPointFromExistingPath(path[wp_index + 1].toNode(), path, da);
                     if(m.replace) m.index_wp_remove = wp_index ;
                 }
                 // add a new waypoint
                 double rand_add = genRandomDouble(1.0);
                 std::vector<Node> candidates;

                 if (rand_add < 0.2) {
                     // Use only 1-hop neighbors
                     //candidates = sol.getNeighbors(source, target, ds, dt, false);
                     candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, false, 8);

                 } else {
                     // Use both 1-hop and 2-hop neighbors
                     candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, true, 8);
                     //candidates = sol.getNeighbors(source, target, ds, dt, true);
                 }

                 if (!candidates.empty()) {

                     int idx = genRandomInt(candidates.size());
                     std::vector<Node> waypoints = {candidates[idx]};

                     if(m.replace && sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments)) {
                         // insertion successful
                         m.index_wp_add = sol.inst.network.id(candidates[idx]) ;
                     }
                     else m.replace = false ;
                 }
             }
         }
    }
        else {
        // No waypoints yet, always add one
        double rand_add = genRandomDouble(1.0);
        std::vector<Node> candidates;
        if (rand_add < 0.1) {
            // Use only 1-hop neighbors
           // candidates = sol.getNeighbors(source, target, ds, dt, false);
            candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, false, 8);
        } else {
            // Use both 1-hop and 2-hop neighbors
           // candidates = sol.getNeighbors(source, target, ds, dt,  true);
            candidates = sol.getPromisingWaypoints(source, target, ds,dt, congested_arc, true, 8);
        }

        if (!candidates.empty()) {
            int idx = genRandomInt(candidates.size());
            std::vector<Node> waypoints = {candidates[idx]};

            if(sol.insertWaypointsIntoExistingPath(path, da, waypoints, sol.scenario.i_max_segments)) {
                // insertion successful
                m.add = true ;
                m.index_wp_add = sol.inst.network.id(candidates[idx]) ;
            }
        }
    }
        m.cost_move = (t == 0) ? dist(sol.rs.getSrPath(t + 1 , da), path) :
                    dist(sol.rs.getSrPath(t - 1, da), path);

    } while ( ((old_cost + m.cost_move) > budget
        ||(!m.add && !m.remove && !m.replace))
        && iter < 10);
    if (iter < 10 && (m.add || m.remove || m.replace)) m.valid = true ;
    // if solution invalid after 10 iterations, revert to original path and return -1
    if (m.valid==false) {
        path = std::move(backup);
        return -1;
    }
    // Evaluate the new path and compute MLU
    int most_congested_arc_id ;
    new_mlu = sol.computeMLU_no_dpa(sol.sr, t,  most_congested_arc_id,da,
        backup, path, true ) ;
    for (ArcIt arc(sol.inst.network); arc != nt::INVALID; ++arc) {
        m.saturations.push_back(sol.sr.saturation(arc, sol.inst.capacities[arc])) ;
    }
    std::sort(
    m.saturations.begin(),
    m.saturations.end(),
    [](const auto& a, const auto& b) {
        return a > b; // Sort descending by saturation
    }
    );

    // route back to original state
    // Remove new flow, add old flow
    double flow = inst.dvms[t][da];
    sol.sr.routeFlow(path, -flow);
    sol.sr.routeFlow(backup, flow);
    path = std::move(backup);

    return 0.0;
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
            std::vector<Node> waypoints = {sol.inst.network.nodeFromId(m.index_wp_add)};
            sol.insertWaypointsIntoExistingPath(path, m.da, waypoints, sol.scenario.i_max_segments);
        }
    }
    else {
        if (m.add) {
            if (m.index_wp_add !=-1) {
                std::vector<Node> waypoints = {sol.inst.network.nodeFromId(m.index_wp_add)};
                sol.insertWaypointsIntoExistingPath(path, m.da, waypoints, sol.scenario.i_max_segments);
            }
        }
        else if (m.remove) {
            if (m.index_wp_remove!=-1) {
                sol.removeWayPointFromExistingPath(path[m.index_wp_remove + 1].toNode(), path, m.da);
            }
        }
    }


    int most_congested_arc_id ;
    sol.solution_mlu[t] = sol.computeMLU_no_dpa(sol.sr, t,  most_congested_arc_id,m.da,
        backup, path, true ) ;

}

void solver::optimize() {
    // 1- initial solution
    solution s(inst, use_ftxui, result_builder, scenario, sr );
    total_cost = s.newHeuristicRun();

    int nbNeighbors = 50;
    // 2- generate 10 neighbors of initial solution by applying moves on them in while loop and keep always the best of them to
    //  replace the initial solution by the neighbor if the neighbor is better.
    for (int t = 0 ; t < s.inst.i_num_time_slots ; t++) {
        InterventionGuard intervention_guard(s.inst.network, s.inst.metrics, s.scenario.interventions[t]);
        std::cout << "=== Time Slot " << t << std::endl;
        int iterations = 1000 ;
        while (iterations--) {
            vector<neighbor> moves(nbNeighbors) ;
            vector<DemandArc> demand_neighbors(nbNeighbors) ;
            Digraph::ArcMap<DemandArray> dpa(s.inst.network);
            int congested = route_solution(s, t,dpa);
            for (int i = 0 ; i < nbNeighbors ; i++) {
                int arc_id = getOneOfTheMostCongestedArcs(s,congested);
                if (arc_id == -1) continue;
                demand_neighbors[i] = getOneOfTheMostContributingDemandsToArc(s, arc_id, t,dpa);
                if (demand_neighbors[i] == nt::INVALID) continue;
                update_sr_path_for_demand_arcs(s, demand_neighbors[i], arc_id, t,moves[i]);
            }
            // find the best neighbor
            int best_neighbor_index = -1 ;
            int where = -1 ;
            int nb_not_valid = 0 ;
            for (int i = 0; i < nbNeighbors; ++i) {
                if (!moves[i].valid) {
                    nb_not_valid++;
                    continue;
                }
                if (best_neighbor_index == -1) {
                    best_neighbor_index = i; // first valid candidate
                    continue;
                }
                bool equal = false;
                if (isBetter(moves[i].saturations, moves[best_neighbor_index].saturations, where, equal) ||
                    (equal && moves[i].cost_move < moves[best_neighbor_index].cost_move)) {
                    best_neighbor_index = i;
                    }
            }
            if (best_neighbor_index == -1) {
                continue; // no valid neighbor this iteration
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
              /*  cout << "MLU improved " << s.solution_saturations[t][where] << " on the "<<
                    where+1 <<" most saturated arc and total cost is " << total_cost <<  endl ;
                cout << "nb of nvalid neighbors " << nb_not_valid<< endl ;*/
                }
        }
    }




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
        nt::JSONDocument doc = buildSrpathsJsonFromRs(s.inst, s.rs);

        const std::string out_path = "../solution.json"; // adapte le chemin
        std::ofstream out(out_path);
        if (!out) {
            std::cerr << "Cannot open output file: " << out_path << std::endl;
        } else {
            out << nt::JSONDocument::toString(doc, 12) << '\n';
            out.close();
            std::cout << "JSON written to " << out_path << std::endl;
            // visualize the results
            string command = "python ../visualizer.py " + nInstance + " " + out_path;
            cout << "Running command: " << command << endl;
            system(command.c_str());
        }
    }
}

