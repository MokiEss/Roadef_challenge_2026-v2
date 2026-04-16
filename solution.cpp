//
// Created by uha on 26/03/2026.
//

#include "solution.h"


vector<Node> solution::getNeighbors(const Node& source, bool TwoHops) const {
    vector<Node> neighbors;
    for (OutArcIt a(inst.network, source); a != nt::INVALID; ++a) {
        Node neigh = inst.network.target(a);
        if (neigh != source) {
            neighbors.push_back(neigh);
            if (TwoHops) {
                for (OutArcIt b(inst.network, neigh); b != nt::INVALID; ++b) {
                    Node neigh2 = inst.network.target(b);
                    if (neigh2 != neigh && neigh2 != source) {
                        neighbors.push_back(neigh2);
                    }
                }
            }
        }
    }
    return neighbors;
}




bool solution::printNodesUsedByDemandPath(const Instance& inst,
                                SegmentRouting& sr,
                                DemandArc demand_arc,
                                const SrPathBit& path) {
    if (path.segmentNum() == 0) {
        std::cout << "Demand " << inst.demand_graph.id(demand_arc) << " has empty SR path\n";
        return false;
    }

    const Node source = inst.demand_graph.source(demand_arc);
    const Node target = inst.demand_graph.target(demand_arc);

    nt::graphs::NodeSet<Digraph> visited(inst.network);
    std::vector<Node> ordered;
    ordered.reserve(inst.network.nodeNum());

    auto add_once = [&](Node n) {
        if (!visited.contains(n)) {
            visited.insert(n);
            ordered.push_back(n);
        }
        return true;
    };

    bool ok = path.forEachSegment(
        inst.network,
        [&](Node s, Node t) {
            add_once(s);
            if (s == t) {
                add_once(t);
                return true;
            }
            const bool seg_ok = sr.underlyingProtocol().forEachNode(
                s, t, [&](Node n) { return add_once(n); });
            add_once(t);
            return seg_ok;
        },
        [&](Arc a) {
            add_once(inst.network.source(a));
            add_once(inst.network.target(a));
        });

    // Print all internal/other nodes first, then force source and target at the end.
    std::cout << "Demand " << inst.demand_graph.id(demand_arc) << " uses nodes: ";
    std::cout << inst.network.id(source) << " ";
    for (Node n : ordered) {
        if (n != source && n != target) {
            std::cout << inst.network.id(n) << " ";
        }
    }

    if (target != source) std::cout << inst.network.id(target) << " ";
    std::cout << "\n";

    return ok;
}


bool solution::buildPathWithWaypointsCapped(SrPathBit& out_path,Node source,Node target,
                                            int max_segments, int requested_waypoints) const
{

    if (max_segments < 1) return false;

    // Max number of waypoint-nodes we can insert while respecting segment limit.
    // segments = 1 + num_waypoints
    const int max_waypoints_by_segments = max_segments - 1;
    const int max_waypoints_by_nodes = std::max(0, inst.network.nodeNum() - 2);
    const int wp_limit = std::min(max_waypoints_by_segments, max_waypoints_by_nodes);

    const int wp_count = std::max(0, std::min(requested_waypoints, wp_limit));

    std::vector<Node> candidates;
    candidates.reserve(inst.network.nodeNum());
    for (NodeIt n(inst.network); n != nt::INVALID; ++n) {
        if (n != source && n != target) candidates.push_back(n);
    }

    static thread_local std::mt19937 rng(std::random_device{}());
    std::shuffle(candidates.begin(), candidates.end(), rng);

    out_path.init(inst.network, wp_count + 2); // source + waypoints + target
    out_path.addSegment(source);

    Node last = source;
    for (int k = 0; k < wp_count; ++k) {
        const Node wp = candidates[k];
        out_path.addSegment(wp, last);
        last = wp;
    }

    out_path.finalize(target);

    return (out_path.segmentNum() - 1) <= max_segments;

}


bool solution::insertWaypointsIntoExistingPath(
    SrPathBit& io_path,
    DemandArc demand_arc,
    const std::vector<Node>& waypoints,
    int max_segments
) const {
    if (max_segments < 1) return false;

    // Handle empty path: initialize with source and target

    if (io_path.segmentNum() == 0) {
        Node source = inst.demand_graph.source(demand_arc);
        Node target = inst.demand_graph.target(demand_arc);
        io_path.init(inst.network, 2);  // capacity for source + target
        io_path.addSegment(source);
        io_path.finalize(target);
    }

    // Now path has at least 2 segments (source + target)
    if (io_path.segmentNum() < 2) return false;

    // SrPathBit mask logic is node-to-node; keep this method node-segment only.
    for (int i = 0; i < io_path.segmentNum(); ++i) {
        if (!io_path[i].isNode()) return false;
    }

    // Extract current node sequence from path.
    std::vector<Node> nodes;
    nodes.reserve(io_path.segmentNum());
    for (int i = 0; i < io_path.segmentNum(); ++i) {
        nodes.push_back(io_path[i].toNode());
    }

    const Node path_source = nodes.front();
    const Node path_target = nodes.back();

    // Current segment count is node_count - 1.
    const int current_segments = static_cast<int>(nodes.size()) - 1;
    const int remaining_segments = max_segments - current_segments;
    if (remaining_segments <= 0) return false;

    // Build filtered waypoint list:
    // - exclude source/target
    // - exclude nodes already in path
    // - deduplicate input order-preserving
    nt::graphs::NodeSet<Digraph> used(inst.network);
    for (Node n : nodes) used.insert(n);

    std::vector<Node> to_insert;
    to_insert.reserve(std::min<int>(waypoints.size(), remaining_segments));

    for (Node w : waypoints) {
        if (w == nt::INVALID) continue;
        if (w == path_source || w == path_target) continue;
        if (used.contains(w)) continue;

        to_insert.push_back(w);
        used.insert(w);

        if (static_cast<int>(to_insert.size()) == remaining_segments) break;
    }

    // If no waypoints to insert, path is already valid (source→target)
    if (to_insert.empty()) return true;

    // Rebuild path: keep old internal nodes, then append new waypoints before target.
    std::vector<Node> rebuilt_nodes;
    rebuilt_nodes.reserve(nodes.size() + to_insert.size());

    rebuilt_nodes.push_back(path_source);
    for (int i = 1; i < static_cast<int>(nodes.size()) - 1; ++i) {
        rebuilt_nodes.push_back(nodes[i]);
    }
    for (Node w : to_insert) {
        rebuilt_nodes.push_back(w);
    }
    rebuilt_nodes.push_back(path_target);

    SrPathBit rebuilt;
    rebuilt.init(inst.network, static_cast<int>(rebuilt_nodes.size()));
    rebuilt.addSegment(rebuilt_nodes[0]);

    Node last = rebuilt_nodes[0];
    for (int i = 1; i < static_cast<int>(rebuilt_nodes.size()) - 1; ++i) {
        rebuilt.addSegment(rebuilt_nodes[i], last);
        last = rebuilt_nodes[i];
    }
    rebuilt.finalize(path_target);

    // Final safety check.
    if ((rebuilt.segmentNum() - 1) > max_segments) return false;

    io_path = std::move(rebuilt);
    return true;
}

bool solution::removeWayPointFromExistingPath(const Node wp, SrPathBit& io_path, const DemandArc & demand_arc) {
    const Node source = inst.demand_graph.source(demand_arc);
    const Node target = inst.demand_graph.target(demand_arc);

    // Invalid request or forbidden removal (endpoints are not waypoints).
    if (wp == nt::INVALID || wp == source || wp == target) return false;

    // Keep behavior aligned with insert helper: initialize empty path to direct path.
    if (io_path.segmentNum() == 0) {
        io_path.init(inst.network, 2);
        io_path.addSegment(source);
        io_path.finalize(target);
        return false; // Nothing removed.
    }

    if (io_path.segmentNum() < 2) return false;

    // This helper assumes node-only SR paths (same assumption as insert helper).
    for (int i = 0; i < io_path.segmentNum(); ++i) {
        if (!io_path[i].isNode()) return false;
    }

    std::vector<Node> nodes;
    nodes.reserve(io_path.segmentNum());
    for (int i = 0; i < io_path.segmentNum(); ++i) {
        nodes.push_back(io_path[i].toNode());
    }

    // Defensive check: path should match demand endpoints.
    if (nodes.front() != source || nodes.back() != target) return false;

    bool removed = false;
    std::vector<Node> kept_internal;
    kept_internal.reserve(std::max(0, static_cast<int>(nodes.size()) - 2));

    // Remove all occurrences of wp in internal positions.
    for (int i = 1; i < static_cast<int>(nodes.size()) - 1; ++i) {
        if (nodes[i] == wp) {
            removed = true;
        } else {
            kept_internal.push_back(nodes[i]);
        }
    }

    if (!removed) return false;

    SrPathBit rebuilt;
    rebuilt.init(inst.network, static_cast<int>(kept_internal.size()) + 2);
    rebuilt.addSegment(source);

    Node last = source;
    for (Node n : kept_internal) {
        rebuilt.addSegment(n, last);
        last = n;
    }
    rebuilt.finalize(target);

    io_path = std::move(rebuilt);
    return true;
}


double solution::computeMLU(SegmentRouting & sr, int time_slot,  int& most_congested_arc_id,
    DemandArc demand_arc,const SrPathBit& old_path, const SrPathBit& path, Digraph::ArcMap<DemandArray> & dpa, bool update ) {
    double flow = inst.dvms[time_slot][demand_arc];
    // Remove old flow, add new flow

    sr.routeFlow(old_path, -flow,demand_arc,dpa);
    sr.routeFlow(path, flow,demand_arc,dpa);

    auto most = sr.mostLoadedArc(inst.capacities);
    double mlu = most.second;
    most_congested_arc_id = (most.first == nt::INVALID) ? -1 : Digraph::id(most.first);
    // Undo: restore to original state
    if (update == false) {
        sr.routeFlow(path, -flow,demand_arc,dpa);
        sr.routeFlow(old_path, flow,demand_arc,dpa);
    }
    return mlu ;
}



Node solution::selectGeometricWaypoint(Node s, Node d, Arc worst_arc, const NetworkPrecompute& precomp) const {
    const Node u = inst.network.source(worst_arc);
    const Node v = inst.network.target(worst_arc);

    const int sid = inst.network.id(s);
    const int did = inst.network.id(d);
    const int uid = inst.network.id(u);
    const int vid = inst.network.id(v);

    const double d_sd = precomp.dist_matrix[sid][did];
    if (!std::isfinite(d_sd) || d_sd <= 0.0) return nt::INVALID;

    Node best_w = nt::INVALID;
    double best_score = -std::numeric_limits<double>::infinity();

    for (NodeIt w(inst.network); w != nt::INVALID; ++w) {
        if (w == s || w == d || w == u || w == v) continue;

        const int wid = inst.network.id(w);
        const double d_sw = precomp.dist_matrix[sid][wid];
        const double d_wd = precomp.dist_matrix[wid][did];
        if (!std::isfinite(d_sw) || !std::isfinite(d_wd)) continue;

        const double detour = (d_sw + d_wd) / d_sd;
        if (detour > 1.35) continue;

        const double sep = std::min(precomp.dist_matrix[wid][uid], precomp.dist_matrix[wid][vid]);
        if (!std::isfinite(sep)) continue;

        const double balance = std::abs(d_sw - d_wd) / (d_sw + d_wd + 1e-9);
        const double score = 2.0 * sep - 1.2 * detour - 0.5 * balance;

        if (score > best_score) {
            best_score = score;
            best_w = w;
        }
    }

    return best_w;
}



double solution::computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id) {
    sr.clear();
    InterventionGuard intervention_guard(inst.network, inst.metrics, scenario.interventions[time_slot]);
    const int i_num_routed = sr.run(inst.demand_graph, inst.dvms[time_slot], test_rs.getSrPathsAt(time_slot));

    if (i_num_routed != inst.demand_graph.arcNum()) {
        most_congested_arc_id = -1;
        return std::numeric_limits<double>::infinity(); // Invalid routing
    }

    double f_mlu = 0.;
    most_congested_arc_id = -1;
    nt::Pair< Arc, double >  most = sr.mostLoadedArc(inst.capacities) ;
    f_mlu = most.second ;
    most_congested_arc_id = inst.network.id(most.first);

    return f_mlu;
}

vector<Node> getWayPointsCandidates(const Instance& inst, Node source, Node target) {
    vector<Node> candidates;
    for (NodeIt node(inst.network); node != nt::INVALID; ++node) {
        if (node != source && node != target) {
            candidates.push_back(node);
        }
    }

    return candidates;
}

void solution::computeAllPairsShortestPaths(NetworkPrecompute& precomp) {
    const int N = inst.network.nodeNum();
    precomp.dist_matrix.assign(N, std::vector<double>(N, std::numeric_limits<double>::infinity()));

    // Initialize diagonal to 0
    for (int i = 0; i < N; ++i) {
        precomp.dist_matrix[i][i] = 0.0;
    }

    // Build arc weight map: weight(a) = 1.0 / capacity(a)
    // Prefers high-capacity arcs — routes naturally avoid thin links
    typename Digraph::template ArcMap<double> arc_weight(inst.network);

    for (ArcIt a(inst.network); a != nt::INVALID; ++a) {
        double cap = inst.capacities[a];
        arc_weight[a] = (cap > 0.0) ? (1.0 / cap) : std::numeric_limits<double>::infinity();
    }

    // Dijkstra from every source node
    // Constructor requires both graph AND length_map
    nt::graphs::Dijkstra<Digraph, double> dijkstra(inst.network, arc_weight);

    for (NodeIt src(inst.network); src != nt::INVALID; ++src) {
        int s_id = inst.network.id(src);
        dijkstra.run(src);

        for (NodeIt dst(inst.network); dst != nt::INVALID; ++dst) {
            int d_id = inst.network.id(dst);
            if (dijkstra.reached(dst))
                precomp.dist_matrix[s_id][d_id] = dijkstra.dist(dst);
        }
    }
}


// ===========================
// New targeted heuristic
// ===========================



bool solution::newHeuristicRun() {
    result_builder.setValid(true);
    vector<int> costInterventions(inst.demand_graph.arcNum(),0);
    std::vector<vector<int>> nbSegmentsByDemand(inst.i_num_time_slots, vector<int>(inst.demand_graph.arcNum(), 1));
    int total_cost = 0;
    solution_mlu.reserve(inst.i_num_time_slots);

    static std::mt19937 rng(std::random_device{}());

    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        InterventionGuard intervention_guard(inst.network, inst.metrics, scenario.interventions[t]);

        // Initial direct paths
        for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
            DemandArc demand_arc = inst.demand_graph.arcFromId(i);
            SrPathBit & path = rs.getSrPath(t, demand_arc);
            // Initialize with direct path
            if (t==0) {
                std::vector<Node> waypoints = {};
               if (!insertWaypointsIntoExistingPath(path,demand_arc, waypoints,scenario.i_max_segments))
                    cout << "erreur " ;

            }
            else {
                path.copyFrom(rs.getSrPath(t-1, demand_arc));

            }
            nbSegmentsByDemand[t][i] = rs.getSrPath(t, demand_arc).segmentNum() - 1;
        }

        int max_iterations = 1000;

        while (max_iterations--) {
            // route  all the sr paths and get the objective
            sr.clear();
            int worst_arc_id;
            Digraph::ArcMap<DemandArray> dpa(inst.network);
            sr.run(inst.demand_graph, inst.dvms[t], rs.getSrPathsAt(t),dpa);
            auto most = sr.mostLoadedArc(inst.capacities);
            double best_mlu = most.second;
            solution_mlu[t] = best_mlu ;
            worst_arc_id = (most.first == nt::INVALID) ? -1 : Digraph::id(most.first);
            // end routing

            if (worst_arc_id == -1) break;

            Arc worst_arc = inst.network.arcFromId(worst_arc_id);
            Node u = inst.network.source(worst_arc);
            Node v = inst.network.target(worst_arc);

            // Collect neighbors of u (excluding v)
            std::vector<Node> neighbors;
            for (OutArcIt a(inst.network, u); a != nt::INVALID; ++a) {
                Node neigh = inst.network.target(a);
                if (neigh != v) {
                    neighbors.push_back(neigh);
                    for (OutArcIt b(inst.network, neigh); b != nt::INVALID; ++b) {
                        Node neigh2 = inst.network.target(b);
                        if (neigh2 != neigh && neigh2 != v) {
                            neighbors.push_back(neigh2);
                        }
                    }
                }
            }
            // Pick random neighbor as waypoint
            int RandomWayPoint = genRandomInt(neighbors.size());
            Node wp = neighbors[RandomWayPoint];
            if (neighbors.empty()) continue;
            int k = 0 ;
            int tmp_arc = worst_arc_id ;
            bool improved = false;
            // get the demands that are responsible for the most congested arc
            DemandArray &users = dpa[worst_arc];

            if (users.size() == 0) continue;
            int nb_users = users.size();

            while (k < nb_users  && (improved == false || tmp_arc == worst_arc_id)) {
                DemandArc demand_arc = users[k];
                int i = inst.demand_graph.id(demand_arc);
                SrPathBit &path = rs.getSrPath(t, demand_arc);
                SrPathBit backup;
                backup.copyFrom(path);

                // Try rerouting via wp
                path.init(inst.network, 3);  // source + waypoint + target
                path.clear();
                std::vector<Node> waypoints = {wp};
                if (!insertWaypointsIntoExistingPath(path,demand_arc, waypoints,scenario.i_max_segments))
                    cout << "erreur " ;

                double new_mlu = computeMLU(sr, t,  tmp_arc, demand_arc,backup, path, dpa, false );
                int cost = (t == 0) ? 0 :
                dist(rs.getSrPath(t - 1, demand_arc), path);
                if (new_mlu < best_mlu) {
                    if (t == 0 || (  total_cost-costInterventions[i]) + cost <= scenario.budget[t]) {
                        best_mlu = computeMLU(sr, t,  tmp_arc, demand_arc,backup, path, dpa, true );
                        if (t > 0)  {
                            total_cost-=costInterventions[i];
                            total_cost += cost;
                            costInterventions[i] = cost ;
                        }
                     /*  std::cout << "Improved MLU: " << best_mlu << " from demand " << i
                                  << " via arc " << tmp_arc << " total cost is " <<total_cost<< " using waypoint " << inst.network.id(wp) <<
                                      std::endl;*/
                        improved = true ;
                        solution_mlu[t] = best_mlu ;
                    }
                    else {
                        rs.setSrPaths(t,demand_arc, std::move(backup));
                    }
                }
                else {
                    rs.setSrPaths(t,demand_arc, std::move(backup));
                }
                nbSegmentsByDemand[t][i] = rs.getSrPath(t,demand_arc).segmentNum() - 1;
                k++;
            }

        }
        if (t > 0 && total_cost >= scenario.budget[t]) {
            cout << "Budget reached or excessed " << endl ;
            break ;
        }
    }

    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return false;
    }


    result_builder._i_total_segments = 0 ;
    result_builder._i_total_srpaths = 0 ;
    for(int i = 0 ; i < nbSegmentsByDemand.size() ; i++) {
        for (int j = 0 ; j < nbSegmentsByDemand[i].size() ; j++) {
            result_builder._i_total_segments += nbSegmentsByDemand[i][j];
            result_builder._i_total_srpaths += (nbSegmentsByDemand[i][j]>1);
        }
    }



    result_builder._i_total_cost = total_cost;
    std::sort(
    result_builder._sat_values.begin(),
    result_builder._sat_values.end(),
    [](const auto& a, const auto& b) {
        return a.third > b.third; // Sort descending by saturation
    }
    );
    // Copy sorted saturation values to solution_saturations
    solution_saturations.clear();
    solution_saturations.resize(inst.i_num_time_slots);
    for (const auto& triplet : result_builder._sat_values) {
        solution_saturations[triplet.first].push_back(triplet.third);
    }
    result_builder._time_slots.clear();
    result_builder._mlu_values.clear();
    result_builder._inv_values.clear();
    result_builder._jain_values.clear();
    result_builder._sat_values.clear();
    result_builder._i_total_cost = 0;
    result_builder._i_total_segments = 0;
    result_builder._i_total_srpaths = 0;
    result_builder.setValid(false);
    return true;
}
