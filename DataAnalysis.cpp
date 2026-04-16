//
// Created by uha on 01/04/2026.
//

#include "DataAnalysis.h"
#include <iostream>
#include <iomanip>
#include <queue>
#include <deque>
#include <set>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

PreprocessingEngine::PreprocessingEngine(const Instance& inst, const Scenario& scenario)
    : _inst(inst), _scenario(scenario) {}

// ============================================================================
// MAIN PREPROCESSING METHOD
// ============================================================================

PreprocessingData PreprocessingEngine::preprocess() {
    LOG_F(INFO, "========== Starting Preprocessing Engine ==========");

    // 1. Analyze network topology
    analyzeNetwork();
    LOG_F(INFO, "Network analysis completed");

    // 2. Analyze traffic matrix
    analyzeTraffic();
    LOG_F(INFO, "Traffic analysis completed");

    // 3. Analyze scenario constraints
    analyzeScenario();
    LOG_F(INFO, "Scenario analysis completed");

    // 4. Generate routing suggestions
    suggestWaypoints();
    LOG_F(INFO, "Waypoint suggestion completed");

    // 5. Check feasibility
    checkFeasibility();
    LOG_F(INFO, "Feasibility check completed");

    LOG_F(INFO, "========== Preprocessing Engine Finished ==========");
    return _data;
}

// ============================================================================
// NETWORK ANALYSIS METHODS
// ============================================================================

void PreprocessingEngine::analyzeNetwork() {
    computeNodeDegrees();
    identifyCriticalArcs();
    computeNetworkDiameter();

    // Compute average node degree
    double total_degree = 0.0;
    for (const auto& pair : _data.node_in_degree) {
        total_degree += pair.second;
    }
    _data.average_node_degree = total_degree / _inst.network.nodeNum();
}

void PreprocessingEngine::computeNodeDegrees() {
    // Initialize all nodes with degree 0
    for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
        _data.node_in_degree[node] = 0;
        _data.node_out_degree[node] = 0;
    }

    // Count degrees
    for (ArcIt arc(_inst.network); arc != nt::INVALID; ++arc) {
        Node source = _inst.network.source(arc);
        Node target = _inst.network.target(arc);
        _data.node_out_degree[source]++;
        _data.node_in_degree[target]++;
    }
    std::cout << "Node degrees computed for " << _inst.network.nodeNum() << " nodes" << std::endl;
}

void PreprocessingEngine::identifyCriticalArcs() {
    // Compute criticality for each arc
    for (ArcIt arc(_inst.network); arc != nt::INVALID; ++arc) {
        ArcMetrics metrics;
        metrics.arc = arc;
        metrics.arc_id = _inst.network.id(arc);
        metrics.capacity = _inst.capacities[arc];
        metrics.metric = _inst.metrics[arc];
        metrics.delay = _inst.delays[arc];

        Node source = _inst.network.source(arc);
        Node target = _inst.network.target(arc);
        metrics.in_degree = _data.node_in_degree[target];
        metrics.out_degree = _data.node_out_degree[source];

        // Compute criticality: low capacity + high demand potential
        // Criticality = 1/capacity * (in_degree + out_degree)
        double capacity_factor = (metrics.capacity > 0) ? (1.0 / metrics.capacity) : 1000.0;
        double degree_factor = metrics.in_degree + metrics.out_degree;
        metrics.criticality_score = capacity_factor * degree_factor;

        _data.critical_arcs.push_back(metrics);
    }

    // Sort by criticality descending
    std::sort(_data.critical_arcs.begin(), _data.critical_arcs.end(),
              [](const ArcMetrics& a, const ArcMetrics& b) {
                  return a.criticality_score > b.criticality_score;
              });

    // Count critical arcs (top 20% by criticality)
    _data.num_critical_arcs = std::max(1, (int)(_data.critical_arcs.size() * 0.2));

    std::cout<< "Identified "<<_data.num_critical_arcs<<" critical arcs out of "<<(int)_data.critical_arcs.size()<<std::endl;
}

void PreprocessingEngine::computeNetworkDiameter() {
    int diameter = 0;

    // BFS from each node to find eccentricity
    for (NodeIt source(_inst.network); source != nt::INVALID; ++source) {
        std::map<Node, int> distances;
        std::queue<Node> q;

        // Initialize
        for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
            distances[node] = -1;
        }

        distances[source] = 0;
        q.push(source);

        // BFS
        while (!q.empty()) {
            Node current = q.front();
            q.pop();

            for (OutArcIt arc(_inst.network, current); arc != nt::INVALID; ++arc) {
                Node next = _inst.network.target(arc);
                if (distances[next] == -1) {
                    distances[next] = distances[current] + 1;
                    q.push(next);
                }
            }
        }

        // Find maximum distance (eccentricity)
        int eccentricity = 0;
        for (const auto& pair : distances) {
            if (pair.second > 0) {
                eccentricity = std::max(eccentricity, pair.second);
            }
        }

        diameter = std::max(diameter, eccentricity);
    }

    _data.network_diameter = diameter;
    std::cout << "Network diameter: " << diameter <<" hops" << std::endl;
}

// ============================================================================
// TRAFFIC ANALYSIS METHODS
// ============================================================================

void PreprocessingEngine::analyzeTraffic() {
    computeTotalDemand();
    computeDemandMetrics();
    rankDemandCriticality();
}

void PreprocessingEngine::computeTotalDemand() {
    _data.total_demand_t0 = 0.0;
    _data.total_demand_t1 = 0.0;

    // Time slot 0
    if (_inst.i_num_time_slots > 0) {
        for (DemandArcIt arc(_inst.demand_graph); arc != nt::INVALID; ++arc) {
            _data.total_demand_t0 += _inst.dvms[0][arc];
        }
    }

    // Time slot 1
    if (_inst.i_num_time_slots > 1) {
        for (DemandArcIt arc(_inst.demand_graph); arc != nt::INVALID; ++arc) {
            _data.total_demand_t1 += _inst.dvms[1][arc];
        }
    }


    std::cout << "Total demand t0: " << _data.total_demand_t0 << " t=1 "
    << _data.total_demand_t1 << std::endl;
}

void PreprocessingEngine::computeDemandMetrics() {
    int demand_id = 0;

    for (DemandArcIt arc(_inst.demand_graph); arc != nt::INVALID; ++arc, ++demand_id) {
        DemandMetrics metrics;
        metrics.demand_arc = arc;
        metrics.demand_id = demand_id;
        metrics.source = _inst.demand_graph.source(arc);
        metrics.target = _inst.demand_graph.target(arc);

        // Get volumes
        metrics.volume_t0 = (_inst.i_num_time_slots > 0) ? _inst.dvms[0][arc] : 0.0;
        metrics.volume_t1 = (_inst.i_num_time_slots > 1) ? _inst.dvms[1][arc] : 0.0;
        metrics.total_volume = metrics.volume_t0 + metrics.volume_t1;


        // Compute shortest path length
        metrics.hop_count = bfsShortestPathLength(metrics.source, metrics.target);
        metrics.shortest_path = dijkstraShortestPath(metrics.source, metrics.target);

        _data.demand_metrics.push_back(metrics);
    }
}

void PreprocessingEngine::rankDemandCriticality() {
    // Compute criticality for each demand based on volume
    for (auto& metrics : _data.demand_metrics) {
        // Criticality = total volume (normalized)
        metrics.criticality_score = metrics.total_volume / std::max(_data.total_demand_t0, _data.total_demand_t1);

        _data.demand_criticality.push_back(
            {metrics.demand_id, metrics.criticality_score}
        );
    }

    // Sort by criticality descending
    std::sort(_data.demand_criticality.begin(), _data.demand_criticality.end(),
              [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                  return a.second > b.second;
              });
    std::cout << "Ranked " << (int)_data.demand_criticality.size() << " demands by criticality" << std::endl;

}

// ============================================================================
// SCENARIO ANALYSIS METHODS
// ============================================================================

void PreprocessingEngine::analyzeScenario() {
    analyzeInterventionImpacts();
}

void PreprocessingEngine::analyzeInterventionImpacts() {
    // For each time slot, check interventions
    for (int t = 0; t < _inst.i_num_time_slots; ++t) {
        if (t >= (int)_scenario.interventions.size()) break;

        const auto& interventions_at_t = _scenario.interventions[t];

        // For each arc ID in interventions
        for (int i = 0; i < interventions_at_t.size(); ++i) {
            int arc_id = interventions_at_t[i];

            // Find actual arc with this ID
            Arc intervention_arc = nt::INVALID;
            for (ArcIt arc(_inst.network); arc != nt::INVALID; ++arc) {
                if (_inst.network.id(arc) == arc_id) {
                    intervention_arc = arc;
                    break;
                }
            }

            if (intervention_arc == nt::INVALID) continue;

            InterventionImpact impact;
            impact.affected_arc = intervention_arc;
            impact.time_slot = t;
            impact.total_demand_loss = 0.0;
            impact.num_affected_demands = 0;

            // Find demands using this arc
            Node arc_source = _inst.network.source(intervention_arc);
            Node arc_target = _inst.network.target(intervention_arc);

            for (DemandArcIt demand(_inst.demand_graph); demand != nt::INVALID; ++demand) {
                Node s = _inst.demand_graph.source(demand);
                Node t_node = _inst.demand_graph.target(demand);

                // Check if this demand would naturally use this arc on shortest path
                // Simplified: if source-target path could reasonably use this arc
                if ((s == arc_source || t_node == arc_target) ||
                    (isConnected(s, arc_source) && isConnected(arc_target, t_node))) {

                    impact.affected_demands.push_back(demand);
                    if (t < _inst.i_num_time_slots) {
                        impact.total_demand_loss += _inst.dvms[t][demand];
                    }
                    impact.num_affected_demands++;
                }
            }

            _data.intervention_impacts.push_back(impact);
        }
    }
    std::cout << "Analyzed " << (int)_data.intervention_impacts.size() << " intervention impacts across " << _scenario.interventions.size() << " time slots" << std::endl;

}

void PreprocessingEngine::checkFeasibility() {
    _data.is_feasible = true;
    _data.feasibility_message = "Instance appears feasible";

    // Check 1: Network connectivity
    int num_connected = 0;
    for (DemandArcIt arc(_inst.demand_graph); arc != nt::INVALID; ++arc) {
        Node source = _inst.demand_graph.source(arc);
        Node target = _inst.demand_graph.target(arc);
        if (isConnected(source, target)) {
            num_connected++;
        }
    }

    if (num_connected < _inst.demand_graph.arcNum()) {
        _data.is_feasible = false;
        _data.feasibility_message = "Network is not fully connected for some demands";
        LOG_F(WARNING, "Only %d/%d demands have connected paths",
              num_connected, _inst.demand_graph.arcNum());
        return;
    }

    // Check 2: Capacity feasibility (rough estimate)
    double total_capacity = 0.0;
    for (ArcIt arc(_inst.network); arc != nt::INVALID; ++arc) {
        total_capacity += _inst.capacities[arc];
    }

    double max_demand = std::max(_data.total_demand_t0, _data.total_demand_t1);

    if (max_demand > total_capacity * 0.9) {
        LOG_F(WARNING, "High utilization: demand %.2f vs capacity %.2f (%.1f%%)",
              max_demand, total_capacity, (max_demand / total_capacity) * 100);
    }

    // Check 3: Budget constraints
    bool budget_tight = false;
    for (int t = 1; t < _scenario.budget.size(); ++t) {
        if (_scenario.budget[t] < 10) {
            budget_tight = true;

            std::cout <<"WARNING Tight budget at t= " <<t <<":"<<
                _scenario.budget[t] << " changes allowed"<< std::endl;
        }
    }


    std::cout <<"Feasibility check: " ;
    if (_data.is_feasible) {
        std::cout <<"PASS" << std::endl;
    }
    else {
        std::cout <<"FAIL" << std::endl;
    }
}

// ============================================================================
// ROUTING SUGGESTION METHODS
// ============================================================================

void PreprocessingEngine::suggestWaypoints() {
    // For each demand, generate waypoint candidates
    int demand_id = 0;
    for (DemandArcIt arc(_inst.demand_graph); arc != nt::INVALID; ++arc, ++demand_id) {
        Node source = _inst.demand_graph.source(arc);
        Node target = _inst.demand_graph.target(arc);

        // Get waypoints that avoid congestion
        std::vector<Node> candidates = getCongestionAvoidingWaypoints(source, target, 5);
        _data.waypoint_candidates[arc] = candidates;
    }


    std::cout << "Generated waypoint candidates for "<<  (int)_data.waypoint_candidates.size() << "demands\n" ;
}

std::vector<Node> PreprocessingEngine::dijkstraShortestPath(Node source, Node target) {
    std::vector<Node> path;
    std::map<Node, double> distances;
    std::map<Node, Node> predecessors;
    std::set<Node> unvisited;

    // Initialize
    for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
        distances[node] = std::numeric_limits<double>::infinity();
        unvisited.insert(node);
    }
    distances[source] = 0;

    while (!unvisited.empty()) {
        // Find unvisited node with minimum distance
        Node current = *unvisited.begin();
        double min_dist = distances[current];

        for (Node node : unvisited) {
            if (distances[node] < min_dist) {
                current = node;
                min_dist = distances[node];
            }
        }

        if (current == target || min_dist == std::numeric_limits<double>::infinity()) {
            break;
        }

        unvisited.erase(current);

        // Check neighbors
        for (OutArcIt arc(_inst.network, current); arc != nt::INVALID; ++arc) {
            Node next = _inst.network.target(arc);
            if (unvisited.find(next) != unvisited.end()) {
                double new_dist = distances[current] + _inst.metrics[arc];
                if (new_dist < distances[next]) {
                    distances[next] = new_dist;
                    predecessors[next] = current;
                }
            }
        }
    }

    // Reconstruct path
    Node current = target;
    while (current != source && predecessors.find(current) != predecessors.end()) {
        path.insert(path.begin(), current);
        current = predecessors[current];
    }
    path.insert(path.begin(), source);

    return path;
}

std::vector<std::vector<Node>> PreprocessingEngine::findAlternatePaths(Node source, Node target, int k) {
    std::vector<std::vector<Node>> paths;

    // For now, implement simple alternate path finding
    // This could be enhanced with Yen's algorithm or similar

    // Path 1: Direct shortest path
    paths.push_back(dijkstraShortestPath(source, target));

    // For additional paths, we could use node-disjoint or edge-disjoint paths
    // For simplicity, generating k random intermediate waypoints

    std::vector<Node> all_nodes;
    for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
        if (node != source && node != target) {
            all_nodes.push_back(node);
        }
    }

    for (int i = 1; i < k && i < (int)all_nodes.size(); ++i) {
        Node waypoint = all_nodes[i];
        std::vector<Node> path1 = dijkstraShortestPath(source, waypoint);
        std::vector<Node> path2 = dijkstraShortestPath(waypoint, target);

        std::vector<Node> combined_path = path1;
        // Avoid duplication of waypoint
        for (int j = 1; j < path2.size(); ++j) {
            combined_path.push_back(path2[j]);
        }

        paths.push_back(combined_path);
    }

    return paths;
}

double PreprocessingEngine::scoreWaypoint(Node waypoint, Node source, Node target) {
    // Score based on how well this waypoint helps avoid critical arcs
    double score = 0.0;

    // Get path through waypoint
    std::vector<Node> path1 = dijkstraShortestPath(source, waypoint);
    std::vector<Node> path2 = dijkstraShortestPath(waypoint, target);

    // Count how many critical arcs this path avoids
    int avoided_critical = 0;
    for (const ArcMetrics& critical : _data.critical_arcs) {
        if (avoided_critical >= _data.num_critical_arcs / 2) break;

        bool uses_critical = false;
        // Check if path uses this arc
        for (int i = 0; i < path1.size() - 1; ++i) {
            for (OutArcIt arc(_inst.network, path1[i]); arc != nt::INVALID; ++arc) {
                if (_inst.network.target(arc) == path1[i + 1] &&
                    _inst.network.id(arc) == critical.arc_id) {
                    uses_critical = true;
                    break;
                }
            }
        }

        if (!uses_critical) {
            avoided_critical++;
        }
    }

    score = avoided_critical;
    return score;
}

std::vector<Node> PreprocessingEngine::getCongestionAvoidingWaypoints(Node source, Node target, int max_candidates) {
    std::vector<std::pair<double, Node>> scored_candidates;

    // Score all potential waypoints
    for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
        if (node != source && node != target) {
            double score = scoreWaypoint(node, source, target);
            scored_candidates.push_back({score, node});
        }
    }

    // Sort by score descending
    std::sort(scored_candidates.begin(), scored_candidates.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    // Return top candidates
    std::vector<Node> result;
    for (int i = 0; i < std::min(max_candidates, (int)scored_candidates.size()); ++i) {
        result.push_back(scored_candidates[i].second);
    }

    return result;
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

int PreprocessingEngine::bfsShortestPathLength(Node source, Node target) {
    if (source == target) return 0;

    std::map<Node, int> distances;
    std::queue<Node> q;

    for (NodeIt node(_inst.network); node != nt::INVALID; ++node) {
        distances[node] = -1;
    }

    distances[source] = 0;
    q.push(source);

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        if (current == target) {
            return distances[target];
        }

        for (OutArcIt arc(_inst.network, current); arc != nt::INVALID; ++arc) {
            Node next = _inst.network.target(arc);
            if (distances[next] == -1) {
                distances[next] = distances[current] + 1;
                q.push(next);
            }
        }
    }

    return -1;  // Not connected
}

bool PreprocessingEngine::isConnected(Node source, Node target) {
    return bfsShortestPathLength(source, target) != -1;
}

std::string PreprocessingEngine::arcToString(const Arc& arc) const {
    std::stringstream ss;
    Node source = _inst.network.source(arc);
    Node target = _inst.network.target(arc);
    ss << "(" << _inst.names[source] << " -> " << _inst.names[target] << ")";
    return ss.str();
}

std::string PreprocessingEngine::nodeToString(const Node& node) const {
    return _inst.names[node];
}

// ============================================================================
// DISPLAY METHODS
// ============================================================================

void PreprocessingEngine::displayResults(int max_decimal_places) const {
    std::cout << "\n";
    std::cout << "========== PREPROCESSING RESULTS ==========" << std::endl;

    // Network statistics
    std::cout << "\n--- Network Statistics ---" << std::endl;
    std::cout << "Nodes: " << _inst.network.nodeNum() << std::endl;
    std::cout << "Arcs: " << _inst.network.arcNum() << std::endl;
    std::cout << "Network Diameter: " << _data.network_diameter << " hops" << std::endl;
    std::cout << "Average Node Degree: " << std::fixed << std::setprecision(2)
              << _data.average_node_degree << std::endl;

    // Traffic statistics
    std::cout << "\n--- Traffic Statistics ---" << std::endl;
    std::cout << "Total Demands: " << _inst.demand_graph.arcNum() << std::endl;
    std::cout << "Total Volume t=0: " << std::fixed << std::setprecision(2)
              << _data.total_demand_t0 << std::endl;
    std::cout << "Total Volume t=1: " << std::fixed << std::setprecision(2)
              << _data.total_demand_t1 << std::endl;

    // Top critical arcs
    std::cout << "\n--- Top 5 Critical Arcs ---" << std::endl;
    for (int i = 0; i < std::min(5, (int)_data.critical_arcs.size()); ++i) {
        const auto& arc_metric = _data.critical_arcs[i];
        std::cout << "  Arc " << arc_metric.arc_id
                  << ": Capacity=" << std::fixed << std::setprecision(2)
                  << arc_metric.capacity
                  << ", Criticality=" << arc_metric.criticality_score << std::endl;
    }

    // Top critical demands
    std::cout << "\n--- Top 5 Critical Demands ---" << std::endl;
    for (int i = 0; i < std::min(5, (int)_data.demand_criticality.size()); ++i) {
        const auto& demand = _data.demand_criticality[i];
        std::cout << "  Demand " << demand.first
                  << ": Criticality=" << std::fixed << std::setprecision(4)
                  << demand.second << std::endl;
    }

    // Intervention impacts
    std::cout << "\n--- Intervention Impacts ---" << std::endl;
    std::cout << "Total Interventions: " << _data.intervention_impacts.size() << std::endl;
    for (int i = 0; i < std::min(3, (int)_data.intervention_impacts.size()); ++i) {
        const auto& impact = _data.intervention_impacts[i];
        std::cout << "  Intervention at t=" << impact.time_slot
                  << ": Arc " << _inst.network.id(impact.affected_arc)
                  << ", Demands Affected: " << impact.num_affected_demands << std::endl;
    }

    // Feasibility
    std::cout << "\n--- Feasibility ---" << std::endl;
    std::cout << "Status: " << ((_data.is_feasible) ? "FEASIBLE" : "INFEASIBLE") << std::endl;
    std::cout << "Message: " << _data.feasibility_message << std::endl;

    std::cout << "\n========== END OF REPORT ==========" << std::endl;
}
