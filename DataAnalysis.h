//
// Created by uha on 01/04/2026.
//

#ifndef ROADEF_CHALLENGE_2026_DATAANALYSIS_H
#define ROADEF_CHALLENGE_2026_DATAANALYSIS_H

#include "checker.h"
#include <map>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
// ============================================================================
// DATA STRUCTURES FOR PREPROCESSING
// ============================================================================

/**
 * @struct ArcMetrics
 * @brief Metrics for a single arc in the network
 */
struct ArcMetrics {
    Arc arc;
    int arc_id;
    double capacity;
    double metric;
    double delay;
    int in_degree;
    int out_degree;
    double criticality_score;  // based on capacity and demand
};

/**
 * @struct DemandMetrics
 * @brief Metrics for a single demand
 */
struct DemandMetrics {
    DemandArc demand_arc;
    int demand_id;
    Node source;
    Node target;
    double volume_t0;
    double volume_t1;
    double total_volume;
    double criticality_score;  // impact on network congestion
    std::vector<Node> shortest_path;
    double hop_count;
};

/**
 * @struct InterventionImpact
 * @brief Impact analysis for a maintenance intervention
 */
struct InterventionImpact {
    Arc affected_arc;
    int time_slot;
    std::vector<DemandArc> affected_demands;
    double total_demand_loss;  // total volume loss if arc fails
    int num_affected_demands;
};

/**
 * @struct PreprocessingData
 * @brief Comprehensive preprocessing results
 */
struct PreprocessingData {
    // Network analysis
    std::map<Node, int> node_in_degree;
    std::map<Node, int> node_out_degree;
    std::vector<ArcMetrics> critical_arcs;  // sorted by criticality descending

    // Traffic analysis
    std::vector<DemandMetrics> demand_metrics;
    std::vector<std::pair<int, double>> demand_criticality;  // (demand_id, score)
    double total_demand_t0 = 0.0;
    double total_demand_t1 = 0.0;

    // Routing suggestions
    std::map<DemandArc, std::vector<Node>> waypoint_candidates;
    std::map<DemandArc, std::vector<Node>> shortest_paths;

    // Intervention impact analysis
    std::vector<InterventionImpact> intervention_impacts;
    std::vector<DemandArc> affected_demands;

    // Network topology insights
    int network_diameter = 0;
    double average_node_degree = 0.0;
    int num_critical_arcs = 0;  // arcs with low capacity

    // Feasibility metrics
    bool is_feasible = true;
    std::string feasibility_message;
};

// ============================================================================
// PREPROCESSING ENGINE CLASS
// ============================================================================

/**
 * @class PreprocessingEngine
 * @brief Comprehensive preprocessing for the ROADEF 2026 challenge
 *
 * Performs network analysis, traffic profiling, demand ranking, and
 * generates routing suggestions and feasibility assessments.
 */
class PreprocessingEngine {
public:
    /**
     * @brief Constructor
     * @param inst Reference to the problem instance
     * @param scenario Reference to the scenario constraints
     */
    PreprocessingEngine(const Instance& inst, const Scenario& scenario);

    /**
     * @brief Main preprocessing method
     * @return PreprocessingData with all analysis results
     */
    PreprocessingData preprocess();

    /**
     * @brief Display preprocessing results
     */
    void displayResults(int max_decimal_places = 12) const;

private:
    // References to problem data
    const Instance& _inst;
    const Scenario& _scenario;
    PreprocessingData _data;

    // ========================================================================
    // PRIVATE METHODS FOR NETWORK ANALYSIS
    // ========================================================================

    /**
     * @brief Analyze network topology (degrees, connectivity)
     */
    void analyzeNetwork();

    /**
     * @brief Compute node degrees (in/out)
     */
    void computeNodeDegrees();

    /**
     * @brief Identify critical arcs based on capacity and demand
     */
    void identifyCriticalArcs();

    /**
     * @brief Compute network diameter using BFS
     */
    void computeNetworkDiameter();

    // ========================================================================
    // PRIVATE METHODS FOR TRAFFIC ANALYSIS
    // ========================================================================

    /**
     * @brief Analyze traffic matrix and demands
     */
    void analyzeTraffic();

    /**
     * @brief Compute total demand per time slot
     */
    void computeTotalDemand();

    /**
     * @brief Rank demands by criticality
     */
    void rankDemandCriticality();

    /**
     * @brief Compute demand metrics (volume, path length, etc.)
     */
    void computeDemandMetrics();

    // ========================================================================
    // PRIVATE METHODS FOR SCENARIO ANALYSIS
    // ========================================================================

    /**
     * @brief Analyze scenario constraints and interventions
     */
    void analyzeScenario();

    /**
     * @brief Analyze intervention impacts on demands
     */
    void analyzeInterventionImpacts();

    /**
     * @brief Check feasibility of constraints
     */
    void checkFeasibility();

    // ========================================================================
    // PRIVATE METHODS FOR ROUTING SUGGESTIONS
    // ========================================================================

    /**
     * @brief Generate waypoint candidates for each demand
     */
    void suggestWaypoints();

    /**
     * @brief Find shortest path between two nodes (Dijkstra)
     * @param source Source node
     * @param target Target node
     * @return Vector of nodes representing shortest path
     */
    std::vector<Node> dijkstraShortestPath(Node source, Node target);

    /**
     * @brief Find k shortest paths for congestion avoidance
     * @param source Source node
     * @param target Target node
     * @param k Number of paths to find
     * @return Vector of alternate paths
     */
    std::vector<std::vector<Node>> findAlternatePaths(Node source, Node target, int k = 3);

    /**
     * @brief Score waypoint based on congestion avoidance potential
     * @param waypoint Candidate waypoint
     * @param source Source node
     * @param target Target node
     * @return Score (higher is better for avoiding congested areas)
     */
    double scoreWaypoint(Node waypoint, Node source, Node target);

    /**
     * @brief Get nodes that avoid congested arcs
     * @param source Source node
     * @param target Target node
     * @param max_candidates Maximum candidates to return
     * @return Vector of candidate waypoints sorted by score
     */
    std::vector<Node> getCongestionAvoidingWaypoints(Node source, Node target, int max_candidates = 5);

    // ========================================================================
    // UTILITY METHODS
    // ========================================================================

    /**
     * @brief BFS to compute shortest path length
     */
    int bfsShortestPathLength(Node source, Node target);

    /**
     * @brief Check if two nodes are connected
     */
    bool isConnected(Node source, Node target);

    /**
     * @brief Convert arc to string for logging
     */
    std::string arcToString(const Arc& arc) const;

    /**
     * @brief Convert node to string for logging
     */
    std::string nodeToString(const Node& node) const;
};

#endif //ROADEF_CHALLENGE_2026_DATAANALYSIS_H