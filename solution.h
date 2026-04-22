//
// Created by uha on 26/03/2026.
//

#ifndef ROADEF_CHALLENGE_2026_HEURISTIC_H
#define ROADEF_CHALLENGE_2026_HEURISTIC_H
#include "readerInstance.h"
using DemandArray = nt::TrivialDynamicArray<DemandArc>;
// In Heuristic.h

struct NetworkPrecompute {
    std::vector<std::vector<double>>          dist_matrix;
    std::vector<std::unordered_set<int>>   neighbors;
};

struct CongestedArc {
    Arc arc;
    double saturation;

    bool operator<(const CongestedArc& other) const {
        return saturation > other.saturation;  // Sort descending
    }
};


// Remove flow of one demand from the network state
// Returns new arc loads, saturations, and MLU — without rerouting other demands


class solution {


public:
    NetworkPrecompute precomp;
    Instance   &   inst;
    vector<double> solution_mlu ;
    vector<vector<double>> solution_saturations;
    bool use_ftxui = false ;
    ResultBuilder & result_builder;
    Scenario & scenario;
    RoutingScheme rs;
    int i_max_decimal_places = 12;
    SegmentRouting & sr ;
    solution(Instance    &  inst,
        bool use_ftxui,
        ResultBuilder & result_builder,
        Scenario & scenario, SegmentRouting & sr):
    inst(inst),use_ftxui(use_ftxui),
            result_builder(result_builder),scenario(scenario),rs(inst), sr(sr) {
        solution_mlu.resize(inst.i_num_time_slots, std::numeric_limits<double>::infinity());
        solution_saturations.resize(inst.i_num_time_slots);  // Initialize in constructor
    } ;
    bool buildPathWithWaypointsCapped(
    SrPathBit& out_path,
    Node source,
    Node target,
    int max_segments,int requested_waypoints) const;


    int newHeuristicRun();
    double computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id);
    double computeMLU(SegmentRouting & sr, int time_slot,  int& most_congested_arc_id,
    DemandArc demand_arc,const SrPathBit& old_path, const SrPathBit& path, Digraph::ArcMap<DemandArray> & dpa, bool update);

    double computeMLU_no_dpa(SegmentRouting & sr, int time_slot,  int& most_congested_arc_id,
    DemandArc demand_arc,const SrPathBit& old_path, const SrPathBit& path, bool update );
    bool insertWaypointsIntoExistingPath(
    SrPathBit& io_path,
    DemandArc demand_arc,
    const std::vector<Node>& waypoints,
    int max_segments
) const ;
    bool removeWayPointFromExistingPath(const Node wp, SrPathBit& io_path, const DemandArc & demand_arc);

    bool printNodesUsedByDemandPath(const Instance& inst,
                                    SegmentRouting& sr,
                                    DemandArc demand_arc,
                                    const SrPathBit& path);
    void computeAllPairsShortestPaths(NetworkPrecompute& precomp) ;
    Node selectGeometricWaypoint(Node s, Node d, Arc worst_arc, const NetworkPrecompute& precomp) const;
    vector<Node> getNeighbors(const Node& source, const Node & target, const Node& ds,
    const Node& dt, bool TwoHops) const;
    vector<Node> getPromisingWaypoints(
    const Node& arc_source,
    const Node& arc_target,
    const Node& ds,
    const Node& dt,
    const Arc& worst_arc,
    bool twoHops,
    int top_k
) const;



private:



};


#endif //ROADEF_CHALLENGE_2026_HEURISTIC_H