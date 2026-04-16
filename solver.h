//
// Created by uha on 14/04/2026.
//

#ifndef ROADEF_CHALLENGE_2026_SOLVER_H
#define ROADEF_CHALLENGE_2026_SOLVER_H
#include "solution.h"

struct neighbor {
    DemandArc da ;
    bool add = false;
    bool remove= false;
    bool replace= false ;
    int index_wp_remove=-1;
    int index_wp_add=-1;
    vector<double> saturations;
};

class solver {

public:

    Instance   &   inst;
    bool use_ftxui = false ;
    ResultBuilder & result_builder;
    Scenario & scenario;
    RoutingScheme * rs;
    int i_max_decimal_places = 12;
    SegmentRouting & sr ;
    solver(Instance   &   inst, bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario, SegmentRouting & sr):inst(inst),use_ftxui(use_ftxui),
            result_builder(result_builder),scenario(scenario), sr(sr) {
        rs = new RoutingScheme(inst) ;
    } ;

    // using roulette wheel, select one of the most congested arcs of a solution given in parameter
    int getOneOfTheMostCongestedArcs(const solution & sol, int t);
    // using roulette wheel, select one of the most contributing demands to an arc given in parameter
    DemandArc getOneOfTheMostContributingDemandsToArc(const solution & sol, int arc, int t);

    double update_sr_path_for_demand_arcs( solution & sol, const DemandArc & da, int t, neighbor & m);

    void apply_move_to_solution(solution & sol, const neighbor & m, int t);
    // optimize
    void optimize();

    bool isBetter(const vector<double> & sat1, const vector<double> & sat2, int & where);
};
#endif //ROADEF_CHALLENGE_2026_SOLVER_H