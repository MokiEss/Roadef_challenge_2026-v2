/*
 * Software Name : networktools
 * SPDX-FileCopyrightText: Copyright (c) Orange SA
 * SPDX-License-Identifier: MIT
 *
 * This software is distributed under the MIT licence,
 * see the "LICENSE" file for more details or https://opensource.org/license/MIT
 *
 * Authors: see CONTRIBUTORS.md
 * Software description: An efficient C++ library for modeling and solving network optimization problems
 */

/**
 * @file
 * @brief
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SRTE_H_
#define _NT_SRTE_H_

#include "../../mp/mp.h"
#include "../../te/algorithms/segment_routing.h"
#include "../../graphs/demand_graph.h"
#include "../../core/queue.h"
#include "../../core/tolerance.h"

#define SRTE_X_NAME(IDEMAND, IPATH) "X_" + std::to_string(IDEMAND) + "_" + std::to_string(IPATH)
#define SRTE_CT_LOAD_NAME(IARC) "ct_load_" + std::to_string(IARC)
#define SRTE_CT_DEMAND_NAME(IDEMAND) "ct_demand_" + std::to_string(IDEMAND)

#define DIST_INDEX(ISEG, IVERTEX) ((IVERTEX) + (ISEG)*_digraph.nodeNum())
#define W_INDEX(I, J) ((I)*_digraph.nodeNum() + (J))

namespace nt {
  namespace te {

    /**
     * @class SrteLp
     * @headerfile srte.h
     * @brief Segment Routing Traffic Engineering using Column Generation (SRTE-LP).
     *
     * This class implements the SRTE-LP model for solving the Segment Routing Traffic Engineering (SRTE) problem,
     * using column generation. The model maximizes satisfied demand volume while respecting capacity constraints
     * and SR path segment limits.
     *
     * Mathematical formulation:
     *
     * max  ∑   ∑     v(d) · Xdp
     *  x  p∈P d∈D(p)
     *
     *  s.t.
     *       ∑  Xdp ≤ 1, ∀d∈D
     *      p∈P
     *
     *       ∑   ∑     r (e, p) · v (d) · Xdp ≤ λ·c(e), ∀e∈E
     *      p∈P d∈D(p)
     *
     *      Xdp ∈ {0,1} ∀p∈P, ∀d∈D(p)
     *
     * Where:
     * - Xdp: binary variable indicating if demand d uses SR path p
     * - v(d): volume of demand d
     * - r(e,p): ratio of flow on arc e when using path p
     * - λ: maximum link utilization factor
     * - c(e): capacity of arc e
     *
     * Example usage:
     * @code
     * using MPEnv = nt::mp::glpk::MPEnv;
     * using LPModel = nt::mp::glpk::LPModel;
     *
     * // Setup network and demands
     * nt::graphs::ListDigraph network;
     * nt::graphs::DemandGraph<nt::graphs::ListDigraph> demand_graph(network);
     * nt::graphs::ListDigraph::ArcMap<double> capacities(network);
     * nt::graphs::ListDigraph::ArcMap<double> metrics(network);
     * nt::graphs::DemandGraph<nt::graphs::ListDigraph>::ArcMap<double> demands(demand_graph);
     *
     * // ... populate network, demands, capacities, metrics ...
     *
     * // Create LP solver and SRTE-LP instance
     * MPEnv env;
     * LPModel lp_solver("lp_solver", env);
     * nt::te::SrteLp<nt::graphs::ListDigraph, LPModel> srte(
     *  network, demand_graph, capacities, metrics, demands, lp_solver);
     *
     * // Configure parameters
     * srte._max_segments = 4;  // Max SR path length
     * srte.updateLambda(1.0);  // Set max utilization
     *
     * // Build and solve
     * if (srte.build()) {
     *   auto status = srte.solve();
     *   if (status == nt::mp::ResultStatus::OPTIMAL) {
     *     std::cout << "Satisfied demand: " << srte.optimalValue() << std::endl;
     *     std::cout << "Generated paths: " << srte.generatedColumns().size() << std::endl;
     *   }
     * }
     * @endcode
     *
     * Reference:
     * [1] Mathieu Jadin, Francois Aubry, Pierre Schaus, Olivier Bonaventure:
     *     CG4SR: Near Optimal Traffic Engineering for Segment Routing with Column Generation.
     *     INFOCOM 2019: 1333-1341
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam LPS The type of the LP solver used to solve the model.
     * @tparam FPTYPE Floating-point precision (float, double, long double, ...).
     * @tparam VEC The type of the vector used to store demands, flows, saturation values.
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam MM A concepts::ReadMap "readable" arc map storing the metrics of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     */
    template < typename GR,
               typename LPS,
               typename FPTYPE = double,
               typename VEC = Vec< FPTYPE, 1 >,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename MM = typename GR::template ArcMap< FPTYPE >,
               typename DVM = typename DG::template ArcMap< VEC > >
    struct SrteLp {
      using Digraph = GR;
      using LPModel = LPS;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(LPModel);

      using DemandGraph = DG;
      using DemandOutArcIt = typename DemandGraph::OutArcIt;
      using DemandInArcIt = typename DemandGraph::InArcIt;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;
      using DemandArcArray = TrivialDynamicArray< DemandArc >;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;
      using MetricMap = MM;
      using Metric = typename MetricMap::Value;
      using DemandValueMap = DVM;

      using VecTraits = VectorTraits< VEC >;
      using DemandVecTraits = VectorTraits< typename DVM::Value >;
      using DemandValue = typename DemandVecTraits::value_type;

      using SpArcs = TrivialDynamicArray< Arc >;

      using Float = FPTYPE;

      using FlowArcMap = typename Digraph::template StaticArcMap< VEC >;

      /**
       * @brief
       *
       */
      template < class MASTER >
      struct Pricer {
        using Master = MASTER;
        Master& _master;

        struct Prev {
          enum E_TYPE { E_NONE = 0, E_NODE = 1, E_ARC = 2 };

          int i_node;
          int i_arc;

          Prev() : i_node(-1), i_arc(-1) {}

          int type() const noexcept {
            if (i_node >= 0 && i_arc >= 0) return E_NODE | E_ARC;
            if (i_node >= 0) return E_NODE;
            if (i_arc >= 0) return E_ARC;
            return E_NONE;
          }

          void setPrevNode(int node) noexcept { i_node = node; }
          void setPrevArc(int arc) noexcept { i_arc = arc; }
        };

        TrivialMultiDimArray< Float, 2 > sol;    //< Dimension = MaxSegment * |V|
        TrivialMultiDimArray< Prev, 2 >  prev;   //< Dimension = MaxSegment * |V|
        TrivialMultiDimArray< Float, 2 > w;      //< Dimension = |V| * |V|

        Pricer(Master& _master) : _master(_master) { init(); }

        /**
         * @brief Generate a new column based on the dynamic algorithm minWeightSrPath from Aubry's
         * thesis.
         *
         * @param demand_arc
         *
         */
        bool generateColumnForDemand(DemandArc demand_arc) {
          const Digraph&     digraph = _master._digraph;
          const DemandGraph& demand_graph = _master._demand_graph;

          const int         i_orig = demand_graph.id(demand_graph.source(demand_arc));
          const int         i_dest = demand_graph.id(demand_graph.target(demand_arc));
          const DemandValue f_max_volume = _master.demandValueMax(demand_arc);

          for (int i_node = 0; i_node < digraph.nodeNum(); ++i_node)
            sol(0, i_node) = std::numeric_limits< Float >::max();

          sol(0, i_orig) = 0;
          prev.setBitsToOne();

          for (int i_seg = 1; i_seg <= _master._max_segments; ++i_seg) {
            for (int i_cur = 0; i_cur < digraph.nodeNum(); ++i_cur) {
              // Case 1 : Do not use the additional segment
              sol(i_seg, i_cur) = sol(i_seg - 1, i_cur);
              prev(i_seg, i_cur) = prev(i_seg - 1, i_cur);

              // Case 2 : Select a new vertex segment to reach i_cur from i_prev with enough
              // capacity at minimum cost
              for (int i_prev = 0; i_prev < digraph.nodeNum(); ++i_prev) {
                if (i_prev == i_cur) continue;

                const Float f_e2e_capacity =
                   _master._e2e_capacity(digraph.nodeFromId(i_prev), digraph.nodeFromId(i_cur));
                if (f_e2e_capacity * f_max_volume > _master._f_lambda) continue;

                const Float f_dist = sol(i_seg - 1, i_prev) + w(i_prev, i_cur);
                if (f_dist < sol(i_seg, i_cur)) {
                  sol(i_seg, i_cur) = f_dist;
                  prev(i_seg, i_cur).setPrevNode(i_prev);
                }
              }

              // TODO(Morgan) Case 3 : Select a new arc segment
              // ...
            }
          }

          // Check whether we found a path
          if (prev(_master._max_segments, i_dest).type() == Prev::E_NONE) return false;

          // Build the sr path
          Column c(digraph);
          c.demand_arc = demand_arc;
          SrPath< Digraph >& sr_path = c.sr_path;
          int                i_node = i_dest;
          Float              f_path_cost = 0.;
          int                i_max_segments = _master._max_segments;
          while (prev(i_max_segments, i_node).type() != Prev::E_NONE) {
            const Prev& p = prev(i_max_segments, i_node);
            switch (p.type()) {
              case Prev::E_NODE: {
                sr_path.addSegment(digraph.nodeFromId(p.i_node));
                const Float w_cpy = w(p.i_node, i_node);
                for (int t = 0; t < Master::VecTraits::size(); ++t)
                  f_path_cost += w_cpy * _master.demandValue(demand_arc, t);
                i_max_segments -= 1;
                i_node = p.i_node;
              } break;

              case Prev::E_ARC: {
                const Arc prev_arc = digraph.arcFromId(p.i_arc);
                sr_path.addSegment(prev_arc);
                for (int t = 0; t < Master::VecTraits::size(); ++t)
                  f_path_cost += _master.dualCapacity(prev_arc, t);
                i_max_segments -= 2;
                i_node = digraph.id(digraph.source(prev_arc));
              } break;

              case Prev::E_NODE | Prev::E_ARC: {
                const Arc prev_arc = digraph.arcFromId(p.i_arc);
                sr_path.addSegment(prev_arc);
                const Float w_cpy = w(p.i_node, digraph.id(digraph.source(prev_arc)));
                for (int t = 0; t < Master::VecTraits::size(); ++t) {
                  f_path_cost += w_cpy * _master.demandValue(demand_arc, t) + _master.dualCapacity(prev_arc, t);
                }
                i_max_segments -= 2;
                i_node = p.i_node;
              } break;

              default: assert(false); break;
            }
          }

          assert(sr_path.segmentNum() > 0);

          assert(digraph.id(sr_path.back().source(digraph)) == i_orig);

          const Float f_reduced_cost = _master.dualDemand(demand_arc) + f_path_cost - f_max_volume;
          if (_master._tolerance.positiveOrZero(f_reduced_cost)) return false;

          sr_path.reverse();
          if (digraph.id(sr_path.back().target(digraph)) != i_dest) sr_path.addSegment(digraph.nodeFromId(i_dest));

          // We cannot create a variable here with a call to makeNumVar() as this will desynchronize
          // the model and OR-tools will return a warning and all dual values will be 0
          c.p_var = nullptr;   // _master.makeNumVar(...)

          _master._segment_routing.computeRatios(c.sr_path, c.ratios);

          _master.pushColumn(std::move(c));

          return true;
        }

        /**
         * @brief
         *
         * @return
         */
        void init() noexcept {
          sol.setDimensions(_master._max_segments + 1, _master._digraph.nodeNum());
          prev.setDimensions(_master._max_segments + 1, _master._digraph.nodeNum());
          w.setDimensions(_master._digraph.nodeNum(), _master._digraph.nodeNum());
        }

        /**
         * @brief
         *
         * @return
         */
        int run() noexcept {
          const Digraph&     digraph = _master._digraph;
          const DemandGraph& demand_graph = _master._demand_graph;

          w.setBitsToZero();
          for (NodeIt u(digraph); u != INVALID; ++u) {
            for (NodeIt v(digraph); v != INVALID; ++v) {
              if (u == v) continue;
              const FlowArcMap& ratio_map = _master._e2e_ratio_map(u, v);
              const SpArcs&     sp_arcs_map = _master._e2e_sp_arcs_map(u, v);
              for (const Arc arc: sp_arcs_map)
                for (int t = 0; t < Master::VecTraits::size(); ++t)
                  w(digraph.id(u), digraph.id(v)) += ratio_map[arc].max() * _master.dualCapacity(arc, t);
            }
          }

          int i_num_columns = 0;
          for (DemandArcIt demand_arc(demand_graph);
               demand_arc != INVALID && i_num_columns < _master._i_max_gen_columns;
               ++demand_arc)
            i_num_columns += generateColumnForDemand(demand_arc);

          return i_num_columns;
        }
      };

      struct Column {
        MPVariable*       p_var;
        SrPath< Digraph > sr_path;
        DemandArc         demand_arc;
        FlowArcMap        ratios;
        Column(const Digraph& g) : ratios(g) {}
      };

      const Digraph&        _digraph;
      const DemandGraph&    _demand_graph;
      const CapacityMap&    _cm;
      const DemandValueMap& _dvm;

      LPModel& _lps;

      DynamicArray< Column > _columns;

      // Corresponds to cap(u,v) in Aubry's thesis notation
      graphs::StaticGraphMap< Digraph, Float, Node, Node >      _e2e_capacity;
      graphs::StaticGraphMap< Digraph, FlowArcMap, Node, Node > _e2e_ratio_map;
      graphs::StaticGraphMap< Digraph, SpArcs, Node, Node >     _e2e_sp_arcs_map;

      DynamicSegmentRouting< Digraph, FPTYPE, VEC, MM > _segment_routing;

      using ConstraintsArray = TrivialMultiDimArray< MPConstraint*, 2 >;   //< Dim : |Demands| x T
      ConstraintsArray                                       _capacity_constraints;
      typename DemandGraph::template ArcMap< MPConstraint* > _demand_constraints;

      Float _f_lambda;
      int   _i_max_iter;
      int   _i_total_gen_columns;
      int   _i_max_gen_columns;
      int   _max_segments;

      Pricer< SrteLp > _pricer;

      Tolerance< FPTYPE > _tolerance;

      SrteLp(const Digraph&        digraph,
             const DemandGraph&    demand_graph,
             const CapacityMap&    cm,
             const MetricMap&      mm,
             const DemandValueMap& dvm,
             LPModel&              lps) :
          _digraph(digraph),
          _demand_graph(demand_graph), _cm(cm), _dvm(dvm), _lps(lps), _e2e_capacity(digraph), _e2e_ratio_map(digraph),
          _e2e_sp_arcs_map(digraph), _segment_routing(digraph, mm), _demand_constraints(demand_graph), _f_lambda(1),
          _i_max_iter(NT_INT32_MAX), _i_total_gen_columns(0), _max_segments(4), _i_max_gen_columns(NT_INT32_MAX),
          _pricer(*this), _tolerance(1e-5){};

      /**
       * @brief
       *
       */
      void clear() {
        _lps.clear();
        _e2e_capacity.setBitsToZero();
        _columns.removeAll();
      }

      /**
       * @brief
       *
       * @param i_arc
       * @return
       */
      Float               dualCapacity(Arc arc, int t) const { return getCapacityConstraint(arc, t).dual_value(); }
      MPConstraint&       getCapacityConstraint(Arc arc, int t) { return *_capacity_constraints(_digraph.id(arc), t); }
      const MPConstraint& getCapacityConstraint(Arc arc, int t) const {
        return *_capacity_constraints(_digraph.id(arc), t);
      }

      /**
       * @brief
       *
       * @param i_arc
       * @return
       */
      Float         dualDemand(DemandArc demand_arc) const { return getDemandConstraint(demand_arc).dual_value(); }
      MPConstraint& getDemandConstraint(DemandArc demand_arc) { return *_demand_constraints[demand_arc]; }
      const MPConstraint& getDemandConstraint(DemandArc demand_arc) const { return *_demand_constraints[demand_arc]; }

      /**
       * @brief
       *
       * @param c
       */
      void pushColumn(Column&& c) { _columns.add(std::forward< Column >(c)); }

      /**
       * @brief
       *
       */
      const DynamicArray< Column >& generatedColumns() const noexcept { return _columns; }

      /**
       * @brief
       *
       * @param f_lambda
       */
      void updateLambda(Float f_lambda) {
        LOG_SCOPE_FUNCTION(INFO);
        _f_lambda = f_lambda;
        for (ArcIt arc(_digraph); arc != INVALID; ++arc)
          for (int t = 0; t < VecTraits::size(); ++t)
            getCapacityConstraint(arc, t).setUB(_f_lambda * capacity(arc));
        LOG_F(INFO, "New lambda : {}", _f_lambda);
      }


      constexpr DemandValue demandValue(DemandArc demand_arc, int t) const noexcept {
        return DemandVecTraits::getElement(_dvm[demand_arc], t);
      }

      constexpr DemandValue demandValueMax(DemandArc demand_arc) const noexcept {
        return DemandVecTraits::max(_dvm[demand_arc]);
      }

      constexpr Capacity capacity(Arc arc) const noexcept { return _cm[arc]; }

      /**
       * @brief Solves the SRTE-LP problem using column generation.
       *
       * @return The status of the final LP solve (OPTIMAL, INFEASIBLE, etc.).
       */
      mp::ResultStatus solve() {
        LOG_SCOPE_FUNCTION(INFO);
        _i_total_gen_columns = 0;

        mp::ResultStatus result = _lps.solve();
        if (result != mp::ResultStatus::OPTIMAL) return result;
        LOG_F(INFO, "Initial optimal value : {}", optimalValue());

        _pricer.init();
        for (int k = 0; k < _i_max_iter; ++k) {
          LOG_F(INFO, "Iteration {}", k);
          const int i_num_gen_columns = _pricer.run();
          if (!i_num_gen_columns) {
            LOG_F(INFO, " No new columns found. Stop.");
            break;
          }
          _updateModel(i_num_gen_columns);
          LOG_F(INFO, " Added columns : {} ( max : {})", i_num_gen_columns, _i_max_gen_columns);
          result = _lps.solve();
          if (result != mp::ResultStatus::OPTIMAL) return result;
          LOG_F(INFO, " Optimal value : {}", optimalValue());

          _i_total_gen_columns += i_num_gen_columns;
        }

        LOG_F(INFO, "Problem solved");
        LOG_F(INFO, " Optimal value : {}", optimalValue());
        LOG_F(INFO, " Number of generated columns : {}", _i_total_gen_columns);

        return result;
      }

      /**
       * @brief Returns the optimal objective value.
       *
       * This represents the total satisfied demand volume in the optimal solution.
       *
       * @return The optimal objective value (total routed demand).
       * @pre solve() must have been called and returned an optimal solution.
       */
      Float optimalValue() const noexcept { return _lps.objective().value(); }

      /**
       * @brief Builds the initial LP model with direct paths for each demand.
       *
       * @return true if the model was built successfully, false if graphs are empty.
       */
      bool build() {
        if (!_digraph.arcNum() || !_demand_graph.arcNum()) {
          LOG_F(ERROR, "Empty graph or demand graph");
          return false;
        }

        _init();

        // Create the objective function
        MPObjective* const p_objective = _lps.mutableObjective();
        p_objective->setMaximization();

        // Insert initial SR paths variables and set the objective's coeff
        _columns.removeAll();
        _columns.ensure(_demand_graph.arcNum());
        for (DemandArcIt demand_arc(_demand_graph); demand_arc != INVALID; ++demand_arc) {
          const int i_demand = _demand_graph.id(demand_arc);
          Column&   c = _columns[_columns.addEmplace(_digraph)];

          c.sr_path.addSegment(_demand_graph.source(demand_arc));
          c.sr_path.addSegment(_demand_graph.target(demand_arc));
          c.p_var = _lps.makeNumVar(0., mp::infinity(), SRTE_X_NAME(i_demand, 0));
          c.demand_arc = demand_arc;

          _segment_routing.computeRatios(c.sr_path, c.ratios);

          // Set obj coeff
          p_objective->setCoefficient(c.p_var, demandValueMax(demand_arc));

          // Create the associated demand constraint
          MPConstraint* const p_constraint = _lps.makeRowConstraint(0., 1., SRTE_CT_DEMAND_NAME(i_demand));
          p_constraint->setCoefficient(c.p_var, 1.);
          _demand_constraints[demand_arc] = p_constraint;
        }

        // Capacity constraints
        for (ArcIt arc(_digraph); arc != INVALID; ++arc) {
          for (int t = 0; t < VecTraits::size(); ++t) {
            MPConstraint* const p_constraint =
               _lps.makeRowConstraint(0., _f_lambda * capacity(arc), SRTE_CT_LOAD_NAME(0));
            _capacity_constraints(_digraph.id(arc), t) = p_constraint;
            for (const Column& c: _columns) {
              const Float r = VecTraits::getElement(c.ratios[arc], 0);
              if (r > 0.) p_constraint->setCoefficient(c.p_var, r * demandValue(c.demand_arc, t));
            }
          }
        }

        // Compute the e2e capacity values
        _e2e_capacity.setBitsToZero();
        for (NodeIt u(_digraph); u != INVALID; ++u) {
          for (NodeIt v(_digraph); v != INVALID; ++v) {
            if (u == v) continue;
            Float f_max_ratio = -1.0;

            FlowArcMap& ratio_map = _e2e_ratio_map(u, v);
            ratio_map.build(_digraph);

            SpArcs& sp_arcs_map = _e2e_sp_arcs_map(u, v);
            if (_segment_routing.computeRatios(u, v, ratio_map, sp_arcs_map)) {
              for (const Arc arc: sp_arcs_map) {
                const Float f_ratio = (ratio_map[arc] / capacity(arc)).max();
                if (f_ratio > f_max_ratio) {
                  _e2e_capacity(u, v) = f_ratio;
                  f_max_ratio = f_ratio;
                }
              }
            }
          }
        }

        return true;
      }

      /**
       * @brief
       *
       */
      void _init() noexcept {
        clear();
        _columns.ensure(_demand_graph.arcNum());
        _capacity_constraints.setDimensions(_digraph.arcNum(), VecTraits::size());
      }

      /**
       * @brief
       *
       * @param i_added_column
       */
      void _updateModel(int i_added_column) {
        SrteLp& m = *this;
        //--
        while (i_added_column) {
          const int                i_col_index = _columns._i_count - i_added_column;
          Column&                  c = _columns[i_col_index];
          const SrPath< Digraph >& sr_path = c.sr_path;

          c.p_var = _lps.makeNumVar(0., mp::infinity(), SRTE_X_NAME(0, 0));

          MPObjective* const p_objective = _lps.mutableObjective();
          p_objective->setCoefficient(c.p_var, demandValueMax(c.demand_arc));

          getDemandConstraint(c.demand_arc).setCoefficient(c.p_var, 1.);

          for (ArcIt arc(_digraph); arc != INVALID; ++arc) {
            const Float r = VecTraits::getElement(c.ratios[arc], 0);
            if (r > 0.) {
              for (int t = 0; t < VecTraits::size(); ++t)
                getCapacityConstraint(arc, t).setCoefficient(c.p_var, r * demandValue(c.demand_arc, t));
            }
          }

          --i_added_column;
        }
      }
    };

    /**
     * @class SrteUtilIlp
     * @headerfile srte.h
     * @brief Segment Routing Traffic Engineering for minimizing maximum link utilization (SRTE-UTIL-ILP).
     *
     * This class implements the SRTE-UTIL-ILP model which minimizes the maximum link utilization
     * (λ) while routing all demands using Segment Routing paths. Unlike SRTE-LP which maximizes
     * satisfied demand, this model assumes all demands must be routed and minimizes network congestion.
     *
     * Mathematical formulation:
     *
     *  min λ
     *    s.t.
     *          ∑ Xdp = 1, ∀d∈D
     *         p∈P
     *
     *          ∑    ∑  r (e, p) · v (d) · Xdp ≤ λ·c(e), ∀e∈E
     *         p∈P d∈D(p)
     *
     *         Xdp ∈ {0,1} ∀p∈P, ∀d∈D(p)
     *
     * Where:
     * - λ: maximum link utilization (to minimize)
     * - Xdp: binary variable indicating if demand d uses SR path p
     * - r(e,p): ratio of flow on arc e when using path p
     * - v(d): volume of demand d
     * - c(e): capacity of arc e
     *
     * This model requires a pre-generated set of candidate SR paths (typically from SRTE-LP),
     * then selects one path per demand to minimize maximum utilization. It uses MIP solving
     * to get exact integer solutions.
     *
     * Example usage:
     * @code
     *
     * using MPEnv = nt::mp::glpk::MPEnv;
     * using MIPModel = nt::mp::glpk::MIPModel;
     *
     * // Setup network and demands
     * nt::graphs::ListDigraph network;
     * nt::graphs::DemandGraph<nt::graphs::ListDigraph> demand_graph(network);
     * nt::graphs::ListDigraph::ArcMap<double> capacities(network);
     * nt::graphs::ListDigraph::ArcMap<double> metrics(network);
     * nt::graphs::DemandGraph<nt::graphs::ListDigraph>::ArcMap<double> demands(demand_graph);
     *
     * // ... populate network, demands ...
     * 
     * MPEnv env;
     *
     * // Create MIP solver and SRTE-UTIL-ILP instance
     * MIPModel mip_solver("mip_solver", env);
     * nt::te::SrteUtilIlp<nt::graphs::ListDigraph, MIPModel> srte_ilp(
     *   network, demand_graph, capacities, metrics, demands, mip_solver);
     *
     * // Add generated columns from SRTE-LP
     * srte_ilp.addColumn(demand_arc_0, sr_path_0, ratios_0);
     * srte_ilp.addColumn(demand_arc_1, sr_path_1, ratios_1);
     * srte_ilp.addColumn(demand_arc_2, sr_path_2, ratios_2);
     *
     * // Build and solve
     * if (srte_ilp.build()) {
     *   auto status = srte_ilp.solve();
     *   if (status == nt::mp::ResultStatus::OPTIMAL) {
     *     std::cout << "Min max utilization: " << srte_ilp.optimalValue() << std::endl;
     *     // Get optimal paths for each demand
     *     for (auto demand_arc : demand_graph) {
     *       auto* path = srte_ilp.optimalSRPath(demand_arc);
     *       // Use the path...
     *     }
     *   }
     * }
     * @endcode
     *
     * Reference:
     * [1] Mathieu Jadin, Francois Aubry, Pierre Schaus, Olivier Bonaventure:
     *     CG4SR: Near Optimal Traffic Engineering for Segment Routing with Column Generation.
     *     INFOCOM 2019: 1333-1341
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam MIPS The type of the MIP solver used to solve the model.
     * @tparam FPTYPE Floating-point precision (float, double, long double, ...).
     * @tparam VEC The type of the vector used to store demands, flows, saturation values.
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam MM A concepts::ReadMap "readable" arc map storing the metrics of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     */
    template < typename GR,
               typename MIPS,
               typename FPTYPE = double,
               typename VEC = Vec< FPTYPE, 1 >,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename MM = typename GR::template ArcMap< FPTYPE >,
               typename DVM = typename DG::template ArcMap< VEC > >
    struct SrteUtilIlp {
      using Digraph = GR;
      using MIPModel = MIPS;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(MIPModel);

      template < typename T >
      using ArcMap = typename Digraph::template ArcMap< T >;

      using DemandGraph = DG;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;
      using MetricMap = MM;
      using Metric = typename MetricMap::Value;
      using DemandValueMap = DVM;

      using VecTraits = VectorTraits< VEC >;
      using DemandVecTraits = VectorTraits< typename DVM::Value >;
      using DemandValue = typename DemandVecTraits::value_type;

      using Float = FPTYPE;

      using FlowArcMap = typename Digraph::template StaticArcMap< VEC >;

      struct Column {
        MPVariable const*        p_var;
        DemandArc                demand_arc;
        SrPath< Digraph > const* p_sr_path;
        FlowArcMap const*        p_ratios;
      };

      using ColumnArray = TrivialDynamicArray< Column >;

      const Digraph&        _digraph;
      const DemandGraph&    _demand_graph;
      CapacityMap const&    _cm;
      MetricMap const&      _mm;
      DemandValueMap const& _dvm;

      MIPModel& _mips;

      NumericalMap< int64_t, ColumnArray > _columns_per_demand;   // TODO : replace with DynamicArray

      Tolerance< FPTYPE > _tolerance;

      /**
       * @brief
       *
       * @param digraph
       * @param demand_graph
       * @param mips
       */
      SrteUtilIlp(const Digraph&        digraph,
                  const DemandGraph&    demand_graph,
                  const CapacityMap&    cm,
                  const MetricMap&      mm,
                  const DemandValueMap& dvm,
                  MIPModel&             mips) :
          _digraph(digraph),
          _demand_graph(demand_graph), _cm(cm), _mm(mm), _dvm(dvm), _mips(mips), _tolerance(1e-5){};

      /**
       * @brief Adds a candidate SR path (column) for a specific demand.
       *
       * Creates a new decision variable Xdp representing the option to route demand d
       * using SR path p. The path's flow ratios are stored for use in capacity constraints.
       * Call this method for each candidate path before calling build().
       *
       * @param k The demand arc this path serves.
       * @param p The SR path (sequence of segments).
       * @param r Arc flow ratios when routing demand k on path p.
       * @return Reference to the created column.
       */
      const Column& addColumn(const DemandArc& k, const SrPath< Digraph >& p, const FlowArcMap& r) noexcept {
        ColumnArray&      columns = _columns_per_demand[_demand_graph.id(k)];
        const MPVariable* pX = _mips.isMIP()
                                  ? _mips.makeBoolVar(SRTE_X_NAME(_demand_graph.id(k), columns.size()))
                                  : _mips.makeNumVar(0., 1., SRTE_X_NAME(_demand_graph.id(k), columns.size()));
        Column            c = {pX, k, &p, &r};
        return columns[columns.add(c)];
      }

      /**
       * @brief Returns the columns associated to the given demand
       *
       * @param demand_arc A demand arc
       *
       * @return An array of columns
       */
      constexpr const ColumnArray& getColumns(const DemandArc demand_arc) const noexcept {
        return _columns_per_demand[_demand_graph.id(demand_arc)];
      }

      /**
       * @brief Returns the optimal SR path of the given demand.
       *
       * @param demand_arc
       * @return
       */
      const SrPath< Digraph >* optimalSRPath(DemandArc demand_arc) const noexcept {
        const ColumnArray& columns = _columns_per_demand[_demand_graph.id(demand_arc)];
        for (const Column& c: columns) {
          if (_tolerance.positive(c.p_var->solution_value())) return c.p_sr_path;
        }
        assert(false);
        return nullptr;
      }

      /**
       * @brief
       *
       * @param demand_arc
       */
      constexpr DemandValue demandValue(DemandArc demand_arc, int t) const noexcept {
        return DemandVecTraits::getElement(_dvm[demand_arc], t);
      }

      /**
       * @brief
       *
       * @param arc
       */
      Capacity capacity(Arc arc) const noexcept { return _cm[arc]; }

      /**
       * @brief Builds the MIP model from added columns.
       *
       * Constructs the complete integer programming formulation including:
       * - Variable λ representing maximum link utilization (to minimize)
       * - Demand constraints ensuring exactly one path is selected per demand
       * - Capacity constraints ensuring arc loads don't exceed λ times capacity
       * - All columns (paths) previously added via addColumn()
       *
       * @return true if the model was built successfully, false if graphs are empty.
       * @pre addColumn() must have been called to add candidate paths for each demand.
       */
      bool build() noexcept {
        if (!_demand_graph.arcNum() || !_digraph.arcNum()) return false;

        // objective
        MPVariable*        p_L = _mips.makeNumVar(0., mp::infinity(), "Lambda");
        MPObjective* const p_objective = _mips.mutableObjective();
        p_objective->setCoefficient(p_L, 1);
        p_objective->setMinimization();

        // Demand constraints
        for (DemandArcIt demand_arc(_demand_graph); demand_arc != INVALID; ++demand_arc) {
          MPConstraint* const p_constraint =
             _mips.makeRowConstraint(1., 1., SRTE_CT_DEMAND_NAME(_demand_graph.id(demand_arc)));
          const ColumnArray& columns = _columns_per_demand[_demand_graph.id(demand_arc)];
          for (const Column& c: columns)
            p_constraint->setCoefficient(c.p_var, 1.);
        }

        // Capacity constraints
        for (ArcIt arc(_digraph); arc != INVALID; ++arc) {
          for (int t = 0; t < VecTraits::size(); ++t) {
            MPConstraint* const p_constraint =
               _mips.makeRowConstraint(-mp::infinity(), 0., SRTE_CT_LOAD_NAME(_digraph.id(arc)));
            p_constraint->setCoefficient(p_L, -capacity(arc));

            for (DemandArcIt demand_arc(_demand_graph); demand_arc != INVALID; ++demand_arc) {
              const ColumnArray& columns = _columns_per_demand[_demand_graph.id(demand_arc)];

              for (const Column& c: columns) {
                const Float r = VecTraits::getElement((*c.p_ratios)[arc], 0);
                if (r > 0.) p_constraint->setCoefficient(c.p_var, r * demandValue(demand_arc, t));
              }
            }
          }
        }

        return true;
      }


      /**
       * @brief Solves the MIP model to find optimal path selection.
       *
       * Invokes the MIP solver to find the path assignment that minimizes the
       * maximum link utilization while routing all demands.
       *
       * @return The status of the MIP solve (OPTIMAL, INFEASIBLE, etc.).
       */
      mp::ResultStatus solve() noexcept { return _mips.solve(); }


      /**
       * @brief Returns the optimal maximum link utilization (λ).
       *
       * This is the minimum achievable maximum link utilization when routing all demands
       * using the provided candidate paths.
       *
       * @return The optimal λ value (minimum maximum utilization).
       * @pre solve() must have been called and returned an optimal solution.
       */
      double optimalValue() const noexcept { return _mips.objective().value(); }
    };

  }   // namespace te
}   // namespace nt
#endif
