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
 * @brief Models for solving the Multi-Commodity Flow (MCF) problem using compact and path-based formulations.
 *
 * This file provides two implementations of the Multi-Commodity Flow problem:
 * - McfArc: Arc-based compact formulation with flow variables on (demand, arc) pairs
 * - McfPath: Path-based formulation with column generation
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_MCF_H_
#define _NT_MCF_H_

#include "../tools.h"
#include "../demand_graph.h"
#include "../../mp/mp.h"
#include "../algorithms/dijkstra.h"
#include "../../core/tolerance.h"

#define CT_LOAD_NAME(IARC, ITIME) "ct_load_" + std::to_string(_p_digraph->id(IARC)) + "_" + std::to_string(ITIME)
#define CT_FLOW_NAME(IVERTEX, IDEMAND) \
  "ct_flow_" + std::to_string(_p_digraph->id(IVERTEX)) + "_" + std::to_string(_p_demand_graph->id(IDEMAND))
#define MCFPATH_X_NAME(IDEMAND, IPATH) "X_" + std::to_string(IDEMAND) + "_" + std::to_string(IPATH)
#define MCFPATH_CT_DEMAND_NAME(IDEMAND) "ct_demand_" + std::to_string(IDEMAND)
#define MCFPATH_CT_CAPACITY_NAME(IARC) "ct_capacity_" + std::to_string(IARC)

namespace nt {
  namespace graphs {

    /**
     * @class McfArc
     * @headerfile mcf.h
     * @brief Model for solving the Multi-Commodity Flow (MCF) problem using an arc-based compact formulation.
     *
     * This class implements the arc-based formulation of the Multi-Commodity Flow problem, where
     * flow variables are defined on the Cartesian product of demands and arcs. The model minimizes
     * the maximum link load subject to flow conservation and capacity constraints. It supports both
     * LP (continuous flow) and MIP (binary flow) formulations.
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam MPS The type of the LP/MIP solver used to solve the model.
     * @tparam FPTYPE Floating-point precision type (e.g., float, double, long double).
     * @tparam VEC Vector type for multi-period or multi-class flows (default: nt::Vec<FPTYPE, 1>).
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     */
    template < typename GR,
               typename MPS,
               typename FPTYPE = float,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename DVM = typename DG::template ArcMap< VEC > >
    struct McfArc {
      using Digraph = GR;
      using MPSolver = MPS;

      using FpType = FPTYPE;

      using DemandGraph = DG;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;
      using DemandValueMap = DVM;
      using DemandValue = typename DVM::Value;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(MPSolver);

      MPSolver&          _mps;
      Digraph const*     _p_digraph;
      DemandGraph const* _p_demand_graph;

      MPMultiDimVariable< 2 > _flow_vars;    //< Defined on the cartesian product : DemandArc x Arc
      MPVariable*             _p_load_var;   //< Load variable

      //--
      McfArc(MPSolver& mps) : _mps(mps), _p_digraph(nullptr), _p_demand_graph(nullptr), _p_load_var(nullptr){};

      /**
       * @brief Build the multi-commodity flow model with flow conservation and capacity constraints.
       *
       * This method constructs the complete LP/MIP model by creating flow variables for each
       * (demand, arc) pair, defining the objective function to minimize maximum load, and adding
       * flow conservation constraints at each node for each demand and capacity constraints for
       * each arc.
       *
       * @param digraph The network topology (directed graph).
       * @param demand_graph The demand graph defining source-destination pairs.
       * @param p_cm Pointer to the capacity map (nullptr means unit capacity).
       * @param p_dvm Pointer to the demand value map (nullptr means unit demand).
       */
      void build(const Digraph&        digraph,
                 const DemandGraph&    demand_graph,
                 const CapacityMap*    p_cm = nullptr,
                 const DemandValueMap* p_dvm = nullptr) {
        _p_digraph = &digraph;
        _p_demand_graph = &demand_graph;
        _mps.clear();

        // Create the load and flow variables
        _p_load_var = _mps.makeNumVar(0., mp::infinity(), "L");

        if (_mps.isLP())
          _flow_vars = _mps.makeMultiDimNumVar(0., mp::infinity(), "X_", demand_graph.arcNum(), digraph.arcNum());
        else
          _flow_vars = _mps.makeMultiDimBoolVar("X_", demand_graph.arcNum(), digraph.arcNum());

        // Create the objective function
        MPObjective* const p_objective = _mps.mutableObjective();
        p_objective->setCoefficient(loadVar(), 1);
        p_objective->setMinimization();

        // Add flow conservation constraints
        for (NodeIt node(digraph); node != INVALID; ++node) {
          for (DemandArcIt demand_arc(demand_graph); demand_arc != INVALID; ++demand_arc) {
            double f_diff_value = 0.;
            if (demand_graph.source(demand_arc) == node)
              f_diff_value = -1.;
            else if (demand_graph.target(demand_arc) == node)
              f_diff_value = 1.;

            MPConstraint* const p_constraint =
               _mps.makeRowConstraint(f_diff_value, f_diff_value, CT_FLOW_NAME(node, demand_arc));

            MPVariable* p_var = nullptr;
            for (OutArcIt out_arc(digraph, node); out_arc != INVALID; ++out_arc) {
              p_var = flowVar(demand_arc, out_arc);
              assert(p_var);
              if (p_var) p_constraint->setCoefficient(p_var, -1.0);
            }

            for (InArcIt in_arc(digraph, node); in_arc != INVALID; ++in_arc) {
              p_var = flowVar(demand_arc, in_arc);
              assert(p_var);
              if (p_var) p_constraint->setCoefficient(p_var, 1.0);
            }
          }
        }

        // Add load constraints
        for (ArcIt arc(digraph); arc != INVALID; ++arc) {
          for (int t = 0; t < VEC::size(); ++t) {
            MPConstraint* const p_constraint = _mps.makeRowConstraint(-mp::infinity(), 0., CT_LOAD_NAME(arc, 0));
            for (DemandArcIt demand_arc(demand_graph); demand_arc != INVALID; ++demand_arc) {
              const MPVariable* p_var = flowVar(demand_arc, arc);
              assert(p_var);
              p_constraint->setCoefficient(p_var, p_dvm ? (*p_dvm)[demand_arc][t] : 1.);
            }
            p_constraint->setCoefficient(loadVar(), p_cm ? -(*p_cm)[arc] : 1.);
          }
        }
      }

      /**
       * @brief Solve the multi-commodity flow problem.
       *
       * @return The result status of the optimization (OPTIMAL, INFEASIBLE, etc.).
       */
      mp::ResultStatus solve() { return _mps.solve(); }

      /**
       * @brief Get the optimal maximum load value.
       *
       * @return The objective value representing the maximum link load across all arcs.
       */
      double getLoad() const { return _mps.objective().value(); }

      /**
       * @brief Return the flow variable associated to the given demand and arc.
       *
       * @param demand_arc The demand arc from the demand graph.
       * @param arc The arc from the network digraph.
       * @return Pointer to the flow variable for this (demand, arc) pair.
       */
      MPVariable* flowVar(DemandArc demand_arc, Arc arc) noexcept {
        assert(_p_digraph && _p_demand_graph);
        return _flow_vars(_p_demand_graph->id(demand_arc), _p_digraph->id(arc));
      }

      /**
       * @brief Return the load variable representing the maximum load across all arcs.
       *
       * @return Pointer to the load variable L that appears in the objective function.
       */
      MPVariable* loadVar() noexcept {
        assert(_p_load_var);
        return _p_load_var;
      }
    };

    /**
     * @class McfPath
     * @headerfile mcf.h
     * @brief Model for solving the Multi-Commodity Flow (MCF) problem using a path-based formulation with column
     * generation.
     *
     * This class implements the path-based formulation of the Multi-Commodity Flow problem using
     * column generation. Flow variables are defined on paths rather than individual arcs. The model
     * starts with a restricted set of paths (shortest paths) and iteratively generates new improving paths
     * using Dijkstra's algorithm. This approach is particularly efficient for large networks.
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam LPS The type of the LP solver used to solve the model.
     * @tparam FPTYPE Floating-point precision type (e.g., float, double, long double).
     * @tparam VEC Vector type for multi-period or multi-class flows (default: nt::Vec<FPTYPE, 1>).
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     */
    template < typename GR,
               typename LPS,
               typename FPTYPE = float,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename DVM = typename DG::template ArcMap< VEC > >
    struct McfPath {
      using Digraph = GR;
      using LPSolver = LPS;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(LPSolver);

      using DemandGraph = DG;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;

      using VecTraits = VectorTraits< VEC >;
      using DemandValueMap = DVM;
      using DemandVecTraits = VectorTraits< typename DVM::Value >;
      using DemandValue = typename DemandVecTraits::value_type;

      using Dijkstra = graphs::Dijkstra< Digraph >;
      using LengthMap = Dijkstra::LengthMap;
      using DemandConsMap = typename DemandGraph::template StaticArcMap< MPConstraint* >;
      using ConstraintsArray = nt::TrivialMultiDimArray< MPConstraint*, 2 >;   //< Dim : |Demands| x T

      struct Column {
        MPVariable*                _p_var;
        TrivialDynamicArray< Arc > _path;
        DemandArc                  _demand_arc;

        constexpr double                            value() const noexcept { return _p_var->solution_value(); }
        constexpr const TrivialDynamicArray< Arc >& path() const noexcept { return _path; }
        constexpr int                               pathSize() const noexcept { return _path.size(); }
        constexpr DemandArc                         demand() const noexcept { return _demand_arc; }
        constexpr bool              hasArc(Arc arc) const noexcept { return _path.find(arc) != _path.size(); }
        constexpr const MPVariable* variable() const noexcept { return _p_var; }
      };

      LPSolver&          _lps;
      const Digraph&     _digraph;
      const DemandGraph& _demand_graph;
      Dijkstra           _dijkstra;
      LengthMap          _length_map;

      nt::DynamicArray< Column > _columns;
      MPVariable*                _p_load_var;   //< Load variable

      CapacityMap const*    _p_cm;
      DemandValueMap const* _p_dvm;

      ConstraintsArray _capacity_constraints;
      DemandConsMap    _demand_constraints;

      int _i_max_iter;
      int _i_total_gen_columns;

      nt::Tolerance< FPTYPE > _tolerance;

      /**
       * @brief Construct a new McfPath object.
       *
       * @param digraph The network topology (directed graph).
       * @param demand_graph The demand graph defining source-destination pairs.
       * @param lps Reference to the LP solver.
       */
      McfPath(const Digraph& digraph, const DemandGraph& demand_graph, LPSolver& lps) :
          _lps(lps), _digraph(digraph), _demand_graph(demand_graph), _dijkstra(digraph, _length_map),
          _length_map(digraph), _p_load_var(nullptr), _p_cm(nullptr), _p_dvm(nullptr), _i_max_iter(NT_INT32_MAX),
          _i_total_gen_columns(0), _tolerance(1e-5){};

      /**
       * @brief Clear the model by removing all columns and resetting the LP solver.
       */
      void clear() {
        _lps.clear();
        _columns.removeAll();
      }

      /**
       * @brief Add a new path variable (column) to the model for a given demand.
       *
       * @param demand_arc The demand arc this path serves.
       * @param path The sequence of arcs forming the path (moved into the column).
       * @return Reference to the newly created column.
       */
      Column& addColumn(const DemandArc& demand_arc, TrivialDynamicArray< Arc >&& path) noexcept {
        Column& c = _columns[_columns.add()];
        c._p_var = _lps.makeNumVar(0., mp::infinity(), MCFPATH_X_NAME(_demand_graph.id(demand_arc), _columns.size()));
        c._demand_arc = demand_arc;
        c._path = std::move(path);
        return c;
      }

      /**
       * @brief Get all initial and generated columns (path variables).
       *
       * @return Const reference to the array of all columns in the model.
       */
      constexpr const nt::DynamicArray< Column >& columns() const noexcept { return _columns; }

      /**
       * @brief Get the total number of columns generated during column generation.
       *
       * @return Total number of paths added to the model during the solve process.
       */
      constexpr int genColNum() const noexcept { return _i_total_gen_columns; }

      /**
       * @brief Build the initial restricted master problem with shortest paths.
       *
       * This method initializes the model by creating the load variable, objective function,
       * and generating initial feasible paths using shortest path routing. Demand and capacity
       * constraints are also created.
       *
       * @param p_cm Pointer to the capacity map (nullptr means unit capacity).
       * @param p_dvm Pointer to the demand value map (nullptr means unit demand).
       * @return true if the model was successfully built, false if the graph is empty or no paths exist.
       */
      bool build(const CapacityMap* p_cm = nullptr, const DemandValueMap* p_dvm = nullptr) {
        if (!_digraph.arcNum() || !_demand_graph.arcNum()) {
          LOG_F(ERROR, "Empty graph or demand graph");
          return false;
        }
        _p_cm = p_cm;
        _p_dvm = p_dvm;

        _init();

        // Create the load variable
        _p_load_var = _lps.makeNumVar(0., mp::infinity(), "L");

        // Create the objective function
        MPObjective* const p_objective = _lps.mutableObjective();
        p_objective->setCoefficient(loadVar(), 1);
        p_objective->setMinimization();

        //
        for (ArcIt arc(_digraph); arc != nt::INVALID; ++arc) {
          if (!capacity(arc))   // If arc capacity is 0, do not route over that arc
            _length_map[arc] = Dijkstra::Operations::infinity();
          else
            _length_map[arc] = 1;
        }

        // Insert initial SR paths variables and set the objective's coeff
        DemandArcIt demand_arc(_demand_graph);
        Node        prev = _demand_graph.source(demand_arc);
        _dijkstra.run(prev);

        for (; demand_arc != nt::INVALID; ++demand_arc) {
          const Node s = _demand_graph.source(demand_arc);
          if (prev != s) {
            _dijkstra.run(s);
            prev = s;
          }

          TrivialDynamicArray< Arc > path = _dijkstra.path(_demand_graph.target(demand_arc));
          if (path.empty()) {
            LOG_F(ERROR,
                  "There exists no path to route the demand {} -> {}",
                  _demand_graph.id(s),
                  _demand_graph.id(_demand_graph.target(demand_arc)));
            return false;
          }

          const Column& c = addColumn(demand_arc, std::move(path));

          // Create the associated demand constraint
          MPConstraint* const p_constraint =
             _lps.makeRowConstraint(1., mp::infinity(), MCFPATH_CT_DEMAND_NAME(_demand_graph.id(demand_arc)));
          p_constraint->setCoefficient(c.variable(), 1.);
          _demand_constraints[demand_arc] = p_constraint;
        }

        // Capacity constraints
        for (ArcIt arc(_digraph); arc != nt::INVALID; ++arc) {
          for (int t = 0; t < VecTraits::size(); ++t) {
            MPConstraint* const p_constraint =
               _lps.makeRowConstraint(-mp::infinity(), 0., MCFPATH_CT_CAPACITY_NAME(_digraph.id(arc)));
            p_constraint->setCoefficient(loadVar(), -capacity(arc));
            _capacity_constraints(_digraph.id(arc), t) = p_constraint;
            for (const Column& c: _columns) {
              if (c.hasArc(arc)) p_constraint->setCoefficient(c.variable(), demandValue(c.demand(), t));
            }
          }
        }

        return true;
      }


      /**
       * @brief Solve the multi-commodity flow problem using column generation.
       *
       * This method implements the column generation algorithm by iteratively solving the
       * restricted master problem, pricing new columns (paths), and adding them to the model
       * until no improving columns are found or the maximum number of iterations is reached.
       *
       * @return The result status of the optimization (OPTIMAL, INFEASIBLE, etc.).
       */
      mp::ResultStatus solve() noexcept {
        LOG_SCOPE_FUNCTION(INFO);

        _i_total_gen_columns = 0;

        mp::ResultStatus result = _lps.solve();
        if (result != mp::ResultStatus::OPTIMAL) {
          LOG_F(ERROR, "Cannot solve model. Error code : {}", getLoad());
          return result;
        }
        LOG_F(INFO, "Initial optimal value : {}", getLoad());

        for (int k = 0; k < _i_max_iter; ++k) {
          LOG_F(INFO, "Iteration {}", k);
          const int i_num_gen_columns = _priceColumns();
          if (!i_num_gen_columns) {
            LOG_F(INFO, "No new columns found. Stop.");
            break;
          }
          _updateModel(i_num_gen_columns);
          LOG_F(INFO, "Added columns : {}", i_num_gen_columns);
          result = _lps.solve();
          if (result != mp::ResultStatus::OPTIMAL) {
            LOG_F(ERROR, "Cannot solve model. Error code : {}", getLoad());
            return result;
          }
          LOG_F(INFO, "Optimal value : {}", getLoad());
          _i_total_gen_columns += i_num_gen_columns;
        }

        LOG_F(INFO, "Problem solved");
        LOG_F(INFO, "Optimal value : {}", getLoad());
        LOG_F(INFO, "Number of generated columns : {}", _i_total_gen_columns);

        return result;
      }

      /**
       * @brief Get the optimal maximum load value.
       *
       * @return The objective value representing the maximum link load across all arcs.
       */
      double getLoad() const noexcept { return _lps.objective().value(); }

      /**
       * @brief Get the dual value of a capacity constraint for a specific arc and time period.
       *
       * @param arc The arc from the network digraph.
       * @param t The time period or class index.
       * @return The dual variable value associated with the capacity constraint.
       */
      constexpr double dualCapacity(Arc arc, int t) const noexcept { return getCapacityConstraint(arc, t)._dual_value; }

      /**
       * @brief Get the capacity constraint associated with a specific arc and time period.
       *
       * @param arc The arc from the network digraph.
       * @param t The time period or class index.
       * @return The capacity constraint associated with the specified arc and time period.
       */
      constexpr MPConstraint& getCapacityConstraint(Arc arc, int t) noexcept {
        return *_capacity_constraints(_digraph.id(arc), t);
      }
      constexpr const MPConstraint& getCapacityConstraint(Arc arc, int t) const noexcept {
        return *_capacity_constraints(_digraph.id(arc), t);
      }

      /**
       * @brief Get the dual value of a demand constraint.
       *
       * @param demand_arc The demand arc from the demand graph.
       * @return The dual variable value associated with the demand satisfaction constraint.
       */
      constexpr double dualDemand(DemandArc demand_arc) const noexcept {
        return getDemandConstraint(demand_arc)._dual_value;
      }

      /**
       * @brief Get the demand constraint associated with a specific demand arc.
       *
       * @param demand_arc The demand arc from the demand graph.
       * @return The demand constraint associated with the specified demand arc.
       */
      constexpr MPConstraint& getDemandConstraint(DemandArc demand_arc) noexcept {
        return *_demand_constraints[demand_arc];
      }
      constexpr const MPConstraint& getDemandConstraint(DemandArc demand_arc) const noexcept {
        return *_demand_constraints[demand_arc];
      }


      /**
       * @brief Get the demand value for a specific demand arc and time period.
       *
       * @param demand_arc The demand arc from the demand graph.
       * @param t The time period or class index.
       * @return The demand value (returns 1 if no demand value map is provided).
       */
      constexpr DemandValue demandValue(DemandArc demand_arc, int t) const noexcept {
        if (_p_dvm) return DemandVecTraits::getElement((*_p_dvm)[demand_arc], t);
        return 1;
      }

      /**
       * @brief Get the capacity of a specific arc.
       *
       * @param arc The arc from the network digraph.
       * @return The capacity value (returns 1 if no capacity map is provided).
       */
      constexpr Capacity capacity(Arc arc) const noexcept {
        if (_p_cm) return (*_p_cm)[arc];
        return 1;
      }

      /**
       * @brief Return the load variable representing the maximum load across all arcs.
       *
       * @return Pointer to the load variable L that appears in the objective function.
       */
      constexpr MPVariable* loadVar() noexcept {
        assert(_p_load_var);
        return _p_load_var;
      }

      /**
       * @brief Initialize internal data structures for the column generation algorithm.
       *
       * This private method allocates memory for columns, builds the weight map for Dijkstra,
       * and initializes constraint arrays for capacity and demand constraints.
       */
      void _init() noexcept {
        clear();
        _columns.ensure(_demand_graph.arcNum());
        _capacity_constraints.setDimensions(_digraph.arcNum(), VecTraits::size());
        _demand_constraints.build(_demand_graph);
      }


      /**
       * @brief Price out new columns (paths) with negative reduced cost.
       *
       * This private method implements the pricing problem by computing reduced costs for each
       * demand using Dijkstra's algorithm with arc weights set to dual values.
       *
       * @return The number of new columns generated in this pricing iteration.
       */
      int _priceColumns() noexcept {
        int i_gen_columns = 0;

        for (DemandArcIt demand_arc(_demand_graph); demand_arc != nt::INVALID; ++demand_arc) {
          for (ArcIt arc(_digraph); arc != nt::INVALID; ++arc) {
            double w = 0.;
            for (int t = 0; t < VecTraits::size(); ++t)
              w += -dualCapacity(arc, t) * demandValue(demand_arc, t);
            assert(_tolerance.positiveOrZero(w));
            if (_tolerance.isZero(w)) w = 0.;
            _length_map[arc] = w;
          }

          const Node s = _demand_graph.source(demand_arc);
          const Node t = _demand_graph.target(demand_arc);
          _dijkstra.run(s, t);

          const double f_reduced_cost = -dualDemand(demand_arc) + _dijkstra.dist(t);
          if (_tolerance.positiveOrZero(f_reduced_cost)) continue;

          TrivialDynamicArray< Arc > path = _dijkstra.path(t);
          addColumn(demand_arc, std::move(path));
          ++i_gen_columns;
        }

        return i_gen_columns;
      }

      /**
       * @brief Update the model by adding coefficients for newly generated columns.
       *
       * This private method updates demand and capacity constraints by setting the appropriate
       * coefficients for the newly added path variables (columns).
       *
       * @param i_added_column The number of columns that were added and need to be integrated.
       */
      void _updateModel(int i_added_column) noexcept {
        while (i_added_column) {
          const int     i_col_index = _columns.size() - i_added_column;
          const Column& c = _columns[i_col_index];

          getDemandConstraint(c.demand()).setCoefficient(c.variable(), 1.);

          for (ArcIt arc(_digraph); arc != nt::INVALID; ++arc) {
            for (int t = 0; t < VecTraits::size(); ++t) {
              if (c._path.find(arc) != c._path.size())
                getCapacityConstraint(arc, t).setCoefficient(c.variable(), demandValue(c.demand(), t));
            }
          }

          --i_added_column;
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
