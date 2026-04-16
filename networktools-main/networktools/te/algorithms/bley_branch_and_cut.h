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

#ifndef _NT_BLEY_BRANCH_AND_CUT_H_
#define _NT_BLEY_BRANCH_AND_CUT_H_

#include "../../core/arrays.h"
#include "../../mp/mp.h"
#include "../../graphs/demand_graph.h"
#include "../../graphs/algorithms/isp.h"

#define X_NAME(IDEMAND, IARC)                                                \
  "X_" + std::to_string(demand_graph.id(demand_graph.source(IDEMAND))) + "," \
     + std::to_string(demand_graph.id(demand_graph.target(IDEMAND))) + "_"   \
     + std::to_string(graph.id(graph.source(IARC))) + "," + std::to_string(graph.id(graph.target(IARC)))
#define L_NAME() "L"
#define BLEY_CT_FLOW_NAME(IDEMAND, IVERTEX) \
  "ct_flow_d" + std::to_string(demand_graph.id(IDEMAND)) + "_v" + std::to_string(graph.id(IVERTEX))
#define BLEY_CT_LOAD_NAME(IARC) "ct_load_a" + std::to_string(graph.id(IARC))

namespace nt {
  namespace te {

    /**
     * @class BleyBranchAndCut
     * @headerfile bley_branch_and_cut.h
     * @brief Branch-and-cut algorithm (based on [1]) for solving the minimum
     * congestion USPR (unsplittable shortest path routing) problem defined as
     * follows
     *
     * Minimum Congestion USPR
     * Input : A digraph D=(V,A) with arc capacities c_a ∊ Z, a ∊ A, and a
     * commodity set K with demands d_st ∊ Z for st ∊ K.
     * Output : A metric m_a ∊ Z for each a ∊ A such that the shortest (s,t)-path
     * w.r.t. to the metrics is uniquely determined for each commodity st ∊ K.
     * Objective : Minimize the most congested arc.
     *
     * [1] Andreas Bley:
     * An Integer Programming Algorithm for Routing Optimization in IP Networks
     * Algorithmica 60: 21-45 (2011)
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam MIPS The type of the MIP solver used to solve the master model.
     * @tparam LPS The type of the LP solver used to solve Inverse Shortest Path (ISP) problem.
     * @tparam DG The type of the digraph representing the demand graph. It must be defined on the
     * same node set as GR.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam DVM A concepts::ReadMap "readable" arc map storing the demand values.
     */
    template < typename GR,
               typename MIPS,
               typename LPS,
               typename DG = graphs::DemandGraph< GR >,
               typename CM = typename GR::template ArcMap< float >,
               typename DVM = typename DG::template ArcMap< float > >
    struct BleyBranchAndCut {
      using Digraph = GR;
      using LPModel = LPS;
      using MIPModel = MIPS;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(MIPModel);

      using DemandGraph = DG;
      using DemandOutArcIt = typename DemandGraph::OutArcIt;
      using DemandInArcIt = typename DemandGraph::InArcIt;
      using DemandArcIt = typename DemandGraph::ArcIt;
      using DemandArc = typename DemandGraph::Arc;

      using CapacityMap = CM;
      using Capacity = typename CM::Value;
      using DemandValueMap = DVM;
      using DemandValue = typename DVM::Value;

      using InverseShortestPath = graphs::InverseShortestPath< Digraph, LPModel >;
      using ForwardArc = typename InverseShortestPath::ForwardArc;

      enum {
        SUBPATH = 1 << 0,    // Subpath consistency constraints
        USPF = 1 << 1,       // Unique forwarding shortest path constraints
        ANTIARBO = 1 << 2,   // Anti arborescence constraint
        DE = 1 << 3          // Direct demand constraint
      };

      struct LazyCutCallback: MPCallback {
        BleyBranchAndCut&                            master;
        nt::TrivialDynamicArray< const MPVariable* > forwarding_vars;

        LazyCutCallback(BleyBranchAndCut& Master) : master(Master), MPCallback(false, true, false) {}
        void runCallback(MPCallbackContext* p_context) override {
          assert(p_context);
          LazyCutCallback& m = *this;

          // Get problem data
          assert(m.master._p_graph && m.master._p_demand_graph);
          const Digraph&     graph = *m.master._p_graph;
          const DemandGraph& demand_graph = *m.master._p_demand_graph;

          // Add subpath inequalities
          if (m.master._i_inequalities_flag & BleyBranchAndCut::SUBPATH) {
            ZoneScoped;
            for (ArcIt arc(graph); arc != nt::INVALID; ++arc) {
              for (DemandArcIt st_demand(demand_graph); st_demand != nt::INVALID; ++st_demand) {
                const Node s = demand_graph.source(st_demand);

                MPVariable* const X_st_a = m.master.X(st_demand, arc);
                const double      f_X_st_a = p_context->variableValue(X_st_a);

                //
                for (DemandOutArcIt sv_demand(demand_graph, s); sv_demand != nt::INVALID; ++sv_demand) {
                  if (sv_demand == st_demand) continue;
                  MPVariable* const X_sv_a = m.master.X(sv_demand, arc);
                  const double      f_X_sv_a = p_context->variableValue(X_sv_a);

                  double     f_sum = 0.;
                  LinearExpr sum_expr;
                  const Node v = demand_graph.target(sv_demand);
                  for (InArcIt in_arc(graph, v); in_arc != nt::INVALID; ++in_arc) {
                    MPVariable* const X_st_e = m.master.X(st_demand, in_arc);
                    f_sum += p_context->variableValue(X_st_e);
                    sum_expr += X_st_e;
                  }

                  if (f_X_sv_a - f_X_st_a + f_sum > 1.5) {
                    sum_expr += X_sv_a;
                    sum_expr -= X_st_a;
                    const LinearRange lazy_cut(-mp::infinity(), sum_expr, 1.);
                    ++master._i_num_cutoff_subpath;
                    p_context->addLazyConstraint(lazy_cut);
                    return;
                  }
                }

                //
                const Node t = demand_graph.target(st_demand);
                for (DemandInArcIt vt_demand(demand_graph, t); vt_demand != nt::INVALID; ++vt_demand) {
                  if (vt_demand == st_demand) continue;
                  MPVariable* const X_vt_a = m.master.X(vt_demand, arc);
                  const double      f_X_vt_a = p_context->variableValue(X_vt_a);

                  double     f_sum = 0.;
                  LinearExpr sum_expr;
                  const Node v = demand_graph.source(vt_demand);
                  for (InArcIt in_arc(graph, v); in_arc != nt::INVALID; ++in_arc) {
                    MPVariable* const X_st_e = m.master.X(st_demand, in_arc);
                    f_sum += p_context->variableValue(X_st_e);
                    sum_expr += X_st_e;
                  }

                  if (f_X_vt_a - f_X_st_a + f_sum > 1.5) {
                    sum_expr += X_vt_a;
                    sum_expr -= X_st_a;
                    const LinearRange lazy_cut(-mp::infinity(), sum_expr, 1.);
                    ++master._i_num_cutoff_subpath;
                    p_context->addLazyConstraint(lazy_cut);
                    return;
                  }
                }

                //
                for (DemandOutArcIt sv_demand(demand_graph, s); sv_demand != nt::INVALID; ++sv_demand) {
                  for (DemandInArcIt vt_demand(demand_graph, t); vt_demand != nt::INVALID; ++vt_demand) {
                    if (demand_graph.source(vt_demand) != demand_graph.target(sv_demand)) continue;

                    MPVariable* const X_sv_a = m.master.X(sv_demand, arc);
                    const double      f_X_sv_a = p_context->variableValue(X_sv_a);
                    MPVariable* const X_vt_a = m.master.X(vt_demand, arc);
                    const double      f_X_vt_a = p_context->variableValue(X_vt_a);

                    double     f_sum = 0.;
                    LinearExpr sum_expr;
                    const Node v = demand_graph.source(vt_demand);
                    for (InArcIt in_arc(graph, v); in_arc != nt::INVALID; ++in_arc) {
                      MPVariable* const X_st_e = m.master.X(st_demand, in_arc);
                      f_sum += p_context->variableValue(X_st_e);
                      sum_expr += X_st_e;
                    }

                    if (f_X_sv_a + f_X_vt_a - f_X_st_a + 2 * f_sum > 2.5) {
                      sum_expr *= 2;
                      sum_expr += X_sv_a;
                      sum_expr += X_vt_a;
                      sum_expr -= X_st_a;
                      const LinearRange lazy_cut(-mp::infinity(), sum_expr, 2.);
                      ++master._i_num_cutoff_subpath;
                      p_context->addLazyConstraint(lazy_cut);
                      return;
                    }
                  }
                }
              }
            }
          }

          // Add anti arborescence inequalities
          if (m.master._i_inequalities_flag & BleyBranchAndCut::ANTIARBO) {
            ZoneScoped;
            for (NodeIt v(graph); v != nt::INVALID; ++v) {
              for (NodeIt t(graph); t != nt::INVALID; ++t) {
                //--
                DemandInArcIt demand_arc(demand_graph, t);
                if (demand_arc == nt::INVALID) continue;

                LinearExpr new_constraint;
                double     f_sum = 0.;
                for (OutArcIt out_arc(graph, v); out_arc != nt::INVALID; ++out_arc) {
                  double      f_max_value = 0.;
                  MPVariable* X = nullptr;
                  do {
                    X = m.master.X(demand_arc, out_arc);
                    f_max_value = p_context->variableValue(X);
                    if (f_max_value > .5) break;
                  } while (++demand_arc != nt::INVALID);
                  new_constraint += X;
                  f_sum += f_max_value;
                  demand_arc = DemandInArcIt(demand_graph, t);
                }
                if (f_sum > 1.5) {
                  LinearRange lazy_cut(0., new_constraint, 1.);
                  ++master._i_num_cutoff_antiarbo;
                  p_context->addLazyConstraint(lazy_cut);
                  return;
                }
                //--
              }
            }
          }

          // Add conflict inequalities
          if (p_context->event() == mp::MPCallbackEvent::kMipSolution
              && (m.master._i_inequalities_flag & BleyBranchAndCut::USPF)) {
            ZoneScoped;
            m.master._isp.removeAllForwardingArcs();
            m.forwarding_vars.removeAll();

            // Build the forwarding set.
            for (ArcIt arc(graph); arc != nt::INVALID; ++arc) {
              for (NodeIt t(graph); t != nt::INVALID; ++t) {
                //--
                DemandInArcIt demand_arc(demand_graph, t);
                if (demand_arc == nt::INVALID) continue;

                do {
                  const MPVariable* X = m.master.X(demand_arc, arc);
                  if (p_context->variableValue(X) > .5) {
                    m.master._isp.addForwardingArc(arc, t);
                    m.forwarding_vars.add(X);
                    break;
                  };
                } while (++demand_arc != nt::INVALID);
              }
            }

            m.master._isp.build(graph);

            mp::MPSolverParameters solver_parameters;

            // Use these solver parameters to get access to the farkas dual values.
            solver_parameters.setIntegerParam(mp::MPSolverParameters::LP_ALGORITHM, mp::MPSolverParameters::DUAL);
            solver_parameters.setIntegerParam(mp::MPSolverParameters::PRESOLVE, mp::MPSolverParameters::PRESOLVE_OFF);

            // If the problem is infeasable then search for the minimal conflicts set
            // and add the coresponding constraint to the master problem.
            if (m.master._isp.solve(solver_parameters) == mp::ResultStatus::INFEASIBLE) {
              // 1 - Remove the forward arcs that do not contribute to the
              // infeasability of the problem according to farkas dual values.
              for (int i_forward_arc = m.master._isp.forwardingArcs().size() - 1; i_forward_arc >= 0; --i_forward_arc) {
                const auto constraints = m.master._isp.forwardingArcContraints(i_forward_arc);
                int        k;

                for (k = 0; k < constraints.size(); ++k)
                  if (std::abs(constraints[k]->dual_farkas_value()) > 0) break;

                if (k == constraints.size()) {
                  m.master._isp.deactivateForwardingArcContraints(i_forward_arc);
                  m.forwarding_vars.remove(i_forward_arc);
                }
              }

              // 2 - Greedily search for the minimal conflict set
              for (int i_forward_arc = m.master._isp.forwardingArcs().size() - 1; i_forward_arc >= 0; --i_forward_arc) {
                // Build the model by exluding i_forward_arc
                m.master._isp.deactivateForwardingArcContraints(i_forward_arc);
                if (m.master._isp.solve(solver_parameters) == mp::ResultStatus::INFEASIBLE)
                  m.forwarding_vars.remove(i_forward_arc);
                else
                  m.master._isp.activateForwardingArcContraints(i_forward_arc);
              }

              // 3 - Add the constraint
              LinearExpr conflict_constraint;

              for (int i_var = 0; i_var < m.forwarding_vars.size(); ++i_var) {
                conflict_constraint += m.forwarding_vars[i_var];
              }
              assert(m.forwarding_vars.size());
              assert(conflict_constraint.terms().size() == m.forwarding_vars.size());

              LinearRange lazy_cut(0., conflict_constraint, m.forwarding_vars.size() - 1);
              ++master._i_num_cutoff_isp;
              p_context->addLazyConstraint(lazy_cut);
              return;
            }
          }
        }
      };

      Digraph const*        _p_graph;
      DemandGraph const*    _p_demand_graph;
      CapacityMap const*    _p_capacity_map;
      DemandValueMap const* _p_demand_map;

      MIPModel&                                       _mips;
      graphs::InverseShortestPath< Digraph, LPModel > _isp;
      LazyCutCallback                                 _lazy_cut_callback;
      int                                             _i_inequalities_flag;

      // Stats
      int _i_num_cutoff_subpath;
      int _i_num_cutoff_antiarbo;
      int _i_num_cutoff_isp;

      BleyBranchAndCut(MIPModel& mips, LPModel& lps) :
          _mips(mips), _isp(lps), _lazy_cut_callback(*this), _i_num_cutoff_subpath(0), _i_inequalities_flag(0),
          _i_num_cutoff_antiarbo(0), _i_num_cutoff_isp(0){};

      /**
       * @brief Create the model.
       *
       *  @param i_inequalities_flag Flags that indicate which inequalities to
       * incorporate in the model.
       *
       * @return true if the model was successfully created, false otherwise.
       */
      bool build(const Digraph&        graph,
                 const DemandGraph&    demand_graph,
                 CapacityMap const*    p_cm = nullptr,
                 DemandValueMap const* p_dvm = nullptr,
                 int                   i_inequalities_flag = 0) {
        ZoneScoped;
        BleyBranchAndCut& m = *this;

        m._p_graph = &graph;
        m._p_demand_graph = &demand_graph;
        m._p_capacity_map = p_cm;
        m._p_demand_map = p_dvm;
        m._i_inequalities_flag = i_inequalities_flag;

        if (!_mips.supportsCallbacks()) return false;
        if (!graph.arcNum()) return false;
        if (!demand_graph.arcNum()) return false;

        _mips.clear();

        // Create X variables
        for (ArcIt arc(graph); arc != nt::INVALID; ++arc)
          for (DemandArcIt demand_arc(demand_graph); demand_arc != nt::INVALID; ++demand_arc)
            _mips.makeBoolVar(X_NAME(demand_arc, arc));

        // Create L variable
        _mips.makeNumVar(0., mp::infinity(), L_NAME());

        // Add objective
        MPObjective* const p_objective = _mips.mutableObjective();
        p_objective->setCoefficient(m.L(), 1);
        p_objective->setMinimization();

        // Add flow conservation constraints
        for (NodeIt v(graph); v != nt::INVALID; ++v) {
          for (DemandArcIt demand_arc(demand_graph); demand_arc != nt::INVALID; ++demand_arc) {
            double f_diff_value = 0.;
            if (demand_graph.source(demand_arc) == v)
              f_diff_value = -1.;
            else if (demand_graph.target(demand_arc) == v)
              f_diff_value = 1.;

            MPConstraint* const p_constraint =
               _mips.makeRowConstraint(f_diff_value, f_diff_value, BLEY_CT_FLOW_NAME(demand_arc, v));

            for (OutArcIt out_arc(graph, v); out_arc != nt::INVALID; ++out_arc)
              p_constraint->setCoefficient(m.X(demand_arc, out_arc), -1.0);

            for (InArcIt in_arc(graph, v); in_arc != nt::INVALID; ++in_arc)
              p_constraint->setCoefficient(m.X(demand_arc, in_arc), 1.0);
          }
        }

        // Add load constraints
        double f_max_capacity = _p_capacity_map ? 0 : 1.;
        for (ArcIt arc(graph); arc != nt::INVALID; ++arc) {
          if (_p_capacity_map)
            f_max_capacity = std::max(f_max_capacity, static_cast< double >((*_p_capacity_map)[arc]));

          MPConstraint* const p_constraint = _mips.makeRowConstraint(-mp::infinity(), 0., BLEY_CT_LOAD_NAME(arc));

          for (DemandArcIt demand_arc(demand_graph); demand_arc != nt::INVALID; ++demand_arc) {
            MPVariable* p_var = m.X(demand_arc, arc);
            assert(p_var);
            if (p_dvm)
              p_constraint->setCoefficient(p_var, (*p_dvm)[demand_arc]);
            else
              p_constraint->setCoefficient(p_var, 1.);
          }

          if (p_cm)
            p_constraint->setCoefficient(m.L(), -(*p_cm)[arc]);
          else
            p_constraint->setCoefficient(m.L(), -1.);
        }

        // DE constraints
        if (m._i_inequalities_flag & BleyBranchAndCut::DE) {
          for (DemandArcIt demand_arc(demand_graph); demand_arc != nt::INVALID; ++demand_arc) {
            const Node s = demand_graph.source(demand_arc);
            const Node t = demand_graph.target(demand_arc);
            for (ArcIt arc(graph); arc != nt::INVALID; ++arc) {
              const Node u = graph.source(arc);
              const Node v = graph.target(arc);
              if (s == u && t == v && (!_p_capacity_map || (*_p_capacity_map)[arc] >= f_max_capacity)) {
                MPConstraint* const p_constraint = _mips.makeRowConstraint(1.0, 1.0, "DE");
                MPVariable*         p_var = m.X(demand_arc, arc);
                assert(p_var);
                p_constraint->setCoefficient(p_var, 1.);
              }
            }
          }
        }

        _mips.setCallback(&m._lazy_cut_callback);

        return true;
      }

      /**
       * @brief Get the optimal weights. The method solve() must be called before.
       *
       * @param optimal_weights A "writable" arc map to store the optimal weights.
       */
      template < class WEIGHT_MAP >
      void getOptimalWeights(WEIGHT_MAP& optimal_weights) {
        ZoneScoped;
        for (ArcIt arc(*_p_graph); arc != nt::INVALID; ++arc)
          optimal_weights.set(arc, _isp.W(arc)->solution_value());
      }

      /**
       * @brief
       *
       */
      double getLoad() const { return _mips.objective().value(); }

      /**
       * @brief
       *
       * @return true
       * @return false
       */
      bool checkSolution() {
        ZoneScoped;
        _isp.removeAllForwardingArcs();

        for (NodeIt v(*_p_graph); v != nt::INVALID; ++v) {
          for (NodeIt t(*_p_graph); t != nt::INVALID; ++t) {
            //--
            DemandInArcIt demand_arc(*_p_demand_graph, t);
            if (demand_arc == nt::INVALID) continue;

            for (OutArcIt out_arc(*_p_graph, v); out_arc != nt::INVALID; ++out_arc) {
              do {
                if (X(demand_arc, out_arc)->solution_value() > .5) {
                  _isp.addForwardingArc(out_arc, t);
                  break;
                };
              } while (++demand_arc != nt::INVALID);
              demand_arc = DemandInArcIt(*_p_demand_graph, t);
            }
          }
        }

        _isp.build(*_p_graph);
        const mp::ResultStatus status = _isp.solve();
        return status == mp::ResultStatus::FEASIBLE || status == mp::ResultStatus::OPTIMAL;
      }

      /**
       * @brief
       *
       * @return mp::ResultStatus
       */
      mp::ResultStatus solve() { return _mips.solve(); }

      /**
       * @brief Retrieve the routing path associated to the demand i_demand.
       *
       * @param i_demand Index of a demand.
       *
       * @return Array of links used to route the demand i_demand.
       */
      nt::IntDynamicArray getArcPath(int i_demand) {
        ZoneScoped;
        BleyBranchAndCut& m = *this;

        nt::IntDynamicArray arc_path;
        for (ArcIt arc(*_p_graph); arc != nt::INVALID; ++arc) {
          const double f_X_val = m.X(DemandGraph::nodeFromId(i_demand), arc)->solution_value();
          if (f_X_val > 0.5) arc_path.add(Digraph::id(arc));
        }

        return arc_path;
      }


      // Variables accessor

      constexpr MPVariable*& X(DemandArc demand_arc, Arc arc) noexcept {
        BleyBranchAndCut& m = *this;
        //--
        assert(m._p_demand_graph->id(demand_arc) >= 0 && m._p_graph->id(arc) >= 0);
        assert(m._p_demand_graph->id(demand_arc) < m._p_demand_graph->arcNum()
               && m._p_graph->id(arc) < m._p_graph->arcNum());
        assert(_mips.numVariables() >= 2);
        return _mips._variables[_mips.numVariables() - 2
                                - (m._p_demand_graph->id(demand_arc) + m._p_demand_graph->arcNum() * Digraph::id(arc))];
      }

      constexpr MPVariable*& L() noexcept {
        BleyBranchAndCut& m = *this;
        //--
        assert(_mips.numVariables() > 0);
        return _mips._variables[_mips.numVariables() - 1];
      }
    };
  }   // namespace te
}   // namespace nt

#endif   // _BLEY_BRANCH_AND_CUT_H_