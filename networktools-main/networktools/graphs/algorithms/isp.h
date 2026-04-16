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
 * @brief Linear program for solving the Inverse Shortest Path (InverseShortestPath) problem.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ISP_H_
#define _NT_ISP_H_
//---------------------------------------------------------------------------------------
#include "../tools.h"
#include "../../mp/mp.h"
//---------------------------------------------------------------------------------------
#define W_NAME() "W_"
#define R_NAME() "R_"
#define W_MAX_NAME() "Wmax"
#define CT_BOUND_NAME(IARC) "ct_bound_" + std::to_string(Digraph::id(IARC))
#define CT_IN_NAME(ITERM, IARC) "ct_in_" + std::to_string(Digraph::id(ITERM)) + "_a" + std::to_string(Digraph::id(IARC))
#define CT_NOTIN_NAME(ITERM, IARC) \
  "ct_notin_" + std::to_string(Digraph::id(ITERM)) + "_a" + std::to_string(Digraph::id(IARC))
#define CT_DONTCARE_NAME(ITERM, IARC) \
  "ct_dontcare_" + std::to_string(Digraph::id(ITERM)) + "_a" + std::to_string(Digraph::id(IARC))
#define CT_SYMMETRIC(IARC, IARCR) \
  "ct_symmetric_a" + std::to_string(Digraph::id(IARC)) + "_a" + std::to_string(Digraph::id(IARCR))
//---------------------------------------------------------------------------------------
namespace nt {
  namespace graphs {

    /**
     * @class InverseShortestPath
     * @headerfile isp.h
     * @brief Linear program for solving the Inverse Shortest Path problem.
     *
     * The Inverse Shortest Path (ISP) problem aims to find arc weights such that specified
     * paths are shortest paths in the resulting network. Given a directed graph and a set of
     * forwarding arcs (representing desired routing paths), this solver computes minimal arc
     * weights that guarantee these paths are optimal shortest paths.
     *
     *
     * Example usage:
     * @code
     * using Digraph = nt::graphs::SmartDigraph;
     * using MPEnv = nt::mp::glpk::MPEnv;
     * using LPModel = nt::mp::glpk::LPModel;
     * 
     * // Create a digraph
     * Digraph digraph;
     * auto    n1 = digraph.addNode();
     * auto    n2 = digraph.addNode();
     * auto    n3 = digraph.addNode();
     * auto    a12 = digraph.addArc(n1, n2);
     * auto    a23 = digraph.addArc(n2, n3);
     * auto    a13 = digraph.addArc(n1, n3);
     *
     * // Create LP solver and ISP instance
     * MPEnv                                               env;
     * LPModel                                             lp_solver("lp_solver", env);
     * nt::graphs::InverseShortestPath< Digraph, LPModel > isp(lp_solver);
     *
     * // Specify that path n1->n2->n3 should be shortest for terminal n3
     * isp.addForwardingArc(a12, n3);
     * isp.addForwardingArc(a23, n3);
     *
     * // Build and solve the model
     * isp.build(digraph);
     * auto status = isp.solve();
     *
     * if (status == nt::mp::ResultStatus::OPTIMAL) {
     *   // Retrieve computed weights
     *   for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
     *     double weight = isp.W(arc)->solution_value();
     *     std::cout << "Arc weight: " << weight << std::endl;
     *   }
     * }
     * @endcode
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam LPS The type of the LP solver used to solve the model.
     *
     */
    template < typename GR, typename LPS >
    // requires nt::concepts::Digraph< GR >
    struct InverseShortestPath {
      using Digraph = GR;
      using LPModel = LPS;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(LPModel);

      struct ForwardArc {
        Arc  arc;
        Node term;
        constexpr ForwardArc() noexcept : arc(0), term(0) {}
        explicit ForwardArc(Arc arc, Node term) noexcept : arc(arc), term(term) {}
      };

      Digraph const* _p_digraph;
      LPModel&       _lps;
      bool           _b_symmetric_weights;   // If true, adds constraints to enforce symmetric weights (w(u,v) = w(v,u))

      nt::TrivialDynamicArray< ForwardArc >                        _forwarding_arcs;
      nt::DynamicArray< nt::TrivialDynamicArray< MPConstraint* > > _forwarding_arcs_cons;

      // Model variables
      MPGraphVariable< Digraph, Arc >        _W;
      MPGraphVariable< Digraph, Node, Node > _R;
      MPVariable*                            _p_wmax;

      explicit InverseShortestPath(LPModel& lps, bool symmetric_weights = true) noexcept :
          _lps(lps), _b_symmetric_weights(symmetric_weights), _p_wmax(nullptr){};

      const nt::TrivialDynamicArray< ForwardArc >& forwardingArcs() const { return _forwarding_arcs; }

      int addForwardingArc(Arc arc, Node term) {
        _forwarding_arcs_cons.add();
        return _forwarding_arcs.add(ForwardArc(arc, term));
      }

      void removeAllForwardingArcs() {
        _forwarding_arcs_cons.removeAll();
        _forwarding_arcs.removeAll();
      }

      nt::ArrayView< MPConstraint* > forwardingArcContraints(int i) const { return _forwarding_arcs_cons[i]; }

      void deactivateForwardingArcContraints(int i) {
        for (MPConstraint* p_constraint: _forwarding_arcs_cons[i])
          p_constraint->deactivate();
      }

      void activateForwardingArcContraints(int i) {
        for (MPConstraint* p_constraint: _forwarding_arcs_cons[i])
          p_constraint->activate();
      }

      /**
       * @brief Builds the LP model for the Inverse Shortest Path problem.
       *
       * @param digraph The directed graph for which to solve the ISP problem.
       */
      void build(const Digraph& digraph) noexcept {
        _p_digraph = &digraph;
        _lps.clear();

        const int i_num_variables = digraph.arcNum() + digraph.nodeNum() * digraph.nodeNum() + 1;
        _lps.reserve(i_num_variables, digraph.arcNum());

        // Create W variables
        _W = _lps.template makeGraphNumVar< Digraph, Arc >(1., mp::infinity(), W_NAME(), digraph);

        // Create R variables
        _R = _lps.template makeGraphNumVar< Digraph, Node, Node >(-mp::infinity(), mp::infinity(), R_NAME(), digraph);

        // Create WMax variable
        _p_wmax = _lps.makeNumVar(1, mp::infinity(), W_MAX_NAME());

        // Add objective
        MPObjective* const p_objective = _lps.mutableObjective();
        p_objective->setCoefficient(WMax(), 1);
        p_objective->setMinimization();

        for (ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
          // Add bound constraints for W
          MPConstraint* const p_constraint = _lps.makeRowConstraint(0., mp::infinity(), CT_BOUND_NAME(arc));
          p_constraint->setCoefficient(W(arc), -1.0);
          p_constraint->setCoefficient(WMax(), 1.0);

          // Add non-negativity constraints : w(u,v) − rtu + rtv ≥ 0 (redundant with some "in"/"not
          // in" constraints)
          for (NodeIt t(digraph); t != nt::INVALID; ++t) {
            MPConstraint* const p_constraint = _lps.makeRowConstraint(0., mp::infinity(), CT_DONTCARE_NAME(t, arc));
            p_constraint->setCoefficient(W(arc), 1.0);
            p_constraint->setCoefficient(R(t, digraph.source(arc)), -1.0);
            p_constraint->setCoefficient(R(t, digraph.target(arc)), +1.0);
          }
        }

        //
        assert(_forwarding_arcs.size() == _forwarding_arcs_cons.size());
        for (int i = 0; i < _forwarding_arcs.size(); ++i) {
          const ForwardArc& forward_arc = _forwarding_arcs[i];
          const Node        arc_source = digraph.source(forward_arc.arc);
          const Node        arc_target = digraph.target(forward_arc.arc);

          _forwarding_arcs_cons[i].removeAll();
          // forward_arc.constraints.ensure(digraph.OutDegree(Arc.iSource));

          // "in" constraint : w(u,v) − rtu + rtv = 0
          MPConstraint* const p_constraint =
             _lps.makeRowConstraint(0., 0., CT_IN_NAME(forward_arc.term, forward_arc.arc));
          p_constraint->setCoefficient(W(forward_arc.arc), 1.0);
          p_constraint->setCoefficient(R(forward_arc.term, arc_source), -1.0);
          p_constraint->setCoefficient(R(forward_arc.term, arc_target), +1.0);
          _forwarding_arcs_cons[i].add(p_constraint);

          // "not in" constraint: w(u,v) − rtu + rtv ≥ 1
          for (OutArcIt out_arc(digraph, arc_source); out_arc != nt::INVALID; ++out_arc) {
            if (out_arc == forward_arc.arc) continue;

            MPConstraint* const p_constraint =
               _lps.makeRowConstraint(1., mp::infinity(), CT_NOTIN_NAME(forward_arc.term, out_arc));
            p_constraint->setCoefficient(W(out_arc), 1.0);
            p_constraint->setCoefficient(R(forward_arc.term, digraph.source(out_arc)), -1.0);
            p_constraint->setCoefficient(R(forward_arc.term, digraph.target(out_arc)), +1.0);
            _forwarding_arcs_cons[i].add(p_constraint);
          }
        }

        if (_b_symmetric_weights) {
          // w(u,v) = w(v,u) , (u, v), (v, u) ∈ A
          for (ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
            const Arc opposite_arc = graphs::findArc(digraph, digraph.target(arc), digraph.source(arc));
            if (opposite_arc == INVALID) continue;

            MPConstraint* const p_constraint = _lps.makeRowConstraint(0., 0., CT_SYMMETRIC(arc, opposite_arc));
            p_constraint->setCoefficient(W(arc), 1.0);
            p_constraint->setCoefficient(W(opposite_arc), -1.0);
          }
        }
      }

      /**
       * @brief Solves the LP model.
       *
       * Invokes the LP solver to find optimal arc weights that satisfy all forwarding
       * arc constraints while minimizing the maximum arc weight.
       *
       * @return The status of the solver (OPTIMAL, INFEASIBLE, UNBOUNDED, etc.)
       */
      mp::ResultStatus solve() noexcept { return _lps.solve(); }

      /**
       * @brief Solves the LP model with custom solver parameters.
       *
       * @param params Custom parameters for the LP solver.
       * @return The status of the solver (OPTIMAL, INFEASIBLE, UNBOUNDED, etc.)
       */
      mp::ResultStatus solve(const mp::MPSolverParameters& params) noexcept { return _lps.solve(params); }

      // Variables accessor
      constexpr MPVariable*& W(Arc arc) noexcept {
        assert(_p_digraph);
        assert(_p_digraph->id(arc) < _p_digraph->arcNum());
        return _W(arc);
      };

      constexpr MPVariable*& R(Node t, Node v) noexcept {
        assert(_p_digraph);
        assert(_p_digraph->id(t) < _p_digraph->nodeNum() && _p_digraph->id(v) < _p_digraph->nodeNum());
        return _R(t, v);
      };

      constexpr MPVariable*& WMax() noexcept { return _p_wmax; };
    };
    //---------------------------------------------------------------------------------------
  }   // namespace graphs
}   // namespace nt

#endif
