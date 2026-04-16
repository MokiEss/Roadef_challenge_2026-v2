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
 * @brief Linear program for computing the minimum vertex cover of a graph.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_VCLP_H_
#define _NT_VCLP_H_

#include "../tools.h"
#include "../../mp/mp.h"

#define VCLP_X_NAME() "X_"
#define VCLP_CT_NAME(IARC) "ct_" + std::to_string(Digraph::id(IARC))

namespace nt {
  namespace graphs {

    /**
     * @class Vclp
     * @headerfile vclp.h
     * @brief Linear program for computing the minimum vertex cover of a graph.
     *
     * The Minimum Vertex Cover problem seeks to find the smallest set of vertices such that
     * every edge in the graph is incident to at least one vertex in the set. This solver
     * formulates the problem as a Linear Program (LP) or Mixed Integer Program (MIP).
     *
     * For the LP relaxation, the solution may have fractional values. For an exact integer
     * solution, use a MIP solver instead of an LP solver.
     *
     * Example usage:
     * @code
     * // Create a bidirected graph
     * using Digraph = nt::graphs::SmartDigraph;
     * using MPEnv = nt::mp::glpk::MPEnv;
     * using LPModel = nt::mp::glpk::LPModel;
     * 
     * Digraph digraph;
     * auto n1 = digraph.addNode();
     * auto n2 = digraph.addNode();
     * auto n3 = digraph.addNode();
     * auto n4 = digraph.addNode();
     *
     * // Add bidirectional edges
     * digraph.addArc(n1, n2);
     * digraph.addArc(n2, n1);
     * digraph.addArc(n2, n3);
     * digraph.addArc(n3, n2);
     * digraph.addArc(n3, n4);
     * digraph.addArc(n4, n3);
     *
     * // Create MIP solver and Vclp instance for exact integer solution
     * MPEnv                                               env;
     * LPModel                                             mip_solver("lp_solver", env);
     * nt::graphs::Vclp<Digraph, LPModel> vclp(mip_solver);
     *
     * // Build and solve the model
     * vclp.build(digraph);
     * auto status = vclp.solve();
     *
     * if (status == nt::mp::ResultStatus::OPTIMAL) {
     *   // Get the minimum vertex cover size
     *   double cover_size = vclp.getVertexCoverNumber();
     *   std::cout << "Minimum vertex cover size: " << cover_size << std::endl;
     *
     *   // Check which vertices are in the cover
     *   for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
     *     double val = vclp.X(node)->solution_value();
     *     if (val > 0.5) {  // For MIP, will be 0 or 1
     *       std::cout << "Node " << digraph.id(node) << " is in the cover" << std::endl;
     *     }
     *   }
     * }
     * @endcode
     *
     * @tparam GR The type of the digraph the algorithm runs on. The digraph is assumed to be bidirected.
     * @tparam MPS The type of the LP/MIP solver used to solve the model.
     *
     */
    template < typename GR, typename MPS >
    // requires nt::concepts::Digraph< GR >
    struct Vclp {
      using Digraph = GR;
      using MPSolver = MPS;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      TEMPLATE_MODEL_TYPEDEFS(MPSolver);

      MPSolver&                        _mps;
      Digraph const*                   _p_digraph;
      MPGraphVariable< Digraph, Node > _X;

      Vclp(MPSolver& mps) : _mps(mps), _p_digraph(nullptr){};

      /**
       * @brief Builds the LP/MIP model for the minimum vertex cover problem.
       *
       * The digraph is assumed to be bidirected (each edge appears as two arcs in both
       * directions). Only one constraint per edge is created by comparing node IDs.
       *
       * @param digraph The bidirected graph for which to compute the minimum vertex cover.
       */
      void build(const Digraph& digraph) noexcept {
        _p_digraph = &digraph;

        _mps.clear();
        _mps.reserve(digraph.nodeNum(), digraph.arcNum());

        // Create X variables
        if (_mps.isLP())
          _X = _mps.template makeGraphNumVar< Digraph, Node >(0., mp::infinity(), VCLP_X_NAME(), digraph);
        else
          _X = _mps.template makeGraphIntVar< Digraph, Node >(0., mp::infinity(), VCLP_X_NAME(), digraph);

        // Create the objective function
        MPObjective* const p_objective = _mps.mutableObjective();
        p_objective->setMinimization();

        for (NodeIt node(digraph); node != INVALID; ++node)
          p_objective->setCoefficient(X(node), 1);

        // Add arc constraints
        for (ArcIt arc(digraph); arc != INVALID; ++arc) {
          const Node u = digraph.source(arc);
          const Node v = digraph.target(arc);
          if (Digraph::id(u) < Digraph::id(v)) {   // Take into account only one direction
            MPConstraint* const p_constraint = _mps.makeRowConstraint(1, mp::infinity(), VCLP_CT_NAME(arc));
            p_constraint->setCoefficient(X(u), 1.);
            p_constraint->setCoefficient(X(v), 1.);
          }
        }
      }

      /**
       * @brief Solves the LP/MIP model.
       *
       * Invokes the solver to find the minimum vertex cover. For LP solvers, the solution
       * may be fractional (providing a lower bound). For MIP solvers, the solution will be
       * integer-valued (exact vertex cover).
       *
       * @return The status of the solver (OPTIMAL, INFEASIBLE, UNBOUNDED, etc.)
       */
      mp::ResultStatus solve() noexcept { return _mps.solve(); }

      /**
       * @brief Returns the minimum vertex cover number.
       *
       * This is the sum of all X[v] variables in the optimal solution. For MIP solvers,
       * this represents the exact size of the minimum vertex cover. For LP solvers,
       * this is the LP relaxation lower bound (may be fractional).
       *
       * @return The minimum vertex cover number (objective value).
       * @pre solve() must have been called and returned an optimal solution.
       */
      double getVertexCoverNumber() const noexcept { return _mps.objective().value(); }

      /**
       * @brief Returns the decision variable associated with node v.
       *
       * Access the LP/MIP variable X[v] which indicates whether vertex v is in the cover.
       * After solving, use X(v)->solution_value() to get the optimal value.
       *
       * @param v The node whose variable to retrieve.
       * @return Pointer to the MPVariable representing X[v].
       */
      MPVariable* X(Node v) noexcept {
        assert(_p_digraph);
        return _X(v);
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
