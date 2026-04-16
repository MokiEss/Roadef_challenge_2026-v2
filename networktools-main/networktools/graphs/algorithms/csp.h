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
 *
 * ============================================================================
 *
 * This file incorporates work from the LEMON graph library (csp.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2009 Egervary Jeno Kombinatorikus Optimalizalasi
 * Kutatocsoport (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted provided
 * that this copyright notice appears in all copies. For precise terms see the
 * accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind, express or
 * implied, and with no claim as to its suitability for any purpose.
 *
 * ============================================================================
 *
 * List of modifications:
 *   -
 */

/**
 * @file
 * @brief Resource Constrained Shortest Path algorithms
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CSP_H_
#define _NT_CSP_H_

#include "dijkstra.h"
#include "../path.h"
#include "../../core/tolerance.h"

namespace nt {
  namespace graphs {

    /**
     * @class ConstrainedShortestPath
     * @headerfile csp.h
     * @brief ConstrainedShortestPath algorithm for solving the Resource Constrained Shortest Path Problem based on [1]
     *
     * The Resource Constrained Shortest (Least Cost) Path problem is the
     * following. We are given a directed graph with two additive weightings
     * on the arcs, referred as \e cost and \e delay. In addition,
     * a source and a destination node \e s and \e t and a delay
     * constraint \e D is given. A path \e p is called \e feasible
     * if <em>delay(p)\<=D</em>. Then, the task is to find the least cost
     * feasible path.
     *
     * [1] A. Jüttner, B. Szviatovszki, I. Mécs, Z. Rajkó:
     *     "Lagrange Relaxation Based Method for the QoS Routing Problem",
     *     INFOCOM 2001: 859-868
     *
     */
    template < typename GR, typename CM = typename GR::template ArcMap< double >, typename DM = CM >
    struct ConstrainedShortestPath {
      TEMPLATE_DIGRAPH_TYPEDEFS(GR);

      using Path = SimplePath< GR >;

      struct CoMap {
        const CM& _cost;
        const DM& _delay;
        double    _lambda;

        using Key = typename CM::Key;
        using Value = double;
        CoMap(const CM& c, const DM& d) : _cost(c), _delay(d), _lambda(0) {}
        double lambda() const { return _lambda; }
        void   lambda(double l) { _lambda = l; }
        Value  operator[](Key& e) const { return _cost[e] + _lambda * _delay[e]; }
      };

      const GR&           _g;
      Tolerance< double > _tol;
      double              _f_lower_bound;   //< a lower bound on the optimal solution

      const CM& _cost;
      const DM& _delay;

      CoMap _co_map;

      DijkstraLegacy< GR, CoMap > _dij;

      ConstrainedShortestPath(const GR& g, const CM& ct, const DM& dl) :
          _g(g), _cost(ct), _delay(dl), _co_map(ct, dl), _f_lower_bound(0.), _dij(_g, _co_map) {}

      /**
       * @brief Compute the cost of a path
       */
      double cost(const Path& p) const {
        double s = 0.;
        for (typename Path::ArcIt e(p); e != INVALID; ++e)
          s += _cost[e];
        return s;
      }

      /**
       * @brief Compute the delay of a path
       */
      double delay(const Path& p) const {
        double s = 0.;
        for (typename Path::ArcIt e(p); e != INVALID; ++e)
          s += _delay[e];
        return s;
      }

      /**
       * @brief This function runs a Lagrange relaxation based heuristic to find a delay constrained least cost path.
       *
       * @param s source node
       * @param t target node
       * @param delta upper bound on the delta
       *
       * \return the found path or an empty
       */
      Path larac(Node s, Node t, double delta, double& lo_bo) {
        double lambda = 0;
        double cp, cq, dp, dq, cr, dr;
        Path   p;
        Path   q;
        Path   r;
        {
          DijkstraLegacy< GR, CM > dij(_g, _cost);
          dij.run(s, t);
          if (!dij.reached(t)) return Path();
          p = dij.path(t);
          cp = cost(p);
          dp = delay(p);
        }
        if (delay(p) <= delta) return p;
        {
          DijkstraLegacy< GR, DM > dij(_g, _delay);
          dij.run(s, t);
          q = dij.path(t);
          cq = cost(q);
          dq = delay(q);
        }
        if (delay(q) > delta) return Path();
        while (true) {
          lambda = (cp - cq) / (dq - dp);
          _co_map.lambda(lambda);
          _dij.run(s, t);
          r = _dij.path(t);
          cr = cost(r);
          dr = delay(r);
          if (!_tol.less(cr + lambda * dr, cp + lambda * dp)) {
            lo_bo = cq + lambda * (dq - delta);
            return q;
          } else if (_tol.less(dr, delta)) {
            q = r;
            cq = cr;
            dq = dr;
          } else if (_tol.less(delta, dr)) {
            p = r;
            cp = cr;
            dp = dr;
          } else
            return r;
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
