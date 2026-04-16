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
 * This file incorporates work from the LEMON graph library (nearest_neighbor_tsp.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2013 Egervary Jeno Kombinatorikus Optimalizalasi
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
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Replaced std::vector with nt::TrivialDynamicArray/nt::DynamicArray
 *   - Updated include paths to networktools structure
 *   - Adapted LEMON concept checking to networktools
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Updated header guard macros
 */

/**
 * @ingroup tsp
 * @file
 * @brief Nearest neighbor algorithm for symmetric TSP
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_NEAREST_NEIGHBOUR_TSP_H_
#define _NT_NEAREST_NEIGHBOUR_TSP_H_

#include "../full_graph.h"
#include "../../core/maps.h"
#include "../graph_maps.h"
#include <deque>
#include <limits>

namespace nt {
  namespace graphs {

    /**
     * @ingroup tsp
     *
     * @brief Nearest neighbor algorithm for symmetric TSP.
     *
     * NearestNeighborTsp implements the nearest neighbor heuristic for solving
     * symmetric @ref tsp "TSP".
     *
     * This is probably the simplest TSP heuristic.
     * It starts with a minimum cost edge and at each step, it connects the
     * nearest unvisited node to the current path.
     * Finally, it connects the two end points of the path to form a tour.
     *
     * This method runs in O(n<sup>2</sup>) time.
     * It quickly finds a relatively short tour for most TSP instances,
     * but it could also yield a really bad (or even the worst) solution
     * in special cases.
     *
     * @tparam CM Type of the cost map.
     */
    template < typename CM >
    class NearestNeighborTsp {
      public:
      /** Type of the cost map */
      using CostMap = CM;
      /** Type of the edge costs */
      using Cost = typename CM::Value;

      private:
      GRAPH_TYPEDEFS(FullGraph);

      const FullGraph&                _gr;
      const CostMap&                  _cost;
      Cost                            _sum;
      nt::TrivialDynamicArray< Node > _path;

      public:
      /**
       * @brief Constructor
       *
       * Constructor.
       * @param gr The @ref FullGraph "full graph" the algorithm runs on.
       * @param cost The cost map.
       */
      NearestNeighborTsp(const FullGraph& gr, const CostMap& cost) : _gr(gr), _cost(cost) {}

      /**
       * \name Execution Control
       * @{
       */

      /**
       * @brief Runs the algorithm.
       *
       * This function runs the algorithm.
       *
       * @return The total cost of the found tour.
       */
      Cost run() {
        _path.clear();
        if (_gr.nodeNum() == 0) {
          return _sum = 0;
        } else if (_gr.nodeNum() == 1) {
          _path.push_back(_gr(0));
          return _sum = 0;
        }

        std::deque< Node > path_dq;
        Edge               min_edge1 = INVALID, min_edge2 = INVALID;

        min_edge1 = mapMin(_gr, _cost);
        Node n1 = _gr.u(min_edge1), n2 = _gr.v(min_edge1);
        path_dq.push_back(n1);
        path_dq.push_back(n2);

        FullGraph::NodeMap< bool > used(_gr, false);
        used[n1] = true;
        used[n2] = true;

        min_edge1 = INVALID;
        while (int(path_dq.size()) != _gr.nodeNum()) {
          if (min_edge1 == INVALID) {
            for (IncEdgeIt e(_gr, n1); e != INVALID; ++e) {
              if (!used[_gr.runningNode(e)] && (min_edge1 == INVALID || _cost[e] < _cost[min_edge1])) { min_edge1 = e; }
            }
          }

          if (min_edge2 == INVALID) {
            for (IncEdgeIt e(_gr, n2); e != INVALID; ++e) {
              if (!used[_gr.runningNode(e)] && (min_edge2 == INVALID || _cost[e] < _cost[min_edge2])) { min_edge2 = e; }
            }
          }

          if (_cost[min_edge1] < _cost[min_edge2]) {
            n1 = _gr.oppositeNode(n1, min_edge1);
            path_dq.push_front(n1);

            used[n1] = true;
            min_edge1 = INVALID;

            if (_gr.u(min_edge2) == n1 || _gr.v(min_edge2) == n1) min_edge2 = INVALID;
          } else {
            n2 = _gr.oppositeNode(n2, min_edge2);
            path_dq.push_back(n2);

            used[n2] = true;
            min_edge2 = INVALID;

            if (_gr.u(min_edge1) == n2 || _gr.v(min_edge1) == n2) min_edge1 = INVALID;
          }
        }

        n1 = path_dq.back();
        n2 = path_dq.front();
        _path.push_back(n2);
        _sum = _cost[_gr.edge(n1, n2)];
        for (int i = 1; i < int(path_dq.size()); ++i) {
          n1 = n2;
          n2 = path_dq[i];
          _path.push_back(n2);
          _sum += _cost[_gr.edge(n1, n2)];
        }

        return _sum;
      }

      /** @} */

      /**
       * \name Query Functions
       * @{
       */

      /**
       * @brief The total cost of the found tour.
       *
       * This function returns the total cost of the found tour.
       *
       * @pre run() must be called before using this function.
       */
      Cost tourCost() const { return _sum; }

      /**
       * @brief Returns a const reference to the node sequence of the
       * found tour.
       *
       * This function returns a const reference to a vector
       * that stores the node sequence of the found tour.
       *
       * @pre run() must be called before using this function.
       */
      const nt::TrivialDynamicArray< Node >& tourNodes() const { return _path; }

      /**
       * @brief Gives back the node sequence of the found tour.
       *
       * This function copies the node sequence of the found tour into
       * an STL container through the given output iterator. The
       * <tt>value_type</tt> of the container must be <tt>FullGraph::Node</tt>.
       * For example,
       * @code
       * nt::TrivialDynamicArray<FullGraph::Node> nodes(nt::graphs::countNodes(graph));
       * tsp.tourNodes(nodes.begin());
       * @endcode
       * or
       * @code
       * std::list<FullGraph::Node> nodes;
       * tsp.tourNodes(std::back_inserter(nodes));
       * @endcode
       *
       * @pre run() must be called before using this function.
       */
      template < typename Iterator >
      void tourNodes(Iterator out) const {
        std::copy(_path.begin(), _path.end(), out);
      }

      /**
       * @brief Gives back the found tour as a path.
       *
       * This function copies the found tour as a list of arcs/edges into
       * the given @ref nt::concepts::Path "path structure".
       *
       * @pre run() must be called before using this function.
       */
      template < typename Path >
      void tour(Path& path) const {
        path.clear();
        for (int i = 0; i < int(_path.size()) - 1; ++i) {
          path.addBack(_gr.arc(_path[i], _path[i + 1]));
        }
        if (int(_path.size()) >= 2) { path.addBack(_gr.arc(_path.back(), _path.front())); }
      }

      /** @} */
    };

  }   // namespace graphs
}   // namespace nt

#endif
