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
 * This file incorporates work from the LEMON graph library (greedy_tsp.h).
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
 * @brief Greedy algorithm for symmetric TSP
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_GREEDY_TSP_H_
#define _NT_GREEDY_TSP_H_

#include "../full_graph.h"
#include "../../core/unionfind.h"
#include <algorithm>

namespace nt {
  namespace graphs {

    /**
     * @ingroup tsp
     *
     * @brief Greedy algorithm for symmetric TSP.
     *
     * GreedyTsp implements the greedy heuristic for solving
     * symmetric @ref tsp "TSP".
     *
     * This algorithm is quite similar to the @ref NearestNeighborTsp
     * "nearest neighbor" heuristic, but it maintains a set of disjoint paths.
     * At each step, the shortest possible edge is added to these paths
     * as long as it does not create a cycle of less than n edges and it does
     * not increase the degree of any node above two.
     *
     * This method runs in O(n<sup>2</sup>) time.
     * It quickly finds a relatively short tour for most TSP instances,
     * but it could also yield a really bad (or even the worst) solution
     * in special cases.
     *
     * @tparam CM Type of the cost map.
     */
    template < typename CM >
    class GreedyTsp {
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

      private:
      // Functor class to compare edges by their costs
      class EdgeComp {
        private:
        const CostMap& _cost;

        public:
        EdgeComp(const CostMap& cost) : _cost(cost) {}

        bool operator()(const Edge& a, const Edge& b) const { return _cost[a] < _cost[b]; }
      };

      public:
      /**
       * @brief Constructor
       *
       * Constructor.
       * @param gr The @ref FullGraph "full graph" the algorithm runs on.
       * @param cost The cost map.
       */
      GreedyTsp(const FullGraph& gr, const CostMap& cost) : _gr(gr), _cost(cost) {}

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

        if (_gr.nodeNum() == 0)
          return _sum = 0;
        else if (_gr.nodeNum() == 1) {
          _path.push_back(_gr(0));
          return _sum = 0;
        }

        nt::TrivialDynamicArray< int > plist;
        plist.resize(_gr.nodeNum() * 2, -1);

        nt::TrivialDynamicArray< Edge > sorted_edges;
        sorted_edges.reserve(_gr.edgeNum());
        for (EdgeIt e(_gr); e != INVALID; ++e)
          sorted_edges.push_back(e);
        std::sort(sorted_edges.begin(), sorted_edges.end(), EdgeComp(_cost));

        FullGraph::NodeMap< int >              item_int_map(_gr);
        UnionFind< FullGraph::NodeMap< int > > union_find(item_int_map);
        for (NodeIt n(_gr); n != INVALID; ++n)
          union_find.insert(n);

        FullGraph::NodeMap< int > degree(_gr, 0);

        int nodesNum = 0, i = 0;
        while (nodesNum != _gr.nodeNum() - 1) {
          Edge e = sorted_edges[i++];
          Node u = _gr.u(e), v = _gr.v(e);

          if (degree[u] <= 1 && degree[v] <= 1) {
            if (union_find.join(u, v)) {
              const int uid = _gr.id(u), vid = _gr.id(v);

              plist[uid * 2 + degree[u]] = vid;
              plist[vid * 2 + degree[v]] = uid;

              ++degree[u];
              ++degree[v];
              ++nodesNum;
            }
          }
        }

        for (int i = 0, n = -1; i < _gr.nodeNum() * 2; ++i) {
          if (plist[i] == -1) {
            if (n == -1) {
              n = i;
            } else {
              plist[n] = i / 2;
              plist[i] = n / 2;
              break;
            }
          }
        }

        for (int i = 0, next = 0, last = -1; i != _gr.nodeNum(); ++i) {
          _path.push_back(_gr.nodeFromId(next));
          if (plist[2 * next] != last) {
            last = next;
            next = plist[2 * next];
          } else {
            last = next;
            next = plist[2 * next + 1];
          }
        }

        _sum = _cost[_gr.edge(_path.back(), _path.front())];
        for (int i = 0; i < int(_path.size()) - 1; ++i) {
          _sum += _cost[_gr.edge(_path[i], _path[i + 1])];
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
       * nt::TrivialDynamicArray<FullGraph::Node> nodes(lemon::countNodes(graph));
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
       * the given @ref lemon::concepts::Path "path structure".
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
