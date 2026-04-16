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
 * This file incorporates work from the LEMON graph library (christofides_tsp.h).
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
 * @brief Christofides algorithm for symmetric TSP
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CHRISTOFIDES_TSP_H_
#define _NT_CHRISTOFIDES_TSP_H_

#include "euler.h"
#include "../full_graph.h"
#include "kruskal.h"
#include "matching.h"
#include "../smart_graph.h"

namespace nt {
  namespace graphs {

    /**
     * @ingroup tsp
     *
     * @brief Christofides algorithm for symmetric TSP.
     *
     * ChristofidesTsp implements Christofides' heuristic for solving
     * symmetric @ref tsp "TSP".
     *
     * This a well-known approximation method for the TSP problem with
     * metric cost function.
     * It has a guaranteed approximation factor of 3/2 (i.e. it finds a tour
     * whose total cost is at most 3/2 of the optimum), but it usually
     * provides better solutions in practice.
     * This implementation runs in O(n<sup>3</sup>log(n)) time.
     *
     * The algorithm starts with a @ref spantree "minimum cost spanning tree" and
     * finds a @ref MaxWeightedPerfectMatching "minimum cost perfect matching"
     * in the subgraph induced by the nodes that have odd degree in the
     * spanning tree.
     * Finally, it constructs the tour from the @ref EulerIt "Euler traversal"
     * of the union of the spanning tree and the matching.
     * During this last step, the algorithm simply skips the visited nodes
     * (i.e. creates shortcuts) assuming that the triangle inequality holds
     * for the cost function.
     *
     * @tparam CM Type of the cost map.
     *
     * @warning CM::Value must be a signed number type.
     */
    template < typename CM >
    class ChristofidesTsp {
      public:
      /** Type of the cost map */
      using CostMap = CM;
      /** Type of the edge costs */
      using Cost = typename CM::Value;

      private:
      GRAPH_TYPEDEFS(FullGraph);

      const FullGraph&                _gr;
      const CostMap&                  _cost;
      nt::TrivialDynamicArray< Node > _path;
      Cost                            _sum;

      public:
      /**
       * @brief Constructor
       *
       * Constructor.
       * @param gr The @ref FullGraph "full graph" the algorithm runs on.
       * @param cost The cost map.
       */
      ChristofidesTsp(const FullGraph& gr, const CostMap& cost) : _gr(gr), _cost(cost) {}

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
        } else if (_gr.nodeNum() == 2) {
          _path.push_back(_gr(0));
          _path.push_back(_gr(1));
          return _sum = 2 * _cost[_gr.edge(_gr(0), _gr(1))];
        }

        // Compute min. cost spanning tree
        nt::TrivialDynamicArray< Edge > tree;
        kruskal(_gr, _cost, std::back_inserter(tree));

        FullGraph::NodeMap< int > deg(_gr, 0);
        for (int i = 0; i != int(tree.size()); ++i) {
          Edge e = tree[i];
          ++deg[_gr.u(e)];
          ++deg[_gr.v(e)];
        }

        // Copy the induced subgraph of odd nodes
        nt::TrivialDynamicArray< Node > odd_nodes;
        for (NodeIt u(_gr); u != INVALID; ++u) {
          if (deg[u] % 2 == 1) odd_nodes.push_back(u);
        }

        SmartGraph                  sgr;
        SmartGraph::EdgeMap< Cost > scost(sgr);
        for (int i = 0; i != int(odd_nodes.size()); ++i) {
          sgr.addNode();
        }
        for (int i = 0; i != int(odd_nodes.size()); ++i) {
          for (int j = 0; j != int(odd_nodes.size()); ++j) {
            if (j == i) continue;
            SmartGraph::Edge e = sgr.addEdge(sgr.nodeFromId(i), sgr.nodeFromId(j));
            scost[e] = -_cost[_gr.edge(odd_nodes[i], odd_nodes[j])];
          }
        }

        // Compute min. cost perfect matching
        MaxWeightedPerfectMatching< SmartGraph, SmartGraph::EdgeMap< Cost > > mwpm(sgr, scost);
        mwpm.run();

        for (SmartGraph::EdgeIt e(sgr); e != INVALID; ++e) {
          if (mwpm.matching(e)) { tree.push_back(_gr.edge(odd_nodes[sgr.id(sgr.u(e))], odd_nodes[sgr.id(sgr.v(e))])); }
        }

        // Join the spanning tree and the matching
        sgr.clear();
        for (int i = 0; i != _gr.nodeNum(); ++i) {
          sgr.addNode();
        }
        for (int i = 0; i != int(tree.size()); ++i) {
          int ui = _gr.id(_gr.u(tree[i])), vi = _gr.id(_gr.v(tree[i]));
          sgr.addEdge(sgr.nodeFromId(ui), sgr.nodeFromId(vi));
        }

        // Compute the tour from the Euler traversal
        SmartGraph::NodeMap< bool > visited(sgr, false);
        for (EulerIt< SmartGraph > e(sgr); e != INVALID; ++e) {
          SmartGraph::Node n = sgr.target(e);
          if (!visited[n]) {
            _path.push_back(_gr(sgr.id(n)));
            visited[n] = true;
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
