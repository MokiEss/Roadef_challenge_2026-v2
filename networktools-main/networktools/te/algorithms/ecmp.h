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
 * @brief Equal-Cost Multi-Path (ECMP) routing algorithm implementation.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_ECMP_H_
#define _NT_ECMP_H_

#include "../../core/algebra.h"
#include "../../core/arrays.h"
#include "../../graphs/algorithms/dijkstra.h"

namespace nt {
  namespace te {

    template < typename DIJKSTRA,
               typename VEC = nt::Vec< double, 1 >,
               typename AFM = typename DIJKSTRA::Digraph::template ArcMap< VEC >,
               typename NFM = typename DIJKSTRA::Digraph::template NodeMap< VEC > >
    struct EcmpBase {
      static_assert(DIJKSTRA::flags() & (graphs::DijkstraFlags::BACKWARD | graphs::DijkstraFlags::PRED_MAP),
                    "Ecmp : DIJKSTRA must have BACKWARD and PRED_MAP flags set");

      using Dijkstra = DIJKSTRA;
      using Digraph = typename Dijkstra::Digraph;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using Heap = typename Dijkstra::Heap;

      using Vector = VEC;
      using VecTraits = VectorTraits< VEC >;
      using ArcFlowMap = AFM;
      using NodeFlowMap = NFM;

      /**
       * The result of computing the split factors.
       */
      enum ResultStatus : int { E_SINGLE_PATH = 0, E_MULTI_PATH, E_NO_PATH };

      TrivialDynamicArray< Arc, 10 > _unique_path;   // SSO = 10 (networks have small graph diameter)

      /**
       * @brief Routes a flow demand from source to target using the ECMP algorithm.
       *
       * @param dijkstra The Dijkstra algorithm instance used for shortest path computations.
       * @param source The source node of the flow demand.
       * @param target The target node of the flow demand.
       * @param flow The amount of flow to route.
       * @param node_flow The node map to store the flow values at each node.
       * @param arc_flow The arc map to store the flow values on each arc.
       * @return true if the demand has been successfully routed, false otherwise (e.g no path)
       */
      ResultStatus run(Dijkstra&    dijkstra,
                       const Node   source,
                       const Node   target,
                       const Vector flow,
                       NodeFlowMap& node_flow,
                       ArcFlowMap&  arc_flow) noexcept {
        return run(dijkstra, source, target, flow, node_flow, [&](const Arc arc, const Vector f) noexcept {
          arc_flow[arc] += f;
        });
      }

      /**
       * @brief Routes a flow demand from source to target using the ECMP algorithm.
       *
       * @param dijkstra The Dijkstra algorithm instance used for shortest path computations.
       * @param source The source node of the flow demand.
       * @param target The target node of the flow demand.
       * @param flow The amount of flow to route.
       * @param node_flow The node map to store the flow values at each node.
       * @param doVisitArc A callable object (e.g., lambda function) that processes each arc and its assigned flow.
       *                   It should have the signature: void(const Arc arc, const Vector flow).
       * @return true if the demand has been successfully routed, false otherwise (e.g no path)
       */
      template < typename F >
      ResultStatus run(Dijkstra&    dijkstra,
                       const Node   source,
                       const Node   target,
                       const Vector flow,
                       NodeFlowMap& node_flow,
                       F            doVisitArc) noexcept {
        // Run dijkstra (backward) from target if not already done
        if (dijkstra.source() != target) dijkstra.run(target);

        // If there is no path from source to target, then the demand cannot be routed => return false
        if (!dijkstra.reached(source)) return E_NO_PATH;

        // If there is a unique shortest path from source to target, then each arc along that path receives the full
        // amount of flow
        _unique_path.removeAll();
        if (dijkstra.hasUniqueShortestPath(source, [&](const Arc arc) { _unique_path.add(arc); })) {
          for (const Arc arc: _unique_path)
            doVisitArc(arc, flow);
          return E_SINGLE_PATH;
        }

        // If there are multiple shortest paths from source to target, then we need to apply doVisitNode (e.g ECMP)
        // to determine the distribution of the flow among the different shortest paths
        node_flow.setBitsToZero();
        node_flow[source] += flow;

        Heap& heap = dijkstra.heap();
        heap.init(dijkstra.digraph().maxNodeId() + 1);
        heap.push(source, -dijkstra.dist(source));

        while (!heap.empty()) {
          const Node v = heap.top();
          heap.pop();
          const Vector f_split = node_flow[v] / dijkstra.numPred(v);

          for (int k = 0; k < dijkstra.numPred(v); ++k) {
            const Arc arc = dijkstra.predArc(v, k);
            doVisitArc(arc, f_split);

            const Node w = dijkstra.digraph().target(arc);
            node_flow[w] += f_split;
            if (heap.state(w) == Heap::PRE_HEAP) heap.push(w, -dijkstra.dist(w));
          }
        }

        return E_MULTI_PATH;
      }
    };

    /**
     * @class Ecmp
     * @headerfile ecmp.h
     * @brief This class provides an implementation of the Equal-Cost Multi-Path (ECMP) routing
     * strategy.
     *
     * **Example usage:**
     * @code
     * SmartDigraph graph;
     * // ... (initialize graph with nodes and arcs)
     * SmartDigraph::ArcMap<double>  weights(graph, 1.0); // uniform weights
     * SmartDigraph::NodeMap<double> node_flow(graph, 0.0);
     * SmartDigraph::ArcMap<double>  arc_flow(graph, 0.0);
     *
     * te::Ecmp< Digraph, double >   ecmp;
     * te::Ecmp< Digraph >::Dijkstra dijkstra(graph, weights);
     * ecmp.run(dijkstra, source_node, target_node, demand_flow, node_flow, arc_flow);
     * @endcode
     *
     * @tparam GR The type of the graph the algorithm runs on.
     * @tparam VEC A vector type to represent flow values.
     * @tparam AFM A concepts::ReadMap "readable" arc map to store the flows of the arcs.
     * @tparam NFM A concepts::ReadMap "readable" node map to store the flows of the nodes.
     *
     */
    template < typename GR,
               typename VEC = nt::Vec< double, 1 >,
               typename AFM = typename GR::template ArcMap< VEC >,
               typename NFM = typename GR::template NodeMap< VEC > >
    using Ecmp = EcmpBase< graphs::DijkstraBackward< GR >, VEC, AFM, NFM >;

  }   // namespace te
}   // namespace nt

#endif   // _ECMP_H_
