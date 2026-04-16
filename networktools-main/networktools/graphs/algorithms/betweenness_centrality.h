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
 * @brief This class implements betweenness centrality computation for directed graphs
 *
 * @author Qiao Zhang (qiao.zhang@orange.com)
 */

#ifndef _NT_BETWEENNESS_CENTRALITY_H_
#define _NT_BETWEENNESS_CENTRALITY_H_

#include "../tools.h"
#include "../graph_element_set.h"
#include "../../core/tolerance.h"
#include "../../core/arrays.h"
#include "../../core/stack.h"
#include "../../core/queue.h"
#include "dijkstra.h"

namespace nt {
  namespace graphs {

    /**
     * @class BetweennessCentrality
     * @headerfile betweenness_centrality.h
     * @brief This class implements betweenness centrality computation for directed graphs
     *
     * Betweenness centrality is a measure of centrality based on shortest paths.
     * For every pair of vertices in a graph, there exists at least one shortest path
     * between the vertices. The betweenness centrality for each vertex is the number
     * of these shortest paths that pass through the vertex.
     *
     * Reference python source code of the algorithm betweenness_centrality:
     * https://networkx.org/documentation/stable/_modules/networkx/algorithms/centrality/betweenness.html#betweenness_centrality
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam WM The type of the weight map (defaults to a map with uniform weights).
     */
    template < typename GR, typename WM = typename GR::template ArcMap< int > >
    struct BetweennessCentrality {
      using Digraph = GR;
      using WeightMap = WM;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      struct Parameters {
        bool             normalized = true;
        bool             endpoints = false;
        double           tolerance = 1e-10;
        const WeightMap* p_weight_map = nullptr;

        Parameters() = default;
        explicit Parameters(const WeightMap& wm) : p_weight_map(&wm) {}
      };

      const Digraph&                               _graph;
      typename Digraph::template NodeMap< double > _centrality;
      Parameters                                   _parameters;

      explicit BetweennessCentrality(const Digraph& g) : _graph(g), _centrality(g) {}

      explicit BetweennessCentrality(const Digraph& g, const WeightMap& weight_map) :
          _graph(g), _centrality(g), _parameters(weight_map) {}

      explicit BetweennessCentrality(const Digraph& g, const Parameters& params) :
          _graph(g), _centrality(g), _parameters(params) {}

      /**
       * @brief Run the betweenness centrality algorithm using default parameters
       */
      void run() { run(_parameters.normalized, _parameters.endpoints); }

      /**
       * @brief Run the betweenness centrality algorithm with custom parameters
       * @param params Parameters structure containing algorithm settings
       */
      void run(const Parameters& params) {
        // Update internal parameters (except weight_map which is a reference)
        _parameters.normalized = params.normalized;
        _parameters.endpoints = params.endpoints;
        _parameters.tolerance = params.tolerance;
        run(params.normalized, params.endpoints);
      }

      /**
       * @brief Run the betweenness centrality algorithm
       * @param normalized Whether to apply normalization (default: true)
       * @param endpoints Whether to include endpoints in the calculation (default: false)
       */
      void run(bool normalized, bool endpoints) {
        // Update internal parameters to match the call parameters
        _parameters.normalized = normalized;
        _parameters.endpoints = endpoints;

        const int n = _graph.nodeNum();

        // Ensure the centrality map is properly sized for the current graph
        // This is important when nodes are added to the graph after object creation
        for (NodeIt v(_graph); v != nt::INVALID; ++v) {
          // Access each node to ensure the map is properly initialized
          _centrality[v] = 0.0;
        }

        // For each node s, compute shortest paths and accumulate centrality
        for (NodeIt s(_graph); s != nt::INVALID; ++s) {
          _computeCentralityFromSource(s);
        }

        // Normalize centrality values using rescale function
        NodeSet< Digraph > sampled_nodes(_graph);
        // use all nodes as sampled nodes
        for (NodeIt v(_graph); v != nt::INVALID; ++v) {
          sampled_nodes.insert(v);
        }
        rescale(_graph, _centrality, n, normalized, true, -1, endpoints, &sampled_nodes, _parameters.tolerance);
      }

      /**
       * @brief Get the betweenness centrality value for a node
       * @param node The node
       * @return The betweenness centrality value
       */
      double centrality(Node node) const noexcept { return _centrality[node]; }

      /**
       * @brief Get the centrality map
       * @return Reference to the centrality map
       */
      const typename Digraph::template NodeMap< double >& centralityMap() const noexcept { return _centrality; }

      /**
       * @brief Get the current parameters
       * @return Reference to the parameters structure
       */
      const Parameters& getParameters() const noexcept { return _parameters; }

      /**
       * @brief Set individual parameter values
       * Note: weight_map cannot be changed as it's a reference
       */
      void setNormalized(bool normalized) noexcept { _parameters.normalized = normalized; }
      void setEndpoints(bool endpoints) noexcept { _parameters.endpoints = endpoints; }
      void setTolerance(double tolerance) noexcept { _parameters.tolerance = tolerance; }

      /**
       * @brief Rescale/normalize betweenness centrality values
       * @param graph Reference to the graph
       * @param betweenness Reference to the centrality map to rescale
       * @param n Number of nodes in the graph
       * @param normalized Whether to apply normalization
       * @param directed Whether the graph is directed
       * @param k Number of sampled source nodes (if applicable)
       * @param endpoints Whether to include endpoints in the calculation
       * @param sampled_nodes Set of sampled nodes (if applicable)
       * @param tolerance Tolerance value for numerical comparisons
       */
      static void rescale(const Digraph&                                graph,
                          typename Digraph::template NodeMap< double >& betweenness,
                          int                                           n,
                          bool                                          normalized = true,
                          bool                                          directed = true,
                          int                                           k = -1,
                          bool                                          endpoints = false,
                          const NodeSet< Digraph >*                     sampled_nodes = nullptr,
                          double                                        tolerance = 1e-10) {
        // N is used to count the number of valid (s, t) pairs where s != t that
        // could have a path pass through v. If endpoints is False, then v must
        // not be the target t, hence why we subtract by 1.

        if (n <= 2) {
          // For graphs with 2 or fewer nodes, centrality should be 0
          return;
        }

        // Declare local variables
        double scale = 1.0;
        int    N = (endpoints) ? (n) : (n - 1);
        int    k_score = (k < 0) ? N : k;
        double correction = (directed) ? 1.0 : 2.0;

        if (k < 0 || endpoints) {
          // Check for potential division by zero
          if (k_score == 0 || (N - 1) == 0) { return; }

          if (normalized) {
            // For normalized betweenness centrality
            // Standard normalization for directed graphs: 1/((n-1)(n-2))
            scale = 1.0 / (static_cast< double >(k_score) * (N - 1));
          } else {
            // scale to the full BC
            scale = static_cast< double >(N) / (static_cast< double >(k_score) * correction);
          }

          // Apply scaling if scale is not approximately 1.0
          if (!nt::Tolerance< double >(tolerance).equal(scale, 1.0)) {
            for (NodeIt v(graph); v != nt::INVALID; ++v) {
              betweenness[v] *= scale;
            }
          }
          return;
        }

        // Sampling adjustment needed when excluding endpoints when using k. In this
        // case, we need to handle source nodes differently from non-source nodes,
        // because source nodes can't include themselves since endpoints are excluded.
        // Without this, k == n would be a special case that would violate the
        // assumption that node `v` is not one of the (s, t) node pairs.
        double scale_source = 0.0;
        double scale_nonsource = 0.0;

        // Check for potential division by zero
        if ((N - 1) == 0 || k_score == 0) { return; }

        if (normalized) {
          scale_source = (k_score > 1) ? (1.0 / (static_cast< double >(k_score) * (N - 1))) : 0.0;
          scale_nonsource = 1.0 / (static_cast< double >(k_score) * (N - 1));
        } else {
          scale_source =
             (k_score > 1) ? (static_cast< double >(N) / (static_cast< double >(k_score - 1) * correction)) : 0.0;
          scale_nonsource = static_cast< double >(N) / (static_cast< double >(k_score) * correction);
        }

        for (NodeIt v(graph); v != nt::INVALID; ++v) {
          if (sampled_nodes && sampled_nodes->contains(v)) {
            if (scale_source > 0.0) { betweenness[v] *= scale_source; }
          } else {
            betweenness[v] *= scale_nonsource;
          }
        }
      }

      private:
      /**
       * @brief Check if the weight map has uniform weights (all weights are equal)
       * @return True if all weights are equal, false otherwise
       */
      bool _isWeightMapUniform() const {
        if (!_parameters.p_weight_map) return true;

        ArcIt arc(_graph);
        if (arc == nt::INVALID) return true;   // Check if graph has no arcs - uniform by definition

        const typename WeightMap::Value reference_weight = (*_parameters.p_weight_map)[arc];
        ++arc;

        for (; arc != nt::INVALID; ++arc)
          if ((*_parameters.p_weight_map)[arc] != reference_weight) return false;   // Found a different weight

        return true;   // All weights are equal
      }

      /**
       * @brief Accumulate centrality values including endpoints
       * @param S Stack of nodes from BFS traversal
       * @param P Predecessors map
       * @param sigma Number of shortest paths
       * @param delta Dependency values
       * @param s Source node
       */
      void _accumulateEndpoints(nt::Stack< Node >&                                                           S,
                                const typename Digraph::template NodeMap< nt::TrivialDynamicArray< Node > >& P,
                                const typename Digraph::template NodeMap< int >&                             sigma,
                                typename Digraph::template NodeMap< double >&                                delta,
                                Node                                                                         s) {
        // Add contribution for source node (endpoints included)
        _centrality[s] += static_cast< double >(S.size() - 1);

        // Back-propagation of dependencies
        while (!S.empty()) {
          Node w = S.top();
          S.pop();

          double coeff = (1.0 + delta[w]) / static_cast< double >(sigma[w]);
          for (Node v: P[w]) {
            delta[v] += static_cast< double >(sigma[v]) * coeff;
          }

          if (w != s) { _centrality[w] += delta[w] + 1.0; }
        }
      }

      /**
       * @brief Accumulate centrality values excluding endpoints (basic version)
       * @param S Stack of nodes from BFS traversal
       * @param P Predecessors map
       * @param sigma Number of shortest paths
       * @param delta Dependency values
       * @param s Source node
       */
      void _accumulateBasic(nt::Stack< Node >&                                                           S,
                            const typename Digraph::template NodeMap< nt::TrivialDynamicArray< Node > >& P,
                            const typename Digraph::template NodeMap< int >&                             sigma,
                            typename Digraph::template NodeMap< double >&                                delta,
                            Node                                                                         s) {
        // Back-propagation of dependencies
        while (!S.empty()) {
          Node w = S.top();
          S.pop();

          double coeff = (1.0 + delta[w]) / static_cast< double >(sigma[w]);
          for (Node v: P[w]) {
            delta[v] += static_cast< double >(sigma[v]) * coeff;
          }

          if (w != s) { _centrality[w] += delta[w]; }
        }
      }

      /**
       * @brief Compute betweenness centrality contribution from a single source using BFS (for uniform weights)
       * @param s Source node
       */
      void _computeCentralityFromSourceBFS(Node s) {
        // Brandes algorithm for betweenness centrality with BFS
        nt::Stack< Node >                                                     stack;
        typename Digraph::template NodeMap< nt::TrivialDynamicArray< Node > > P(_graph);
        typename Digraph::template NodeMap< int >                             sigma(_graph);
        typename Digraph::template NodeMap< int >                             d(_graph);
        typename Digraph::template NodeMap< double >                          delta(_graph);

        // Initialize
        for (NodeIt v(_graph); v != nt::INVALID; ++v) {
          P[v].clear();
          sigma[v] = 0;
          d[v] = -1;
          delta[v] = 0.0;
        }

        sigma[s] = 1;
        d[s] = 0;

        nt::Queue< Node > queue;
        queue.push(s);

        // BFS to find shortest paths
        while (!queue.empty()) {
          Node v = queue.front();
          queue.pop();
          stack.push(v);

          for (OutArcIt arc(_graph, v); arc != nt::INVALID; ++arc) {
            Node w = _graph.target(arc);

            // First time we find w?
            if (d[w] < 0) {
              queue.push(w);
              d[w] = d[v] + 1;
            }

            // Shortest path to w via v?
            if (d[w] == d[v] + 1) {
              sigma[w] += sigma[v];
              P[w].push_back(v);
            }
          }
        }

        // Accumulation - back-propagation of dependencies
        _accumulate(stack, P, sigma, delta, s);
      }

      /**
       * @brief Compute betweenness centrality contribution from a single source using Dijkstra (for non-uniform
       * weights)
       * @param s Source node
       */
      void _computeCentralityFromSourceDijkstra(Node s) {
        assert(_parameters.p_weight_map);

        // Brandes algorithm for betweenness centrality using nt::graphs::Dijkstra
        nt::Stack< Node >                                                     S;
        typename Digraph::template NodeMap< nt::TrivialDynamicArray< Node > > P(_graph);
        typename Digraph::template NodeMap< int >                             sigma(_graph);
        typename Digraph::template NodeMap< double >                          delta(_graph);

        // Initialize
        for (NodeIt v(_graph); v != nt::INVALID; ++v) {
          P[v].clear();
          sigma[v] = 0;
          delta[v] = 0.0;
        }

        sigma[s] = 1;

        // Use nt::graphs::Dijkstra to compute shortest paths
        using DijkstraType = nt::graphs::Dijkstra< Digraph, typename WeightMap::Value >;
        DijkstraType dijkstra(_graph, *_parameters.p_weight_map);

        // Run Dijkstra from source s to find all shortest paths
        dijkstra.run(s);

        // Build the stack S with nodes in order of non-increasing distance from source
        // and compute sigma values simultaneously
        nt::TrivialDynamicArray< nt::Pair< typename WeightMap::Value, Node > > distance_node_pairs;

        for (NodeIt v(_graph); v != nt::INVALID; ++v) {
          if (dijkstra.reached(v)) { distance_node_pairs.push_back({dijkstra.dist(v), v}); }
        }

        // Sort by distance (ascending) for processing in topological order
        nt::sort(distance_node_pairs,
                 [](const nt::Pair< typename WeightMap::Value, Node >& a,
                    const nt::Pair< typename WeightMap::Value, Node >& b) { return a.first < b.first; });

        // Build predecessors and compute sigma using topological order
        for (const auto& pair: distance_node_pairs) {
          Node v = pair.second;

          if (v == s) continue;

          // Get all predecessor arcs for this node
          int num_preds = dijkstra.numPred(v);
          for (int i = 0; i < num_preds; ++i) {
            Arc  pred_arc = dijkstra.predArc(v, i);
            Node pred_node = _graph.source(pred_arc);
            P[v].push_back(pred_node);
            // Accumulate sigma from this predecessor
            sigma[v] += sigma[pred_node];
          }
        }

        // Build stack in reverse topological order (decreasing distance)
        // We want to process nodes in order of decreasing distance from source
        // So the node with highest distance should be processed first (top of stack)
        for (const auto& pair: distance_node_pairs) {
          S.push(pair.second);
        }

        // Accumulation - back-propagation of dependencies
        _accumulate(S, P, sigma, delta, s);
      }

      /**
       * @brief Accumulate centrality values based on endpoints parameter
       * @param S Stack of nodes from traversal
       * @param P Predecessors map
       * @param sigma Number of shortest paths
       * @param delta Dependency values
       * @param s Source node
       */
      void _accumulate(nt::Stack< Node >&                                                           S,
                       const typename Digraph::template NodeMap< nt::TrivialDynamicArray< Node > >& P,
                       const typename Digraph::template NodeMap< int >&                             sigma,
                       typename Digraph::template NodeMap< double >&                                delta,
                       Node                                                                         s) {
        if (_parameters.endpoints) {
          _accumulateEndpoints(S, P, sigma, delta, s);
        } else {
          _accumulateBasic(S, P, sigma, delta, s);
        }
      }

      /**
       * @brief Compute betweenness centrality contribution from a single source
       * @param s Source node
       */
      void _computeCentralityFromSource(Node s) {
        // Use BFS for uniform weights, Dijkstra for non-uniform weights
        if (_isWeightMapUniform()) {
          _computeCentralityFromSourceBFS(s);
        } else {
          _computeCentralityFromSourceDijkstra(s);
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif   // _NT_BETWEENNESS_CENTRALITY_H_
