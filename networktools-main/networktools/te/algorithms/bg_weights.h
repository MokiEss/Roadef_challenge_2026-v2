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
 * @author Nirussan Sivanand (nirussan.sivanand@orange.com)
 */

#ifndef _NT_BG_WEIGHTS_H_
#define _NT_BG_WEIGHTS_H_

#include "../../graphs/algorithms/bfs.h"

namespace nt {
  namespace te {

    /**
     * @class BGWeights
     * @headerfile bg_weights.h
     * @brief Algorithm based on [1] that computes a set of integer arc weights/metrics for a given graph G such that :
     *          (i) the weight of every arc is a positive integer ≤ 6R − 1 (R = radius of G),
     *          (ii) every arc of G belongs to at least one shortest path in the sense of these weights,
     *          (iii) there is exactly one shortest path between any pair of vertices.
     *
     * Example of usage :
     *
     * @code
     * int main() {
     *  // Create a directed graph along with an arc map to store the computed weights/metrics
     *  Digraph digraph;
     *  Digraph::ArcMap<float> mm(digraph);
     *
     *  // Instantiate BGWeights using the previous graph
     *  BGWeights< Digraph > bg_weights(digraph);
     *
     *  // Set the arc map that will receive the calculated weight
     *  bg_weights.metricMap(mm);
     *
     *  // Run BGWeights's algorithm
     *  bg_weights.run();
     *
     *  return 0;
     * }
     * @endcode
     *
     * @tparam GR The type of the digraph the algorithm runs on. The digraph is assumed to be bidirected.
     * @tparam MV Type of the metrics (int, float, ...)
     * @tparam MM A concepts::ReadMap "readable" arc map storing the metric values.
     * @tparam W0 Value of w0 used in the formula to compute the weihts (see [1])
     *
     *  [1] Walid Ben-Ameur, Éric Gourdin:
     *      Internet Routing and Related Topology Issues.
     *      SIAM J. Discret. Math. 17(1): 18-49 (2003)
     */
    template < typename GR, typename MV = double, typename MM = typename GR::template ArcMap< MV >, int W0 = 3 >
    // requires nt::concepts::Digraph< GR >
    struct BGWeights {
      static_assert(W0 >= 3, "W0 must be greater or equal to 3.");
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using MetricMap = MM;
      using Metric = typename MM::Value;

      using NodeMap = typename Digraph::template StaticNodeMap< Node >;
      using NodeArray = nt::TrivialDynamicArray< Node >;
      using ArcSet = graphs::ArcSet< Digraph >;

      struct BGWeightsBfsVisitor {
        BGWeights& _bg;
        bool       _b_build;

        BGWeightsBfsVisitor(BGWeights& bg) : _bg(bg), _b_build(false){};

        constexpr void start(const Node& node) noexcept {}
        constexpr void reach(const Node& node) noexcept {}
        constexpr void process(const Node& node) noexcept {}
        constexpr void discover(const Arc& arc) noexcept {
          assert(_bg._p_mm);
          if (_b_build) {
            MetricMap& mm = *_bg._p_mm;

            const Digraph& g = _bg.digraph();
            const Node     src = g.target(arc);
            const Node     trg = g.source(arc);

            _bg._T0[src] = trg;
            mm[arc] = W0;

            const Arc reverse = graphs::findArc(g, src, trg);
            mm[reverse] = W0;
          }
        }
        constexpr void examine(const Arc& arc) noexcept {}

        void buildTree(bool b_build) { _b_build = b_build; }
      };

      const Digraph&      _digraph;
      BGWeightsBfsVisitor _bg_weights_visitor;
      graphs::BfsDistanceVisit< Digraph, BGWeightsBfsVisitor >
                 _bfs;    // < Used to compute the routed tree T0 and the central vertex
      MetricMap* _p_mm;   // < Contains the computed weights
      NodeMap    _T0;

      Node _central_vertex;

      explicit BGWeights(const Digraph& g) :
          _digraph(g), _bg_weights_visitor(*this), _bfs(g, _bg_weights_visitor), _p_mm(nullptr), _T0(_digraph) {}

      /**
       * @brief Compute the arc weights such that the properties (i), (ii) and (iii) are met
       *
       */
      void run() noexcept {
        if (!_p_mm) return;
        _searchCentralVertex();
        _buildT0();
        _computeWeights();
      }

      /**
       * @brief Set the map that stores the arcs metrics.
       *
       */
      constexpr void metricMap(MetricMap& mm) noexcept { _p_mm = &mm; }

      /**
       * @brief Return a const reference to the digraph
       *
       */
      const Digraph& digraph() const noexcept { return _digraph; }

      //---------------------------------------------------------------------------
      // Auxiliary methods
      //---------------------------------------------------------------------------

      /**
       * @brief Compute a central vertex v (indexed by 0) of the graph G. (Recall that a
       *        central vertex is a vertex such that the maximum distance from it to any
       *        other vertex is minimal)
       */
      void _searchCentralVertex() noexcept {
        int i_central_vertex_distance = NT_INT32_MAX;
        for (NodeIt v(_digraph); v != nt::INVALID; ++v) {
          _bfs.run(v);

          int i_max_distance = 0;
          for (NodeIt n(_digraph); n != nt::INVALID; ++n) {
            const int i_distance = _bfs.dist(n);
            if (i_distance > i_max_distance) i_max_distance = i_distance;
          }

          if (i_max_distance <= i_central_vertex_distance) {
            i_central_vertex_distance = i_max_distance;
            _central_vertex = v;
          }
        }
      }

      /**
       * @brief Creates the T0 tree and runs a BFS on the central vertex
       *
       */
      void _buildT0() noexcept {
        _bg_weights_visitor.buildTree(true);
        _bfs.run(_central_vertex);
      }

      /**
       * @brief Associate a weight W0 ×δ0(i, j)−k0(i, j) with each edge e = ij ∈ E \E0, and W0 otherwise.
       *
       */
      void _computeWeights() noexcept {
        assert(_p_mm);
        NodeArray  path_ij;
        NodeArray  path_j;
        MetricMap& mm = *_p_mm;

        for (ArcIt a(_digraph); a != nt::INVALID; ++a) {
          if (mm[a] == W0) continue;
          const Node i = _digraph.source(a);
          const Node j = _digraph.target(a);
          if (j < i) continue;

          path_ij.removeAll();
          path_j.removeAll();

          const int gap = _bfs.dist(i) - _bfs.dist(j);
          if (gap < 0) {
            path_ij.add(j);
            _findPath(j, i, path_ij, path_j, -gap);
          } else {
            path_ij.add(i);
            _findPath(i, j, path_ij, path_j, gap);
          }

          // calculates k0
          int k0 = 0;
          for (int x = 0; x < path_ij.size() - 2; ++x)
            for (int z = x + 2; z < path_ij.size(); ++z)
              if (graphs::findArc(_digraph, path_ij[x], path_ij[z]) != nt::INVALID) ++k0;

          // applies we = W0 × δ0(i, j)−k0(i, j) with each edge e = ij ∈ E \E0.
          const int i_dist = path_ij.size() - 1;
          mm[a] = W0 * i_dist - k0;
          const Arc reverse = graphs::findArc(_digraph, j, i);
          mm[reverse] = W0 * i_dist - k0;
        }
      }

      /**
       * @brief Stores in path_ij the nodes passed by in order to get from i to j
       *
       */
      void _findPath(Node i, Node j, NodeArray& path_ij, NodeArray& path_j, int gap) const noexcept {
        while (gap-- > 1) {
          i = _T0[i];
          path_ij.add(i);
        }

        while (i != _central_vertex) {
          i = _T0[i];
          path_ij.add(i);
          if (i == j) break;
          path_j.add(j);
          j = _T0[j];
          if (i == j) break;
        }

        for (int i = path_j.size() - 1; i >= 0; --i)
          path_ij.push_back(path_j[i]);
      }
    };

  }   // namespace te
}   // namespace nt

#endif