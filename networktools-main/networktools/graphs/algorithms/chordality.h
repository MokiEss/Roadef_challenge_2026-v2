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
 * @brief Algorithms for chordal graphs.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CHORDALITY_H_
#define _NT_CHORDALITY_H_

#include "connectivity.h"   // < isClique
#include "../../graphs/mutable_graph.h"
#include "../../core/quick_heap.h"
#include "../../core/maps.h"

namespace nt {
  namespace graphs {

    /**
     * @class Chordality
     * @headerfile chordality.h
     * @brief Algorithms for chordal graphs. A graph is chordal if every cycle of length
     * at least 4 has a chord (an edge joining two non-adjacent vertices in a cycle) [1].
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam CGR The type of the digraph used to represent the chordal graph
     *
     * Note that the minimum fill-in  problem (find the minimum set
     * of edges to add in a graph to make it chordal) is NP-complete [2].
     *
     * [1] R. E. Tarjan and M. Yannakakis,
     *     "Simple linear-time algorithms to test chordality of graphs, test
     *      acyclicity of hypergraphs, and selectively reduce acyclic hypergraphs",
     *     SIAM J. Comput., 13 (1984), pp. 566–579.
     *
     * [2] M. Yannakakis,
     *     "Computing the Minimum Fill-In is NP-Complete",
     *     SIAM. J. on Algebraic and Discrete Methods, 2(1), 77–79, 1981
     */
    template < class GR, class CGR = MutableGraph< GR, true > >
    struct Chordality {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);
      using ChordalGraph = CGR;

      using DisabledEdgesMap = nt::BoolMap;
      using HeapMap = IntStaticNodeMap;
      using Heap = nt::OptimalHeap< int, HeapMap >;

      enum HEURISTIC { GREEDY = 0 };

      const Digraph& _g;
      Heap           _heap;

      Chordality(const Digraph& g) : _g(g) { _heap.init(g.nodeNum()); }

      static int fillInHeur(const ChordalGraph& g, Node v) { return 0; }
      static int degreeHeur(const ChordalGraph& g, Node v) { return g.degree(v); }

      /**
       * @brief Check whether the given graph is chordal.
       *
       * @return treewidth value if the graph is chordal, 0 otherwise.
       *
       */
      int isChordal() const noexcept {
        // Treewidth is defined for connected graphs of at least 2 vertices
        if (_g.nodeNum() < 2) return 0;

        NodeSet< Digraph > treated(_g);
        NodeSet< Digraph > clique(_g);
        int                i_max_clique = 0;

        for (int i = 0; i < _g.nodeNum(); ++i) {
          const Node max_card_node = _maxCardinalityNode(treated);
          treated.insert(max_card_node);

          clique.clear();

          // The input digraph is assumed to be bidirected wlog we iterate on outgoing arcs of max_card_node
          for (OutArcIt e(_g, max_card_node); e != INVALID; ++e) {
            const Node neighbor = _g.target(e);
            if (treated.contains(neighbor)) clique.insert(neighbor);
          }

          if (!isClique(_g, clique)) return 0;
          if (i_max_clique < clique.size()) i_max_clique = clique.size();
        }

        return i_max_clique;
      }

      /**
       * @brief Compute an elimination order using minimum degree heuristic.
       *
       * This is a convenience method that creates a temporary MutableGraph internally.
       *
       * @param elim_order Output array that will store the elimination order.
       * @return The treewidth upper bound (maximum clique size during elimination).
       */
      int findElimOrderMinDegree(TrivialDynamicArray< Node >& elim_order) noexcept {
        ChordalGraph chordal_g(_g);
        return findElimOrder(elim_order, chordal_g, degreeHeur);
      }

      /**
       * @brief Compute an elimination order using minimum degree heuristic with an external chordal graph.
       *
       * This overload allows reusing a MutableGraph instance, which is useful when you need
       * to inspect the chordal completion or perform multiple operations on it.
       *
       * @param elim_order Output array that will store the elimination order.
       * @param chordal_g Mutable copy of the graph that will be triangulated during elimination.
       * @return The treewidth upper bound (maximum clique size during elimination).
       */
      int findElimOrderMinDegree(TrivialDynamicArray< Node >& elim_order, ChordalGraph& chordal_g) noexcept {
        return findElimOrder(elim_order, chordal_g, degreeHeur);
      }

      /**
       * @brief Compute an elimination order according to the given heuristic function.
       *
       * This is the main algorithm that computes an elimination order by iteratively selecting
       * vertices to eliminate based on the heuristic score, triangulating their neighborhoods,
       * and updating the graph accordingly.
       *
       * @tparam HeurFct Heuristic function type with signature: int(const ChordalGraph&, Node).
       * @tparam bUseUB If true, abort early if a vertex degree exceeds the upper bound.
       * @tparam bApplyChanges If true, also modify the original input graph.
       *
       * @param elim_order Output array that will store the elimination order.
       * @param chordal_g Mutable copy of the graph that will be triangulated during elimination.
       * @param heur_fct Heuristic function to evaluate the score of each vertex.
       * @param ub Upper bound on treewidth (only used if bUseUB=true).
       * @return The treewidth upper bound (maximum clique size), or NT_INT32_MAX if bound exceeded.
       */
      template < typename HeurFct, bool bUseUB = false, bool bApplyChanges = false >
      int findElimOrder(TrivialDynamicArray< Node >& elim_order,
                        ChordalGraph&                chordal_g,
                        HeurFct                      heur_fct,
                        int                          ub = NT_INT32_MAX) noexcept {
        elim_order.removeAll();
        elim_order.ensure(chordal_g.nodeNum());

        int i_max_clique = 0;

        _heap.clear();
        for (NodeIt v(_g); v != nt::INVALID; ++v) {
          const int k = heur_fct(chordal_g, v);
          _heap.push(v, k);
        }

        while (!_heap.empty()) {
          const Node u = _heap.top();
          const int  i_score_u = _heap.prio();
          const int  i_new_score = heur_fct(chordal_g, u);
          if (i_score_u != i_new_score) {
            _heap.set(u, i_new_score);
            continue;
          }

          _heap.pop();
          elim_order.add(u);

          nt::ArrayView< Node > neighbors = chordal_g.neighbors(u);

          if constexpr (bUseUB) {
            if (neighbors.size() >= ub) {
              elim_order.removeAll();
              return NT_INT32_MAX;
            }
          }

          if (neighbors.size() > i_max_clique) i_max_clique = neighbors.size();

          // Triangulate
          for (int i = 0; i < neighbors.size() - 1; ++i) {
            const Node u = neighbors[i];
            for (int j = i + 1; j < neighbors.size(); ++j) {
              const Node v = neighbors[j];
              if (!chordal_g.hasArc(u, v)) {
                chordal_g.addEdge(u, v);
                if constexpr (bApplyChanges) { _g.addEdge(u, v); }
              }
            }
          }

          for (const Node v: neighbors) {
            // Perf: removing the arc vu (and not both uv & vu) is sufficient for the validity of the algorithm and
            // leaves untouched the out-neighborhood of u.
            chordal_g.removeArc(v, u);

            // Score value of v may have changed => recompute it and update the heap accordingly
            const int i_new_val = heur_fct(chordal_g, v);
            assert(_heap.state(v) != Heap::POST_HEAP);   // If assert, make sure the graph is bidirected
            if (i_new_val < _heap[v]) _heap.decrease(v, i_new_val);
          }
        }

        // Restore previously deleted arcs
        for (NodeIt u(_g); u != nt::INVALID; ++u) {
          nt::ArrayView< Node > neighbors = chordal_g.neighbors(u);
          for (const Node v: neighbors) {
            chordal_g.addArc(v, u);
          }
        }

        assert(elim_order.size() == chordal_g.nodeNum());
        return i_max_clique + 1;
      }

      //---------------------------------------------------------------------------
      // Auxiliary methods
      //---------------------------------------------------------------------------

      Node _maxCardinalityNode(const NodeSet< Digraph >& treated) const noexcept {
        int  i_max_number = -1;
        Node i_max_cardinality_node = INVALID;

        for (NodeIt n(_g); n != INVALID; ++n) {
          if (treated.contains(n)) continue;

          int i_number = 0;
          for (OutArcIt e(_g, n); e != INVALID; ++e) {
            const Node neighbor = _g.target(e);
            i_number += treated.contains(neighbor);
          }

          if (i_number > i_max_number) {
            i_max_number = i_number;
            i_max_cardinality_node = n;
          }
        }

        return i_max_cardinality_node;
      }
    };
  }   // namespace graphs
}   // namespace nt

#endif
