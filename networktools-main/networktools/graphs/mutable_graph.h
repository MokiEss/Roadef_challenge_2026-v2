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
 * @brief Mutable graph data structure optimized for frequent edge additions/removals and existence checks.
 *
 * This data structure is specifically designed for algorithms that require:
 * - O(1) edge existence checks (via adjacency matrix)
 * - Efficient edge addition and removal
 * - Temporary modifications to a graph copy
 *
 * Primary use case: Chordality and tree decomposition algorithms where graphs are
 * triangulated by adding many edges while checking for existing edges frequently.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_MUTABLE_GRAPH_H_
#define _NT_MUTABLE_GRAPH_H_

#include "tools.h"
#include "../core/arrays.h"

namespace nt {
  namespace graphs {

    /**
     * @class MutableGraph
     * @headerfile mutable_graph.h
     * @brief Mutable graph copy optimized for efficient edge operations and existence checks.
     *
     * This class stores a mutable copy of a graph with O(1) edge existence checks using
     * an adjacency matrix, combined with adjacency lists for efficient neighbor iteration.
     * It's specifically designed for algorithms that need to:
     * - Frequently check if edges exist during computation
     * - Add and remove edges dynamically
     * - Maintain a temporary modified copy of an input graph
     *
     * @tparam GR The type of the graph to copy from.
     * @tparam UNDIR If true, maintains bidirectional edges (undirected graph semantics).
     * @tparam SSO Small-size optimization for adjacency lists (default 8).
     *
     * @see Chordality, TreeDecomposition for primary use cases
     */
    template < class GR, bool UNDIR = false, int SSO = 8 >
    struct MutableGraph {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      nt::DynamicArray< TrivialDynamicArray< Node, SSO > > _adj_list;
      nt::BitArray                                         _matrix;
      int                                                  _i_num_arcs;

      MutableGraph(int n) : _i_num_arcs(0) { init(n); }
      explicit MutableGraph(const Digraph& g) { copyFrom(g); }

      /**
       * @brief Initialize the mutable graph to accomodate n vertices
       *
       * @param n Number of vertices in the graph
       */
      void init(int n) noexcept {
        clear();
        _matrix.extendByBits(n * n);
        _matrix.setBitsToZero();
        _adj_list.ensureAndFill(n);
        _i_num_arcs = 0;
      }

      /**
       * @brief Copies all of the vertices and edges/arcs from graph 'g'
       *
       * @param g The graph to copy from
       */
      void copyFrom(const Digraph& g) noexcept {
        init(g.nodeNum());

        using LinkView_ = LinkView< Digraph >;
        using LinkIt_ = LinkView< Digraph >::LinkIt;

        for (LinkIt_ link(g); link != INVALID; ++link) {
          const Node u = LinkView_::u(g, link);
          const Node v = LinkView_::v(g, link);
          if constexpr (UNDIR)
            addEdge(u, v);
          else
            addArc(u, v);
        }
      }

      /**
       * @brief Remove all vertices and arcs
       *
       */
      void clear() noexcept {
        _matrix.setBitsToZero();
        for (int k = 0; k < _adj_list.size(); ++k)
          _adj_list[k].removeAll();
        _adj_list.removeAll();
        _i_num_arcs = 0;
      }

      /**
       * @brief Return the number of vertices
       *
       */
      constexpr int nodeNum() const noexcept { return _adj_list.size(); }

      /**
       * @brief Return the number of edges
       *
       */
      constexpr int edgeNum() const noexcept { return _i_num_arcs >> 1; }

      /**
       * @brief Return the number of arcs
       *
       */
      constexpr int arcNum() const noexcept { return _i_num_arcs; }

      /**
       * @brief Add an arc from 'u' to 'v'
       *
       */
      void addArc(Node u, Node v) noexcept {
        if (hasArc(u, v)) return;   // Exclude adding multiple arcs
        assert(u != v);
        _adj_list[Digraph::id(u)].add(v);
        _matrix.setOneAt(_idx(u, v));
        ++_i_num_arcs;
      }

      /**
       * @brief Remove the arc uv if it exists
       *
       */
      void removeArc(Node u, Node v) noexcept {
        TrivialDynamicArray< Node, 8 >& list = _adj_list[Digraph::id(u)];
        for (int k = 0; k < list.size(); ++k) {
          const Node& n = list[k];
          if (v == n) {
            list.remove(k);
            _matrix.setZeroAt(_idx(u, v));
            --_i_num_arcs;
            break;
          }
        }
      }

      /**
       * @brief Add an edge between 'u' and 'v'
       * Internally, an edge is represented by two arcs uv and vu
       *
       */
      void addEdge(Node u, Node v) noexcept {
        addArc(u, v);
        addArc(v, u);
      }

      /**
       * @brief Remove the edge uv if it exists
       *
       */
      void removeEdge(Node u, Node v) noexcept {
        removeArc(u, v);
        removeArc(v, u);
      }

      /**
       * @brief Return true if the arc uv exists, false otherwise
       *
       */
      constexpr bool hasArc(Node u, Node v) const noexcept { return _matrix.isOne(_idx(u, v)); }

      /**
       * @brief Return the degree of vertex 'v'
       *
       */
      constexpr int degree(Node v) const noexcept { return _adj_list[Digraph::id(v)].size(); }

      /**
       * @brief Return an array view containing the out-neighbors of vertex 'v'
       *
       */
      constexpr nt::ArrayView< Node > neighbors(Node v) const noexcept { return _adj_list[Digraph::id(v)]; }

      //---------------------------------------------------------------------------
      // Auxiliary methods
      //---------------------------------------------------------------------------

      constexpr int _idx(Node u, Node v) const noexcept { return nodeNum() * Digraph::id(u) + Digraph::id(v); }
    };
  }   // namespace graphs
}   // namespace nt

#endif