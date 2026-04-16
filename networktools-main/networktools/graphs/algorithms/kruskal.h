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
 * This file incorporates work from the LEMON graph library (kruskal.h).
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
 *   - Converted typedef declarations to C++11 using declarations
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Updated header guard macros
 */

/**
 * @ingroup spantree
 * @file
 * @brief Kruskal's algorithm to compute a minimum cost spanning tree
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_KRUSKAL_H_
#define _NT_KRUSKAL_H_

#include "../../core/maps.h"
#include "../../core/unionfind.h"
#include "../../core/common.h"
#include <algorithm>

namespace nt {
  namespace graphs {

    namespace _kruskal_bits {

      // Kruskal for directed graphs.

      template < typename Digraph, typename In, typename Out >
      typename std::enable_if< !nt::concepts::UndirectedTagIndicator< Digraph >,
                               typename In::value_type::second_type >::type
         kruskal(const Digraph& digraph, const In& in, Out& out, nt::details::Dummy< 0 > = 0) {
        using Value = typename In::value_type::second_type;
        using IndexMap = typename Digraph::template NodeMap< int >;
        using Node = typename Digraph::Node;

        IndexMap              index(digraph);
        UnionFind< IndexMap > uf(index);
        for (typename Digraph::NodeIt it(digraph); it != INVALID; ++it) {
          uf.insert(it);
        }

        Value tree_value = 0;
        for (typename In::const_iterator it = in.begin(); it != in.end(); ++it) {
          if (uf.join(digraph.target(it->first), digraph.source(it->first))) {
            out.set(it->first, true);
            tree_value += it->second;
          } else {
            out.set(it->first, false);
          }
        }
        return tree_value;
      }

      // Kruskal for undirected graphs.

      template < typename Graph, typename In, typename Out >
      typename std::enable_if< nt::concepts::UndirectedTagIndicator< Graph >,
                               typename In::value_type::second_type >::type
         kruskal(const Graph& graph, const In& in, Out& out, nt::details::Dummy< 1 > = 1) {
        using Value = typename In::value_type::second_type;
        using IndexMap = typename Graph::template NodeMap< int >;
        using Node = typename Graph::Node;

        IndexMap              index(graph);
        UnionFind< IndexMap > uf(index);
        for (typename Graph::NodeIt it(graph); it != INVALID; ++it) {
          uf.insert(it);
        }

        Value tree_value = 0;
        for (typename In::const_iterator it = in.begin(); it != in.end(); ++it) {
          if (uf.join(graph.u(it->first), graph.v(it->first))) {
            out.set(it->first, true);
            tree_value += it->second;
          } else {
            out.set(it->first, false);
          }
        }
        return tree_value;
      }

      template < typename Sequence >
      struct PairComp {
        using Value = typename Sequence::value_type;
        bool operator()(const Value& left, const Value& right) { return left.second < right.second; }
      };

      template < typename T >
      concept SequenceInputIndicator = requires {
        typename T::value_type::first_type;
      };

      template < typename T >
      concept MapInputIndicator = requires {
        typename T::Value;
      };

      template < typename T >
      concept SequenceOutputIndicator = requires {
        typename T::value_type;
      };

      template < typename T >
      concept MapOutputIndicator = requires {
        typename T::Value;
      };

      template < typename In, typename InEnable = void >
      struct KruskalValueSelector {};

      template < typename In >
      struct KruskalValueSelector< In, typename std::enable_if< SequenceInputIndicator< In >, void >::type > {
        using Value = typename In::value_type::second_type;
      };

      template < typename In >
      struct KruskalValueSelector< In, typename std::enable_if< MapInputIndicator< In >, void >::type > {
        using Value = typename In::Value;
      };

      template < typename Graph, typename In, typename Out, typename InEnable = void >
      struct KruskalInputSelector {};

      template < typename Graph, typename In, typename Out, typename InEnable = void >
      struct KruskalOutputSelector {};

      template < typename Graph, typename In, typename Out >
      struct KruskalInputSelector< Graph,
                                   In,
                                   Out,
                                   typename std::enable_if< SequenceInputIndicator< In >, void >::type > {
        using Value = typename In::value_type::second_type;

        static Value kruskal(const Graph& graph, const In& in, Out& out) {
          return KruskalOutputSelector< Graph, In, Out >::kruskal(graph, in, out);
        }
      };

      template < typename Graph, typename In, typename Out >
      struct KruskalInputSelector< Graph, In, Out, typename std::enable_if< MapInputIndicator< In >, void >::type > {
        using Value = typename In::Value;
        static Value kruskal(const Graph& graph, const In& in, Out& out) {
          using MapArc = typename In::Key;
          using Value = typename In::Value;
          using MapArcIt = typename ItemSetTraits< Graph, MapArc >::ItemIt;
          using Sequence = nt::TrivialDynamicArray< nt::Pair< MapArc, Value > >;
          Sequence seq;

          for (MapArcIt it(graph); it != INVALID; ++it) {
            seq.push_back({it, in[it]});
          }

          std::sort(seq.begin(), seq.end(), PairComp< Sequence >());
          return KruskalOutputSelector< Graph, Sequence, Out >::kruskal(graph, seq, out);
        }
      };

      template < typename T >
      struct RemoveConst {
        using type = T;
      };

      template < typename T >
      struct RemoveConst< const T > {
        using type = T;
      };

      template < typename Graph, typename In, typename Out >
      struct KruskalOutputSelector< Graph,
                                    In,
                                    Out,
                                    typename std::enable_if< SequenceOutputIndicator< Out >, void >::type > {
        using Value = typename In::value_type::second_type;

        static Value kruskal(const Graph& graph, const In& in, Out& out) {
          using Map = LoggerBoolMap< typename RemoveConst< Out >::type >;
          Map map(out);
          return _kruskal_bits::kruskal(graph, in, map);
        }
      };

      template < typename Graph, typename In, typename Out >
      struct KruskalOutputSelector< Graph, In, Out, typename std::enable_if< MapOutputIndicator< Out >, void >::type > {
        using Value = typename In::value_type::second_type;

        static Value kruskal(const Graph& graph, const In& in, Out& out) {
          return _kruskal_bits::kruskal(graph, in, out);
        }
      };

    }   // namespace _kruskal_bits

    /**
     * @ingroup spantree
     *
     * @brief Kruskal's algorithm for finding a minimum cost spanning tree of
     * a graph.
     *
     * This function runs Kruskal's algorithm to find a minimum cost
     * spanning tree of a graph.
     * Due to some C++ hacking, it accepts various input and output types.
     *
     * @param g The graph the algorithm runs on.
     * It can be either @ref concepts::Digraph "directed" or
     * @ref concepts::Graph "undirected".
     * If the graph is directed, the algorithm consider it to be
     * undirected by disregarding the direction of the arcs.
     *
     * @param in This object is used to describe the arc/edge costs.
     * It can be one of the following choices.
     * - An STL compatible 'Forward Container' with
     * <tt>std::pair<GR::Arc,C></tt> or
     * <tt>std::pair<GR::Edge,C></tt> as its <tt>value_type</tt>, where
     * \c C is the type of the costs. The pairs indicates the arcs/edges
     * along with the assigned cost. <em>They must be in a
     * cost-ascending order.</em>
     * - Any readable arc/edge map. The values of the map indicate the
     * arc/edge costs.
     *
     * \retval out Here we also have a choice.
     * - It can be a writable arc/edge map with \c bool value type. After
     * running the algorithm it will contain the found minimum cost spanning
     * tree: the value of an arc/edge will be set to \c true if it belongs
     * to the tree, otherwise it will be set to \c false. The value of
     * each arc/edge will be set exactly once.
     * - It can also be an iteraror of an STL Container with
     * <tt>GR::Arc</tt> or <tt>GR::Edge</tt> as its
     * <tt>value_type</tt>.  The algorithm copies the elements of the
     * found tree into this sequence.  For example, if we know that the
     * spanning tree of the graph \c g has say 53 arcs, then we can
     * put its arcs into an STL vector \c tree with a code like this.
     * @code
     * nt::TrivialDynamicArray<Arc> tree(53);
     * kruskal(g,cost,tree.begin());
     * @endcode
     * Or if we don't know in advance the size of the tree, we can
     * write this.
     * @code
     * nt::TrivialDynamicArray<Arc> tree;
     * kruskal(g,cost,std::back_inserter(tree));
     * @endcode
     *
     * @return The total cost of the found spanning tree.
     *
     * @note If the input graph is not (weakly) connected, a spanning
     * forest is calculated instead of a spanning tree.
     */

#ifdef DOXYGEN
    template < typename Graph, typename In, typename Out >
    Value kruskal(const Graph& g, const In& in, Out& out)
#else
    template < class Graph, class In, class Out >
    inline typename _kruskal_bits::KruskalValueSelector< In >::Value kruskal(const Graph& graph, const In& in, Out& out)
#endif
    {
      return _kruskal_bits::KruskalInputSelector< Graph, In, Out >::kruskal(graph, in, out);
    }

    template < class Graph, class In, class Out >
    inline typename _kruskal_bits::KruskalValueSelector< In >::Value
       kruskal(const Graph& graph, const In& in, const Out& out) {
      return _kruskal_bits::KruskalInputSelector< Graph, In, const Out >::kruskal(graph, in, out);
    }

  }   // namespace graphs
}   // namespace nt

#endif   // NT_KRUSKAL_H
