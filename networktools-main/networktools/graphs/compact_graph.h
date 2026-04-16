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
 * This file incorporates work from the LEMON graph library (compact_graph.h).
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
 *   - Replaced raw arrays with nt::IntDynamicArray
 *   - Replaced LEMON_ASSERT with NT_ASSERT
 *   - Changed namespace from lemon to nt
 *   - Adapted to use nt::graphs::countNodes() and nt::graphs::countArcs()
 */

/**
 * @ingroup graphs
 * @file
 * @brief CompactDigraph class.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_COMPACT_GRAPH_H_
#define _NT_COMPACT_GRAPH_H_

#include "tools.h"
#include "bits/graph_extender.h"
#include "../core/common.h"

namespace nt {

  namespace graphs {

    class CompactDigraphBase {
      public:
      struct Node {
        int id;
        constexpr Node(int _id) : id(_id) {}

        Node() = default;
        constexpr Node(Invalid) : id(-1) {}
        constexpr bool operator==(const Node& node) const noexcept { return id == node.id; }
        constexpr bool operator!=(const Node& node) const noexcept { return id != node.id; }
        constexpr bool operator<(const Node& node) const noexcept { return id < node.id; }
      };

      struct Arc {
        int id;
        int source;
        constexpr Arc(int _id, int _source) : id(_id), source(_source) {}

        Arc() = default;
        constexpr Arc(Invalid) : id(-1), source(-1) {}
        constexpr bool operator==(const Arc& arc) const noexcept { return id == arc.id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return id != arc.id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return id < arc.id; }
      };

      constexpr Node source(const Arc& e) const noexcept { return Node(e.source); }
      constexpr Node target(const Arc& e) const noexcept { return Node(arc_target[e.id]); }

      constexpr void        first(Node& n) const noexcept { n.id = nodeNum() - 1; }
      constexpr static void next(Node& n) noexcept { --n.id; }

      private:
      constexpr void nextSource(Arc& e) const noexcept {
        if (e.id == -1) return;
        int last = node_first_out[e.source] - 1;
        while (e.id == last) {
          --e.source;
          last = node_first_out[e.source] - 1;
        }
      }

      public:
      constexpr void first(Arc& e) const noexcept {
        e.id = arcNum() - 1;
        e.source = nodeNum() - 1;
        nextSource(e);
      }
      constexpr void next(Arc& e) const noexcept {
        --e.id;
        nextSource(e);
      }

      constexpr void firstOut(Arc& e, const Node& n) const noexcept {
        e.source = n.id;
        e.id = node_first_out[n.id];
        if (e.id == node_first_out[n.id + 1]) e = INVALID;
      }
      constexpr void nextOut(Arc& e) const noexcept {
        ++e.id;
        if (e.id == node_first_out[e.source + 1]) e = INVALID;
      }

      constexpr void firstIn(Arc& e, const Node& n) const noexcept {
        first(e);
        while (e != INVALID && target(e) != n) {
          next(e);
        }
      }
      constexpr void nextIn(Arc& e) const noexcept {
        Node arcTarget = target(e);
        do {
          next(e);
        } while (e != INVALID && target(e) != arcTarget);
      }

      constexpr static int  id(const Node& n) noexcept { return n.id; }
      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr int         maxNodeId() const noexcept { return nodeNum() - 1; }

      constexpr static int id(const Arc& e) noexcept { return e.id; }
      constexpr Arc        arcFromId(int id) const noexcept {
               const int* l = std::upper_bound(node_first_out.data(), node_first_out.data() + nodeNum(), id) - 1;
               int        src = l - node_first_out.data();
               return Arc(id, src);
      }
      constexpr int maxArcId() const noexcept { return arcNum() - 1; }

      using NodeNumTag = True;
      using ArcNumTag = True;

      constexpr int nodeNum() const noexcept {
        if (node_first_out.empty()) [[unlikely]]
          return 0;
        return node_first_out.size() - 1;
      }
      constexpr int arcNum() const noexcept { return arc_target.size(); }

      private:
      template < typename Digraph, typename NodeRefMap >
      class ArcLess {
        public:
        using Arc = typename Digraph::Arc;

        ArcLess(const Digraph& _graph, const NodeRefMap& _nodeRef) : digraph(_graph), nodeRef(_nodeRef) {}

        bool operator()(const Arc& left, const Arc& right) const {
          return nodeRef[digraph.target(left)] < nodeRef[digraph.target(right)];
        }

        private:
        const Digraph&    digraph;
        const NodeRefMap& nodeRef;
      };

      public:
      using BuildTag = True;

      constexpr void clear() noexcept {
        node_first_out.removeAll();
        arc_target.removeAll();
      }

      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      constexpr void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        using GNode = typename Digraph::Node;
        using GArc = typename Digraph::Arc;

        const int node_num = nt::graphs::countNodes(digraph);
        const int arc_num = nt::graphs::countArcs(digraph);

        node_first_out.ensureAndFill(node_num + 1);
        arc_target.ensureAndFill(arc_num);

        int node_index = 0;
        for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
          nodeRef[n] = Node(node_index);
          ++node_index;
        }

        ArcLess< Digraph, NodeRefMap > arcLess(digraph, nodeRef);

        int                         arc_index = 0;
        TrivialDynamicArray< GArc > arcs;
        for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
          int source = nodeRef[n].id;
          arcs.removeAll();
          for (typename Digraph::OutArcIt e(digraph, n); e != INVALID; ++e) {
            arcs.push_back(e);
          }
          if (!arcs.empty()) {
            node_first_out[source] = arc_index;
            nt::sort(arcs, arcLess);
            for (typename TrivialDynamicArray< GArc >::iterator it = arcs.begin(); it != arcs.end(); ++it) {
              const int target = nodeRef[digraph.target(*it)].id;
              arcRef[*it] = Arc(arc_index, source);
              arc_target[arc_index] = target;
              ++arc_index;
            }
          } else {
            node_first_out[source] = arc_index;
          }
        }
        node_first_out[node_num] = arc_num;
      }

      template < typename ArcListIterator >
      constexpr void build(int n, ArcListIterator first, ArcListIterator last) noexcept {
        const int arc_num = static_cast< int >(std::distance(first, last));

        node_first_out.ensureAndFill(n + 1);
        arc_target.ensureAndFill(arc_num);

        int arc_index = 0;
        for (int i = 0; i != n; ++i) {
          node_first_out[i] = arc_index;
          for (; first != last && (*first).first == i; ++first) {
            int j = (*first).second;
            NT_ASSERT(j >= 0 && j < n, "Wrong arc list for CompactDigraph::build()");
            arc_target[arc_index] = j;
            ++arc_index;
          }
        }
        NT_ASSERT(first == last, "Wrong arc list for CompactDigraph::build()");
        node_first_out[n] = arc_num;
      }

      template < typename NeighborsArray >
      constexpr void build(int m, NeighborsArray&& neighbors) noexcept {
        node_first_out.ensureAndFill(neighbors.size() + 1);
        arc_target.ensureAndFill(m);
        const int n = neighbors.size();

        int arc_index = 0;
        for (int i = 0; i != n; ++i) {
          node_first_out[i] = arc_index;
          for (const int j: neighbors[i]) {
            NT_ASSERT(j >= 0 && j < n, "Wrong arc list for CompactDigraph::build()");
            NT_ASSERT(arc_index < m, "Wrong arc list for CompactDigraph::build()");
            arc_target[arc_index] = j;
            ++arc_index;
          }
        }
        NT_ASSERT(arc_index == m, "Wrong arc list for CompactDigraph::build()");
        node_first_out[n] = m;
      }

      protected:
      IntDynamicArray node_first_out;
      IntDynamicArray arc_target;
    };

    using ExtendedCompactDigraphBase = DigraphExtender< CompactDigraphBase >;

    /**
     * @ingroup graphs
     *
     * @brief A static directed graph class.
     *
     * @ref CompactDigraph is a highly efficient digraph implementation
     * similar to @ref StaticDigraph. It is more memory efficient but does
     * not provide efficient iteration over incoming arcs.
     *
     * It stores only one \c int values for each node and one \c int value
     * for each arc. Its @ref InArcIt implementation is inefficient and
     * provided only for compatibility with the @ref concepts::Digraph "Digraph
     * concept".
     *
     * This type fully conforms to the @ref concepts::Digraph "Digraph concept".
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * \sa concepts::Digraph
     */
    class CompactDigraph: public ExtendedCompactDigraphBase {
      private:
      /**
       * Graphs are \e not copy constructible. Use DigraphCopy instead.
       */
      CompactDigraph(const CompactDigraph&) : ExtendedCompactDigraphBase(){};
      /**
       * @brief Assignment of a graph to another one is \e not allowed.
       * Use DigraphCopy instead.
       */
      constexpr void operator=(const CompactDigraph&) noexcept {}

      public:
      using Parent = ExtendedCompactDigraphBase;

      public:
      /**
       * @brief Constructor
       *
       * Default constructor.
       */
      CompactDigraph() : Parent() {}

      /**
       * @brief The node with the given index.
       *
       * This function returns the node with the given index.
       * \sa index()
       */
      constexpr static Node node(int ix) noexcept { return Parent::nodeFromId(ix); }

      /**
       * @brief The arc with the given index.
       *
       * This function returns the arc with the given index.
       * \sa index()
       */
      constexpr Arc arc(int ix) noexcept { return arcFromId(ix); }

      /**
       * @brief The index of the given node.
       *
       * This function returns the index of the the given node.
       * \sa node()
       */
      constexpr static int index(Node node) noexcept { return Parent::id(node); }

      /**
       * @brief The index of the given arc.
       *
       * This function returns the index of the the given arc.
       * \sa arc()
       */
      constexpr static int index(Arc arc) noexcept { return Parent::id(arc); }

      /**
       * @brief Build the digraph copying another digraph.
       *
       * This function builds the digraph copying another digraph of any
       * kind. It can be called more than once, but in such case, the whole
       * structure and all maps will be cleared and rebuilt.
       *
       * This method also makes possible to copy a digraph to a CompactDigraph
       * structure using @ref DigraphCopy.
       *
       * @param digraph An existing digraph to be copied.
       * @param nodeRef The node references will be copied into this map.
       * Its key type must be \c Digraph::Node and its value type must be
       * \c CompactDigraph::Node.
       * It must conform to the @ref concepts::ReadWriteMap "ReadWriteMap"
       * concept.
       * @param arcRef The arc references will be copied into this map.
       * Its key type must be \c Digraph::Arc and its value type must be
       * \c CompactDigraph::Arc.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       *
       * @note If you do not need the arc references, then you could use
       * @ref NullMap for the last parameter. However the node references
       * are required by the function itself, thus they must be readable
       * from the map.
       */
      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      constexpr void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        Parent::clear();
        Parent::build(digraph, nodeRef, arcRef);
      }

      /**
       * @brief Build the digraph from an arc list.
       *
       * This function builds the digraph from the given arc list.
       * It can be called more than once, but in such case, the whole
       * structure and all maps will be cleared and rebuilt.
       *
       * The list of the arcs must be given in the range <tt>[begin, end)</tt>
       * specified by STL compatible itartors whose \c value_type must be
       * <tt>std::pair<int,int></tt>.
       * Each arc must be specified by a pair of integer indices
       * from the range <tt>[0..n-1]</tt>. <i>The pairs must be in a
       * non-decreasing order with respect to their first values.</i>
       * If the k-th pair in the list is <tt>(i,j)</tt>, then
       * <tt>arc(k-1)</tt> will connect <tt>node(i)</tt> to <tt>node(j)</tt>.
       *
       * @param n The number of nodes.
       * @param begin An iterator pointing to the beginning of the arc list.
       * @param end An iterator pointing to the end of the arc list.
       *
       * For example, a simple digraph can be constructed like this.
       * @code
       * nt::TrivialDynamicArray<nt::Pair<int,int> > arcs;
       * arcs.push_back({0,1});
       * arcs.push_back({0,2});
       * arcs.push_back({1,3});
       * arcs.push_back({1,2});
       * arcs.push_back({3,0});
       * CompactDigraph gr;
       * gr.build(4, arcs.begin(), arcs.end());
       * @endcode
       */
      template < typename ArcListIterator >
      constexpr void build(int n, ArcListIterator begin, ArcListIterator end) noexcept {
        Parent::clear();
        CompactDigraphBase::build(n, begin, end);
        notifier(Node()).build();
        notifier(Arc()).build();
      }

      /**
       * @brief Build the digraph from a list of neighbors
       *
       * @tparam NeighborsArray
       * @param m
       * @param neighbors
       *
       * For example, a simple digraph can be constructed like this.
       * @code
       * nt::TrivialDynamicArray< nt::IntDynamicArray > arcs(4);
       * arcs[0].add({1,2});
       * arcs[1].add({3,2});
       * arcs[3].add({0});
       * CompactDigraph gr;
       * gr.build(5, arcs);
       * @endcode
       *
       */
      template < typename NeighborsArray >
      constexpr void build(int m, NeighborsArray&& neighbors) noexcept {
        Parent::clear();
        CompactDigraphBase::build< NeighborsArray >(m, std::forward< NeighborsArray >(neighbors));
        notifier(Node()).build();
        notifier(Arc()).build();
      }

      /**
       * @brief Clear the digraph.
       *
       * This function erases all nodes and arcs from the digraph.
       */
      constexpr void clear() noexcept { Parent::clear(); }

      public:
      constexpr Node baseNode(const OutArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }

      constexpr Node runningNode(const OutArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }

      constexpr Node baseNode(const InArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }

      constexpr Node runningNode(const InArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }
    };

  }   // namespace graphs
}   // namespace nt
#endif
