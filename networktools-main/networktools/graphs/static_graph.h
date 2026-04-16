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
 * This file incorporates work from the LEMON graph library (static_graph.h).
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
 *   - Changed LEMON_ASSERT to NT_ASSERT
 *   - Adapted LEMON concept checking to networktools
 *   - Converted typedef declarations to C++11 using declarations
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Updated header guard macros
 */

/**
 * @ingroup graphs
 * @file
 * @brief StaticDigraph class.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_STATIC_GRAPH_H_
#define _NT_STATIC_GRAPH_H_

#include "../core/common.h"
#include "../core/maps.h"
#include "bits/graph_extender.h"

namespace nt {
  namespace graphs {

    struct StaticDigraphBase {
      StaticDigraphBase() : node_num(0), arc_num(0) {}

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
        constexpr Arc(int _id) : id(_id) {}

        Arc() = default;
        constexpr Arc(Invalid) : id(-1) {}
        constexpr bool operator==(const Arc& arc) const noexcept { return id == arc.id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return id != arc.id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return id < arc.id; }
      };

      constexpr Node source(const Arc& e) const noexcept { return Node(arc_source[e.id]); }
      constexpr Node target(const Arc& e) const noexcept { return Node(arc_target[e.id]); }

      constexpr void        first(Node& n) const noexcept { n.id = node_num - 1; }
      constexpr static void next(Node& n) noexcept { --n.id; }

      constexpr void        first(Arc& e) const noexcept { e.id = arc_num - 1; }
      constexpr static void next(Arc& e) noexcept { --e.id; }

      constexpr void firstOut(Arc& e, const Node& n) const noexcept {
        e.id = node_first_out[n.id] != node_first_out[n.id + 1] ? node_first_out[n.id] : -1;
      }
      constexpr void nextOut(Arc& e) const noexcept { e.id = arc_next_out[e.id]; }

      constexpr void firstIn(Arc& e, const Node& n) const noexcept { e.id = node_first_in[n.id]; }
      constexpr void nextIn(Arc& e) const noexcept { e.id = arc_next_in[e.id]; }

      constexpr static int  id(const Node& n) noexcept { return n.id; }
      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr int         maxNodeId() const { return node_num - 1; }

      constexpr static int id(const Arc& e) noexcept { return e.id; }
      constexpr static Arc arcFromId(int id) noexcept { return Arc(id); }
      constexpr int        maxArcId() const noexcept { return arc_num - 1; }

      using NodeNumTag = True;
      using ArcNumTag = True;

      constexpr int nodeNum() const noexcept { return node_first_out.size(); }
      constexpr int arcNum() const noexcept { return arc_num; }

      private:
      template < typename Digraph, typename NodeRefMap >
      class ArcLess {
        public:
        using Arc = typename Digraph::Arc;

        constexpr ArcLess(const Digraph& _graph, const NodeRefMap& _nodeRef) : digraph(_graph), nodeRef(_nodeRef) {}

        constexpr bool operator()(const Arc& left, const Arc& right) const noexcept {
          return nodeRef[digraph.target(left)] < nodeRef[digraph.target(right)];
        }

        private:
        const Digraph&    digraph;
        const NodeRefMap& nodeRef;
      };

      public:
      using BuildTag = nt::True;

      constexpr void clear() noexcept {
        node_first_out.removeAll();
        node_first_in.removeAll();
        arc_source.removeAll();
        arc_target.removeAll();
        arc_next_out.removeAll();
        arc_next_in.removeAll();
      }

      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      constexpr void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        using GArc = typename Digraph::Arc;

        node_num = digraph.nodeNum();
        arc_num = digraph.arcNum();

        node_first_out.ensureAndFill(node_num + 1);
        node_first_in.ensureAndFill(node_num);

        arc_source.ensureAndFill(arc_num);
        arc_target.ensureAndFill(arc_num);
        arc_next_out.ensureAndFill(arc_num);
        arc_next_in.ensureAndFill(arc_num);

        int node_index = 0;
        for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
          nodeRef[n] = Node(node_index);
          node_first_in[node_index] = -1;
          ++node_index;
        }

        ArcLess< Digraph, NodeRefMap > arcLess(digraph, nodeRef);

        int arc_index = 0;
        for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
          const int                   source = nodeRef[n].id;
          TrivialDynamicArray< GArc > arcs;
          for (typename Digraph::OutArcIt e(digraph, n); e != INVALID; ++e) {
            arcs.push_back(e);
          }
          if (!arcs.empty()) {
            node_first_out[source] = arc_index;
            nt::sort(arcs, arcLess);
            for (typename TrivialDynamicArray< GArc >::iterator it = arcs.begin(); it != arcs.end(); ++it) {
              const int target = nodeRef[digraph.target(*it)].id;
              arcRef[*it] = Arc(arc_index);
              arc_source[arc_index] = source;
              arc_target[arc_index] = target;
              arc_next_in[arc_index] = node_first_in[target];
              node_first_in[target] = arc_index;
              arc_next_out[arc_index] = arc_index + 1;
              ++arc_index;
            }
            arc_next_out[arc_index - 1] = -1;
          } else {
            node_first_out[source] = arc_index;
          }
        }
        node_first_out[node_num] = arc_num;
      }

      template < typename ArcListIterator >
      constexpr void build(int n, ArcListIterator first, ArcListIterator last) noexcept {
        node_num = n;
        arc_num = static_cast< int >(last - first);

        node_first_out.ensureAndFill(node_num + 1);
        node_first_in.ensureAndFill(node_num);

        arc_source.ensureAndFill(arc_num);
        arc_target.ensureAndFill(arc_num);
        arc_next_out.ensureAndFill(arc_num);
        arc_next_in.ensureAndFill(arc_num);

        for (int i = 0; i != node_num; ++i)
          node_first_in[i] = -1;

        int arc_index = 0;
        for (int i = 0; i != node_num; ++i) {
          node_first_out[i] = arc_index;
          for (; first != last && (*first).first == i; ++first) {
            int j = (*first).second;
            NT_ASSERT(j >= 0 && j < node_num, "Wrong arc list for StaticDigraph::build()");
            arc_source[arc_index] = i;
            arc_target[arc_index] = j;
            arc_next_in[arc_index] = node_first_in[j];
            node_first_in[j] = arc_index;
            arc_next_out[arc_index] = arc_index + 1;
            ++arc_index;
          }
          if (arc_index > node_first_out[i]) arc_next_out[arc_index - 1] = -1;
        }
        NT_ASSERT(first == last, "Wrong arc list for StaticDigraph::build()");
        node_first_out[node_num] = arc_num;
      }

      template < typename NeighborsArray >
      constexpr void build(int m, NeighborsArray&& neighbors) noexcept {
        node_num = neighbors.size();
        arc_num = m;

        node_first_out.ensureAndFill(node_num + 1);
        node_first_in.ensureAndFill(node_num);

        arc_source.ensureAndFill(arc_num);
        arc_target.ensureAndFill(arc_num);
        arc_next_out.ensureAndFill(arc_num);
        arc_next_in.ensureAndFill(arc_num);

        for (int i = 0; i != node_num; ++i)
          node_first_in[i] = -1;

        int arc_index = 0;
        for (int i = 0; i != node_num; ++i) {
          node_first_out[i] = arc_index;
          for (const int j: neighbors[i]) {
            NT_ASSERT(j >= 0 && j < node_num, "Wrong arc list for StaticDigraph::build()");
            NT_ASSERT(arc_index < m, "Wrong arc list for CompactDigraph::build()");
            arc_source[arc_index] = i;
            arc_target[arc_index] = j;
            arc_next_in[arc_index] = node_first_in[j];
            node_first_in[j] = arc_index;
            arc_next_out[arc_index] = arc_index + 1;
            ++arc_index;
          }
          if (arc_index > node_first_out[i]) arc_next_out[arc_index - 1] = -1;
        }
        NT_ASSERT(arc_index == m, "Wrong arc list for StaticDigraph::build()");
        node_first_out[node_num] = arc_num;
      }

      protected:
      constexpr void fastFirstOut(Arc& e, const Node& n) const noexcept { e.id = node_first_out[n.id]; }

      constexpr static void fastNextOut(Arc& e) noexcept { ++e.id; }
      constexpr void        fastLastOut(Arc& e, const Node& n) const noexcept { e.id = node_first_out[n.id + 1]; }

      protected:
      int                 node_num;
      int                 arc_num;
      nt::IntDynamicArray node_first_out;
      nt::IntDynamicArray node_first_in;
      nt::IntDynamicArray arc_source;
      nt::IntDynamicArray arc_target;
      nt::IntDynamicArray arc_next_in;
      nt::IntDynamicArray arc_next_out;
    };

    using ExtendedStaticDigraphBase = DigraphExtender< StaticDigraphBase >;

    /**
     * @ingroup graphs
     *
     * @brief A static directed graph class.
     *
     * @ref StaticDigraph is a highly efficient digraph implementation,
     * but it is fully static.
     * It stores only two \c int values for each node and only four \c int
     * values for each arc. Moreover it provides faster item iteration than
     * @ref ListDigraph and @ref SmartDigraph, especially using \c OutArcIt
     * iterators, since its arcs are stored in an appropriate order.
     * However it only provides build() and clear() functions and does not
     * support any other modification of the digraph.
     *
     * Since this digraph structure is completely static, its nodes and arcs
     * can be indexed with integers from the ranges <tt>[0..nodeNum()-1]</tt>
     * and <tt>[0..arcNum()-1]</tt>, respectively.
     * The index of an item is the same as its ID, it can be obtained
     * using the corresponding @ref index() or @ref concepts::Digraph::id()
     * "id()" function. A node or arc with a certain index can be obtained
     * using node() or arc().
     *
     * This type fully conforms to the @ref concepts::Digraph "Digraph concept".
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides constant time counting for nodes and arcs.
     *
     * \sa concepts::Digraph
     */
    class StaticDigraph: public ExtendedStaticDigraphBase {
      public:
      using Parent = ExtendedStaticDigraphBase;

      public:
      /**
       * @brief Constructor
       *
       * Default constructor.
       */
      StaticDigraph() : Parent() {}

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
      constexpr static Arc arc(int ix) noexcept { return Parent::arcFromId(ix); }

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
       * @brief Number of nodes.
       *
       * This function returns the number of nodes.
       */
      constexpr int nodeNum() const noexcept { return node_num; }

      /**
       * @brief Number of arcs.
       *
       * This function returns the number of arcs.
       */
      constexpr int arcNum() const noexcept { return arc_num; }

      /**
       * @brief Build the digraph copying another digraph.
       *
       * This function builds the digraph copying another digraph of any
       * kind. It can be called more than once, but in such case, the whole
       * structure and all maps will be cleared and rebuilt.
       *
       * This method also makes possible to copy a digraph to a StaticDigraph
       * structure using @ref DigraphCopy.
       *
       * @param digraph An existing digraph to be copied.
       * @param node_ref The node references will be copied into this map.
       * Its key type must be \c Digraph::Node and its value type must be
       * \c StaticDigraph::Node.
       * It must conform to the @ref concepts::ReadWriteMap "ReadWriteMap"
       * concept.
       * @param arc_ref The arc references will be copied into this map.
       * Its key type must be \c Digraph::Arc and its value type must be
       * \c StaticDigraph::Arc.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       *
       * @note If you do not need the arc references, then you could use
       * @ref NullMap for the last parameter. However the node references
       * are required by the function itself, thus they must be readable
       * from the map.
       */
      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      constexpr void build(const Digraph& digraph, NodeRefMap& node_ref, ArcRefMap& arc_ref) noexcept {
        Parent::clear();
        Parent::build(digraph, node_ref, arc_ref);
      }

      /**
       * @brief Build the digraph copying another digraph.
       *
       * This function builds the digraph copying another digraph of any
       * kind. It can be called more than once, but in such case, the whole
       * structure and all maps will be cleared and rebuilt.
       *
       * This method also makes possible to copy a digraph to a StaticDigraph
       * structure using @ref DigraphCopy.
       *
       * @param digraph An existing digraph to be copied.
       *
       * @note This version of the function creates internal node and arc
       * reference maps automatically.
       *
       */
      template < typename Digraph >
      constexpr void build(const Digraph& digraph) noexcept {
        using StaticNodeRefMap =
           details::StaticMapBase< Digraph, typename Digraph::Node, nt::DynamicArray< StaticDigraph::Node > >;
        using StaticArcRefMap = nt::NullMap< typename Digraph::Arc, StaticDigraph::Arc >;

        StaticNodeRefMap node_ref(digraph);
        StaticArcRefMap  arc_ref;
        build(digraph, node_ref, arc_ref);
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
       * StaticDigraph gr;
       * gr.build(4, arcs.begin(), arcs.end());
       * @endcode
       */
      template < typename ArcListIterator >
      constexpr void build(int n, ArcListIterator begin, ArcListIterator end) noexcept {
        Parent::clear();
        StaticDigraphBase::build(n, begin, end);
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
       * StaticDigraph gr;
       * gr.build(5, arcs);
       * @endcode
       *
       */
      template < typename NeighborsArray >
      constexpr void build(int m, NeighborsArray&& neighbors) noexcept {
        Parent::clear();
        StaticDigraphBase::build< NeighborsArray >(m, std::forward< NeighborsArray >(neighbors));
        notifier(Node()).build();
        notifier(Arc()).build();
      }

      /**
       * @brief Clear the digraph.
       *
       * This function erases all nodes and arcs from the digraph.
       */
      constexpr void clear() noexcept { Parent::clear(); }

      protected:
      using Parent::fastFirstOut;
      using Parent::fastLastOut;
      using Parent::fastNextOut;

      public:
      class OutArcIt: public Arc {
        public:
        OutArcIt() = default;

        constexpr OutArcIt(Invalid i) : Arc(i) {}

        constexpr OutArcIt(const StaticDigraph& digraph, const Node& node) {
          digraph.fastFirstOut(*this, node);
          digraph.fastLastOut(last, node);
          if (last == *this) *this = INVALID;
        }

        constexpr OutArcIt(const StaticDigraph& digraph, const Arc& arc) : Arc(arc) {
          if (arc != INVALID) { digraph.fastLastOut(last, digraph.source(arc)); }
        }

        constexpr OutArcIt& operator++() noexcept {
          StaticDigraph::fastNextOut(*this);
          if (last == *this) *this = INVALID;
          return *this;
        }

        private:
        Arc last;
      };

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
