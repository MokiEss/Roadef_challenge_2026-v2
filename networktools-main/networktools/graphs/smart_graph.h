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
 * This file incorporates work from the LEMON graph library (smart_graph.h).
 *
 * Original LEMON Copyright Notice:
 * Copyright (C) 2003-2009 Egervary Jeno Kombinatorikus Optimalizalasi
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
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Added move semantics for performance
 *   - Updated header guard macros
 *   - Variable renaming to follow networktools code convention
 */

/**
 * @ingroup graphs
 * @file
 * @brief SmartDigraph and SmartGraph classes.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SMART_GRAPH_H_
#define _NT_SMART_GRAPH_H_

#include "../core/common.h"
#include "bits/graph_extender.h"

namespace nt {
  namespace graphs {

    struct SmartDigraph;

    struct SmartDigraphBase {
      using Digraph = SmartDigraphBase;
      using NodeNumTag = nt::True;
      using ArcNumTag = nt::True;

      struct NodeT {
        int first_in, first_out;
      };

      struct ArcT {
        int target, source, next_in, next_out;
      };

      struct Node {
        int _id;

        Node() = default;
        constexpr explicit Node(int id) : _id(id) {}
        constexpr Node(Invalid) : _id(-1) {}

        constexpr bool operator==(const Node i) const noexcept { return _id == i._id; }
        constexpr bool operator!=(const Node i) const noexcept { return _id != i._id; }
        constexpr bool operator<(const Node i) const noexcept { return _id < i._id; }
      };

      struct Arc {
        int _id;

        Arc() = default;
        constexpr explicit Arc(int id) : _id(id) {}
        constexpr Arc(Invalid) : _id(-1) {}

        constexpr bool operator==(const Arc i) const noexcept { return _id == i._id; }
        constexpr bool operator!=(const Arc i) const noexcept { return _id != i._id; }
        constexpr bool operator<(const Arc i) const noexcept { return _id < i._id; }
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      nt::TrivialDynamicArray< ArcT >  _arcs;

      SmartDigraphBase() = default;
      SmartDigraphBase(SmartDigraphBase&& g) : _nodes(std::move(g._nodes)), _arcs(std::move(g._arcs)) {}

      constexpr int nodeNum() const noexcept { return _nodes.size(); }
      constexpr int arcNum() const noexcept { return _arcs.size(); }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr Node addNode() noexcept {
        const int n = _nodes.add();
        _nodes[n].first_in = -1;
        _nodes[n].first_out = -1;
        return Node(n);
      }

      constexpr Arc addArc(Node u, Node v) noexcept {
        const int n = _arcs.add();
        ArcT&     new_arc = _arcs[n];
        new_arc.source = u._id;
        new_arc.target = v._id;
        new_arc.next_out = _nodes[u._id].first_out;
        new_arc.next_in = _nodes[v._id].first_in;
        _nodes[u._id].first_out = _nodes[v._id].first_in = n;

        return Arc(n);
      }

      constexpr void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
      }

      constexpr Node source(Arc a) const noexcept { return Node(_arcs[a._id].source); }
      constexpr Node target(Arc a) const noexcept { return Node(_arcs[a._id].target); }

      constexpr static int id(Node v) noexcept { return v._id; }
      constexpr static int id(Arc a) noexcept { return a._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }

      constexpr bool valid(Node n) const noexcept { return n._id >= 0 && n._id < static_cast< int >(_nodes.size()); }
      constexpr bool valid(Arc a) const noexcept { return a._id >= 0 && a._id < static_cast< int >(_arcs.size()); }

      constexpr void        first(Node& node) const noexcept { node._id = _nodes.size() - 1; }
      constexpr static void next(Node& node) noexcept { --node._id; }

      constexpr void        first(Arc& arc) const noexcept { arc._id = _arcs.size() - 1; }
      constexpr static void next(Arc& arc) noexcept { --arc._id; }

      constexpr void firstOut(Arc& arc, const Node& node) const noexcept { arc._id = _nodes[node._id].first_out; }
      constexpr void nextOut(Arc& arc) const noexcept { arc._id = _arcs[arc._id].next_out; }

      constexpr void firstIn(Arc& arc, const Node& node) const noexcept { arc._id = _nodes[node._id].first_in; }
      constexpr void nextIn(Arc& arc) const noexcept { arc._id = _arcs[arc._id].next_in; }
    };

    using ExtendedSmartDigraphBase = DigraphExtender< SmartDigraphBase >;

    /**
     * @ingroup graphs
     *
     * @brief A smart directed graph class.
     *
     * @ref SmartDigraph is a simple and fast digraph implementation.
     * It is also quite memory efficient but at the price
     * that it does not support node and arc deletion
     * (except for the Snapshot feature).
     *
     * This type fully conforms to the @ref concepts::Digraph "Digraph concept"
     * and it also provides some additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides constant time counting for nodes and arcs.
     *
     * \sa concepts::Digraph
     * \sa SmartGraph
     */
    struct SmartDigraph: public ExtendedSmartDigraphBase {
      using Parent = ExtendedSmartDigraphBase;

      /**
       * Digraphs are \e not copy constructible. Use DigraphCopy instead.
       */
      SmartDigraph(const SmartDigraph&) = delete;
      /**
       * @brief Assignment of a digraph to another one is \e not allowed.
       * Use DigraphCopy instead.
       */
      void operator=(const SmartDigraph&) = delete;

      /** Constructor */

      /**
       * Constructor.
       *
       */
      SmartDigraph() = default;

      /** Add a new node to the digraph. */

      /**
       * This function adds a new node to the digraph.
       * @return The new node.
       */
      Node addNode() { return Parent::addNode(); }

      /** Add a new arc to the digraph. */

      /**
       * This function adds a new arc to the digraph with source node \c s
       * and target node \c t.
       * @return The new arc.
       */
      Arc addArc(Node s, Node t) { return Parent::addArc(s, t); }

      /**
       * @brief Node validity check
       *
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the digraph.
       *
       * @warning A removed node (using Snapshot) could become valid again
       * if new nodes are added to the digraph.
       */
      bool valid(Node n) const { return Parent::valid(n); }

      /**
       * @brief Arc validity check
       *
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the digraph.
       *
       * @warning A removed arc (using Snapshot) could become valid again
       * if new arcs are added to the graph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /** Split a node. */

      /**
       * This function splits the given node. First, a new node is added
       * to the digraph, then the source of each outgoing arc of node \c n
       * is moved to this new node.
       * If the second parameter \c connect is \c true (this is the default
       * value), then a new arc from node \c n to the newly created node
       * is also added.
       * @return The newly created node.
       *
       * @note All iterators remain valid.
       *
       * @warning This functionality cannot be used together with the Snapshot
       * feature.
       */
      Node split(Node n, bool connect = true) {
        Node b = addNode();
        _nodes[b._id].first_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = -1;
        for (int i = _nodes[b._id].first_out; i != -1; i = _arcs[i].next_out) {
          _arcs[i].source = b._id;
        }
        if (connect) addArc(n, b);
        return b;
      }

      /** Clear the digraph. */

      /**
       * This function erases all nodes and arcs from the digraph.
       *
       */
      void clear() { Parent::clear(); }

      /** Reserve memory for nodes. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the digraph you want to build will
       * be large (e.g. it will contain millions of nodes and/or arcs),
       * then it is worth reserving space for this amount before starting
       * to build the digraph.
       * \sa reserveArc()
       */
      void reserveNode(int n) { _nodes.reserve(n); };

      /** Reserve memory for arcs. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the digraph you want to build will
       * be large (e.g. it will contain millions of nodes and/or arcs),
       * then it is worth reserving space for this amount before starting
       * to build the digraph.
       * \sa reserveNode()
       */
      void reserveArc(int m) { _arcs.reserve(m); };

      /**
       * Class to make a snapshot of the digraph and to restore it later.
       */

      /**
       * Class to make a snapshot of the digraph and to restore it later.
       *
       * The newly added nodes and arcs can be removed using the
       * restore() function. This is the only way for deleting nodes and/or
       * arcs from a SmartDigraph structure.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and arcs again using
       * another Snapshot instance.
       *
       * @warning Node splitting cannot be restored.
       * @warning The validity of the snapshot is not stored due to
       * performance reasons. If you do not use the snapshot correctly,
       * it can cause broken program, invalid or not restored state of
       * the digraph or no change.
       */
      struct Snapshot {
        SmartDigraph* _graph;

        unsigned int _node_num;
        unsigned int _arc_num;

        /** Default constructor. */

        /**
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : _graph(nullptr) {}
        /** Constructor that immediately makes a snapshot */

        /**
         * This constructor immediately makes a snapshot of the given digraph.
         *
         */
        Snapshot(SmartDigraph& gr) : _graph(&gr) {
          _node_num = _graph->_nodes.size();
          _arc_num = _graph->_arcs.size();
        }

        /** Make a snapshot. */

        /**
         * This function makes a snapshot of the given digraph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        void save(SmartDigraph& gr) {
          _graph = &gr;
          _node_num = _graph->_nodes.size();
          _arc_num = _graph->_arcs.size();
        }

        /** Undo the changes until a snapshot. */

        /**
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(SmartDigraph&).
         */
        void restore() { _graph->_restoreSnapshot(*this); }
      };

      void _restoreSnapshot(const Snapshot& s) {
        while (s._arc_num < _arcs.size()) {
          Arc arc = arcFromId(_arcs.size() - 1);
          Parent::notifier(Arc()).erase(arc);
          _nodes[_arcs.back().source].first_out = _arcs.back().next_out;
          _nodes[_arcs.back().target].first_in = _arcs.back().next_in;
          _arcs.pop_back();
        }
        while (s._node_num < _nodes.size()) {
          Node node = nodeFromId(_nodes.size() - 1);
          Parent::notifier(Node()).erase(node);
          _nodes.pop_back();
        }
      }
    };

    struct SmartGraphBase {
      using Graph = SmartGraphBase;
      using NodeNumTag = nt::True;
      using EdgeNumTag = nt::True;
      using ArcNumTag = nt::True;

      struct NodeT {
        int first_out;
      };

      struct ArcT {
        int target;
        int next_out;
      };

      struct Node {
        int _id;

        Node() = default;
        constexpr Node(Invalid) { _id = -1; }
        constexpr explicit Node(int id) { _id = id; }

        constexpr bool operator==(const Node& node) const noexcept { return _id == node._id; }
        constexpr bool operator!=(const Node& node) const noexcept { return _id != node._id; }
        constexpr bool operator<(const Node& node) const noexcept { return _id < node._id; }
      };

      struct Edge {
        int _id;

        Edge() = default;
        constexpr Edge(Invalid) { _id = -1; }
        constexpr explicit Edge(int id) { _id = id; }

        constexpr bool operator==(const Edge& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Edge& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Edge& arc) const noexcept { return _id < arc._id; }
      };

      struct Arc {
        int _id;

        Arc() = default;
        constexpr Arc(Invalid) { _id = -1; }
        constexpr explicit Arc(int id) { _id = id; }

        constexpr operator Edge() const noexcept { return _id != -1 ? edgeFromId(_id / 2) : INVALID; }

        constexpr bool operator==(const Arc& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return _id < arc._id; }
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      nt::TrivialDynamicArray< ArcT >  _arcs;

      SmartGraphBase() = default;

      constexpr int nodeNum() const noexcept { return _nodes.size(); }
      constexpr int edgeNum() const noexcept { return _arcs.size() / 2; }
      constexpr int arcNum() const noexcept { return _arcs.size(); }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxEdgeId() const noexcept { return _arcs.size() / 2 - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr Node source(Arc e) const noexcept { return Node(_arcs[e._id ^ 1].target); }
      constexpr Node target(Arc e) const noexcept { return Node(_arcs[e._id].target); }

      constexpr Node u(Edge e) const noexcept { return Node(_arcs[2 * e._id].target); }
      constexpr Node v(Edge e) const noexcept { return Node(_arcs[2 * e._id + 1].target); }

      constexpr static bool direction(Arc e) noexcept { return (e._id & 1) == 1; }

      constexpr static Arc direct(Edge e, bool d) noexcept { return Arc(e._id * 2 + (d ? 1 : 0)); }

      constexpr void first(Node& node) const noexcept { node._id = _nodes.size() - 1; }

      constexpr static void next(Node& node) noexcept { --node._id; }

      constexpr void first(Arc& arc) const noexcept { arc._id = _arcs.size() - 1; }

      constexpr static void next(Arc& arc) noexcept { --arc._id; }

      constexpr void first(Edge& arc) const noexcept { arc._id = _arcs.size() / 2 - 1; }

      constexpr static void next(Edge& arc) noexcept { --arc._id; }

      constexpr void firstOut(Arc& arc, const Node& v) const noexcept { arc._id = _nodes[v._id].first_out; }
      constexpr void nextOut(Arc& arc) const noexcept { arc._id = _arcs[arc._id].next_out; }

      constexpr void firstIn(Arc& arc, const Node& v) const noexcept {
        arc._id = ((_nodes[v._id].first_out) ^ 1);
        if (arc._id == -2) arc._id = -1;
      }
      constexpr void nextIn(Arc& arc) const noexcept {
        arc._id = ((_arcs[arc._id ^ 1].next_out) ^ 1);
        if (arc._id == -2) arc._id = -1;
      }

      constexpr void firstInc(Edge& arc, bool& d, const Node& v) const noexcept {
        int de = _nodes[v._id].first_out;
        if (de != -1) {
          arc._id = de / 2;
          d = ((de & 1) == 1);
        } else {
          arc._id = -1;
          d = true;
        }
      }
      constexpr void nextInc(Edge& arc, bool& d) const noexcept {
        int de = (_arcs[(arc._id * 2) | (d ? 1 : 0)].next_out);
        if (de != -1) {
          arc._id = de / 2;
          d = ((de & 1) == 1);
        } else {
          arc._id = -1;
          d = true;
        }
      }

      constexpr static int id(Node v) noexcept { return v._id; }
      constexpr static int id(Arc e) noexcept { return e._id; }
      constexpr static int id(Edge e) noexcept { return e._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }
      constexpr static Edge edgeFromId(int id) noexcept { return Edge(id); }

      constexpr bool valid(Node n) const noexcept { return n._id >= 0 && n._id < static_cast< int >(_nodes.size()); }
      constexpr bool valid(Arc a) const noexcept { return a._id >= 0 && a._id < static_cast< int >(_arcs.size()); }
      constexpr bool valid(Edge e) const noexcept { return e._id >= 0 && 2 * e._id < static_cast< int >(_arcs.size()); }

      Node addNode() noexcept {
        const int n = _nodes.add();
        _nodes[n].first_out = -1;

        return Node(n);
      }

      Edge addEdge(Node u, Node v) noexcept {
        const int n = _arcs.add();
        _arcs.add();

        _arcs[n].target = u._id;
        _arcs[n | 1].target = v._id;

        _arcs[n].next_out = _nodes[v._id].first_out;
        _nodes[v._id].first_out = n;

        _arcs[n | 1].next_out = _nodes[u._id].first_out;
        _nodes[u._id].first_out = (n | 1);

        return Edge(n / 2);
      }

      void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
      }
    };

    using ExtendedSmartGraphBase = GraphExtender< SmartGraphBase >;

    /**
     * @ingroup graphs
     *
     * @brief A smart undirected graph class.
     *
     * @ref SmartGraph is a simple and fast graph implementation.
     * It is also quite memory efficient but at the price
     * that it does not support node and edge deletion
     * (except for the Snapshot feature).
     *
     * This type fully conforms to the @ref concepts::Graph "Graph concept"
     * and it also provides some additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides constant time counting for nodes, edges and arcs.
     *
     * \sa concepts::Graph
     * \sa SmartDigraph
     */
    struct SmartGraph: public ExtendedSmartGraphBase {
      using Parent = ExtendedSmartGraphBase;

      /**
       * Graphs are \e not copy constructible. Use GraphCopy instead.
       */
      SmartGraph(const SmartGraph&) = delete;
      /**
       * @brief Assignment of a graph to another one is \e not allowed.
       * Use GraphCopy instead.
       */
      void operator=(const SmartGraph&) = delete;

      /** Constructor */

      /**
       * Constructor.
       *
       */
      SmartGraph() = default;

      /**
       * @brief Add a new node to the graph.
       *
       * This function adds a new node to the graph.
       * @return The new node.
       */
      Node addNode() { return Parent::addNode(); }

      /**
       * @brief Add a new edge to the graph.
       *
       * This function adds a new edge to the graph between nodes
       * \c u and \c v with inherent orientation from node \c u to
       * node \c v.
       * @return The new edge.
       */
      Edge addEdge(Node u, Node v) { return Parent::addEdge(u, v); }

      /**
       * @brief Node validity check
       *
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the graph.
       *
       * @warning A removed node (using Snapshot) could become valid again
       * if new nodes are added to the graph.
       */
      bool valid(Node n) const { return Parent::valid(n); }

      /**
       * @brief Edge validity check
       *
       * This function gives back \c true if the given edge is valid,
       * i.e. it is a real edge of the graph.
       *
       * @warning A removed edge (using Snapshot) could become valid again
       * if new edges are added to the graph.
       */
      bool valid(Edge e) const { return Parent::valid(e); }

      /**
       * @brief Arc validity check
       *
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the graph.
       *
       * @warning A removed arc (using Snapshot) could become valid again
       * if new edges are added to the graph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /** Clear the graph. */

      /**
       * This function erases all nodes and arcs from the graph.
       *
       */
      void clear() { Parent::clear(); }

      /** Reserve memory for nodes. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the graph you want to build will
       * be large (e.g. it will contain millions of nodes and/or edges),
       * then it is worth reserving space for this amount before starting
       * to build the graph.
       * \sa reserveEdge()
       */
      void reserveNode(int n) { _nodes.reserve(n); };

      /** Reserve memory for edges. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the graph you want to build will
       * be large (e.g. it will contain millions of nodes and/or edges),
       * then it is worth reserving space for this amount before starting
       * to build the graph.
       * \sa reserveNode()
       */
      void reserveEdge(int m) { _arcs.reserve(2 * m); };


      /**
       * Class to make a snapshot of the graph and to restore it later.
       */

      /**
       * Class to make a snapshot of the graph and to restore it later.
       *
       * The newly added nodes and edges can be removed using the
       * restore() function. This is the only way for deleting nodes and/or
       * edges from a SmartGraph structure.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and edges again using
       * another Snapshot instance.
       *
       * @warning The validity of the snapshot is not stored due to
       * performance reasons. If you do not use the snapshot correctly,
       * it can cause broken program, invalid or not restored state of
       * the graph or no change.
       */
      struct Snapshot {
        SmartGraph* _graph;

        unsigned int _node_num;
        unsigned int _arc_num;

        /** Default constructor. */

        /**
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : _graph(0) {}
        /** Constructor that immediately makes a snapshot */

        /**
         * This constructor immediately makes a snapshot of the given graph.
         *
         */
        Snapshot(SmartGraph& gr) { gr._saveSnapshot(*this); }

        /** Make a snapshot. */

        /**
         * This function makes a snapshot of the given graph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        void save(SmartGraph& gr) { gr._saveSnapshot(*this); }

        /** Undo the changes until the last snapshot. */

        /**
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(SmartGraph&).
         */
        void restore() { _graph->_restoreSnapshot(*this); }
      };


      void _saveSnapshot(Snapshot& s) {
        s._graph = this;
        s._node_num = _nodes.size();
        s._arc_num = _arcs.size();
      }

      void _restoreSnapshot(const Snapshot& s) {
        while (s._arc_num < _arcs.size()) {
          int  n = _arcs.size() - 1;
          Edge arc = edgeFromId(n / 2);
          Parent::notifier(Edge()).erase(arc);
          nt::TrivialDynamicArray< Arc > dir;
          dir.push_back(arcFromId(n));
          dir.push_back(arcFromId(n - 1));
          Parent::notifier(Arc()).erase(dir);
          _nodes[_arcs[n - 1].target].first_out = _arcs[n].next_out;
          _nodes[_arcs[n].target].first_out = _arcs[n - 1].next_out;
          _arcs.pop_back();
          _arcs.pop_back();
        }
        while (s._node_num < _nodes.size()) {
          int  n = _nodes.size() - 1;
          Node node = nodeFromId(n);
          Parent::notifier(Node()).erase(node);
          _nodes.pop_back();
        }
      }
    };

    struct SmartBpGraphBase {
      using Graph = SmartBpGraphBase;
      using NodeNumTag = nt::True;
      using EdgeNumTag = nt::True;
      using ArcNumTag = nt::True;

      struct NodeT {
        int  first_out;
        int  partition_next;
        int  partition_index;
        bool red;
      };

      struct ArcT {
        int target;
        int next_out;
      };

      struct Node {
        int _id;

        Node() = default;
        constexpr Node(Invalid) { _id = -1; }
        constexpr explicit Node(int id) { _id = id; }

        constexpr bool operator==(const Node& node) const noexcept { return _id == node._id; }
        constexpr bool operator!=(const Node& node) const noexcept { return _id != node._id; }
        constexpr bool operator<(const Node& node) const noexcept { return _id < node._id; }
      };

      struct RedNode: public Node {
        RedNode() = default;
        constexpr RedNode(const RedNode& node) : Node(node) {}
        constexpr RedNode(Invalid) : Node(INVALID) {}
        constexpr explicit RedNode(int pid) : Node(pid) {}
      };

      struct BlueNode: public Node {
        BlueNode() = default;
        constexpr BlueNode(const BlueNode& node) : Node(node) {}
        constexpr BlueNode(Invalid) : Node(INVALID) {}
        constexpr explicit BlueNode(int pid) : Node(pid) {}
      };

      struct Edge {
        int _id;

        Edge() = default;
        constexpr Edge(Invalid) { _id = -1; }
        constexpr explicit Edge(int id) { _id = id; }

        constexpr bool operator==(const Edge& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Edge& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Edge& arc) const noexcept { return _id < arc._id; }
      };

      struct Arc {
        int _id;

        Arc() = default;
        constexpr Arc(Invalid) { _id = -1; }
        constexpr explicit Arc(int id) { _id = id; }

        constexpr operator Edge() const { return _id != -1 ? edgeFromId(_id / 2) : INVALID; }

        constexpr bool operator==(const Arc& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return _id < arc._id; }
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      nt::TrivialDynamicArray< ArcT >  _arcs;

      int first_red, first_blue;
      int max_red, max_blue;

      SmartBpGraphBase() : _nodes(), _arcs(), first_red(-1), first_blue(-1), max_red(-1), max_blue(-1) {}

      constexpr int nodeNum() const noexcept { return _nodes.size(); }
      constexpr int redNum() const noexcept { return max_red + 1; }
      constexpr int blueNum() const noexcept { return max_blue + 1; }
      constexpr int edgeNum() const noexcept { return _arcs.size() / 2; }
      constexpr int arcNum() const noexcept { return _arcs.size(); }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxRedId() const noexcept { return max_red; }
      constexpr int maxBlueId() const noexcept { return max_blue; }
      constexpr int maxEdgeId() const noexcept { return _arcs.size() / 2 - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr bool red(Node n) const noexcept { return _nodes[n._id].red; }
      constexpr bool blue(Node n) const noexcept { return !_nodes[n._id].red; }

      constexpr static RedNode  asRedNodeUnsafe(Node n) noexcept { return RedNode(n._id); }
      constexpr static BlueNode asBlueNodeUnsafe(Node n) noexcept { return BlueNode(n._id); }

      constexpr Node source(Arc a) const noexcept { return Node(_arcs[a._id ^ 1].target); }
      constexpr Node target(Arc a) const noexcept { return Node(_arcs[a._id].target); }

      constexpr RedNode  redNode(Edge e) const noexcept { return RedNode(_arcs[2 * e._id].target); }
      constexpr BlueNode blueNode(Edge e) const noexcept { return BlueNode(_arcs[2 * e._id + 1].target); }

      constexpr static bool direction(Arc a) noexcept { return (a._id & 1) == 1; }

      constexpr static Arc direct(Edge e, bool d) noexcept { return Arc(e._id * 2 + (d ? 1 : 0)); }

      constexpr void first(Node& node) const noexcept { node._id = _nodes.size() - 1; }

      constexpr static void next(Node& node) noexcept { --node._id; }

      constexpr void first(RedNode& node) const noexcept { node._id = first_red; }

      constexpr void next(RedNode& node) const noexcept { node._id = _nodes[node._id].partition_next; }

      constexpr void first(BlueNode& node) const noexcept { node._id = first_blue; }

      constexpr void next(BlueNode& node) const noexcept { node._id = _nodes[node._id].partition_next; }

      constexpr void first(Arc& arc) const noexcept { arc._id = _arcs.size() - 1; }

      constexpr static void next(Arc& arc) noexcept { --arc._id; }

      constexpr void first(Edge& arc) const noexcept { arc._id = _arcs.size() / 2 - 1; }

      constexpr static void next(Edge& arc) noexcept { --arc._id; }

      constexpr void firstOut(Arc& arc, const Node& v) const noexcept { arc._id = _nodes[v._id].first_out; }
      constexpr void nextOut(Arc& arc) const noexcept { arc._id = _arcs[arc._id].next_out; }

      constexpr void firstIn(Arc& arc, const Node& v) const noexcept {
        arc._id = ((_nodes[v._id].first_out) ^ 1);
        if (arc._id == -2) arc._id = -1;
      }
      constexpr void nextIn(Arc& arc) const noexcept {
        arc._id = ((_arcs[arc._id ^ 1].next_out) ^ 1);
        if (arc._id == -2) arc._id = -1;
      }

      constexpr void firstInc(Edge& arc, bool& d, const Node& v) const noexcept {
        int de = _nodes[v._id].first_out;
        if (de != -1) {
          arc._id = de / 2;
          d = ((de & 1) == 1);
        } else {
          arc._id = -1;
          d = true;
        }
      }
      constexpr void nextInc(Edge& arc, bool& d) const noexcept {
        int de = (_arcs[(arc._id * 2) | (d ? 1 : 0)].next_out);
        if (de != -1) {
          arc._id = de / 2;
          d = ((de & 1) == 1);
        } else {
          arc._id = -1;
          d = true;
        }
      }

      constexpr static int id(Node v) noexcept { return v._id; }
      constexpr int        id(RedNode v) const noexcept { return _nodes[v._id].partition_index; }
      constexpr int        id(BlueNode v) const noexcept { return _nodes[v._id].partition_index; }
      constexpr static int id(Arc e) noexcept { return e._id; }
      constexpr static int id(Edge e) noexcept { return e._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }
      constexpr static Edge edgeFromId(int id) noexcept { return Edge(id); }

      constexpr bool valid(Node n) const noexcept { return n._id >= 0 && n._id < static_cast< int >(_nodes.size()); }
      constexpr bool valid(Arc a) const noexcept { return a._id >= 0 && a._id < static_cast< int >(_arcs.size()); }
      constexpr bool valid(Edge e) const noexcept { return e._id >= 0 && 2 * e._id < static_cast< int >(_arcs.size()); }

      RedNode addRedNode() noexcept {
        const int n = _nodes.add();
        _nodes[n].first_out = -1;
        _nodes[n].red = true;
        _nodes[n].partition_index = ++max_red;
        _nodes[n].partition_next = first_red;
        first_red = n;

        return RedNode(n);
      }

      BlueNode addBlueNode() noexcept {
        const int n = _nodes.add();
        _nodes[n].first_out = -1;
        _nodes[n].red = false;
        _nodes[n].partition_index = ++max_blue;
        _nodes[n].partition_next = first_blue;
        first_blue = n;

        return BlueNode(n);
      }

      Edge addEdge(RedNode u, BlueNode v) noexcept {
        const int n = _arcs.add();
        _arcs.add();

        _arcs[n].target = u._id;
        _arcs[n | 1].target = v._id;

        _arcs[n].next_out = _nodes[v._id].first_out;
        _nodes[v._id].first_out = n;

        _arcs[n | 1].next_out = _nodes[u._id].first_out;
        _nodes[u._id].first_out = (n | 1);

        return Edge(n / 2);
      }

      void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
        first_red = -1;
        first_blue = -1;
        max_blue = -1;
        max_red = -1;
      }
    };

    using ExtendedSmartBpGraphBase = BpGraphExtender< SmartBpGraphBase >;

    /**
     * @ingroup graphs
     *
     * @brief A smart undirected bipartite graph class.
     *
     * @ref SmartBpGraph is a simple and fast bipartite graph implementation.
     * It is also quite memory efficient but at the price
     * that it does not support node and edge deletion
     * (except for the Snapshot feature).
     *
     * This type fully conforms to the @ref concepts::BpGraph "BpGraph concept"
     * and it also provides some additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides constant time counting for nodes, edges and arcs.
     *
     * \sa concepts::BpGraph
     * \sa SmartGraph
     */
    struct SmartBpGraph: public ExtendedSmartBpGraphBase {
      using Parent = ExtendedSmartBpGraphBase;

      /**
       * Graphs are \e not copy constructible. Use GraphCopy instead.
       */
      SmartBpGraph(const SmartBpGraph&) = delete;
      /**
       * @brief Assignment of a graph to another one is \e not allowed.
       * Use GraphCopy instead.
       */
      void operator=(const SmartBpGraph&) = delete;

      /** Constructor */

      /**
       * Constructor.
       *
       */
      SmartBpGraph() = default;

      /**
       * @brief Add a new red node to the graph.
       *
       * This function adds a red new node to the graph.
       * @return The new node.
       */
      RedNode addRedNode() { return Parent::addRedNode(); }

      /**
       * @brief Add a new blue node to the graph.
       *
       * This function adds a blue new node to the graph.
       * @return The new node.
       */
      BlueNode addBlueNode() { return Parent::addBlueNode(); }

      /**
       * @brief Add a new edge to the graph.
       *
       * This function adds a new edge to the graph between nodes
       * \c u and \c v with inherent orientation from node \c u to
       * node \c v.
       * @return The new edge.
       */
      Edge addEdge(RedNode u, BlueNode v) { return Parent::addEdge(u, v); }
      Edge addEdge(BlueNode v, RedNode u) { return Parent::addEdge(u, v); }

      /**
       * @brief Node validity check
       *
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the graph.
       *
       * @warning A removed node (using Snapshot) could become valid again
       * if new nodes are added to the graph.
       */
      bool valid(Node n) const { return Parent::valid(n); }

      /**
       * @brief Edge validity check
       *
       * This function gives back \c true if the given edge is valid,
       * i.e. it is a real edge of the graph.
       *
       * @warning A removed edge (using Snapshot) could become valid again
       * if new edges are added to the graph.
       */
      bool valid(Edge e) const { return Parent::valid(e); }

      /**
       * @brief Arc validity check
       *
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the graph.
       *
       * @warning A removed arc (using Snapshot) could become valid again
       * if new edges are added to the graph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /** Clear the graph. */

      /**
       * This function erases all nodes and arcs from the graph.
       *
       */
      void clear() { Parent::clear(); }

      /** Reserve memory for nodes. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the graph you want to build will
       * be large (e.g. it will contain millions of nodes and/or edges),
       * then it is worth reserving space for this amount before starting
       * to build the graph.
       * \sa reserveEdge()
       */
      void reserveNode(int n) { _nodes.reserve(n); };

      /** Reserve memory for edges. */

      /**
       * Using this function, it is possible to avoid superfluous memory
       * allocation: if you know that the graph you want to build will
       * be large (e.g. it will contain millions of nodes and/or edges),
       * then it is worth reserving space for this amount before starting
       * to build the graph.
       * \sa reserveNode()
       */
      void reserveEdge(int m) { _arcs.reserve(2 * m); };


      /**
       * Class to make a snapshot of the graph and to restore it later.
       */

      /**
       * Class to make a snapshot of the graph and to restore it later.
       *
       * The newly added nodes and edges can be removed using the
       * restore() function. This is the only way for deleting nodes and/or
       * edges from a SmartBpGraph structure.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and edges again using
       * another Snapshot instance.
       *
       * @warning The validity of the snapshot is not stored due to
       * performance reasons. If you do not use the snapshot correctly,
       * it can cause broken program, invalid or not restored state of
       * the graph or no change.
       */
      struct Snapshot {
        SmartBpGraph* _graph;

        unsigned int _node_num;
        unsigned int _arc_num;

        /** Default constructor. */

        /**
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : _graph(0) {}
        /** Constructor that immediately makes a snapshot */

        /**
         * This constructor immediately makes a snapshot of the given graph.
         *
         */
        Snapshot(SmartBpGraph& gr) { gr._saveSnapshot(*this); }

        /** Make a snapshot. */

        /**
         * This function makes a snapshot of the given graph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        void save(SmartBpGraph& gr) { gr._saveSnapshot(*this); }

        /** Undo the changes until the last snapshot. */

        /**
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(SmartBpGraph&).
         */
        void restore() { _graph->_restoreSnapshot(*this); }
      };


      void _saveSnapshot(Snapshot& s) {
        s._graph = this;
        s._node_num = _nodes.size();
        s._arc_num = _arcs.size();
      }

      void _restoreSnapshot(const Snapshot& s) {
        while (s._arc_num < _arcs.size()) {
          int  n = _arcs.size() - 1;
          Edge arc = edgeFromId(n / 2);
          Parent::notifier(Edge()).erase(arc);
          nt::TrivialDynamicArray< Arc > dir;
          dir.push_back(arcFromId(n));
          dir.push_back(arcFromId(n - 1));
          Parent::notifier(Arc()).erase(dir);
          _nodes[_arcs[n - 1].target].first_out = _arcs[n].next_out;
          _nodes[_arcs[n].target].first_out = _arcs[n - 1].next_out;
          _arcs.pop_back();
          _arcs.pop_back();
        }
        while (s._node_num < _nodes.size()) {
          int  n = _nodes.size() - 1;
          Node node = nodeFromId(n);
          if (Parent::red(node)) {
            first_red = _nodes[n].partition_next;
            if (first_red != -1) {
              max_red = _nodes[first_red].partition_index;
            } else {
              max_red = -1;
            }
            Parent::notifier(RedNode()).erase(asRedNodeUnsafe(node));
          } else {
            first_blue = _nodes[n].partition_next;
            if (first_blue != -1) {
              max_blue = _nodes[first_blue].partition_index;
            } else {
              max_blue = -1;
            }
            Parent::notifier(BlueNode()).erase(asBlueNodeUnsafe(node));
          }
          Parent::notifier(Node()).erase(node);
          _nodes.pop_back();
        }
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
