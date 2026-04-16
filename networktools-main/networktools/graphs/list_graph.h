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
 * This file incorporates work from the LEMON graph library (list_graph.h).
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
 *   - Replaced std::list with nt::TrivialDynamicArray
 *   - Updated include paths to networktools structure
 *   - Adapted LEMON concept checking to networktools
 *   - Converted typedef declarations to C++11 using declarations
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Added move semantics for performance
 *   - Modified exception handling strategy
 *   - Updated header guard macros
 *   - Added numNode(), numArc(), numEdge(), numBlueNode(), numRedNode() methods
 */

/**
 * @ingroup graphs
 * @file
 * @brief ListDigraph and ListGraph classes.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_LIST_GRAPH_H_
#define _NT_LIST_GRAPH_H_

#include "../core/common.h"
#include "bits/graph_extender.h"

namespace nt {
  namespace graphs {

    class ListDigraph;

    class ListDigraphBase {
      using NodeNumTag = nt::True;
      using ArcNumTag = nt::True;

      protected:
      struct NodeT {
        int first_in, first_out;
        int prev, next;
      };

      struct ArcT {
        int target, source;
        int prev_in, prev_out;
        int next_in, next_out;
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      int                              _i_first_node;
      int                              _i_first_free_node;

      nt::TrivialDynamicArray< ArcT > _arcs;
      int                             _i_first_free_arc;

      int _i_num_node;
      int _i_num_arc;

      public:
      using Digraph = ListDigraphBase;

      class Node {
        friend class ListDigraphBase;
        friend class ListDigraph;

        protected:
        int _id;
        explicit constexpr Node(int pid) { _id = pid; }

        public:
        Node() = default;
        constexpr Node(Invalid) { _id = -1; }
        constexpr bool operator==(const Node& node) const noexcept { return _id == node._id; }
        constexpr bool operator!=(const Node& node) const noexcept { return _id != node._id; }
        constexpr bool operator<(const Node& node) const noexcept { return _id < node._id; }
      };

      class Arc {
        friend class ListDigraphBase;
        friend class ListDigraph;

        protected:
        int _id;
        explicit constexpr Arc(int pid) { _id = pid; }

        public:
        Arc() = default;
        constexpr Arc(Invalid) { _id = -1; }
        constexpr bool operator==(const Arc& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return _id < arc._id; }
      };

      ListDigraphBase() :
          _nodes(), _i_first_node(-1), _i_first_free_node(-1), _arcs(), _i_first_free_arc(-1), _i_num_node(0),
          _i_num_arc(0) {}

      ListDigraphBase(ListDigraphBase&& g) noexcept :
          _nodes(std::move(g._nodes)), _i_first_node(g._i_first_node), _i_first_free_node(g._i_first_free_node),
          _arcs(std::move(g._arcs)), _i_first_free_arc(g._i_first_free_arc), _i_num_node(g._i_num_node),
          _i_num_arc(g._i_num_arc) {
        // Reset source to empty state
        g._i_first_node = -1;
        g._i_first_free_node = -1;
        g._i_first_free_arc = -1;
        g._i_num_node = 0;
        g._i_num_arc = 0;
      }

      ListDigraphBase& operator=(ListDigraphBase&& g) noexcept {
        if (this != &g) {
          _nodes = std::move(g._nodes);
          _i_first_node = g._i_first_node;
          _i_first_free_node = g._i_first_free_node;
          _arcs = std::move(g._arcs);
          _i_first_free_arc = g._i_first_free_arc;
          _i_num_node = g._i_num_node;
          _i_num_arc = g._i_num_arc;

          // Reset source to empty state
          g._i_first_node = -1;
          g._i_first_free_node = -1;
          g._i_first_free_arc = -1;
          g._i_num_node = 0;
          g._i_num_arc = 0;
        }
        return *this;
      }

      constexpr int nodeNum() const noexcept { return _i_num_node; }
      constexpr int arcNum() const noexcept { return _i_num_arc; }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr Node source(Arc e) const noexcept { return Node(_arcs[e._id].source); }
      constexpr Node target(Arc e) const noexcept { return Node(_arcs[e._id].target); }

      constexpr void first(Node& node) const noexcept { node._id = _i_first_node; }

      constexpr void next(Node& node) const noexcept { node._id = _nodes[node._id].next; }

      constexpr void first(Arc& arc) const noexcept {
        int n;
        for (n = _i_first_node; n != -1 && _nodes[n].first_out == -1; n = _nodes[n].next) {}
        arc._id = (n == -1) ? -1 : _nodes[n].first_out;
      }

      constexpr void next(Arc& arc) const noexcept {
        if (_arcs[arc._id].next_out != -1) {
          arc._id = _arcs[arc._id].next_out;
        } else {
          int n;
          for (n = _nodes[_arcs[arc._id].source].next; n != -1 && _nodes[n].first_out == -1; n = _nodes[n].next) {}
          arc._id = (n == -1) ? -1 : _nodes[n].first_out;
        }
      }

      constexpr void firstOut(Arc& e, const Node& v) const noexcept { e._id = _nodes[v._id].first_out; }
      constexpr void nextOut(Arc& e) const noexcept { e._id = _arcs[e._id].next_out; }

      constexpr void firstIn(Arc& e, const Node& v) const noexcept { e._id = _nodes[v._id].first_in; }
      constexpr void nextIn(Arc& e) const noexcept { e._id = _arcs[e._id].next_in; }

      constexpr static int id(Node v) noexcept { return v._id; }
      constexpr static int id(Arc e) noexcept { return e._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }

      constexpr bool valid(Node n) const noexcept {
        return n._id >= 0 && n._id < static_cast< int >(_nodes.size()) && _nodes[n._id].prev != -2;
      }
      constexpr bool valid(Arc a) const noexcept {
        return a._id >= 0 && a._id < static_cast< int >(_arcs.size()) && _arcs[a._id].prev_in != -2;
      }

      Node addNode() noexcept {
        int n;

        if (_i_first_free_node == -1) {
          n = _nodes.add();
        } else {
          n = _i_first_free_node;
          _i_first_free_node = _nodes[n].next;
        }

        _nodes[n].next = _i_first_node;
        if (_i_first_node != -1) _nodes[_i_first_node].prev = n;
        _i_first_node = n;
        _nodes[n].prev = -1;

        _nodes[n].first_in = _nodes[n].first_out = -1;

        ++_i_num_node;
        return Node(n);
      }

      Arc addArc(Node u, Node v) noexcept {
        int n;

        if (_i_first_free_arc == -1) {
          n = _arcs.add();
        } else {
          n = _i_first_free_arc;
          _i_first_free_arc = _arcs[n].next_in;
        }

        _arcs[n].source = u._id;
        _arcs[n].target = v._id;

        _arcs[n].next_out = _nodes[u._id].first_out;
        if (_nodes[u._id].first_out != -1) { _arcs[_nodes[u._id].first_out].prev_out = n; }

        _arcs[n].next_in = _nodes[v._id].first_in;
        if (_nodes[v._id].first_in != -1) { _arcs[_nodes[v._id].first_in].prev_in = n; }

        _arcs[n].prev_in = _arcs[n].prev_out = -1;

        _nodes[u._id].first_out = _nodes[v._id].first_in = n;

        ++_i_num_arc;
        return Arc(n);
      }

      constexpr void erase(const Node& node) noexcept {
        int n = node._id;

        if (_nodes[n].next != -1) { _nodes[_nodes[n].next].prev = _nodes[n].prev; }

        if (_nodes[n].prev != -1) {
          _nodes[_nodes[n].prev].next = _nodes[n].next;
        } else {
          _i_first_node = _nodes[n].next;
        }

        _nodes[n].next = _i_first_free_node;
        _i_first_free_node = n;
        _nodes[n].prev = -2;

        --_i_num_node;
      }

      constexpr void erase(const Arc& arc) noexcept {
        int n = arc._id;

        if (_arcs[n].next_in != -1) { _arcs[_arcs[n].next_in].prev_in = _arcs[n].prev_in; }

        if (_arcs[n].prev_in != -1) {
          _arcs[_arcs[n].prev_in].next_in = _arcs[n].next_in;
        } else {
          _nodes[_arcs[n].target].first_in = _arcs[n].next_in;
        }

        if (_arcs[n].next_out != -1) { _arcs[_arcs[n].next_out].prev_out = _arcs[n].prev_out; }

        if (_arcs[n].prev_out != -1) {
          _arcs[_arcs[n].prev_out].next_out = _arcs[n].next_out;
        } else {
          _nodes[_arcs[n].source].first_out = _arcs[n].next_out;
        }

        _arcs[n].next_in = _i_first_free_arc;
        _i_first_free_arc = n;
        _arcs[n].prev_in = -2;

        --_i_num_arc;
      }

      void copyFrom(const ListDigraphBase& other) noexcept {
        _nodes.copyFrom(other._nodes);
        _i_first_node = other._i_first_node;
        _i_first_free_node = other._i_first_free_node;

        _arcs.copyFrom(other._arcs);
        _i_first_free_arc = other._i_first_free_arc;
        _i_num_node = other._i_num_node;
        _i_num_arc = other._i_num_arc;
      }

      void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
        _i_first_node = _i_first_free_node = _i_first_free_arc = -1;
        _i_num_arc = _i_num_node = 0;
      }

      protected:
      constexpr void changeTarget(Arc e, Node n) noexcept {
        if (_arcs[e._id].next_in != -1) _arcs[_arcs[e._id].next_in].prev_in = _arcs[e._id].prev_in;
        if (_arcs[e._id].prev_in != -1)
          _arcs[_arcs[e._id].prev_in].next_in = _arcs[e._id].next_in;
        else
          _nodes[_arcs[e._id].target].first_in = _arcs[e._id].next_in;
        if (_nodes[n._id].first_in != -1) { _arcs[_nodes[n._id].first_in].prev_in = e._id; }
        _arcs[e._id].target = n._id;
        _arcs[e._id].prev_in = -1;
        _arcs[e._id].next_in = _nodes[n._id].first_in;
        _nodes[n._id].first_in = e._id;
      }
      constexpr void changeSource(Arc e, Node n) noexcept {
        if (_arcs[e._id].next_out != -1) _arcs[_arcs[e._id].next_out].prev_out = _arcs[e._id].prev_out;
        if (_arcs[e._id].prev_out != -1)
          _arcs[_arcs[e._id].prev_out].next_out = _arcs[e._id].next_out;
        else
          _nodes[_arcs[e._id].source].first_out = _arcs[e._id].next_out;
        if (_nodes[n._id].first_out != -1) { _arcs[_nodes[n._id].first_out].prev_out = e._id; }
        _arcs[e._id].source = n._id;
        _arcs[e._id].prev_out = -1;
        _arcs[e._id].next_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = e._id;
      }
    };

    using ExtendedListDigraphBase = DigraphExtender< ListDigraphBase >;

    /**
     * \addtogroup graphs
     * @{
     */

    /** A general directed graph structure. */

    /**
     * @ref ListDigraph is a versatile and fast directed graph
     * implementation based on linked lists that are stored in
     * \c nt::DynamicArray structures.
     *
     * This type fully conforms to the @ref concepts::Digraph "Digraph concept"
     * and it also provides several useful additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides only linear time counting for nodes and arcs.
     *
     * \sa concepts::Digraph
     * \sa ListGraph
     */
    struct ListDigraph: public ExtendedListDigraphBase {
      using Parent = ExtendedListDigraphBase;

      /**
       * @brief Move constructor.
       */
      ListDigraph(ListDigraph&& g) noexcept : Parent(std::move(g)) {}

      /**
       * @brief Move assignment operator.
       *
       * @return Reference to this digraph
       */
      ListDigraph& operator=(ListDigraph&& g) noexcept {
        Parent::operator=(std::move(g));
        return *this;
      }

      /**
       * Digraphs are \e not copy constructible. Use DigraphCopy instead.
       */
      ListDigraph(const ListDigraph&) = delete;
      /**
       * @brief Assignment of a digraph to another one is \e not allowed.
       * Use DigraphCopy instead.
       */
      void operator=(const ListDigraph&) = delete;

      /** Constructor */

      /**
       * Constructor.
       *
       */
      ListDigraph() = default;

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
       * @brief Erase a node from the digraph.
       *
       * This function erases the given node along with its outgoing and
       * incoming arcs from the digraph.
       *
       * @note All iterators referencing the removed node or the connected
       * arcs are invalidated, of course.
       */
      void erase(Node n) { Parent::erase(n); }

      /**
       * @brief Erase an arc from the digraph.
       *
       * This function erases the given arc from the digraph.
       *
       * @note All iterators referencing the removed arc are invalidated,
       * of course.
       */
      void erase(Arc a) { Parent::erase(a); }

      /** Node validity check */

      /**
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the digraph.
       *
       * @warning A removed node could become valid again if new nodes are
       * added to the digraph.
       */
      bool valid(Node n) const { return Parent::valid(n); }

      /** Arc validity check */

      /**
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the digraph.
       *
       * @warning A removed arc could become valid again if new arcs are
       * added to the digraph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /** Change the target node of an arc */

      /**
       * This function changes the target node of the given arc \c a to \c n.
       *
       * @note \c ArcIt and \c OutArcIt iterators referencing the changed
       * arc remain valid, but \c InArcIt iterators are invalidated.
       *
       * @warning This functionality cannot be used together with the Snapshot
       * feature.
       */
      void changeTarget(Arc a, Node n) { Parent::changeTarget(a, n); }
      /** Change the source node of an arc */

      /**
       * This function changes the source node of the given arc \c a to \c n.
       *
       * @note \c InArcIt iterators referencing the changed arc remain
       * valid, but \c ArcIt and \c OutArcIt iterators are invalidated.
       *
       * @warning This functionality cannot be used together with the Snapshot
       * feature.
       */
      void changeSource(Arc a, Node n) { Parent::changeSource(a, n); }

      /** Reverse the direction of an arc. */

      /**
       * This function reverses the direction of the given arc.
       * @note \c ArcIt, \c OutArcIt and \c InArcIt iterators referencing
       * the changed arc are invalidated.
       *
       * @warning This functionality cannot be used together with the Snapshot
       * feature.
       */
      void reverseArc(Arc a) {
        Node t = target(a);
        changeTarget(a, source(a));
        changeSource(a, t);
      }

      /** Contract two nodes. */

      /**
       * This function contracts the given two nodes.
       * Node \c v is removed, but instead of deleting its
       * incident arcs, they are joined to node \c u.
       * If the last parameter \c r is \c true (this is the default value),
       * then the newly created loops are removed.
       *
       * @note The moved arcs are joined to node \c u using changeSource()
       * or changeTarget(), thus \c ArcIt and \c OutArcIt iterators are
       * invalidated for the outgoing arcs of node \c v and \c InArcIt
       * iterators are invalidated for the incoming arcs of \c v.
       * Moreover all iterators referencing node \c v or the removed
       * loops are also invalidated. Other iterators remain valid.
       *
       * @warning This functionality cannot be used together with the Snapshot
       * feature.
       */
      void contract(Node u, Node v, bool r = true) {
        for (OutArcIt e(*this, v); e != INVALID;) {
          OutArcIt f = e;
          ++f;
          if (r && target(e) == u)
            erase(e);
          else
            changeSource(e, u);
          e = f;
        }
        for (InArcIt e(*this, v); e != INVALID;) {
          InArcIt f = e;
          ++f;
          if (r && source(e) == u)
            erase(e);
          else
            changeTarget(e, u);
          e = f;
        }
        erase(v);
      }

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
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
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

      /** Split an arc. */

      /**
       * This function splits the given arc. First, a new node \c v is
       * added to the digraph, then the target node of the original arc
       * is set to \c v. Finally, an arc from \c v to the original target
       * is added.
       * @return The newly created node.
       *
       * @note \c InArcIt iterators referencing the original arc are
       * invalidated. Other iterators remain valid.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      Node split(Arc a) {
        Node v = addNode();
        addArc(v, target(a));
        changeTarget(a, v);
        return v;
      }

      /** Clear the digraph. */

      /**
       * This function erases all nodes and arcs from the digraph.
       *
       * @note All iterators of the digraph are invalidated, of course.
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
       * @brief Copy the content of another digraph.
       *
       */
      void copyFrom(const ListDigraph& other) noexcept { Parent::copyFrom(other); }

      /**
       * @brief Class to make a snapshot of the digraph and restore
       * it later.
       *
       * Class to make a snapshot of the digraph and restore it later.
       *
       * The newly added nodes and arcs can be removed using the
       * restore() function.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and arcs again using
       * another Snapshot instance.
       *
       * @warning Node and arc deletions and other modifications (e.g.
       * reversing, contracting, splitting arcs or nodes) cannot be
       * restored. These events invalidate the snapshot.
       * However, the arcs and nodes that were added to the digraph after
       * making the current snapshot can be removed without invalidating it.
       */
      class Snapshot {
        protected:
        using NodeNotifier = Parent::NodeNotifier;

        class NodeObserverProxy: public NodeNotifier::ObserverBase {
          public:
          NodeObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using NodeNotifier::ObserverBase::attach;
          using NodeNotifier::ObserverBase::attached;
          using NodeNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Node& node) noexcept override {
            snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = nodes.size() - 1; i >= 0; ++i)
              snapshot.addNode(nodes[i]);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Node& node) noexcept override {
            return snapshot.eraseNode(node);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = 0; i < int(nodes.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseNode(nodes[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node))
              snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node)) {
              const ObserverReturnCode r = snapshot.eraseNode(node);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        class ArcObserverProxy: public ArcNotifier::ObserverBase {
          public:
          ArcObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using ArcNotifier::ObserverBase::attach;
          using ArcNotifier::ObserverBase::attached;
          using ArcNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Arc& arc) noexcept override {
            snapshot.addArc(arc);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Arc >& arcs) noexcept override {
            for (int i = arcs.size() - 1; i >= 0; ++i) {
              snapshot.addArc(arcs[i]);
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Arc& arc) noexcept override {
            return snapshot.eraseArc(arc);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Arc >& arcs) noexcept override {
            for (int i = 0; i < int(arcs.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseArc(arcs[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Arc arc;
            for (notifier()->first(arc); arc != INVALID; notifier()->next(arc))
              snapshot.addArc(arc);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Arc arc;
            for (notifier()->first(arc); arc != INVALID; notifier()->next(arc)) {
              const ObserverReturnCode r = snapshot.eraseArc(arc);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        ListDigraph* digraph;

        NodeObserverProxy node_observer_proxy;
        ArcObserverProxy  arc_observer_proxy;

        nt::TrivialDynamicArray< Node > added_nodes;
        nt::TrivialDynamicArray< Arc >  added_arcs;

        constexpr void               addNode(const Node& node) noexcept { added_nodes.push_back(node); }
        constexpr ObserverReturnCode eraseNode(const Node& node) noexcept {
          const int i = added_nodes.find(node);
          if (i == added_nodes.size()) {
            clear();
            arc_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_nodes.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void               addArc(const Arc& arc) noexcept { added_arcs.push_back(arc); }
        constexpr ObserverReturnCode eraseArc(const Arc& arc) noexcept {
          const int i = added_arcs.find(arc);
          if (i == added_arcs.size()) {
            clear();
            node_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_arcs.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void attach(ListDigraph& _digraph) noexcept {
          digraph = &_digraph;
          node_observer_proxy.attach(digraph->notifier(Node()));
          arc_observer_proxy.attach(digraph->notifier(Arc()));
        }

        constexpr void detach() noexcept {
          node_observer_proxy.detach();
          arc_observer_proxy.detach();
        }

        constexpr bool attached() const noexcept { return node_observer_proxy.attached(); }

        constexpr void clear() noexcept {
          added_nodes.clear();
          added_arcs.clear();
        }

        public:
        /**
         * @brief Default constructor.
         *
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : digraph(0), node_observer_proxy(*this), arc_observer_proxy(*this) {}

        /**
         * @brief Constructor that immediately makes a snapshot.
         *
         * This constructor immediately makes a snapshot of the given digraph.
         */
        Snapshot(ListDigraph& gr) : node_observer_proxy(*this), arc_observer_proxy(*this) { attach(gr); }

        /**
         * @brief Make a snapshot.
         *
         * This function makes a snapshot of the given digraph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        void save(ListDigraph& gr) {
          if (attached()) {
            detach();
            clear();
          }
          attach(gr);
        }

        /**
         * @brief Undo the changes until the last snapshot.
         *
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(ListDigraph&).
         *
         * @warning This method invalidates the snapshot, i.e. repeated
         * restoring is not supported unless you call save() again.
         */
        void restore() {
          detach();
          for (const Arc a: added_arcs)
            digraph->erase(a);
          for (const Node v: added_nodes)
            digraph->erase(v);
          clear();
        }

        /**
         * @brief Returns \c true if the snapshot is valid.
         *
         * This function returns \c true if the snapshot is valid.
         */
        bool valid() const { return attached(); }
      };
    };

    /** @} */

    class ListGraphBase {
      using NodeNumTag = nt::True;
      using ArcNumTag = nt::True;
      using EdgeNumTag = nt::True;

      protected:
      struct NodeT {
        int first_out;
        int prev, next;
      };

      struct ArcT {
        int target;
        int prev_out, next_out;
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      int                              _i_first_node;
      int                              _i_first_free_node;

      nt::TrivialDynamicArray< ArcT > _arcs;
      int                             _i_first_free_arc;

      int _i_num_node;
      int _i_num_arc;

      public:
      using Graph = ListGraphBase;

      class Node {
        friend class ListGraphBase;

        protected:
        int _id;
        constexpr explicit Node(int pid) { _id = pid; }

        public:
        Node() = default;
        constexpr Node(Invalid) { _id = -1; }
        constexpr bool operator==(const Node& node) const noexcept { return _id == node._id; }
        constexpr bool operator!=(const Node& node) const noexcept { return _id != node._id; }
        constexpr bool operator<(const Node& node) const noexcept { return _id < node._id; }
      };

      class Edge {
        friend class ListGraphBase;

        protected:
        int _id;
        constexpr explicit Edge(int pid) { _id = pid; }

        public:
        Edge() = default;
        constexpr Edge(Invalid) { _id = -1; }
        constexpr bool operator==(const Edge& edge) const noexcept { return _id == edge._id; }
        constexpr bool operator!=(const Edge& edge) const noexcept { return _id != edge._id; }
        constexpr bool operator<(const Edge& edge) const noexcept { return _id < edge._id; }
      };

      class Arc {
        friend class ListGraphBase;

        protected:
        int _id;
        constexpr explicit Arc(int pid) { _id = pid; }

        public:
        operator Edge() const { return _id != -1 ? edgeFromId(_id / 2) : INVALID; }

        Arc() = default;
        constexpr Arc(Invalid) { _id = -1; }
        constexpr bool operator==(const Arc& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return _id < arc._id; }
      };

      ListGraphBase() :
          _nodes(), _i_first_node(-1), _i_first_free_node(-1), _arcs(), _i_first_free_arc(-1), _i_num_node(0),
          _i_num_arc(0) {}

      ListGraphBase(ListGraphBase&& g) noexcept :
          _nodes(std::move(g._nodes)), _i_first_node(g._i_first_node), _i_first_free_node(g._i_first_free_node),
          _arcs(std::move(g._arcs)), _i_first_free_arc(g._i_first_free_arc), _i_num_node(g._i_num_node),
          _i_num_arc(g._i_num_arc) {
        // Reset source to empty state
        g._i_first_node = -1;
        g._i_first_free_node = -1;
        g._i_first_free_arc = -1;
        g._i_num_node = 0;
        g._i_num_arc = 0;
      }

      ListGraphBase& operator=(ListGraphBase&& g) noexcept {
        if (this != &g) {
          _nodes = std::move(g._nodes);
          _i_first_node = g._i_first_node;
          _i_first_free_node = g._i_first_free_node;
          _arcs = std::move(g._arcs);
          _i_first_free_arc = g._i_first_free_arc;
          _i_num_node = g._i_num_node;
          _i_num_arc = g._i_num_arc;

          // Reset source to empty state
          g._i_first_node = -1;
          g._i_first_free_node = -1;
          g._i_first_free_arc = -1;
          g._i_num_node = 0;
          g._i_num_arc = 0;
        }
        return *this;
      }

      constexpr int nodeNum() const noexcept { return _i_num_node; }
      constexpr int edgeNum() const noexcept { return _i_num_arc / 2; }
      constexpr int arcNum() const noexcept { return _i_num_arc; }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxEdgeId() const noexcept { return _arcs.size() / 2 - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr Node source(Arc e) const noexcept { return Node(_arcs[e._id ^ 1].target); }
      constexpr Node target(Arc e) const noexcept { return Node(_arcs[e._id].target); }

      constexpr Node u(Edge e) const noexcept { return Node(_arcs[2 * e._id].target); }
      constexpr Node v(Edge e) const noexcept { return Node(_arcs[2 * e._id + 1].target); }

      constexpr static bool direction(Arc e) noexcept { return (e._id & 1) == 1; }

      constexpr static Arc direct(Edge e, bool d) noexcept { return Arc(e._id * 2 + (d ? 1 : 0)); }

      constexpr void first(Node& node) const noexcept { node._id = _i_first_node; }

      constexpr void next(Node& node) const noexcept { node._id = _nodes[node._id].next; }

      constexpr void first(Arc& e) const noexcept {
        int n = _i_first_node;
        while (n != -1 && _nodes[n].first_out == -1) {
          n = _nodes[n].next;
        }
        e._id = (n == -1) ? -1 : _nodes[n].first_out;
      }

      constexpr void next(Arc& e) const noexcept {
        if (_arcs[e._id].next_out != -1) {
          e._id = _arcs[e._id].next_out;
        } else {
          int n = _nodes[_arcs[e._id ^ 1].target].next;
          while (n != -1 && _nodes[n].first_out == -1) {
            n = _nodes[n].next;
          }
          e._id = (n == -1) ? -1 : _nodes[n].first_out;
        }
      }

      constexpr void first(Edge& e) const noexcept {
        int n = _i_first_node;
        while (n != -1) {
          e._id = _nodes[n].first_out;
          while ((e._id & 1) != 1) {
            e._id = _arcs[e._id].next_out;
          }
          if (e._id != -1) {
            e._id /= 2;
            return;
          }
          n = _nodes[n].next;
        }
        e._id = -1;
      }

      constexpr void next(Edge& e) const noexcept {
        int n = _arcs[e._id * 2].target;
        e._id = _arcs[(e._id * 2) | 1].next_out;
        while ((e._id & 1) != 1) {
          e._id = _arcs[e._id].next_out;
        }
        if (e._id != -1) {
          e._id /= 2;
          return;
        }
        n = _nodes[n].next;
        while (n != -1) {
          e._id = _nodes[n].first_out;
          while ((e._id & 1) != 1) {
            e._id = _arcs[e._id].next_out;
          }
          if (e._id != -1) {
            e._id /= 2;
            return;
          }
          n = _nodes[n].next;
        }
        e._id = -1;
      }

      constexpr void firstOut(Arc& e, const Node& v) const noexcept { e._id = _nodes[v._id].first_out; }
      constexpr void nextOut(Arc& e) const noexcept { e._id = _arcs[e._id].next_out; }

      constexpr void firstIn(Arc& e, const Node& v) const noexcept {
        e._id = ((_nodes[v._id].first_out) ^ 1);
        if (e._id == -2) e._id = -1;
      }
      constexpr void nextIn(Arc& e) const {
        e._id = ((_arcs[e._id ^ 1].next_out) ^ 1);
        if (e._id == -2) e._id = -1;
      }

      constexpr void firstInc(Edge& e, bool& d, const Node& v) const noexcept {
        int a = _nodes[v._id].first_out;
        if (a != -1) {
          e._id = a / 2;
          d = ((a & 1) == 1);
        } else {
          e._id = -1;
          d = true;
        }
      }
      constexpr void nextInc(Edge& e, bool& d) const noexcept {
        int a = (_arcs[(e._id * 2) | (d ? 1 : 0)].next_out);
        if (a != -1) {
          e._id = a / 2;
          d = ((a & 1) == 1);
        } else {
          e._id = -1;
          d = true;
        }
      }

      constexpr static int id(Node v) noexcept { return v._id; }
      constexpr static int id(Arc e) noexcept { return e._id; }
      constexpr static int id(Edge e) noexcept { return e._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }
      constexpr static Edge edgeFromId(int id) noexcept { return Edge(id); }

      constexpr bool valid(Node n) const noexcept {
        return n._id >= 0 && n._id < static_cast< int >(_nodes.size()) && _nodes[n._id].prev != -2;
      }
      constexpr bool valid(Arc a) const noexcept {
        return a._id >= 0 && a._id < static_cast< int >(_arcs.size()) && _arcs[a._id].prev_out != -2;
      }
      constexpr bool valid(Edge e) const noexcept {
        return e._id >= 0 && 2 * e._id < static_cast< int >(_arcs.size()) && _arcs[2 * e._id].prev_out != -2;
      }

      Node addNode() noexcept {
        int n;

        if (_i_first_free_node == -1) {
          n = _nodes.add();
        } else {
          n = _i_first_free_node;
          _i_first_free_node = _nodes[n].next;
        }

        _nodes[n].next = _i_first_node;
        if (_i_first_node != -1) _nodes[_i_first_node].prev = n;
        _i_first_node = n;
        _nodes[n].prev = -1;

        _nodes[n].first_out = -1;

        ++_i_num_node;
        return Node(n);
      }

      Edge addEdge(Node u, Node v) noexcept {
        int n;

        if (_i_first_free_arc == -1) {
          n = _arcs.add();
          _arcs.add();
        } else {
          n = _i_first_free_arc;
          _i_first_free_arc = _arcs[n].next_out;
        }

        _arcs[n].target = u._id;
        _arcs[n | 1].target = v._id;

        _arcs[n].next_out = _nodes[v._id].first_out;
        if (_nodes[v._id].first_out != -1) { _arcs[_nodes[v._id].first_out].prev_out = n; }
        _arcs[n].prev_out = -1;
        _nodes[v._id].first_out = n;

        _arcs[n | 1].next_out = _nodes[u._id].first_out;
        if (_nodes[u._id].first_out != -1) { _arcs[_nodes[u._id].first_out].prev_out = (n | 1); }
        _arcs[n | 1].prev_out = -1;
        _nodes[u._id].first_out = (n | 1);

        _i_num_arc += 2;
        return Edge(n / 2);
      }

      constexpr void erase(const Node& node) noexcept {
        int n = node._id;

        if (_nodes[n].next != -1) { _nodes[_nodes[n].next].prev = _nodes[n].prev; }

        if (_nodes[n].prev != -1) {
          _nodes[_nodes[n].prev].next = _nodes[n].next;
        } else {
          _i_first_node = _nodes[n].next;
        }

        _nodes[n].next = _i_first_free_node;
        _i_first_free_node = n;
        _nodes[n].prev = -2;
        --_i_num_node;
      }

      constexpr void erase(const Edge& edge) noexcept {
        int n = edge._id * 2;

        if (_arcs[n].next_out != -1) { _arcs[_arcs[n].next_out].prev_out = _arcs[n].prev_out; }

        if (_arcs[n].prev_out != -1) {
          _arcs[_arcs[n].prev_out].next_out = _arcs[n].next_out;
        } else {
          _nodes[_arcs[n | 1].target].first_out = _arcs[n].next_out;
        }

        if (_arcs[n | 1].next_out != -1) { _arcs[_arcs[n | 1].next_out].prev_out = _arcs[n | 1].prev_out; }

        if (_arcs[n | 1].prev_out != -1) {
          _arcs[_arcs[n | 1].prev_out].next_out = _arcs[n | 1].next_out;
        } else {
          _nodes[_arcs[n].target].first_out = _arcs[n | 1].next_out;
        }

        _arcs[n].next_out = _i_first_free_arc;
        _i_first_free_arc = n;
        _arcs[n].prev_out = -2;
        _arcs[n | 1].prev_out = -2;
        _i_num_arc -= 2;
      }

      void copyFrom(const ListGraphBase& other) noexcept {
        _nodes.copyFrom(other._nodes);
        _i_first_node = other._i_first_node;
        _i_first_free_node = other._i_first_free_node;

        _arcs.copyFrom(other._arcs);
        _i_first_free_arc = other._i_first_free_arc;
        _i_num_node = other._i_num_node;
        _i_num_arc = other._i_num_arc;
      }

      void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
        _i_first_node = _i_first_free_node = _i_first_free_arc = -1;
        _i_num_arc = _i_num_node = 0;
      }

      protected:
      constexpr void changeV(Edge e, Node n) noexcept {
        if (_arcs[2 * e._id].next_out != -1) { _arcs[_arcs[2 * e._id].next_out].prev_out = _arcs[2 * e._id].prev_out; }
        if (_arcs[2 * e._id].prev_out != -1) {
          _arcs[_arcs[2 * e._id].prev_out].next_out = _arcs[2 * e._id].next_out;
        } else {
          _nodes[_arcs[(2 * e._id) | 1].target].first_out = _arcs[2 * e._id].next_out;
        }

        if (_nodes[n._id].first_out != -1) { _arcs[_nodes[n._id].first_out].prev_out = 2 * e._id; }
        _arcs[(2 * e._id) | 1].target = n._id;
        _arcs[2 * e._id].prev_out = -1;
        _arcs[2 * e._id].next_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = 2 * e._id;
      }

      constexpr void changeU(Edge e, Node n) noexcept {
        if (_arcs[(2 * e._id) | 1].next_out != -1) {
          _arcs[_arcs[(2 * e._id) | 1].next_out].prev_out = _arcs[(2 * e._id) | 1].prev_out;
        }
        if (_arcs[(2 * e._id) | 1].prev_out != -1) {
          _arcs[_arcs[(2 * e._id) | 1].prev_out].next_out = _arcs[(2 * e._id) | 1].next_out;
        } else {
          _nodes[_arcs[2 * e._id].target].first_out = _arcs[(2 * e._id) | 1].next_out;
        }

        if (_nodes[n._id].first_out != -1) { _arcs[_nodes[n._id].first_out].prev_out = ((2 * e._id) | 1); }
        _arcs[2 * e._id].target = n._id;
        _arcs[(2 * e._id) | 1].prev_out = -1;
        _arcs[(2 * e._id) | 1].next_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = ((2 * e._id) | 1);
      }
    };

    using ExtendedListGraphBase = GraphExtender< ListGraphBase >;

    /**
     * \addtogroup graphs
     * @{
     */

    /** A general undirected graph structure. */

    /**
     * @ref ListGraph is a versatile and fast undirected graph
     * implementation based on linked lists that are stored in
     * \c nt::DynamicArray structures.
     *
     * This type fully conforms to the @ref concepts::Graph "Graph concept"
     * and it also provides several useful additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides only linear time counting for nodes, edges and arcs.
     *
     * \sa concepts::Graph
     * \sa ListDigraph
     */
    class ListGraph: public ExtendedListGraphBase {
      using Parent = ExtendedListGraphBase;

      public:
      /**
       * @brief Move constructor.
       */
      ListGraph(ListGraph&& g) noexcept : Parent(std::move(g)) {}

      /**
       * @brief Move assignment operator.
       *
       * @return Reference to this graph
       */
      ListGraph& operator=(ListGraph&& g) noexcept {
        Parent::operator=(std::move(g));
        return *this;
      }

      /**
       * Graphs are \e not copy constructible. Use GraphCopy instead.
       */
      ListGraph(const ListGraph&) = delete;
      /**
       * @brief Assignment of a graph to another one is \e not allowed.
       * Use GraphCopy instead.
       */
      void operator=(const ListGraph&) = delete;

      /**
       * Constructor.
       *
       */
      ListGraph() = default;

      using IncEdgeIt = Parent::OutArcIt;

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
       * @brief Erase a node from the graph.
       *
       * This function erases the given node along with its incident arcs
       * from the graph.
       *
       * @note All iterators referencing the removed node or the incident
       * edges are invalidated, of course.
       */
      void erase(Node n) { Parent::erase(n); }

      /**
       * @brief Erase an edge from the graph.
       *
       * This function erases the given edge from the graph.
       *
       * @note All iterators referencing the removed edge are invalidated,
       * of course.
       */
      void erase(Edge e) { Parent::erase(e); }
      /** Node validity check */

      /**
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the graph.
       *
       * @warning A removed node could become valid again if new nodes are
       * added to the graph.
       */
      bool valid(Node n) const { return Parent::valid(n); }
      /** Edge validity check */

      /**
       * This function gives back \c true if the given edge is valid,
       * i.e. it is a real edge of the graph.
       *
       * @warning A removed edge could become valid again if new edges are
       * added to the graph.
       */
      bool valid(Edge e) const { return Parent::valid(e); }
      /** Arc validity check */

      /**
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the graph.
       *
       * @warning A removed arc could become valid again if new edges are
       * added to the graph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /**
       * @brief Change the first node of an edge.
       *
       * This function changes the first node of the given edge \c e to \c n.
       *
       * @note \c EdgeIt and \c ArcIt iterators referencing the
       * changed edge are invalidated and all other iterators whose
       * base node is the changed node are also invalidated.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      void changeU(Edge e, Node n) { Parent::changeU(e, n); }
      /**
       * @brief Change the second node of an edge.
       *
       * This function changes the second node of the given edge \c e to \c n.
       *
       * @note \c EdgeIt iterators referencing the changed edge remain
       * valid, but \c ArcIt iterators referencing the changed edge and
       * all other iterators whose base node is the changed node are also
       * invalidated.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      void changeV(Edge e, Node n) { Parent::changeV(e, n); }

      /**
       * @brief Contract two nodes.
       *
       * This function contracts the given two nodes.
       * Node \c b is removed, but instead of deleting
       * its incident edges, they are joined to node \c a.
       * If the last parameter \c r is \c true (this is the default value),
       * then the newly created loops are removed.
       *
       * @note The moved edges are joined to node \c a using changeU()
       * or changeV(), thus all edge and arc iterators whose base node is
       * \c b are invalidated.
       * Moreover all iterators referencing node \c b or the removed
       * loops are also invalidated. Other iterators remain valid.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      void contract(Node a, Node b, bool r = true) {
        for (IncEdgeIt e(*this, b); e != INVALID;) {
          IncEdgeIt f = e;
          ++f;
          if (r && runningNode(e) == a) {
            erase(e);
          } else if (u(e) == b) {
            changeU(e, a);
          } else {
            changeV(e, a);
          }
          e = f;
        }
        erase(b);
      }

      /** Clear the graph. */

      /**
       * This function erases all nodes and arcs from the graph.
       *
       * @note All iterators of the graph are invalidated, of course.
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
       * @brief Copy the content of another graph.
       *
       */
      void copyFrom(const ListGraph& other) noexcept { Parent::copyFrom(other); }

      /**
       * @brief Class to make a snapshot of the graph and restore
       * it later.
       *
       * Class to make a snapshot of the graph and restore it later.
       *
       * The newly added nodes and edges can be removed
       * using the restore() function.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and edges again using
       * another Snapshot instance.
       *
       * @warning Node and edge deletions and other modifications
       * (e.g. changing the end-nodes of edges or contracting nodes)
       * cannot be restored. These events invalidate the snapshot.
       * However, the edges and nodes that were added to the graph after
       * making the current snapshot can be removed without invalidating it.
       */
      class Snapshot {
        protected:
        using NodeNotifier = Parent::NodeNotifier;

        class NodeObserverProxy: public NodeNotifier::ObserverBase {
          public:
          NodeObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using NodeNotifier::ObserverBase::attach;
          using NodeNotifier::ObserverBase::attached;
          using NodeNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Node& node) noexcept override {
            snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = nodes.size() - 1; i >= 0; ++i)
              snapshot.addNode(nodes[i]);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Node& node) noexcept override {
            return snapshot.eraseNode(node);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = 0; i < int(nodes.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseNode(nodes[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node))
              snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node)) {
              const ObserverReturnCode r = snapshot.eraseNode(node);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        class EdgeObserverProxy: public EdgeNotifier::ObserverBase {
          public:
          EdgeObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using EdgeNotifier::ObserverBase::attach;
          using EdgeNotifier::ObserverBase::attached;
          using EdgeNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Edge& edge) noexcept override {
            snapshot.addEdge(edge);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Edge >& edges) noexcept override {
            for (int i = edges.size() - 1; i >= 0; ++i)
              snapshot.addEdge(edges[i]);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Edge& edge) noexcept override {
            return snapshot.eraseEdge(edge);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Edge >& edges) noexcept override {
            for (int i = 0; i < int(edges.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseEdge(edges[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Edge edge;
            for (notifier()->first(edge); edge != INVALID; notifier()->next(edge))
              snapshot.addEdge(edge);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Edge edge;
            for (notifier()->first(edge); edge != INVALID; notifier()->next(edge)) {
              const ObserverReturnCode r = snapshot.eraseEdge(edge);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        ListGraph* graph;

        NodeObserverProxy node_observer_proxy;
        EdgeObserverProxy edge_observer_proxy;

        nt::TrivialDynamicArray< Node > added_nodes;
        nt::TrivialDynamicArray< Edge > added_edges;

        constexpr void               addNode(const Node& node) noexcept { added_nodes.push_back(node); }
        constexpr ObserverReturnCode eraseNode(const Node& node) noexcept {
          const int i = added_nodes.find(node);
          if (i == added_nodes.size()) {
            clear();
            edge_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_nodes.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void               addEdge(const Edge& edge) noexcept { added_edges.push_back(edge); }
        constexpr ObserverReturnCode eraseEdge(const Edge& edge) noexcept {
          const int i = added_edges.find(edge);
          if (i == added_edges.size()) {
            clear();
            node_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_edges.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void attach(ListGraph& _graph) noexcept {
          graph = &_graph;
          node_observer_proxy.attach(graph->notifier(Node()));
          edge_observer_proxy.attach(graph->notifier(Edge()));
        }

        constexpr void detach() noexcept {
          node_observer_proxy.detach();
          edge_observer_proxy.detach();
        }

        constexpr bool attached() const noexcept { return node_observer_proxy.attached(); }

        constexpr void clear() noexcept {
          added_nodes.clear();
          added_edges.clear();
        }

        public:
        /**
         * @brief Default constructor.
         *
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : graph(0), node_observer_proxy(*this), edge_observer_proxy(*this) {}

        /**
         * @brief Constructor that immediately makes a snapshot.
         *
         * This constructor immediately makes a snapshot of the given graph.
         */
        Snapshot(ListGraph& gr) : node_observer_proxy(*this), edge_observer_proxy(*this) { attach(gr); }

        /**
         * @brief Make a snapshot.
         *
         * This function makes a snapshot of the given graph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        constexpr void save(ListGraph& gr) noexcept {
          if (attached()) {
            detach();
            clear();
          }
          attach(gr);
        }

        /**
         * @brief Undo the changes until the last snapshot.
         *
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(ListGraph&).
         *
         * @warning This method invalidates the snapshot, i.e. repeated
         * restoring is not supported unless you call save() again.
         */
        constexpr void restore() noexcept {
          detach();
          for (const Edge e: added_edges)
            graph->erase(e);
          for (const Node v: added_nodes)
            graph->erase(v);
          clear();
        }

        /**
         * @brief Returns \c true if the snapshot is valid.
         *
         * This function returns \c true if the snapshot is valid.
         */
        constexpr bool valid() const noexcept { return attached(); }
      };
    };

    /** @} */

    class ListBpGraphBase {
      using NodeNumTag = nt::True;
      using ArcNumTag = nt::True;
      using EdgeNumTag = nt::True;

      protected:
      struct NodeT {
        int  first_out;
        int  prev, next;
        int  partition_prev, partition_next;
        int  partition_index;
        bool red;
      };

      struct ArcT {
        int target;
        int prev_out, next_out;
      };

      nt::TrivialDynamicArray< NodeT > _nodes;
      int                              _i_first_node, _i_first_red, _i_first_blue;
      int                              _i_max_red, _i_max_blue;

      int _i_first_free_red, _i_first_free_blue;

      nt::TrivialDynamicArray< ArcT > _arcs;
      int                             _i_first_free_arc;

      int _i_num_blue;
      int _i_num_node;
      int _i_num_arc;

      public:
      using BpGraph = ListBpGraphBase;

      class Node {
        friend class ListBpGraphBase;

        protected:
        int _id;
        constexpr explicit Node(int pid) { _id = pid; }

        public:
        Node() = default;
        constexpr Node(Invalid) { _id = -1; }
        constexpr bool operator==(const Node& node) const noexcept { return _id == node._id; }
        constexpr bool operator!=(const Node& node) const noexcept { return _id != node._id; }
        constexpr bool operator<(const Node& node) const noexcept { return _id < node._id; }
      };

      class RedNode: public Node {
        friend class ListBpGraphBase;

        protected:
        constexpr explicit RedNode(int pid) : Node(pid) {}

        public:
        RedNode() = default;
        constexpr RedNode(const RedNode& node) : Node(node) {}
        constexpr RedNode(Invalid) : Node(INVALID) {}
      };

      class BlueNode: public Node {
        friend class ListBpGraphBase;

        protected:
        constexpr explicit BlueNode(int pid) : Node(pid) {}

        public:
        BlueNode() = default;
        constexpr BlueNode(const BlueNode& node) : Node(node) {}
        constexpr BlueNode(Invalid) : Node(INVALID) {}
      };

      class Edge {
        friend class ListBpGraphBase;

        protected:
        int _id;
        constexpr explicit Edge(int pid) { _id = pid; }

        public:
        Edge() = default;
        constexpr Edge(Invalid) { _id = -1; }
        constexpr bool operator==(const Edge& edge) const noexcept { return _id == edge._id; }
        constexpr bool operator!=(const Edge& edge) const noexcept { return _id != edge._id; }
        constexpr bool operator<(const Edge& edge) const noexcept { return _id < edge._id; }
      };

      class Arc {
        friend class ListBpGraphBase;

        protected:
        int _id;
        constexpr explicit Arc(int pid) { _id = pid; }

        public:
        operator Edge() const { return _id != -1 ? edgeFromId(_id / 2) : INVALID; }

        Arc() = default;
        constexpr Arc(Invalid) { _id = -1; }
        constexpr bool operator==(const Arc& arc) const noexcept { return _id == arc._id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return _id != arc._id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return _id < arc._id; }
      };

      ListBpGraphBase() :
          _nodes(), _i_first_node(-1), _i_first_red(-1), _i_first_blue(-1), _i_max_red(-1), _i_max_blue(-1),
          _i_first_free_red(-1), _i_first_free_blue(-1), _arcs(), _i_first_free_arc(-1), _i_num_blue(0), _i_num_node(0),
          _i_num_arc(0) {}

      ListBpGraphBase(ListBpGraphBase&& g) noexcept :
          _nodes(std::move(g._nodes)), _i_first_node(g._i_first_node), _i_first_red(g._i_first_red),
          _i_first_blue(g._i_first_blue), _i_max_red(g._i_max_red), _i_max_blue(g._i_max_blue),
          _i_first_free_red(g._i_first_free_red), _i_first_free_blue(g._i_first_free_blue), _arcs(std::move(g._arcs)),
          _i_first_free_arc(g._i_first_free_arc), _i_num_blue(g._i_num_blue), _i_num_node(g._i_num_node),
          _i_num_arc(g._i_num_arc) {
        // Reset source to empty state
        g._i_first_node = -1;
        g._i_first_red = -1;
        g._i_first_blue = -1;
        g._i_max_red = -1;
        g._i_max_blue = -1;
        g._i_first_free_red = -1;
        g._i_first_free_blue = -1;
        g._i_first_free_arc = -1;
        g._i_num_blue = 0;
        g._i_num_node = 0;
        g._i_num_arc = 0;
      }

      ListBpGraphBase& operator=(ListBpGraphBase&& g) noexcept {
        if (this != &g) {
          _nodes = std::move(g._nodes);
          _i_first_node = g._i_first_node;
          _i_first_red = g._i_first_red;
          _i_first_blue = g._i_first_blue;
          _i_max_red = g._i_max_red;
          _i_max_blue = g._i_max_blue;
          _i_first_free_red = g._i_first_free_red;
          _i_first_free_blue = g._i_first_free_blue;
          _arcs = std::move(g._arcs);
          _i_first_free_arc = g._i_first_free_arc;
          _i_num_blue = g._i_num_blue;
          _i_num_node = g._i_num_node;
          _i_num_arc = g._i_num_arc;

          // Reset source to empty state
          g._i_first_node = -1;
          g._i_first_red = -1;
          g._i_first_blue = -1;
          g._i_max_red = -1;
          g._i_max_blue = -1;
          g._i_first_free_red = -1;
          g._i_first_free_blue = -1;
          g._i_first_free_arc = -1;
          g._i_num_blue = 0;
          g._i_num_node = 0;
          g._i_num_arc = 0;
        }
        return *this;
      }

      constexpr bool red(Node n) const noexcept { return _nodes[n._id].red; }
      constexpr bool blue(Node n) const noexcept { return !_nodes[n._id].red; }

      constexpr static RedNode  asRedNodeUnsafe(Node n) noexcept { return RedNode(n._id); }
      constexpr static BlueNode asBlueNodeUnsafe(Node n) noexcept { return BlueNode(n._id); }

      constexpr int nodeNum() const noexcept { return _i_num_node; }
      constexpr int redNum() const noexcept { return _i_num_node - _i_num_blue; }
      constexpr int blueNum() const noexcept { return _i_num_blue; }
      constexpr int edgeNum() const noexcept { return _i_num_arc / 2; }
      constexpr int arcNum() const noexcept { return _i_num_arc; }

      constexpr int maxNodeId() const noexcept { return _nodes.size() - 1; }
      constexpr int maxRedId() const noexcept { return _i_max_red; }
      constexpr int maxBlueId() const noexcept { return _i_max_blue; }
      constexpr int maxEdgeId() const noexcept { return _arcs.size() / 2 - 1; }
      constexpr int maxArcId() const noexcept { return _arcs.size() - 1; }

      constexpr Node source(Arc e) const noexcept { return Node(_arcs[e._id ^ 1].target); }
      constexpr Node target(Arc e) const noexcept { return Node(_arcs[e._id].target); }

      constexpr RedNode  redNode(Edge e) const noexcept { return RedNode(_arcs[2 * e._id].target); }
      constexpr BlueNode blueNode(Edge e) const noexcept { return BlueNode(_arcs[2 * e._id + 1].target); }

      constexpr static bool direction(Arc e) { return (e._id & 1) == 1; }

      constexpr static Arc direct(Edge e, bool d) { return Arc(e._id * 2 + (d ? 1 : 0)); }

      constexpr void first(Node& node) const noexcept { node._id = _i_first_node; }

      constexpr void next(Node& node) const noexcept { node._id = _nodes[node._id].next; }

      constexpr void first(RedNode& node) const noexcept { node._id = _i_first_red; }

      constexpr void next(RedNode& node) const noexcept { node._id = _nodes[node._id].partition_next; }

      constexpr void first(BlueNode& node) const noexcept { node._id = _i_first_blue; }

      constexpr void next(BlueNode& node) const noexcept { node._id = _nodes[node._id].partition_next; }

      constexpr void first(Arc& e) const noexcept {
        int n = _i_first_node;
        while (n != -1 && _nodes[n].first_out == -1) {
          n = _nodes[n].next;
        }
        e._id = (n == -1) ? -1 : _nodes[n].first_out;
      }

      constexpr void next(Arc& e) const noexcept {
        if (_arcs[e._id].next_out != -1) {
          e._id = _arcs[e._id].next_out;
        } else {
          int n = _nodes[_arcs[e._id ^ 1].target].next;
          while (n != -1 && _nodes[n].first_out == -1) {
            n = _nodes[n].next;
          }
          e._id = (n == -1) ? -1 : _nodes[n].first_out;
        }
      }

      constexpr void first(Edge& e) const noexcept {
        int n = _i_first_node;
        while (n != -1) {
          e._id = _nodes[n].first_out;
          while ((e._id & 1) != 1) {
            e._id = _arcs[e._id].next_out;
          }
          if (e._id != -1) {
            e._id /= 2;
            return;
          }
          n = _nodes[n].next;
        }
        e._id = -1;
      }

      constexpr void next(Edge& e) const noexcept {
        int n = _arcs[e._id * 2].target;
        e._id = _arcs[(e._id * 2) | 1].next_out;
        while ((e._id & 1) != 1) {
          e._id = _arcs[e._id].next_out;
        }
        if (e._id != -1) {
          e._id /= 2;
          return;
        }
        n = _nodes[n].next;
        while (n != -1) {
          e._id = _nodes[n].first_out;
          while ((e._id & 1) != 1) {
            e._id = _arcs[e._id].next_out;
          }
          if (e._id != -1) {
            e._id /= 2;
            return;
          }
          n = _nodes[n].next;
        }
        e._id = -1;
      }

      constexpr void firstOut(Arc& e, const Node& v) const noexcept { e._id = _nodes[v._id].first_out; }
      constexpr void nextOut(Arc& e) const noexcept { e._id = _arcs[e._id].next_out; }

      constexpr void firstIn(Arc& e, const Node& v) const noexcept {
        e._id = ((_nodes[v._id].first_out) ^ 1);
        if (e._id == -2) e._id = -1;
      }
      constexpr void nextIn(Arc& e) const noexcept {
        e._id = ((_arcs[e._id ^ 1].next_out) ^ 1);
        if (e._id == -2) e._id = -1;
      }

      constexpr void firstInc(Edge& e, bool& d, const Node& v) const noexcept {
        int a = _nodes[v._id].first_out;
        if (a != -1) {
          e._id = a / 2;
          d = ((a & 1) == 1);
        } else {
          e._id = -1;
          d = true;
        }
      }
      constexpr void nextInc(Edge& e, bool& d) const noexcept {
        int a = (_arcs[(e._id * 2) | (d ? 1 : 0)].next_out);
        if (a != -1) {
          e._id = a / 2;
          d = ((a & 1) == 1);
        } else {
          e._id = -1;
          d = true;
        }
      }

      constexpr static int id(Node v) { return v._id; }
      constexpr int        id(RedNode v) const noexcept { return _nodes[v._id].partition_index; }
      constexpr int        id(BlueNode v) const noexcept { return _nodes[v._id].partition_index; }
      constexpr static int id(Arc e) { return e._id; }
      constexpr static int id(Edge e) { return e._id; }

      constexpr static Node nodeFromId(int id) noexcept { return Node(id); }
      constexpr static Arc  arcFromId(int id) noexcept { return Arc(id); }
      constexpr static Edge edgeFromId(int id) noexcept { return Edge(id); }

      constexpr bool valid(Node n) const noexcept {
        return n._id >= 0 && n._id < static_cast< int >(_nodes.size()) && _nodes[n._id].prev != -2;
      }
      constexpr bool valid(Arc a) const noexcept {
        return a._id >= 0 && a._id < static_cast< int >(_arcs.size()) && _arcs[a._id].prev_out != -2;
      }
      constexpr bool valid(Edge e) const noexcept {
        return e._id >= 0 && 2 * e._id < static_cast< int >(_arcs.size()) && _arcs[2 * e._id].prev_out != -2;
      }

      RedNode addRedNode() noexcept {
        int n;

        if (_i_first_free_red == -1) {
          n = _nodes.add();
          _nodes[n].partition_index = ++_i_max_red;
          _nodes[n].red = true;
        } else {
          n = _i_first_free_red;
          _i_first_free_red = _nodes[n].next;
        }

        _nodes[n].next = _i_first_node;
        if (_i_first_node != -1) _nodes[_i_first_node].prev = n;
        _i_first_node = n;
        _nodes[n].prev = -1;

        _nodes[n].partition_next = _i_first_red;
        if (_i_first_red != -1) _nodes[_i_first_red].partition_prev = n;
        _i_first_red = n;
        _nodes[n].partition_prev = -1;

        _nodes[n].first_out = -1;

        ++_i_num_node;
        return RedNode(n);
      }

      BlueNode addBlueNode() noexcept {
        int n;

        if (_i_first_free_blue == -1) {
          n = _nodes.add();
          _nodes[n].partition_index = ++_i_max_blue;
          _nodes[n].red = false;
        } else {
          n = _i_first_free_blue;
          _i_first_free_blue = _nodes[n].next;
        }

        _nodes[n].next = _i_first_node;
        if (_i_first_node != -1) _nodes[_i_first_node].prev = n;
        _i_first_node = n;
        _nodes[n].prev = -1;

        _nodes[n].partition_next = _i_first_blue;
        if (_i_first_blue != -1) _nodes[_i_first_blue].partition_prev = n;
        _i_first_blue = n;
        _nodes[n].partition_prev = -1;

        _nodes[n].first_out = -1;

        ++_i_num_blue;
        ++_i_num_node;
        return BlueNode(n);
      }

      Edge addEdge(Node u, Node v) noexcept {
        int n;

        if (_i_first_free_arc == -1) {
          n = _arcs.add();
          _arcs.add();
        } else {
          n = _i_first_free_arc;
          _i_first_free_arc = _arcs[n].next_out;
        }

        _arcs[n].target = u._id;
        _arcs[n | 1].target = v._id;

        _arcs[n].next_out = _nodes[v._id].first_out;
        if (_nodes[v._id].first_out != -1) { _arcs[_nodes[v._id].first_out].prev_out = n; }
        _arcs[n].prev_out = -1;
        _nodes[v._id].first_out = n;

        _arcs[n | 1].next_out = _nodes[u._id].first_out;
        if (_nodes[u._id].first_out != -1) { _arcs[_nodes[u._id].first_out].prev_out = (n | 1); }
        _arcs[n | 1].prev_out = -1;
        _nodes[u._id].first_out = (n | 1);

        _i_num_arc += 2;
        return Edge(n / 2);
      }

      constexpr void erase(const Node& node) noexcept {
        int n = node._id;

        if (_nodes[n].next != -1) { _nodes[_nodes[n].next].prev = _nodes[n].prev; }

        if (_nodes[n].prev != -1) {
          _nodes[_nodes[n].prev].next = _nodes[n].next;
        } else {
          _i_first_node = _nodes[n].next;
        }

        if (_nodes[n].partition_next != -1) {
          _nodes[_nodes[n].partition_next].partition_prev = _nodes[n].partition_prev;
        }

        if (_nodes[n].partition_prev != -1) {
          _nodes[_nodes[n].partition_prev].partition_next = _nodes[n].partition_next;
        } else {
          if (_nodes[n].red) {
            _i_first_red = _nodes[n].partition_next;
          } else {
            _i_first_blue = _nodes[n].partition_next;
          }
        }

        if (_nodes[n].red) {
          _nodes[n].next = _i_first_free_red;
          _i_first_free_red = n;
        } else {
          _nodes[n].next = _i_first_free_blue;
          _i_first_free_blue = n;
          --_i_num_blue;
        }

        --_i_num_node;
        _nodes[n].prev = -2;
      }

      constexpr void erase(const Edge& edge) noexcept {
        int n = edge._id * 2;

        if (_arcs[n].next_out != -1) { _arcs[_arcs[n].next_out].prev_out = _arcs[n].prev_out; }

        if (_arcs[n].prev_out != -1) {
          _arcs[_arcs[n].prev_out].next_out = _arcs[n].next_out;
        } else {
          _nodes[_arcs[n | 1].target].first_out = _arcs[n].next_out;
        }

        if (_arcs[n | 1].next_out != -1) { _arcs[_arcs[n | 1].next_out].prev_out = _arcs[n | 1].prev_out; }

        if (_arcs[n | 1].prev_out != -1) {
          _arcs[_arcs[n | 1].prev_out].next_out = _arcs[n | 1].next_out;
        } else {
          _nodes[_arcs[n].target].first_out = _arcs[n | 1].next_out;
        }

        _arcs[n].next_out = _i_first_free_arc;
        _i_first_free_arc = n;
        _arcs[n].prev_out = -2;
        _arcs[n | 1].prev_out = -2;

        _i_num_arc -= 2;
      }

      void copyFrom(const ListBpGraphBase& other) noexcept {
        _nodes.copyFrom(other._nodes);
        _i_first_node = other._i_first_node;
        _i_first_red = other._i_first_red;
        _i_first_blue = other._i_first_blue;
        _i_max_red = other._i_max_red;
        _i_max_blue = other._i_max_blue;
        _i_first_free_red = other._i_first_free_red;
        _i_first_free_blue = other._i_first_free_blue;

        _arcs.copyFrom(other._arcs);
        _i_first_free_arc = other._i_first_free_arc;
        _i_num_blue = other._i_num_blue;
        _i_num_node = other._i_num_node;
        _i_num_arc = other._i_num_arc;
      }

      void clear() noexcept {
        _arcs.clear();
        _nodes.clear();
        _i_first_node = _i_first_free_arc = _i_first_red = _i_first_blue = _i_max_red = _i_max_blue =
           _i_first_free_red = _i_first_free_blue = -1;
        _i_num_arc = _i_num_node = _i_num_blue = 0;
      }

      protected:
      constexpr void changeRed(Edge e, RedNode n) noexcept {
        if (_arcs[(2 * e._id) | 1].next_out != -1) {
          _arcs[_arcs[(2 * e._id) | 1].next_out].prev_out = _arcs[(2 * e._id) | 1].prev_out;
        }
        if (_arcs[(2 * e._id) | 1].prev_out != -1) {
          _arcs[_arcs[(2 * e._id) | 1].prev_out].next_out = _arcs[(2 * e._id) | 1].next_out;
        } else {
          _nodes[_arcs[2 * e._id].target].first_out = _arcs[(2 * e._id) | 1].next_out;
        }

        if (_nodes[n._id].first_out != -1) { _arcs[_nodes[n._id].first_out].prev_out = ((2 * e._id) | 1); }
        _arcs[2 * e._id].target = n._id;
        _arcs[(2 * e._id) | 1].prev_out = -1;
        _arcs[(2 * e._id) | 1].next_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = ((2 * e._id) | 1);
      }

      constexpr void changeBlue(Edge e, BlueNode n) noexcept {
        if (_arcs[2 * e._id].next_out != -1) { _arcs[_arcs[2 * e._id].next_out].prev_out = _arcs[2 * e._id].prev_out; }
        if (_arcs[2 * e._id].prev_out != -1) {
          _arcs[_arcs[2 * e._id].prev_out].next_out = _arcs[2 * e._id].next_out;
        } else {
          _nodes[_arcs[(2 * e._id) | 1].target].first_out = _arcs[2 * e._id].next_out;
        }

        if (_nodes[n._id].first_out != -1) { _arcs[_nodes[n._id].first_out].prev_out = 2 * e._id; }
        _arcs[(2 * e._id) | 1].target = n._id;
        _arcs[2 * e._id].prev_out = -1;
        _arcs[2 * e._id].next_out = _nodes[n._id].first_out;
        _nodes[n._id].first_out = 2 * e._id;
      }
    };

    using ExtendedListBpGraphBase = BpGraphExtender< ListBpGraphBase >;

    /**
     * \addtogroup graphs
     * @{
     */

    /** A general undirected graph structure. */

    /**
     * @ref ListBpGraph is a versatile and fast undirected graph
     * implementation based on linked lists that are stored in
     * \c nt::DynamicArray structures.
     *
     * This type fully conforms to the @ref concepts::BpGraph "BpGraph concept"
     * and it also provides several useful additional functionalities.
     * Most of its member functions and nested classes are documented
     * only in the concept class.
     *
     * This class provides only linear time counting for nodes, edges and arcs.
     *
     * \sa concepts::BpGraph
     * \sa ListDigraph
     */
    class ListBpGraph: public ExtendedListBpGraphBase {
      using Parent = ExtendedListBpGraphBase;

      public:
      /**
       * @brief Move constructor.
       */
      ListBpGraph(ListBpGraph&& g) noexcept : Parent(std::move(g)) {}

      /**
       * @brief Move assignment operator.
       *
       * @return Reference to this bipartite graph
       */
      ListBpGraph& operator=(ListBpGraph&& g) noexcept {
        Parent::operator=(std::move(g));
        return *this;
      }

      /**
       * BpGraphs are \e not copy constructible. Use BpGraphCopy instead.
       */
      ListBpGraph(const ListBpGraph&) = delete;
      /**
       * @brief Assignment of a graph to another one is \e not allowed.
       * Use BpGraphCopy instead.
       */
      void operator=(const ListBpGraph&) = delete;

      /**
       * Constructor.
       *
       */
      ListBpGraph() = default;

      using IncEdgeIt = Parent::OutArcIt;

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
       * @brief Erase a node from the graph.
       *
       * This function erases the given node along with its incident arcs
       * from the graph.
       *
       * @note All iterators referencing the removed node or the incident
       * edges are invalidated, of course.
       */
      void erase(Node n) { Parent::erase(n); }

      /**
       * @brief Erase an edge from the graph.
       *
       * This function erases the given edge from the graph.
       *
       * @note All iterators referencing the removed edge are invalidated,
       * of course.
       */
      void erase(Edge e) { Parent::erase(e); }
      /** Node validity check */

      /**
       * This function gives back \c true if the given node is valid,
       * i.e. it is a real node of the graph.
       *
       * @warning A removed node could become valid again if new nodes are
       * added to the graph.
       */
      bool valid(Node n) const { return Parent::valid(n); }
      /** Edge validity check */

      /**
       * This function gives back \c true if the given edge is valid,
       * i.e. it is a real edge of the graph.
       *
       * @warning A removed edge could become valid again if new edges are
       * added to the graph.
       */
      bool valid(Edge e) const { return Parent::valid(e); }
      /** Arc validity check */

      /**
       * This function gives back \c true if the given arc is valid,
       * i.e. it is a real arc of the graph.
       *
       * @warning A removed arc could become valid again if new edges are
       * added to the graph.
       */
      bool valid(Arc a) const { return Parent::valid(a); }

      /**
       * @brief Change the red node of an edge.
       *
       * This function changes the red node of the given edge \c e to \c n.
       *
       * @note \c EdgeIt and \c ArcIt iterators referencing the
       * changed edge are invalidated and all other iterators whose
       * base node is the changed node are also invalidated.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      void changeRed(Edge e, RedNode n) { Parent::changeRed(e, n); }
      /**
       * @brief Change the blue node of an edge.
       *
       * This function changes the blue node of the given edge \c e to \c n.
       *
       * @note \c EdgeIt iterators referencing the changed edge remain
       * valid, but \c ArcIt iterators referencing the changed edge and
       * all other iterators whose base node is the changed node are also
       * invalidated.
       *
       * @warning This functionality cannot be used together with the
       * Snapshot feature.
       */
      void changeBlue(Edge e, BlueNode n) { Parent::changeBlue(e, n); }

      /** Clear the graph. */

      /**
       * This function erases all nodes and arcs from the graph.
       *
       * @note All iterators of the graph are invalidated, of course.
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
       * @brief Copy the content of another bipartite graph.
       *
       */
      void copyFrom(const ListBpGraph& other) noexcept { Parent::copyFrom(other); }

      /**
       * @brief Class to make a snapshot of the graph and restore
       * it later.
       *
       * Class to make a snapshot of the graph and restore it later.
       *
       * The newly added nodes and edges can be removed
       * using the restore() function.
       *
       * @note After a state is restored, you cannot restore a later state,
       * i.e. you cannot add the removed nodes and edges again using
       * another Snapshot instance.
       *
       * @warning Node and edge deletions and other modifications
       * (e.g. changing the end-nodes of edges or contracting nodes)
       * cannot be restored. These events invalidate the snapshot.
       * However, the edges and nodes that were added to the graph after
       * making the current snapshot can be removed without invalidating it.
       */
      class Snapshot {
        protected:
        using NodeNotifier = Parent::NodeNotifier;

        class NodeObserverProxy: public NodeNotifier::ObserverBase {
          public:
          NodeObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using NodeNotifier::ObserverBase::attach;
          using NodeNotifier::ObserverBase::attached;
          using NodeNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Node& node) noexcept override {
            snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = nodes.size() - 1; i >= 0; ++i)
              snapshot.addNode(nodes[i]);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Node& node) noexcept override {
            return snapshot.eraseNode(node);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
            for (int i = 0; i < int(nodes.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseNode(nodes[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node))
              snapshot.addNode(node);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Node node;
            for (notifier()->first(node); node != INVALID; notifier()->next(node)) {
              const ObserverReturnCode r = snapshot.eraseNode(node);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        class EdgeObserverProxy: public EdgeNotifier::ObserverBase {
          public:
          EdgeObserverProxy(Snapshot& _snapshot) : snapshot(_snapshot) {}

          using EdgeNotifier::ObserverBase::attach;
          using EdgeNotifier::ObserverBase::attached;
          using EdgeNotifier::ObserverBase::detach;

          protected:
          constexpr virtual ObserverReturnCode add(const Edge& edge) noexcept override {
            snapshot.addEdge(edge);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode add(const nt::ArrayView< Edge >& edges) noexcept override {
            for (int i = edges.size() - 1; i >= 0; ++i)
              snapshot.addEdge(edges[i]);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode erase(const Edge& edge) noexcept override {
            return snapshot.eraseEdge(edge);
          }
          constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Edge >& edges) noexcept override {
            for (int i = 0; i < int(edges.size()); ++i) {
              const ObserverReturnCode r = snapshot.eraseEdge(edges[i]);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode build() noexcept override {
            Edge edge;
            for (notifier()->first(edge); edge != INVALID; notifier()->next(edge))
              snapshot.addEdge(edge);
            return ObserverReturnCode::Ok;
          }
          constexpr virtual ObserverReturnCode clear() noexcept override {
            Edge edge;
            for (notifier()->first(edge); edge != INVALID; notifier()->next(edge)) {
              const ObserverReturnCode r = snapshot.eraseEdge(edge);
              if (r != ObserverReturnCode::Ok) return r;
            }
            return ObserverReturnCode::Ok;
          }

          constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
            // assert(false);
            return ObserverReturnCode::Ok;
          }

          Snapshot& snapshot;
        };

        ListBpGraph* graph;

        NodeObserverProxy node_observer_proxy;
        EdgeObserverProxy edge_observer_proxy;

        nt::TrivialDynamicArray< Node > added_nodes;
        nt::TrivialDynamicArray< Edge > added_edges;

        constexpr void               addNode(const Node& node) noexcept { added_nodes.push_back(node); }
        constexpr ObserverReturnCode eraseNode(const Node& node) noexcept {
          const int i = added_nodes.find(node);
          if (i == added_nodes.size()) {
            clear();
            edge_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_nodes.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void               addEdge(const Edge& edge) noexcept { added_edges.push_back(edge); }
        constexpr ObserverReturnCode eraseEdge(const Edge& edge) noexcept {
          const int i = added_edges.find(edge);
          if (i == added_edges.size()) {
            clear();
            node_observer_proxy.detach();
            assert(false);
            return ObserverReturnCode::ImmediateDetach;
          }
          added_edges.remove(i);
          return ObserverReturnCode::Ok;
        }

        constexpr void attach(ListBpGraph& _graph) noexcept {
          graph = &_graph;
          node_observer_proxy.attach(graph->notifier(Node()));
          edge_observer_proxy.attach(graph->notifier(Edge()));
        }

        constexpr void detach() noexcept {
          node_observer_proxy.detach();
          edge_observer_proxy.detach();
        }

        constexpr bool attached() const noexcept { return node_observer_proxy.attached(); }

        constexpr void clear() noexcept {
          added_nodes.clear();
          added_edges.clear();
        }

        public:
        /**
         * @brief Default constructor.
         *
         * Default constructor.
         * You have to call save() to actually make a snapshot.
         */
        Snapshot() : graph(0), node_observer_proxy(*this), edge_observer_proxy(*this) {}

        /**
         * @brief Constructor that immediately makes a snapshot.
         *
         * This constructor immediately makes a snapshot of the given graph.
         */
        Snapshot(ListBpGraph& gr) : node_observer_proxy(*this), edge_observer_proxy(*this) { attach(gr); }

        /**
         * @brief Make a snapshot.
         *
         * This function makes a snapshot of the given graph.
         * It can be called more than once. In case of a repeated
         * call, the previous snapshot gets lost.
         */
        constexpr void save(ListBpGraph& gr) noexcept {
          if (attached()) {
            detach();
            clear();
          }
          attach(gr);
        }

        /**
         * @brief Undo the changes until the last snapshot.
         *
         * This function undos the changes until the last snapshot
         * created by save() or Snapshot(ListBpGraph&).
         *
         * @warning This method invalidates the snapshot, i.e. repeated
         * restoring is not supported unless you call save() again.
         */
        constexpr void restore() noexcept {
          detach();
          for (const Edge e: added_edges)
            graph->erase(e);
          for (const Node v: added_nodes)
            graph->erase(v);
          clear();
        }

        /**
         * @brief Returns \c true if the snapshot is valid.
         *
         * This function returns \c true if the snapshot is valid.
         */
        constexpr bool valid() const noexcept { return attached(); }
      };
    };

    /** @} */
  }   // namespace graphs
}   // namespace nt

#endif
