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
 * This file incorporates work from the LEMON graph library (edge_set.h).
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
 *   - Replaced std::vector with nt::DynamicArray
 *   - Replaced LEMON_ASSERT with NT_ASSERT
 *   - Changed namespace from lemon to nt
 *   - The classes originally named SmarcArcSet, SmartEdgeSet,... in LEMON were renamed to
 *     SmarcArcGraph, SmartEdgeGraph, ... in order to
 *      (i) be consistent with the fact they are graphs and not sets, and to
 *      (ii) avoid the confusion with the classes NodeSet, ArcSet and EdgetSet defined in graph_element_set.h which are
 *           sets of nodes, arcs and edges.
 */

/**
 * @file
 * @brief ArcGraph and EdgeGraph classes.
 *
 * Graphs which use another graph's node-set as own.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_EDGE_GRAPH_H_
#define _NT_EDGE_GRAPH_H_

#include "../core/maps.h"
#include "bits/edge_graph_extender.h"

namespace nt {
  namespace graphs {

    template < typename GR >
    class ListArcGraphBase {
      public:
      using Node = typename GR::Node;
      using NodeIt = typename GR::NodeIt;

      protected:
      struct NodeT {
        int first_out, first_in;
        NodeT() : first_out(-1), first_in(-1) {}
      };

      using NodesImplBase = typename ItemSetTraits< GR, Node >::template Map< NodeT >::Type;

      NodesImplBase* _nodes;

      struct ArcT {
        Node source, target;
        int  next_out, next_in;
        int  prev_out, prev_in;
        ArcT() : prev_out(-1), prev_in(-1) {}
      };

      TrivialDynamicArray< ArcT > arcs;

      int first_arc;
      int first_free_arc;

      const GR* _graph;

      void initalize(const GR& graph, NodesImplBase& nodes) {
        _graph = &graph;
        _nodes = &nodes;
      }

      public:
      struct Arc {
        int id;

        Arc() = default;
        Arc(int _id) : id(_id) {}
        Arc(Invalid) : id(-1) {}
        bool operator==(const Arc& arc) const { return id == arc.id; }
        bool operator!=(const Arc& arc) const { return id != arc.id; }
        bool operator<(const Arc& arc) const { return id < arc.id; }
      };

      ListArcGraphBase() : first_arc(-1), first_free_arc(-1) {}

      Node addNode() {
        NT_ASSERT(false, "This graph structure does not support node insertion");
        return INVALID;   // avoid warning
      }

      Arc addArc(const Node& u, const Node& v) {
        int n;
        if (first_free_arc == -1) {
          n = arcs.size();
          arcs.push_back(ArcT());
        } else {
          n = first_free_arc;
          first_free_arc = arcs[first_free_arc].next_in;
        }
        arcs[n].next_in = (*_nodes)[v].first_in;
        if ((*_nodes)[v].first_in != -1) { arcs[(*_nodes)[v].first_in].prev_in = n; }
        (*_nodes)[v].first_in = n;
        arcs[n].next_out = (*_nodes)[u].first_out;
        if ((*_nodes)[u].first_out != -1) { arcs[(*_nodes)[u].first_out].prev_out = n; }
        (*_nodes)[u].first_out = n;
        arcs[n].source = u;
        arcs[n].target = v;
        return Arc(n);
      }

      void erase(const Arc& arc) {
        int n = arc.id;
        if (arcs[n].prev_in != -1) {
          arcs[arcs[n].prev_in].next_in = arcs[n].next_in;
        } else {
          (*_nodes)[arcs[n].target].first_in = arcs[n].next_in;
        }
        if (arcs[n].next_in != -1) { arcs[arcs[n].next_in].prev_in = arcs[n].prev_in; }

        if (arcs[n].prev_out != -1) {
          arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
        } else {
          (*_nodes)[arcs[n].source].first_out = arcs[n].next_out;
        }
        if (arcs[n].next_out != -1) { arcs[arcs[n].next_out].prev_out = arcs[n].prev_out; }
      }

      void clear() {
        Node node;
        for (first(node); node != INVALID; next(node)) {
          (*_nodes)[node].first_in = -1;
          (*_nodes)[node].first_out = -1;
        }
        arcs.clear();
        first_arc = -1;
        first_free_arc = -1;
      }

      void first(Node& node) const { _graph->first(node); }

      void next(Node& node) const { _graph->next(node); }

      void first(Arc& arc) const {
        Node node;
        first(node);
        while (node != INVALID && (*_nodes)[node].first_in == -1) {
          next(node);
        }
        arc.id = (node == INVALID) ? -1 : (*_nodes)[node].first_in;
      }

      void next(Arc& arc) const {
        if (arcs[arc.id].next_in != -1) {
          arc.id = arcs[arc.id].next_in;
        } else {
          Node node = arcs[arc.id].target;
          next(node);
          while (node != INVALID && (*_nodes)[node].first_in == -1) {
            next(node);
          }
          arc.id = (node == INVALID) ? -1 : (*_nodes)[node].first_in;
        }
      }

      void firstOut(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_out; }

      void nextOut(Arc& arc) const { arc.id = arcs[arc.id].next_out; }

      void firstIn(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_in; }

      void nextIn(Arc& arc) const { arc.id = arcs[arc.id].next_in; }

      int id(const Node& node) const { return _graph->id(node); }
      int id(const Arc& arc) const { return arc.id; }

      Node nodeFromId(int ix) const { return _graph->nodeFromId(ix); }
      Arc  arcFromId(int ix) const { return Arc(ix); }

      int maxNodeId() const { return _graph->maxNodeId(); };
      int maxArcId() const { return arcs.size() - 1; }

      Node source(const Arc& arc) const { return arcs[arc.id].source; }
      Node target(const Arc& arc) const { return arcs[arc.id].target; }

      using NodeNotifier = typename ItemSetTraits< GR, Node >::ItemNotifier;

      NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); }

      template < typename V >
      class NodeMap: public GR::template NodeMap< V > {
        using Parent = typename GR::template NodeMap< V >;

        public:
        explicit NodeMap(const ListArcGraphBase< GR >& arcset) : Parent(*arcset._graph) {}

        NodeMap(const ListArcGraphBase< GR >& arcset, const V& value) : Parent(*arcset._graph, value) {}

        NodeMap& operator=(const NodeMap& cmap) { return operator=< NodeMap >(cmap); }

        template < typename CMap >
        NodeMap& operator=(const CMap& cmap) {
          Parent::operator=(cmap);
          return *this;
        }
      };
    };

    /**
     * @ingroup graphs
     *
     * @brief Digraph using a node set of another digraph or graph and
     * an own arc set.
     *
     * This structure can be used to establish another directed graph
     * over a node set of an existing one. This class uses the same
     * Node type as the underlying graph, and each valid node of the
     * original graph is valid in this arc set, therefore the node
     * objects of the original graph can be used directly with this
     * class. The node handling functions (id handling, observing, and
     * iterators) works equivalently as in the original graph.
     *
     * This implementation is based on doubly-linked lists, from each
     * node the outgoing and the incoming arcs make up lists, therefore
     * one arc can be erased in constant time. It also makes possible,
     * that node can be removed from the underlying graph, in this case
     * all arcs incident to the given node is erased from the arc set.
     *
     * This class fully conforms to the @ref concepts::Digraph
     * "Digraph" concept.
     * It provides only linear time counting for nodes and arcs.
     *
     * @param GR The type of the graph which shares its node set with
     * this class. Its interface must conform to the
     * @ref concepts::Digraph "Digraph" or @ref concepts::Graph "Graph"
     * concept.
     */
    template < typename GR >
    class ListArcGraph: public ArcGraphExtender< ListArcGraphBase< GR > > {
      using Parent = ArcGraphExtender< ListArcGraphBase< GR > >;

      public:
      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;

      using NodesImplBase = typename Parent::NodesImplBase;

      void eraseNode(const Node& node) {
        Arc arc;
        Parent::firstOut(arc, node);
        while (arc != INVALID) {
          erase(arc);
          Parent::firstOut(arc, node);
        }

        Parent::firstIn(arc, node);
        while (arc != INVALID) {
          erase(arc);
          Parent::firstIn(arc, node);
        }
      }

      void clearNodes() { Parent::clear(); }

      class NodesImpl: public NodesImplBase {
        using Parent = NodesImplBase;

        public:
        NodesImpl(const GR& graph, ListArcGraph& arcset) : Parent(graph), _arcset(arcset) {}

        virtual ~NodesImpl() noexcept {}

        protected:
        virtual ObserverReturnCode erase(const Node& node) noexcept override {
          _arcset.eraseNode(node);
          return Parent::erase(node);
        }
        virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
          for (int i = 0; i < int(nodes.size()); ++i)
            _arcset.eraseNode(nodes[i]);
          return Parent::erase(nodes);
        }
        virtual ObserverReturnCode clear() noexcept override {
          _arcset.clearNodes();
          return Parent::clear();
        }

        private:
        ListArcGraph& _arcset;
      };

      NodesImpl _nodes;

      public:
      /**
       * @brief Constructor of the ArcSet.
       *
       * Constructor of the ArcSet.
       */
      ListArcGraph(const GR& graph) : _nodes(graph, *this) { Parent::initalize(graph, _nodes); }

      /**
       * @brief Add a new arc to the digraph.
       *
       * Add a new arc to the digraph with source node \c s
       * and target node \c t.
       * @return The new arc.
       */
      Arc addArc(const Node& s, const Node& t) { return Parent::addArc(s, t); }

      /**
       * @brief Erase an arc from the digraph.
       *
       * Erase an arc \c a from the digraph.
       */
      void erase(const Arc& a) { return Parent::erase(a); }
    };

    template < typename GR >
    class ListEdgeGraphBase {
      public:
      using Node = typename GR::Node;
      using NodeIt = typename GR::NodeIt;

      protected:
      struct NodeT {
        int first_out;
        NodeT() : first_out(-1) {}
      };

      using NodesImplBase = typename ItemSetTraits< GR, Node >::template Map< NodeT >::Type;

      NodesImplBase* _nodes;

      struct ArcT {
        Node target;
        int  prev_out, next_out;
        ArcT() : prev_out(-1), next_out(-1) {}
      };

      TrivialDynamicArray< ArcT > arcs;

      int first_arc;
      int first_free_arc;

      const GR* _graph;

      void initalize(const GR& graph, NodesImplBase& nodes) {
        _graph = &graph;
        _nodes = &nodes;
      }

      public:
      struct Edge {
        int id;

        Edge() = default;
        explicit Edge(int _id) { id = _id; }
        Edge(Invalid) { id = -1; }
        bool operator==(const Edge& arc) const { return id == arc.id; }
        bool operator!=(const Edge& arc) const { return id != arc.id; }
        bool operator<(const Edge& arc) const { return id < arc.id; }
      };

      struct Arc {
        int id;

        Arc() = default;
        Arc(int _id) : id(_id) {}
        Arc(Invalid) : id(-1) {}

        operator Edge() const { return edgeFromId(id / 2); }
        bool operator==(const Arc& arc) const { return id == arc.id; }
        bool operator!=(const Arc& arc) const { return id != arc.id; }
        bool operator<(const Arc& arc) const { return id < arc.id; }
      };

      ListEdgeGraphBase() : first_arc(-1), first_free_arc(-1) {}

      Node addNode() {
        NT_ASSERT(false, "This graph structure does not support node insertion");
        return INVALID;   // avoid warning
      }

      Edge addEdge(const Node& u, const Node& v) {
        int n;

        if (first_free_arc == -1) {
          n = arcs.size();
          arcs.push_back(ArcT());
          arcs.push_back(ArcT());
        } else {
          n = first_free_arc;
          first_free_arc = arcs[n].next_out;
        }

        arcs[n].target = u;
        arcs[n | 1].target = v;

        arcs[n].next_out = (*_nodes)[v].first_out;
        if ((*_nodes)[v].first_out != -1) { arcs[(*_nodes)[v].first_out].prev_out = n; }
        (*_nodes)[v].first_out = n;
        arcs[n].prev_out = -1;

        if ((*_nodes)[u].first_out != -1) { arcs[(*_nodes)[u].first_out].prev_out = (n | 1); }
        arcs[n | 1].next_out = (*_nodes)[u].first_out;
        (*_nodes)[u].first_out = (n | 1);
        arcs[n | 1].prev_out = -1;

        return Edge(n / 2);
      }

      void erase(const Edge& arc) {
        int n = arc.id * 2;

        if (arcs[n].next_out != -1) { arcs[arcs[n].next_out].prev_out = arcs[n].prev_out; }

        if (arcs[n].prev_out != -1) {
          arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
        } else {
          (*_nodes)[arcs[n | 1].target].first_out = arcs[n].next_out;
        }

        if (arcs[n | 1].next_out != -1) { arcs[arcs[n | 1].next_out].prev_out = arcs[n | 1].prev_out; }

        if (arcs[n | 1].prev_out != -1) {
          arcs[arcs[n | 1].prev_out].next_out = arcs[n | 1].next_out;
        } else {
          (*_nodes)[arcs[n].target].first_out = arcs[n | 1].next_out;
        }

        arcs[n].next_out = first_free_arc;
        first_free_arc = n;
      }

      void clear() {
        Node node;
        for (first(node); node != INVALID; next(node)) {
          (*_nodes)[node].first_out = -1;
        }
        arcs.clear();
        first_arc = -1;
        first_free_arc = -1;
      }

      void first(Node& node) const { _graph->first(node); }

      void next(Node& node) const { _graph->next(node); }

      void first(Arc& arc) const {
        Node node;
        first(node);
        while (node != INVALID && (*_nodes)[node].first_out == -1) {
          next(node);
        }
        arc.id = (node == INVALID) ? -1 : (*_nodes)[node].first_out;
      }

      void next(Arc& arc) const {
        if (arcs[arc.id].next_out != -1) {
          arc.id = arcs[arc.id].next_out;
        } else {
          Node node = arcs[arc.id ^ 1].target;
          next(node);
          while (node != INVALID && (*_nodes)[node].first_out == -1) {
            next(node);
          }
          arc.id = (node == INVALID) ? -1 : (*_nodes)[node].first_out;
        }
      }

      void first(Edge& edge) const {
        Node node;
        first(node);
        while (node != INVALID) {
          edge.id = (*_nodes)[node].first_out;
          while ((edge.id & 1) != 1) {
            edge.id = arcs[edge.id].next_out;
          }
          if (edge.id != -1) {
            edge.id /= 2;
            return;
          }
          next(node);
        }
        edge.id = -1;
      }

      void next(Edge& edge) const {
        Node node = arcs[edge.id * 2].target;
        edge.id = arcs[(edge.id * 2) | 1].next_out;
        while ((edge.id & 1) != 1) {
          edge.id = arcs[edge.id].next_out;
        }
        if (edge.id != -1) {
          edge.id /= 2;
          return;
        }
        next(node);
        while (node != INVALID) {
          edge.id = (*_nodes)[node].first_out;
          while ((edge.id & 1) != 1) {
            edge.id = arcs[edge.id].next_out;
          }
          if (edge.id != -1) {
            edge.id /= 2;
            return;
          }
          next(node);
        }
        edge.id = -1;
      }

      void firstOut(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_out; }

      void nextOut(Arc& arc) const { arc.id = arcs[arc.id].next_out; }

      void firstIn(Arc& arc, const Node& node) const {
        arc.id = (((*_nodes)[node].first_out) ^ 1);
        if (arc.id == -2) arc.id = -1;
      }

      void nextIn(Arc& arc) const {
        arc.id = ((arcs[arc.id ^ 1].next_out) ^ 1);
        if (arc.id == -2) arc.id = -1;
      }

      void firstInc(Edge& arc, bool& dir, const Node& node) const {
        int de = (*_nodes)[node].first_out;
        if (de != -1) {
          arc.id = de / 2;
          dir = ((de & 1) == 1);
        } else {
          arc.id = -1;
          dir = true;
        }
      }
      void nextInc(Edge& arc, bool& dir) const {
        int de = (arcs[(arc.id * 2) | (dir ? 1 : 0)].next_out);
        if (de != -1) {
          arc.id = de / 2;
          dir = ((de & 1) == 1);
        } else {
          arc.id = -1;
          dir = true;
        }
      }

      static bool direction(Arc arc) { return (arc.id & 1) == 1; }

      static Arc direct(Edge edge, bool dir) { return Arc(edge.id * 2 + (dir ? 1 : 0)); }

      int        id(const Node& node) const { return _graph->id(node); }
      static int id(Arc e) { return e.id; }
      static int id(Edge e) { return e.id; }

      Node        nodeFromId(int id) const { return _graph->nodeFromId(id); }
      static Arc  arcFromId(int id) { return Arc(id); }
      static Edge edgeFromId(int id) { return Edge(id); }

      int maxNodeId() const { return _graph->maxNodeId(); };
      int maxEdgeId() const { return arcs.size() / 2 - 1; }
      int maxArcId() const { return arcs.size() - 1; }

      Node source(Arc e) const { return arcs[e.id ^ 1].target; }
      Node target(Arc e) const { return arcs[e.id].target; }

      Node u(Edge e) const { return arcs[2 * e.id].target; }
      Node v(Edge e) const { return arcs[2 * e.id + 1].target; }

      using NodeNotifier = typename ItemSetTraits< GR, Node >::ItemNotifier;

      NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); }

      template < typename V >
      class NodeMap: public GR::template NodeMap< V > {
        using Parent = typename GR::template NodeMap< V >;

        public:
        explicit NodeMap(const ListEdgeGraphBase< GR >& arcset) : Parent(*arcset._graph) {}

        NodeMap(const ListEdgeGraphBase< GR >& arcset, const V& value) : Parent(*arcset._graph, value) {}

        NodeMap& operator=(const NodeMap& cmap) { return operator=< NodeMap >(cmap); }

        template < typename CMap >
        NodeMap& operator=(const CMap& cmap) {
          Parent::operator=(cmap);
          return *this;
        }
      };
    };

    /**
     * @ingroup graphs
     *
     * @brief Graph using a node set of another digraph or graph and an
     * own edge set.
     *
     * This structure can be used to establish another graph over a
     * node set of an existing one. This class uses the same Node type
     * as the underlying graph, and each valid node of the original
     * graph is valid in this arc set, therefore the node objects of
     * the original graph can be used directly with this class. The
     * node handling functions (id handling, observing, and iterators)
     * works equivalently as in the original graph.
     *
     * This implementation is based on doubly-linked lists, from each
     * node the incident edges make up lists, therefore one edge can be
     * erased in constant time. It also makes possible, that node can
     * be removed from the underlying graph, in this case all edges
     * incident to the given node is erased from the arc set.
     *
     * This class fully conforms to the @ref concepts::Graph "Graph"
     * concept.
     * It provides only linear time counting for nodes, edges and arcs.
     *
     * @param GR The type of the graph which shares its node set
     * with this class. Its interface must conform to the
     * @ref concepts::Digraph "Digraph" or @ref concepts::Graph "Graph"
     * concept.
     */
    template < typename GR >
    class ListEdgeGraph: public EdgeGraphExtender< ListEdgeGraphBase< GR > > {
      using Parent = EdgeGraphExtender< ListEdgeGraphBase< GR > >;

      public:
      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;
      using Edge = typename Parent::Edge;

      using NodesImplBase = typename Parent::NodesImplBase;

      void eraseNode(const Node& node) {
        Arc arc;
        Parent::firstOut(arc, node);
        while (arc != INVALID) {
          erase(arc);
          Parent::firstOut(arc, node);
        }
      }

      void clearNodes() { Parent::clear(); }

      class NodesImpl: public NodesImplBase {
        using Parent = NodesImplBase;

        public:
        NodesImpl(const GR& graph, ListEdgeGraph& arcset) : Parent(graph), _arcset(arcset) {}

        virtual ~NodesImpl() noexcept {}

        protected:
        virtual ObserverReturnCode erase(const Node& node) noexcept override {
          _arcset.eraseNode(node);
          return Parent::erase(node);
        }
        virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
          for (int i = 0; i < int(nodes.size()); ++i) {
            _arcset.eraseNode(nodes[i]);
          }
          return Parent::erase(nodes);
        }
        virtual ObserverReturnCode clear() noexcept override {
          _arcset.clearNodes();
          return Parent::clear();
        }

        private:
        ListEdgeGraph& _arcset;
      };

      NodesImpl _nodes;

      public:
      /**
       * @brief Constructor of the EdgeSet.
       *
       * Constructor of the EdgeSet.
       */
      ListEdgeGraph(const GR& graph) : _nodes(graph, *this) { Parent::initalize(graph, _nodes); }

      /**
       * @brief Add a new edge to the graph.
       *
       * Add a new edge to the graph with node \c u
       * and node \c v endpoints.
       * @return The new edge.
       */
      Edge addEdge(const Node& u, const Node& v) { return Parent::addEdge(u, v); }

      /**
       * @brief Erase an edge from the graph.
       *
       * Erase the edge \c e from the graph.
       */
      void erase(const Edge& e) { return Parent::erase(e); }
    };

    template < typename GR >
    class SmartArcGraphBase {
      public:
      using Node = typename GR::Node;
      using NodeIt = typename GR::NodeIt;

      protected:
      struct NodeT {
        int first_out, first_in;
        NodeT() : first_out(-1), first_in(-1) {}
      };

      using NodesImplBase = typename ItemSetTraits< GR, Node >::template Map< NodeT >::Type;

      NodesImplBase* _nodes;

      struct ArcT {
        Node source, target;
        int  next_out, next_in;
        ArcT() = default;
      };

      nt::TrivialDynamicArray< ArcT > arcs;

      const GR* _graph;

      void initalize(const GR& graph, NodesImplBase& nodes) {
        _graph = &graph;
        _nodes = &nodes;
      }

      public:
      struct Arc {
        int id;

        Arc() = default;
        Arc(int _id) : id(_id) {}
        Arc(Invalid) : id(-1) {}
        bool operator==(const Arc& arc) const { return id == arc.id; }
        bool operator!=(const Arc& arc) const { return id != arc.id; }
        bool operator<(const Arc& arc) const { return id < arc.id; }
      };

      SmartArcGraphBase() = default;
      SmartArcGraphBase(SmartArcGraphBase&& other) :
          arcs(std::move(other.arcs)), _graph(other._graph), _nodes(other._nodes) {
        other._graph = nullptr;
        other._nodes = nullptr;
      }

      Node addNode() {
        assert(false);    // This graph structure does not support node insertion
        return INVALID;   // avoid warning
      }

      Arc addArc(const Node& u, const Node& v) {
        const int n = arcs.add();
        ArcT&     new_arc = arcs[n];
        new_arc.source = u;
        new_arc.target = v;
        new_arc.next_out = (*_nodes)[u].first_out;
        new_arc.next_in = (*_nodes)[v].first_in;
        (*_nodes)[u].first_out = (*_nodes)[v].first_in = n;
        return Arc(n);
      }

      void clear() {
        Node node;
        for (first(node); node != INVALID; next(node)) {
          (*_nodes)[node].first_in = -1;
          (*_nodes)[node].first_out = -1;
        }
        arcs.clear();
      }

      void first(Node& node) const { _graph->first(node); }

      void next(Node& node) const { _graph->next(node); }

      void first(Arc& arc) const { arc.id = arcs.size() - 1; }

      static void next(Arc& arc) { --arc.id; }

      void firstOut(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_out; }

      void nextOut(Arc& arc) const { arc.id = arcs[arc.id].next_out; }

      void firstIn(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_in; }

      void nextIn(Arc& arc) const { arc.id = arcs[arc.id].next_in; }

      static int id(const Node& node) noexcept { return GR::id(node); }
      static int id(const Arc& arc) noexcept { return arc.id; }

      Node       nodeFromId(int ix) const { return _graph->nodeFromId(ix); }
      static Arc arcFromId(int ix) { return Arc(ix); }

      int maxNodeId() const { return _graph->maxNodeId(); };
      int maxArcId() const { return arcs.size() - 1; }

      Node source(const Arc& arc) const { return arcs[arc.id].source; }
      Node target(const Arc& arc) const { return arcs[arc.id].target; }

      constexpr int nodeNum() const noexcept { return (*_nodes).size(); }
      constexpr int arcNum() const noexcept { return arcs.size(); }

      using NodeNotifier = typename ItemSetTraits< GR, Node >::ItemNotifier;

      NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); }

      template < typename V >
      class NodeMap: public GR::template NodeMap< V > {
        using Parent = typename GR::template NodeMap< V >;

        public:
        explicit NodeMap(const SmartArcGraphBase< GR >& arcgraph) : Parent(*arcgraph._graph) {}

        NodeMap(const SmartArcGraphBase< GR >& arcgraph, const V& value) : Parent(*arcgraph._graph, value) {}

        NodeMap& operator=(const NodeMap& cmap) { return operator=< NodeMap >(cmap); }

        template < typename CMap >
        NodeMap& operator=(const CMap& cmap) {
          Parent::operator=(cmap);
          return *this;
        }
      };

      template < typename _Value >
      using StaticNodeMap = details::StaticMapBase< SmartArcGraphBase< GR >, Node, nt::DynamicArray< _Value > >;
    };

    /**
     * @ingroup graphs
     *
     * @brief Digraph using a node set of another digraph or graph and
     * an own arc set.
     *
     * This structure can be used to establish another directed graph
     * over a node set of an existing one. This class uses the same
     * Node type as the underlying graph, and each valid node of the
     * original graph is valid in this arc set, therefore the node
     * objects of the original graph can be used directly with this
     * class. The node handling functions (id handling, observing, and
     * iterators) works equivalently as in the original graph.
     *
     * @param GR The type of the graph which shares its node set with
     * this class. Its interface must conform to the
     * @ref concepts::Digraph "Digraph" or @ref concepts::Graph "Graph"
     * concept.
     *
     * This implementation is slightly faster than the \c ListArcGraph,
     * because it uses continuous storage for arcs and it uses just
     * single-linked lists for enumerate outgoing and incoming
     * arcs. Therefore the arcs cannot be erased from the arc sets.
     *
     * This class fully conforms to the @ref concepts::Digraph "Digraph"
     * concept.
     * It provides only linear time counting for nodes and arcs.
     *
     * @warning If a node is erased from the underlying graph and this
     * node is the source or target of one arc in the arc set, then
     * the arc set is invalidated, and it cannot be used anymore. The
     * validity can be checked with the \c valid() member function.
     */
    template < typename GR, bool STATIC = false >
    class SmartArcGraph: public ArcGraphExtender< SmartArcGraphBase< GR >, STATIC > {
      using Parent = ArcGraphExtender< SmartArcGraphBase< GR > >;

      public:
      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;

      protected:
      using NodesImplBase = typename Parent::NodesImplBase;

      ObserverReturnCode eraseNode(const Node& node) noexcept {
        if (typename Parent::InArcIt(*this, node) == INVALID && typename Parent::OutArcIt(*this, node) == INVALID)
          return ObserverReturnCode::Ok;
        return ObserverReturnCode::ImmediateDetach;
      }

      void clearNodes() noexcept { Parent::clear(); }

      class NodesImpl: public NodesImplBase {
        using Parent = NodesImplBase;

        public:
        NodesImpl(const GR& graph, SmartArcGraph& arcgraph) : Parent(graph), _arcgraph(arcgraph) {}
        NodesImpl(NodesImpl&& other) : _arcgraph(other._arcgraph), Parent(std::move(other)) {}

        virtual ~NodesImpl() noexcept {}

        bool attached() const { return Parent::attached(); }

        protected:
        virtual ObserverReturnCode erase(const Node& node) noexcept override {
          if (_arcgraph.eraseNode(node) == ObserverReturnCode::ImmediateDetach) {
            Parent::clear();
            return ObserverReturnCode::ImmediateDetach;
          }
          Parent::erase(node);
          return ObserverReturnCode::Ok;
        }
        virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
          for (int i = 0; i < int(nodes.size()); ++i) {
            if (_arcgraph.eraseNode(nodes[i]) == ObserverReturnCode::ImmediateDetach) {
              Parent::clear();
              return ObserverReturnCode::ImmediateDetach;
            }
          }
          Parent::erase(nodes);
          return ObserverReturnCode::Ok;
        }
        virtual ObserverReturnCode clear() noexcept override {
          _arcgraph.clearNodes();
          Parent::clear();
          return ObserverReturnCode::Ok;
        }

        private:
        SmartArcGraph& _arcgraph;
      };

      NodesImpl _nodes;

      public:
      /**
       * @brief Constructor of the ArcGraph.
       *
       * Constructor of the ArcGraph.
       */
      SmartArcGraph(const GR& graph) : _nodes(graph, *this) { Parent::initalize(graph, _nodes); }

      SmartArcGraph(SmartArcGraph&& other) : _nodes(std::move(other._nodes)), Parent(std::move(other)) {}

      /**
       * @brief Add a new arc to the digraph.
       *
       * Add a new arc to the digraph with source node \c s
       * and target node \c t.
       * @return The new arc.
       */
      Arc addArc(const Node& s, const Node& t) { return Parent::addArc(s, t); }

      constexpr int nodeNum() const noexcept { return Parent::_graph->nodeNum(); }
      constexpr int arcNum() const noexcept { return Parent::arcNum(); }

      void reserveNode(int n) { assert(false); };
      void reserveArc(int m) {
        Parent::arcs.reserve(m);
        Parent::notifier(Arc()).reserve(m);
      };

      /**
       * @brief Validity check
       *
       * This functions gives back false if the ArcGraph is
       * invalidated. It occurs when a node in the underlying graph is
       * erased and it is not isolated in the ArcGraph.
       */
      bool valid() const { return _nodes.attached(); }
    };

    template < typename GR >
    class SmartEdgeGraphBase {
      public:
      using Node = typename GR::Node;
      using NodeIt = typename GR::NodeIt;

      protected:
      struct NodeT {
        int first_out;
        NodeT() : first_out(-1) {}
      };

      using NodesImplBase = typename ItemSetTraits< GR, Node >::template Map< NodeT >::Type;

      NodesImplBase* _nodes;

      struct ArcT {
        Node target;
        int  next_out;
        ArcT() = default;
      };

      nt::TrivialDynamicArray< ArcT > arcs;

      const GR* _graph;

      void initalize(const GR& graph, NodesImplBase& nodes) {
        _graph = &graph;
        _nodes = &nodes;
      }

      public:
      struct Edge {
        int id;

        Edge() = default;
        Edge(Invalid) { id = -1; }
        explicit Edge(int _id) { id = _id; }

        bool operator==(const Edge& arc) const { return id == arc.id; }
        bool operator!=(const Edge& arc) const { return id != arc.id; }
        bool operator<(const Edge& arc) const { return id < arc.id; }
      };

      struct Arc {
        int id;

        Arc() = default;
        Arc(int _id) : id(_id) {}
        Arc(Invalid) : id(-1) {}

        operator Edge() const { return edgeFromId(id / 2); }
        bool operator==(const Arc& arc) const { return id == arc.id; }
        bool operator!=(const Arc& arc) const { return id != arc.id; }
        bool operator<(const Arc& arc) const { return id < arc.id; }
      };

      SmartEdgeGraphBase() {}

      Node addNode() {
        assert(false);    // This graph structure does not support node insertion
        return INVALID;   // avoid warning
      }

      Edge addEdge(const Node& u, const Node& v) {
        int n = arcs.size();
        arcs.push_back(ArcT());
        arcs.push_back(ArcT());

        arcs[n].target = u;
        arcs[n | 1].target = v;

        arcs[n].next_out = (*_nodes)[v].first_out;
        (*_nodes)[v].first_out = n;

        arcs[n | 1].next_out = (*_nodes)[u].first_out;
        (*_nodes)[u].first_out = (n | 1);

        return Edge(n / 2);
      }

      void clear() {
        Node node;
        for (first(node); node != INVALID; next(node)) {
          (*_nodes)[node].first_out = -1;
        }
        arcs.clear();
      }

      void first(Node& node) const { _graph->first(node); }

      void next(Node& node) const { _graph->next(node); }

      void first(Arc& arc) const { arc.id = arcs.size() - 1; }

      static void next(Arc& arc) { --arc.id; }

      void first(Edge& arc) const { arc.id = arcs.size() / 2 - 1; }

      static void next(Edge& arc) { --arc.id; }

      void firstOut(Arc& arc, const Node& node) const { arc.id = (*_nodes)[node].first_out; }

      void nextOut(Arc& arc) const { arc.id = arcs[arc.id].next_out; }

      void firstIn(Arc& arc, const Node& node) const {
        arc.id = (((*_nodes)[node].first_out) ^ 1);
        if (arc.id == -2) arc.id = -1;
      }

      void nextIn(Arc& arc) const {
        arc.id = ((arcs[arc.id ^ 1].next_out) ^ 1);
        if (arc.id == -2) arc.id = -1;
      }

      void firstInc(Edge& arc, bool& dir, const Node& node) const {
        int de = (*_nodes)[node].first_out;
        if (de != -1) {
          arc.id = de / 2;
          dir = ((de & 1) == 1);
        } else {
          arc.id = -1;
          dir = true;
        }
      }
      void nextInc(Edge& arc, bool& dir) const {
        int de = (arcs[(arc.id * 2) | (dir ? 1 : 0)].next_out);
        if (de != -1) {
          arc.id = de / 2;
          dir = ((de & 1) == 1);
        } else {
          arc.id = -1;
          dir = true;
        }
      }

      static bool direction(Arc arc) { return (arc.id & 1) == 1; }

      static Arc direct(Edge edge, bool dir) { return Arc(edge.id * 2 + (dir ? 1 : 0)); }

      static int id(const Node& node) { return GR::id(node); }
      static int id(const Arc& arc) { return arc.id; }
      static int id(const Edge& arc) { return arc.id; }

      Node        nodeFromId(int id) const { return _graph->nodeFromId(id); }
      static Arc  arcFromId(int id) { return Arc(id); }
      static Edge edgeFromId(int id) { return Edge(id); }

      int maxNodeId() const { return _graph->maxNodeId(); };
      int maxArcId() const { return arcs.size() - 1; }
      int maxEdgeId() const { return arcs.size() / 2 - 1; }

      Node source(Arc e) const { return arcs[e.id ^ 1].target; }
      Node target(Arc e) const { return arcs[e.id].target; }

      Node u(Edge e) const { return arcs[2 * e.id].target; }
      Node v(Edge e) const { return arcs[2 * e.id + 1].target; }

      constexpr int nodeNum() const noexcept { return (*_nodes).size(); }
      constexpr int arcNum() const noexcept { return arcs.size(); }
      constexpr int edgeNum() const noexcept { return arcs.size() / 2; }

      using NodeNotifier = typename ItemSetTraits< GR, Node >::ItemNotifier;

      NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); }

      template < typename V >
      class NodeMap: public GR::template NodeMap< V > {
        using Parent = typename GR::template NodeMap< V >;

        public:
        explicit NodeMap(const SmartEdgeGraphBase< GR >& arcgraph) : Parent(*arcgraph._graph) {}

        NodeMap(const SmartEdgeGraphBase< GR >& arcgraph, const V& value) : Parent(*arcgraph._graph, value) {}

        NodeMap& operator=(const NodeMap& cmap) { return operator=< NodeMap >(cmap); }

        template < typename CMap >
        NodeMap& operator=(const CMap& cmap) {
          Parent::operator=(cmap);
          return *this;
        }
      };

      template < typename _Value >
      using StaticNodeMap = details::StaticMapBase< SmartEdgeGraphBase< GR >, Node, nt::DynamicArray< _Value > >;
    };

    /**
     * @ingroup graphs
     *
     * @brief Graph using a node set of another digraph or graph and an
     * own edge set.
     *
     * This structure can be used to establish another graph over a
     * node set of an existing one. This class uses the same Node type
     * as the underlying graph, and each valid node of the original
     * graph is valid in this arc set, therefore the node objects of
     * the original graph can be used directly with this class. The
     * node handling functions (id handling, observing, and iterators)
     * works equivalently as in the original graph.
     *
     * @param GR The type of the graph which shares its node set
     * with this class. Its interface must conform to the
     * @ref concepts::Digraph "Digraph" or @ref concepts::Graph "Graph"
     * concept.
     *
     * This implementation is slightly faster than the \c ListEdgeGraph,
     * because it uses continuous storage for edges and it uses just
     * single-linked lists for enumerate incident edges. Therefore the
     * edges cannot be erased from the edge sets.
     *
     * This class fully conforms to the @ref concepts::Graph "Graph"
     * concept.
     * It provides only linear time counting for nodes, edges and arcs.
     *
     * @warning If a node is erased from the underlying graph and this
     * node is incident to one edge in the edge set, then the edge set
     * is invalidated, and it cannot be used anymore. The validity can
     * be checked with the \c valid() member function.
     */
    template < typename GR, bool STATIC = false >
    class SmartEdgeGraph: public EdgeGraphExtender< SmartEdgeGraphBase< GR >, STATIC > {
      using Parent = EdgeGraphExtender< SmartEdgeGraphBase< GR > >;

      public:
      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;
      using Edge = typename Parent::Edge;

      protected:
      using NodesImplBase = typename Parent::NodesImplBase;

      ObserverReturnCode eraseNode(const Node& node) {
        if (typename Parent::IncEdgeIt(*this, node) == INVALID) return ObserverReturnCode::Ok;
        return ObserverReturnCode::ImmediateDetach;
      }

      void clearNodes() { Parent::clear(); }

      class NodesImpl: public NodesImplBase {
        using Parent = NodesImplBase;

        public:
        NodesImpl(const GR& graph, SmartEdgeGraph& arcgraph) : Parent(graph), _arcgraph(arcgraph) {}

        virtual ~NodesImpl() noexcept {}

        bool attached() const { return Parent::attached(); }

        protected:
        virtual ObserverReturnCode erase(const Node& node) noexcept override {
          if (_arcgraph.eraseNode(node) == ObserverReturnCode::ImmediateDetach) {
            Parent::clear();
            return ObserverReturnCode::ImmediateDetach;
          }
          Parent::erase(node);
          return ObserverReturnCode::Ok;
        }
        virtual ObserverReturnCode erase(const nt::ArrayView< Node >& nodes) noexcept override {
          for (int i = 0; i < int(nodes.size()); ++i) {
            if (_arcgraph.eraseNode(nodes[i]) == ObserverReturnCode::ImmediateDetach) {
              Parent::clear();
              return ObserverReturnCode::ImmediateDetach;
            }
          }
          Parent::erase(nodes);
          return ObserverReturnCode::Ok;
        }
        virtual ObserverReturnCode clear() noexcept override {
          _arcgraph.clearNodes();
          Parent::clear();
          return ObserverReturnCode::Ok;
        }

        private:
        SmartEdgeGraph& _arcgraph;
      };

      NodesImpl _nodes;

      public:
      /**
       * @brief Constructor of the EdgeGraph.
       *
       * Constructor of the EdgeGraph.
       */
      SmartEdgeGraph(const GR& graph) : _nodes(graph, *this) { Parent::initalize(graph, _nodes); }

      /**
       * @brief Add a new edge to the graph.
       *
       * Add a new edge to the graph with node \c u
       * and node \c v endpoints.
       * @return The new edge.
       */
      Edge addEdge(const Node& u, const Node& v) { return Parent::addEdge(u, v); }

      constexpr int nodeNum() const noexcept { return Parent::_graph->nodeNum(); }
      constexpr int arcNum() const noexcept { return Parent::arcNum(); }
      constexpr int edgeNum() const noexcept { return Parent::edgeNum(); }

      void reserveNode(int n) { assert(false); };
      void reserveArc(int m) {
        Parent::arcs.reserve(m);
        Parent::notifier(Arc()).reserve(m);
      };
      void reserveEdge(int m) {
        Parent::arcs.reserve(2 * m);
        Parent::notifier(Arc()).reserve(2 * m);
      };

      /**
       * @brief Validity check
       *
       * This functions gives back false if the EdgeGraph is
       * invalidated. It occurs when a node in the underlying graph is
       * erased and it is not isolated in the EdgeGraph.
       */
      bool valid() const { return _nodes.attached(); }
    };

  }   // namespace graphs
}   // namespace nt

#endif
