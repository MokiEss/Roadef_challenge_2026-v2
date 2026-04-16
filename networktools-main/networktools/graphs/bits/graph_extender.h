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
 * This file incorporates work from the LEMON graph library (graph_extender.h).
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
 *   - Add STATIC template parameter
 *   - Add constexpr, noexcept specifiers
 *   - Replace typedef with using
 */

/**
 * @ingroup graphbits
 * @file
 * @brief Extenders for the graph types
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_GRAPH_EXTENDER_H_
#define _NT_GRAPH_EXTENDER_H_

#include "default_map.h"
#include "map_extender.h"
#include "static_map.h"

namespace nt {
  namespace graphs {

    /**
     * @ingroup graphbits
     *
     * @brief Extender for the digraph implementations
     */
    template < typename Base, bool STATIC = false >
    class DigraphExtender: public Base {
      using Parent = Base;

      public:
      using Digraph = DigraphExtender;

      /** Base extensions */
      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;

      constexpr int maxId(Node) const { return Parent::maxNodeId(); }
      constexpr int maxId(Arc) const { return Parent::maxArcId(); }

      constexpr static Node fromId(int id, Node) noexcept { return Parent::nodeFromId(id); }
      constexpr static Arc  fromId(int id, Arc) noexcept { return Parent::arcFromId(id); }

      constexpr Node oppositeNode(const Node& node, const Arc& arc) const noexcept {
        if (node == Parent::source(arc))
          return Parent::target(arc);
        else if (node == Parent::target(arc))
          return Parent::source(arc);
        else
          return INVALID;
      }

      /** Alterable extension */
      using NodeNotifier = AlterationNotifier< DigraphExtender, Node, STATIC >;
      using ArcNotifier = AlterationNotifier< DigraphExtender, Arc, STATIC >;

      protected:
      mutable NodeNotifier node_notifier;
      mutable ArcNotifier  arc_notifier;

      public:
      constexpr NodeNotifier& notifier(Node) const noexcept { return node_notifier; }
      constexpr ArcNotifier&  notifier(Arc) const noexcept { return arc_notifier; }

      struct NodeIt: public Node {
        const Digraph* _digraph;

        NodeIt() = default;
        NodeIt(Invalid i) : Node(i) {}
        explicit NodeIt(const Digraph& digraph) : _digraph(&digraph) { _digraph->first(static_cast< Node& >(*this)); }
        NodeIt(const Digraph& digraph, const Node& node) : Node(node), _digraph(&digraph) {}

        constexpr NodeIt& operator++() noexcept {
          _digraph->next(*this);
          return *this;
        }
      };

      struct ArcIt: public Arc {
        const Digraph* _digraph;

        ArcIt() = default;
        ArcIt(Invalid i) : Arc(i) {}
        explicit ArcIt(const Digraph& digraph) : _digraph(&digraph) { _digraph->first(static_cast< Arc& >(*this)); }
        ArcIt(const Digraph& digraph, const Arc& arc) : Arc(arc), _digraph(&digraph) {}

        constexpr ArcIt& operator++() noexcept {
          _digraph->next(*this);
          return *this;
        }
      };

      struct OutArcIt: public Arc {
        const Digraph* _digraph;

        OutArcIt() = default;
        OutArcIt(Invalid i) : Arc(i) {}
        OutArcIt(const Digraph& digraph, const Node& node) : _digraph(&digraph) { _digraph->firstOut(*this, node); }
        OutArcIt(const Digraph& digraph, const Arc& arc) : Arc(arc), _digraph(&digraph) {}

        constexpr OutArcIt& operator++() noexcept {
          _digraph->nextOut(*this);
          return *this;
        }
      };

      struct InArcIt: public Arc {
        const Digraph* _digraph;

        InArcIt() = default;
        InArcIt(Invalid i) : Arc(i) {}
        InArcIt(const Digraph& digraph, const Node& node) : _digraph(&digraph) { _digraph->firstIn(*this, node); }
        InArcIt(const Digraph& digraph, const Arc& arc) : Arc(arc), _digraph(&digraph) {}

        constexpr InArcIt& operator++() noexcept {
          _digraph->nextIn(*this);
          return *this;
        }
      };

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (i.e. the source in this case) of the iterator
       */
      constexpr Node baseNode(const OutArcIt& arc) const noexcept { return Parent::source(arc); }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (i.e. the target in this case) of the
       * iterator
       */
      constexpr Node runningNode(const OutArcIt& arc) const noexcept { return Parent::target(arc); }

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (i.e. the target in this case) of the iterator
       */
      constexpr Node baseNode(const InArcIt& arc) const noexcept { return Parent::target(arc); }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (i.e. the source in this case) of the
       * iterator
       */
      constexpr Node runningNode(const InArcIt& arc) const noexcept { return Parent::source(arc); }

      template < typename _Value >
      using StaticNodeMap = details::StaticMapBase< Digraph, Node, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticArcMap = details::StaticMapBase< Digraph, Arc, nt::DynamicArray< _Value > >;

      template < typename _Value, typename... ELEMENTS >
      using StaticGraphMap = StaticGraphMap< Digraph, _Value, ELEMENTS... >;

      /**
       * If STATIC is true then any NodeMap or ArcMap must be of type StaticArcMap
       */

      template < typename _Value >
      using NodeMap = typename std::
         conditional< STATIC, StaticNodeMap< _Value >, MapExtender< DefaultMap< Digraph, Node, _Value > > >::type;

      template < typename _Value >
      using ArcMap = typename std::
         conditional< STATIC, StaticArcMap< _Value >, MapExtender< DefaultMap< Digraph, Arc, _Value > > >::type;


      constexpr Node addNode() noexcept {
        const Node node = Parent::addNode();
        notifier(Node()).add(node);
        return node;
      }

      constexpr Arc addArc(const Node& from, const Node& to) noexcept {
        const Arc arc = Parent::addArc(from, to);
        notifier(Arc()).add(arc);
        return arc;
      }

      constexpr void clear() noexcept {
        notifier(Arc()).clear();
        notifier(Node()).clear();
        Parent::clear();
      }

      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      constexpr void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        Parent::build(digraph, nodeRef, arcRef);
        notifier(Node()).build();
        notifier(Arc()).build();
      }

      constexpr void erase(const Node& node) noexcept {
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

        notifier(Node()).erase(node);
        Parent::erase(node);
      }

      constexpr void erase(const Arc& arc) noexcept {
        notifier(Arc()).erase(arc);
        Parent::erase(arc);
      }

      void copyFrom(const DigraphExtender& other) noexcept {
        Parent::copyFrom(other);
        arc_notifier.copyFrom(other.arc_notifier);
        node_notifier.copyFrom(other.node_notifier);
      }

      DigraphExtender() noexcept {
        node_notifier.setContainer(*this);
        arc_notifier.setContainer(*this);
      }

      DigraphExtender(DigraphExtender&& g) noexcept :
          Parent(std::move(g)), arc_notifier(std::move(g.arc_notifier)), node_notifier(std::move(g.node_notifier)) {}

      DigraphExtender& operator=(DigraphExtender&& g) noexcept {
        if (this == &g) return *this;
        Parent::operator=(std::move(g));
        arc_notifier = std::move(g.arc_notifier);
        node_notifier = std::move(g.node_notifier);
        return *this;
      }

      ~DigraphExtender() noexcept {
        arc_notifier.clear();
        node_notifier.clear();
      }
    };

    /**
     * @ingroup _graphbits
     *
     * @brief Extender for the Graphs
     */
    template < typename Base, bool STATIC = false >
    class GraphExtender: public Base {
      using Parent = Base;

      public:
      using Graph = GraphExtender;

      using UndirectedTag = nt::True;

      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;
      using Edge = typename Parent::Edge;

      /** Graph extension */

      constexpr int maxId(Node) const noexcept { return Parent::maxNodeId(); }
      constexpr int maxId(Arc) const noexcept { return Parent::maxArcId(); }
      constexpr int maxId(Edge) const noexcept { return Parent::maxEdgeId(); }

      constexpr static Node fromId(int id, Node) noexcept { return Parent::nodeFromId(id); }
      constexpr static Arc  fromId(int id, Arc) noexcept { return Parent::arcFromId(id); }
      constexpr static Edge fromId(int id, Edge) noexcept { return Parent::edgeFromId(id); }

      constexpr Node oppositeNode(const Node& n, const Edge& e) const noexcept {
        if (n == Parent::u(e))
          return Parent::v(e);
        else if (n == Parent::v(e))
          return Parent::u(e);
        else
          return INVALID;
      }

      constexpr Arc oppositeArc(const Arc& arc) const noexcept { return Parent::direct(arc, !Parent::direction(arc)); }

      using Parent::direct;
      constexpr Arc direct(const Edge& edge, const Node& node) const noexcept {
        return Parent::direct(edge, Parent::u(edge) == node);
      }

      /** Alterable extension */

      using NodeNotifier = AlterationNotifier< GraphExtender, Node, STATIC >;
      using ArcNotifier = AlterationNotifier< GraphExtender, Arc, STATIC >;
      using EdgeNotifier = AlterationNotifier< GraphExtender, Edge, STATIC >;

      protected:
      mutable NodeNotifier node_notifier;
      mutable ArcNotifier  arc_notifier;
      mutable EdgeNotifier edge_notifier;

      public:
      constexpr NodeNotifier& notifier(Node) const noexcept { return node_notifier; }
      constexpr ArcNotifier&  notifier(Arc) const noexcept { return arc_notifier; }
      constexpr EdgeNotifier& notifier(Edge) const noexcept { return edge_notifier; }

      struct NodeIt: public Node {
        const Graph* _graph;

        NodeIt() = default;
        NodeIt(Invalid i) : Node(i) {}
        explicit NodeIt(const Graph& graph) : _graph(&graph) { _graph->first(static_cast< Node& >(*this)); }
        NodeIt(const Graph& graph, const Node& node) : Node(node), _graph(&graph) {}

        constexpr NodeIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      struct ArcIt: public Arc {
        const Graph* _graph;

        ArcIt() = default;
        ArcIt(Invalid i) : Arc(i) {}
        explicit ArcIt(const Graph& graph) : _graph(&graph) { _graph->first(static_cast< Arc& >(*this)); }
        ArcIt(const Graph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr ArcIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      struct OutArcIt: public Arc {
        const Graph* _graph;

        OutArcIt() = default;
        OutArcIt(Invalid i) : Arc(i) {}
        OutArcIt(const Graph& graph, const Node& node) : _graph(&graph) { _graph->firstOut(*this, node); }
        OutArcIt(const Graph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr OutArcIt& operator++() noexcept {
          _graph->nextOut(*this);
          return *this;
        }
      };

      struct InArcIt: public Arc {
        const Graph* _graph;

        InArcIt() = default;
        InArcIt(Invalid i) : Arc(i) {}
        InArcIt(const Graph& graph, const Node& node) : _graph(&graph) { _graph->firstIn(*this, node); }
        InArcIt(const Graph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr InArcIt& operator++() noexcept {
          _graph->nextIn(*this);
          return *this;
        }
      };

      struct EdgeIt: public Parent::Edge {
        const Graph* _graph;

        EdgeIt() = default;
        EdgeIt(Invalid i) : Edge(i) {}
        explicit EdgeIt(const Graph& graph) : _graph(&graph) { _graph->first(static_cast< Edge& >(*this)); }
        EdgeIt(const Graph& graph, const Edge& edge) : Edge(edge), _graph(&graph) {}

        constexpr EdgeIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      struct IncEdgeIt: public Parent::Edge {
        friend class GraphExtender;
        const Graph* _graph;
        bool         _direction;

        IncEdgeIt() = default;
        IncEdgeIt(Invalid i) : Edge(i), _direction(false) {}
        IncEdgeIt(const Graph& graph, const Node& node) : _graph(&graph) { _graph->firstInc(*this, _direction, node); }
        IncEdgeIt(const Graph& graph, const Edge& edge, const Node& node) : _graph(&graph), Edge(edge) {
          _direction = (_graph->source(edge) == node);
        }

        constexpr IncEdgeIt& operator++() noexcept {
          _graph->nextInc(*this, _direction);
          return *this;
        }
      };

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the source in this case) of the iterator
       */
      constexpr Node baseNode(const OutArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the target in this case) of the
       * iterator
       */
      constexpr Node runningNode(const OutArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the target in this case) of the iterator
       */
      constexpr Node baseNode(const InArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the source in this case) of the
       * iterator
       */
      constexpr Node runningNode(const InArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }

      /**
       * Base node of the iterator
       *
       * Returns the base node of the iterator
       */
      constexpr Node baseNode(const IncEdgeIt& edge) const noexcept {
        return edge._direction ? this->u(edge) : this->v(edge);
      }
      /**
       * Running node of the iterator
       *
       * Returns the running node of the iterator
       */
      constexpr Node runningNode(const IncEdgeIt& edge) const noexcept {
        return edge._direction ? this->v(edge) : this->u(edge);
      }

      /** Mappable extension */

      template < typename _Value >
      using StaticNodeMap = details::StaticMapBase< Graph, Node, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticArcMap = details::StaticMapBase< Graph, Arc, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticEdgeMap = details::StaticMapBase< Graph, Edge, nt::DynamicArray< _Value > >;

      template < typename _Value, typename... ELEMENTS >
      using StaticGraphMap = StaticGraphMap< Graph, _Value, ELEMENTS... >;

      /**
       * If STATIC is true then any NodeMap, ArcMap or EdgeMap must be of type StaticArcMap
       */

      template < typename _Value >
      using NodeMap = typename std::
         conditional< STATIC, StaticNodeMap< _Value >, MapExtender< DefaultMap< Graph, Node, _Value > > >::type;

      template < typename _Value >
      using ArcMap = typename std::
         conditional< STATIC, StaticArcMap< _Value >, MapExtender< DefaultMap< Graph, Arc, _Value > > >::type;

      template < typename _Value >
      using EdgeMap = typename std::
         conditional< STATIC, StaticArcMap< _Value >, MapExtender< DefaultMap< Graph, Edge, _Value > > >::type;

      /** Alteration extension */

      constexpr Node addNode() noexcept {
        Node node = Parent::addNode();
        notifier(Node()).add(node);
        return node;
      }

      constexpr Edge addEdge(const Node& from, const Node& to) noexcept {
        Edge edge = Parent::addEdge(from, to);
        notifier(Edge()).add(edge);
        const Arc ev[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).add(ev);
        return edge;
      }

      constexpr void clear() noexcept {
        notifier(Arc()).clear();
        notifier(Edge()).clear();
        notifier(Node()).clear();
        Parent::clear();
      }

      template < typename Graph, typename NodeRefMap, typename EdgeRefMap >
      constexpr void build(const Graph& graph, NodeRefMap& nodeRef, EdgeRefMap& edgeRef) noexcept {
        Parent::build(graph, nodeRef, edgeRef);
        notifier(Node()).build();
        notifier(Edge()).build();
        notifier(Arc()).build();
      }

      constexpr void erase(const Node& node) noexcept {
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

        notifier(Node()).erase(node);
        Parent::erase(node);
      }

      constexpr void erase(const Edge& edge) noexcept {
        const Arc av[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).erase(av);
        notifier(Edge()).erase(edge);
        Parent::erase(edge);
      }

      GraphExtender() noexcept {
        node_notifier.setContainer(*this);
        arc_notifier.setContainer(*this);
        edge_notifier.setContainer(*this);
      }

      GraphExtender(GraphExtender&& g) noexcept :
          Parent(std::move(g)), arc_notifier(std::move(g.arc_notifier)), node_notifier(std::move(g.node_notifier)),
          edge_notifier(std::move(g.edge_notifier)) {}

      GraphExtender& operator=(GraphExtender&& g) noexcept {
        if (this == &g) return *this;
        Parent::operator=(std::move(g));
        arc_notifier = std::move(g.arc_notifier);
        node_notifier = std::move(g.node_notifier);
        edge_notifier = std::move(g.edge_notifier);
        return *this;
      }

      ~GraphExtender() noexcept {
        edge_notifier.clear();
        arc_notifier.clear();
        node_notifier.clear();
      }
    };

    /**
     * @ingroup _graphbits
     *
     * @brief Extender for the BpGraphs
     */
    template < typename Base, bool STATIC = false >
    class BpGraphExtender: public Base {
      using Parent = Base;

      public:
      using BpGraph = BpGraphExtender;

      using UndirectedTag = nt::True;

      using Node = typename Parent::Node;
      using RedNode = typename Parent::RedNode;
      using BlueNode = typename Parent::BlueNode;
      using Arc = typename Parent::Arc;
      using Edge = typename Parent::Edge;

      /** BpGraph extension */

      using Parent::first;
      using Parent::id;
      using Parent::next;

      constexpr int maxId(Node) const noexcept { return Parent::maxNodeId(); }
      constexpr int maxId(RedNode) const noexcept { return Parent::maxRedId(); }
      constexpr int maxId(BlueNode) const noexcept { return Parent::maxBlueId(); }
      constexpr int maxId(Arc) const noexcept { return Parent::maxArcId(); }
      constexpr int maxId(Edge) const noexcept { return Parent::maxEdgeId(); }

      constexpr static Node fromId(int id, Node) noexcept { return Parent::nodeFromId(id); }
      constexpr static Arc  fromId(int id, Arc) noexcept { return Parent::arcFromId(id); }
      constexpr static Edge fromId(int id, Edge) noexcept { return Parent::edgeFromId(id); }

      constexpr Node u(Edge e) const noexcept { return this->redNode(e); }
      constexpr Node v(Edge e) const noexcept { return this->blueNode(e); }

      constexpr Node oppositeNode(const Node& n, const Edge& e) const noexcept {
        if (n == u(e))
          return v(e);
        else if (n == v(e))
          return u(e);
        else
          return INVALID;
      }

      constexpr Arc oppositeArc(const Arc& arc) const noexcept { return Parent::direct(arc, !Parent::direction(arc)); }

      using Parent::direct;
      constexpr Arc direct(const Edge& edge, const Node& node) const noexcept {
        return Parent::direct(edge, Parent::redNode(edge) == node);
      }

      constexpr RedNode asRedNode(const Node& node) const noexcept {
        if (node == INVALID || Parent::blue(node)) return INVALID;
        return Parent::asRedNodeUnsafe(node);
      }

      constexpr BlueNode asBlueNode(const Node& node) const noexcept {
        if (node == INVALID || Parent::red(node)) return INVALID;
        return Parent::asBlueNodeUnsafe(node);
      }

      /** Alterable extension */

      using NodeNotifier = AlterationNotifier< BpGraphExtender, Node, STATIC >;
      using RedNodeNotifier = AlterationNotifier< BpGraphExtender, RedNode, STATIC >;
      using BlueNodeNotifier = AlterationNotifier< BpGraphExtender, BlueNode, STATIC >;
      using ArcNotifier = AlterationNotifier< BpGraphExtender, Arc, STATIC >;
      using EdgeNotifier = AlterationNotifier< BpGraphExtender, Edge, STATIC >;

      protected:
      mutable NodeNotifier     node_notifier;
      mutable RedNodeNotifier  red_node_notifier;
      mutable BlueNodeNotifier blue_node_notifier;
      mutable ArcNotifier      arc_notifier;
      mutable EdgeNotifier     edge_notifier;

      public:
      constexpr NodeNotifier&     notifier(Node) const noexcept { return node_notifier; }
      constexpr RedNodeNotifier&  notifier(RedNode) const noexcept { return red_node_notifier; }
      constexpr BlueNodeNotifier& notifier(BlueNode) const noexcept { return blue_node_notifier; }
      constexpr ArcNotifier&      notifier(Arc) const noexcept { return arc_notifier; }
      constexpr EdgeNotifier&     notifier(Edge) const noexcept { return edge_notifier; }

      class NodeIt: public Node {
        const BpGraph* _graph;

        public:
        NodeIt() = default;
        NodeIt(Invalid i) : Node(i) {}
        explicit NodeIt(const BpGraph& graph) : _graph(&graph) { _graph->first(static_cast< Node& >(*this)); }
        NodeIt(const BpGraph& graph, const Node& node) : Node(node), _graph(&graph) {}

        constexpr NodeIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      class RedNodeIt: public RedNode {
        const BpGraph* _graph;

        public:
        RedNodeIt() = default;
        RedNodeIt(Invalid i) : RedNode(i) {}
        explicit RedNodeIt(const BpGraph& graph) : _graph(&graph) { _graph->first(static_cast< RedNode& >(*this)); }
        RedNodeIt(const BpGraph& graph, const RedNode& node) : RedNode(node), _graph(&graph) {}

        constexpr RedNodeIt& operator++() noexcept {
          _graph->next(static_cast< RedNode& >(*this));
          return *this;
        }
      };

      class BlueNodeIt: public BlueNode {
        const BpGraph* _graph;

        public:
        BlueNodeIt() = default;
        BlueNodeIt(Invalid i) : BlueNode(i) {}
        explicit BlueNodeIt(const BpGraph& graph) : _graph(&graph) { _graph->first(static_cast< BlueNode& >(*this)); }
        BlueNodeIt(const BpGraph& graph, const BlueNode& node) : BlueNode(node), _graph(&graph) {}

        constexpr BlueNodeIt& operator++() noexcept {
          _graph->next(static_cast< BlueNode& >(*this));
          return *this;
        }
      };

      class ArcIt: public Arc {
        const BpGraph* _graph;

        public:
        ArcIt() = default;
        ArcIt(Invalid i) : Arc(i) {}
        explicit ArcIt(const BpGraph& graph) : _graph(&graph) { _graph->first(static_cast< Arc& >(*this)); }
        ArcIt(const BpGraph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr ArcIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      class OutArcIt: public Arc {
        const BpGraph* _graph;

        public:
        OutArcIt() = default;
        OutArcIt(Invalid i) : Arc(i) {}
        OutArcIt(const BpGraph& graph, const Node& node) : _graph(&graph) { _graph->firstOut(*this, node); }
        OutArcIt(const BpGraph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr OutArcIt& operator++() noexcept {
          _graph->nextOut(*this);
          return *this;
        }
      };

      class InArcIt: public Arc {
        const BpGraph* _graph;

        public:
        InArcIt() = default;
        InArcIt(Invalid i) : Arc(i) {}
        InArcIt(const BpGraph& graph, const Node& node) : _graph(&graph) { _graph->firstIn(*this, node); }
        InArcIt(const BpGraph& graph, const Arc& arc) : Arc(arc), _graph(&graph) {}

        constexpr InArcIt& operator++() noexcept {
          _graph->nextIn(*this);
          return *this;
        }
      };

      class EdgeIt: public Parent::Edge {
        const BpGraph* _graph;

        public:
        EdgeIt() {}
        EdgeIt(Invalid i) : Edge(i) {}
        explicit EdgeIt(const BpGraph& graph) : _graph(&graph) { _graph->first(static_cast< Edge& >(*this)); }
        EdgeIt(const BpGraph& graph, const Edge& edge) : Edge(edge), _graph(&graph) {}

        constexpr EdgeIt& operator++() noexcept {
          _graph->next(*this);
          return *this;
        }
      };

      class IncEdgeIt: public Parent::Edge {
        friend class BpGraphExtender;
        const BpGraph* _graph;
        bool           _direction;

        public:
        IncEdgeIt() = default;
        IncEdgeIt(Invalid i) : Edge(i), _direction(false) {}
        IncEdgeIt(const BpGraph& graph, const Node& node) : _graph(&graph) {
          _graph->firstInc(*this, _direction, node);
        }
        IncEdgeIt(const BpGraph& graph, const Edge& edge, const Node& node) : _graph(&graph), Edge(edge) {
          _direction = (_graph->source(edge) == node);
        }

        constexpr IncEdgeIt& operator++() noexcept {
          _graph->nextInc(*this, _direction);
          return *this;
        }
      };

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the source in this case) of the iterator
       */
      constexpr Node baseNode(const OutArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the target in this case) of the
       * iterator
       */
      constexpr Node runningNode(const OutArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the target in this case) of the iterator
       */
      constexpr Node baseNode(const InArcIt& arc) const noexcept {
        return Parent::target(static_cast< const Arc& >(arc));
      }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the source in this case) of the
       * iterator
       */
      constexpr Node runningNode(const InArcIt& arc) const noexcept {
        return Parent::source(static_cast< const Arc& >(arc));
      }

      /**
       * Base node of the iterator
       *
       * Returns the base node of the iterator
       */
      constexpr Node baseNode(const IncEdgeIt& edge) const noexcept {
        return edge._direction ? this->u(edge) : this->v(edge);
      }
      /**
       * Running node of the iterator
       *
       * Returns the running node of the iterator
       */
      constexpr Node runningNode(const IncEdgeIt& edge) const noexcept {
        return edge._direction ? this->v(edge) : this->u(edge);
      }

      /** Mappable extension */

      template < typename _Value >
      using StaticNodeMap = details::StaticMapBase< BpGraph, Node, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticRedNodeMap = details::StaticMapBase< BpGraph, RedNode, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticBlueNodeMap = details::StaticMapBase< BpGraph, BlueNode, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticArcMap = details::StaticMapBase< BpGraph, Arc, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticEdgeMap = details::StaticMapBase< BpGraph, Edge, nt::DynamicArray< _Value > >;

      template < typename _Value, typename... ELEMENTS >
      using StaticGraphMap = StaticGraphMap< BpGraph, _Value, ELEMENTS... >;

      /**
       * If STATIC is true then any NodeMap, ArcMap or EdgeMap must be of type StaticArcMap
       */

      template < typename _Value >
      using NodeMap = typename std::
         conditional< STATIC, StaticNodeMap< _Value >, MapExtender< DefaultMap< BpGraph, Node, _Value > > >::type;
      template < typename _Value >
      using RedNodeMap = typename std::
         conditional< STATIC, StaticNodeMap< _Value >, MapExtender< DefaultMap< BpGraph, RedNode, _Value > > >::type;

      template < typename _Value >
      using BlueNodeMap = typename std::
         conditional< STATIC, StaticNodeMap< _Value >, MapExtender< DefaultMap< BpGraph, BlueNode, _Value > > >::type;

      template < typename _Value >
      using ArcMap = typename std::
         conditional< STATIC, StaticArcMap< _Value >, MapExtender< DefaultMap< BpGraph, Arc, _Value > > >::type;

      template < typename _Value >
      using EdgeMap = typename std::
         conditional< STATIC, StaticArcMap< _Value >, MapExtender< DefaultMap< BpGraph, Edge, _Value > > >::type;

      /** Alteration extension */

      constexpr RedNode addRedNode() noexcept {
        RedNode node = Parent::addRedNode();
        notifier(RedNode()).add(node);
        notifier(Node()).add(node);
        return node;
      }

      constexpr BlueNode addBlueNode() noexcept {
        BlueNode node = Parent::addBlueNode();
        notifier(BlueNode()).add(node);
        notifier(Node()).add(node);
        return node;
      }

      constexpr Edge addEdge(const RedNode& from, const BlueNode& to) noexcept {
        Edge edge = Parent::addEdge(from, to);
        notifier(Edge()).add(edge);
        const Arc av[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).add(av);
        return edge;
      }

      constexpr void clear() noexcept {
        notifier(Arc()).clear();
        notifier(Edge()).clear();
        notifier(Node()).clear();
        notifier(BlueNode()).clear();
        notifier(RedNode()).clear();
        Parent::clear();
      }

      template < typename BpGraph, typename NodeRefMap, typename EdgeRefMap >
      constexpr void build(const BpGraph& graph, NodeRefMap& nodeRef, EdgeRefMap& edgeRef) noexcept {
        Parent::build(graph, nodeRef, edgeRef);
        notifier(RedNode()).build();
        notifier(BlueNode()).build();
        notifier(Node()).build();
        notifier(Edge()).build();
        notifier(Arc()).build();
      }

      constexpr void erase(const Node& node) noexcept {
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

        if (Parent::red(node)) {
          notifier(RedNode()).erase(this->asRedNodeUnsafe(node));
        } else {
          notifier(BlueNode()).erase(this->asBlueNodeUnsafe(node));
        }

        notifier(Node()).erase(node);
        Parent::erase(node);
      }

      constexpr void erase(const Edge& edge) noexcept {
        const Arc av[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).erase(av);
        notifier(Edge()).erase(edge);
        Parent::erase(edge);
      }

      BpGraphExtender() noexcept {
        red_node_notifier.setContainer(*this);
        blue_node_notifier.setContainer(*this);
        node_notifier.setContainer(*this);
        arc_notifier.setContainer(*this);
        edge_notifier.setContainer(*this);
      }

      BpGraphExtender(BpGraphExtender&& g) noexcept :
          Parent(std::move(g)), red_node_notifier(std::move(g.red_node_notifier)),
          blue_node_notifier(std::move(g.blue_node_notifier)), node_notifier(std::move(g.node_notifier)),
          arc_notifier(std::move(g.arc_notifier)), edge_notifier(std::move(g.edge_notifier)) {}

      BpGraphExtender& operator=(BpGraphExtender&& g) noexcept {
        if (this == &g) return *this;
        Parent::operator=(std::move(g));
        red_node_notifier = std::move(g.red_node_notifier);
        blue_node_notifier = std::move(g.blue_node_notifier);
        node_notifier = std::move(g.node_notifier);
        arc_notifier = std::move(g.arc_notifier);
        edge_notifier = std::move(g.edge_notifier);
        return *this;
      }

      ~BpGraphExtender() noexcept {
        edge_notifier.clear();
        arc_notifier.clear();
        node_notifier.clear();
        blue_node_notifier.clear();
        red_node_notifier.clear();
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
