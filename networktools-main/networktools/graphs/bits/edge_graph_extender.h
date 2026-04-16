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
 * This file incorporates work from the LEMON graph library (edge_set_extender.h).
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
 *   - The classes originally named ArcSetExtender, EdgeSetExtender,... in LEMON were renamed to
 *     ArcGraphExtender, EdgeGraphExtender, ... in order to
 *      (i) be consistent with the fact they are graphs and not actual sets, and to
 *      (ii) avoid the confusion with the classes NodeSet, ArcSet and EdgetSet defined in
 *   - graph_element_set.h which are actual sets of nodes, arcs and edges.
 */

/**
 * @ingroup digraphbits
 * @file
 * @brief Extenders for the arc set types
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_EDGE_SET_EXTENDER_H_
#define _NT_EDGE_SET_EXTENDER_H_

#include "../tools.h"
#include "default_map.h"
#include "map_extender.h"
#include "static_map.h"

namespace nt {
  namespace graphs {

    /**
     * @ingroup digraphbits
     *
     * @brief Extender for the ArcGraphs
     */
    template < typename Base, bool STATIC = false >
    class ArcGraphExtender: public Base {
      using Parent = Base;

      public:
      using Digraph = ArcGraphExtender;

      /** Base extensions */

      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;

      constexpr int maxId(Node) const noexcept { return Parent::maxNodeId(); }
      constexpr int maxId(Arc) const noexcept { return Parent::maxArcId(); }

      constexpr Node fromId(int id, Node) const noexcept { return Parent::nodeFromId(id); }
      constexpr Arc  fromId(int id, Arc) const noexcept { return Parent::arcFromId(id); }

      constexpr Node oppositeNode(const Node& n, const Arc& e) const noexcept {
        if (n == Parent::source(e))
          return Parent::target(e);
        else if (n == Parent::target(e))
          return Parent::source(e);
        else
          return INVALID;
      }

      /** Alteration notifier extensions */

      /** The arc observer registry. */
      using ArcNotifier = AlterationNotifier< ArcGraphExtender, Arc, STATIC >;

      protected:
      mutable ArcNotifier arc_notifier;

      public:
      using Parent::notifier;

      /** Gives back the arc alteration notifier. */
      constexpr ArcNotifier& notifier(Arc) const noexcept { return arc_notifier; }

      /** Iterable extensions */

      class NodeIt: public Node {
        const Digraph* digraph;

        public:
        NodeIt() = default;
        NodeIt(Invalid i) : Node(i) {}
        explicit NodeIt(const Digraph& _graph) : digraph(&_graph) { _graph.first(static_cast< Node& >(*this)); }
        NodeIt(const Digraph& _graph, const Node& node) : Node(node), digraph(&_graph) {}

        constexpr NodeIt& operator++() noexcept {
          digraph->next(*this);
          return *this;
        }
      };

      class ArcIt: public Arc {
        const Digraph* digraph;

        public:
        ArcIt() = default;
        ArcIt(Invalid i) : Arc(i) {}
        explicit ArcIt(const Digraph& _graph) : digraph(&_graph) { _graph.first(static_cast< Arc& >(*this)); }
        ArcIt(const Digraph& _graph, const Arc& e) : Arc(e), digraph(&_graph) {}

        constexpr ArcIt& operator++() noexcept {
          digraph->next(*this);
          return *this;
        }
      };

      class OutArcIt: public Arc {
        const Digraph* digraph;

        public:
        OutArcIt() = default;
        OutArcIt(Invalid i) : Arc(i) {}
        OutArcIt(const Digraph& _graph, const Node& node) : digraph(&_graph) { _graph.firstOut(*this, node); }
        OutArcIt(const Digraph& _graph, const Arc& arc) : Arc(arc), digraph(&_graph) {}

        constexpr OutArcIt& operator++() noexcept {
          digraph->nextOut(*this);
          return *this;
        }
      };

      class InArcIt: public Arc {
        const Digraph* digraph;

        public:
        InArcIt() = default;
        InArcIt(Invalid i) : Arc(i) {}
        InArcIt(const Digraph& _graph, const Node& node) : digraph(&_graph) { _graph.firstIn(*this, node); }
        InArcIt(const Digraph& _graph, const Arc& arc) : Arc(arc), digraph(&_graph) {}

        constexpr InArcIt& operator++() noexcept {
          digraph->nextIn(*this);
          return *this;
        }
      };

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the source in this case) of the iterator
       */
      constexpr Node baseNode(const OutArcIt& e) const noexcept { return Parent::source(static_cast< const Arc& >(e)); }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the target in this case) of the
       * iterator
       */
      constexpr Node runningNode(const OutArcIt& e) const noexcept {
        return Parent::target(static_cast< const Arc& >(e));
      }

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the target in this case) of the iterator
       */
      constexpr Node baseNode(const InArcIt& e) const noexcept { return Parent::target(static_cast< const Arc& >(e)); }
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the source in this case) of the
       * iterator
       */
      constexpr Node runningNode(const InArcIt& e) const noexcept {
        return Parent::source(static_cast< const Arc& >(e));
      }

      using Parent::first;

      /** Mappable extension */

      template < typename _Value >
      class ArcMap: public MapExtender< DefaultMap< Digraph, Arc, _Value > > {
        using Parent = MapExtender< DefaultMap< Digraph, Arc, _Value > >;

        public:
        explicit ArcMap(const Digraph& _g) : Parent(_g) {}
        ArcMap(const Digraph& _g, const _Value& _v) : Parent(_g, _v) {}

        ArcMap(ArcMap&& other) noexcept : Parent(std::move(other)) {}

        ArcMap& operator=(const ArcMap& cmap) { return operator=< ArcMap >(cmap); }

        ArcMap& operator=(ArcMap&& cmap) noexcept {
          static_cast<Parent&>(*this) = std::move(static_cast<Parent&>(cmap));
          return *this;
        }

        template < typename CMap >
        ArcMap& operator=(const CMap& cmap) noexcept {
          Parent::operator=(cmap);
          return *this;
        }
      };

      template < typename _Value >
      using StaticArcMap = details::StaticMapBase< Digraph, Arc, nt::DynamicArray< _Value > >;

      template < typename _Value, typename... ELEMENTS >
      using StaticGraphMap = StaticGraphMap< Digraph, _Value, ELEMENTS... >;

      /** Alteration extension */

      constexpr Arc addArc(const Node& from, const Node& to) noexcept {
        Arc arc = Parent::addArc(from, to);
        notifier(Arc()).add(arc);
        return arc;
      }

      constexpr void clear() noexcept {
        notifier(Arc()).clear();
        Parent::clear();
      }

      constexpr void erase(const Arc& arc) noexcept {
        notifier(Arc()).erase(arc);
        Parent::erase(arc);
      }

      ArcGraphExtender() noexcept { arc_notifier.setContainer(*this); }
      ArcGraphExtender(ArcGraphExtender&& other) noexcept :
          arc_notifier(std::move(other.arc_notifier)), Parent(std::move(other)) {}
      ~ArcGraphExtender() noexcept { arc_notifier.clear(); }
    };

    /**
     * @ingroup digraphbits
     *
     * @brief Extender for the EdgeGraphs
     */
    template < typename Base, bool STATIC = false >
    class EdgeGraphExtender: public Base {
      using Parent = Base;

      public:
      using Graph = EdgeGraphExtender;

      using UndirectedTag = nt::True;

      using Node = typename Parent::Node;
      using Arc = typename Parent::Arc;
      using Edge = typename Parent::Edge;

      constexpr int maxId(Node) const noexcept { return Parent::maxNodeId(); }
      constexpr int maxId(Arc) const noexcept { return Parent::maxArcId(); }
      constexpr int maxId(Edge) const noexcept { return Parent::maxEdgeId(); }

      constexpr Node fromId(int id, Node) const noexcept { return Parent::nodeFromId(id); }
      constexpr Arc  fromId(int id, Arc) const noexcept { return Parent::arcFromId(id); }
      constexpr Edge fromId(int id, Edge) const noexcept { return Parent::edgeFromId(id); }

      constexpr Node oppositeNode(const Node& n, const Edge& e) const noexcept {
        if (n == Parent::u(e))
          return Parent::v(e);
        else if (n == Parent::v(e))
          return Parent::u(e);
        else
          return INVALID;
      }

      constexpr Arc oppositeArc(const Arc& e) const noexcept { return Parent::direct(e, !Parent::direction(e)); }

      using Parent::direct;
      constexpr Arc direct(const Edge& e, const Node& s) const noexcept { return Parent::direct(e, Parent::u(e) == s); }

      using ArcNotifier = AlterationNotifier< EdgeGraphExtender, Arc, STATIC >;
      using EdgeNotifier = AlterationNotifier< EdgeGraphExtender, Edge, STATIC >;

      protected:
      mutable ArcNotifier  arc_notifier;
      mutable EdgeNotifier edge_notifier;

      public:
      using Parent::notifier;

      constexpr ArcNotifier&  notifier(Arc) const noexcept { return arc_notifier; }
      constexpr EdgeNotifier& notifier(Edge) const noexcept { return edge_notifier; }

      class NodeIt: public Node {
        const Graph* graph;

        public:
        NodeIt() = default;
        NodeIt(Invalid i) : Node(i) {}
        explicit NodeIt(const Graph& _graph) : graph(&_graph) { _graph.first(static_cast< Node& >(*this)); }
        NodeIt(const Graph& _graph, const Node& node) : Node(node), graph(&_graph) {}

        constexpr NodeIt& operator++() noexcept {
          graph->next(*this);
          return *this;
        }
      };

      class ArcIt: public Arc {
        const Graph* graph;

        public:
        ArcIt() = default;
        ArcIt(Invalid i) : Arc(i) {}
        explicit ArcIt(const Graph& _graph) : graph(&_graph) { _graph.first(static_cast< Arc& >(*this)); }
        ArcIt(const Graph& _graph, const Arc& e) : Arc(e), graph(&_graph) {}

        constexpr ArcIt& operator++() noexcept {
          graph->next(*this);
          return *this;
        }
      };

      class OutArcIt: public Arc {
        const Graph* graph;

        public:
        OutArcIt() = default;
        OutArcIt(Invalid i) : Arc(i) {}
        OutArcIt(const Graph& _graph, const Node& node) : graph(&_graph) { _graph.firstOut(*this, node); }
        OutArcIt(const Graph& _graph, const Arc& arc) : Arc(arc), graph(&_graph) {}

        constexpr OutArcIt& operator++() noexcept {
          graph->nextOut(*this);
          return *this;
        }
      };

      class InArcIt: public Arc {
        const Graph* graph;

        public:
        InArcIt() = default;
        InArcIt(Invalid i) : Arc(i) {}
        InArcIt(const Graph& _graph, const Node& node) : graph(&_graph) { _graph.firstIn(*this, node); }
        InArcIt(const Graph& _graph, const Arc& arc) : Arc(arc), graph(&_graph) {}

        constexpr InArcIt& operator++() noexcept {
          graph->nextIn(*this);
          return *this;
        }
      };

      class EdgeIt: public Parent::Edge {
        const Graph* graph;

        public:
        EdgeIt() = default;
        EdgeIt(Invalid i) : Edge(i) {}
        explicit EdgeIt(const Graph& _graph) : graph(&_graph) { _graph.first(static_cast< Edge& >(*this)); }
        EdgeIt(const Graph& _graph, const Edge& e) : Edge(e), graph(&_graph) {}

        constexpr EdgeIt& operator++() noexcept {
          graph->next(*this);
          return *this;
        }
      };

      class IncEdgeIt: public Parent::Edge {
        friend class EdgeGraphExtender;
        const Graph* graph;
        bool         direction;

        public:
        IncEdgeIt() = default;
        IncEdgeIt(Invalid i) : Edge(i), direction(false) {}
        IncEdgeIt(const Graph& _graph, const Node& n) : graph(&_graph) { _graph.firstInc(*this, direction, n); }
        IncEdgeIt(const Graph& _graph, const Edge& ue, const Node& n) : graph(&_graph), Edge(ue) {
          direction = (_graph.source(ue) == n);
        }

        constexpr IncEdgeIt& operator++() noexcept {
          graph->nextInc(*this, direction);
          return *this;
        }
      };

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the source in this case) of the iterator
       */
      constexpr Node baseNode(const OutArcIt& e) const noexcept { return Parent::source(static_cast< const Arc& >(e)); }
      
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the target in this case) of the
       * iterator
       */
      constexpr Node runningNode(const OutArcIt& e) const noexcept {
        return Parent::target(static_cast< const Arc& >(e));
      }

      /**
       * @brief Base node of the iterator
       *
       * Returns the base node (ie. the target in this case) of the iterator
       */
      constexpr Node baseNode(const InArcIt& e) const noexcept { return Parent::target(static_cast< const Arc& >(e)); }
      
      /**
       * @brief Running node of the iterator
       *
       * Returns the running node (ie. the source in this case) of the
       * iterator
       */
      constexpr Node runningNode(const InArcIt& e) const noexcept {
        return Parent::source(static_cast< const Arc& >(e));
      }

      /**
       * Base node of the iterator
       *
       * Returns the base node of the iterator
       */
      constexpr Node baseNode(const IncEdgeIt& e) const noexcept { return e.direction ? this->u(e) : this->v(e); }
      
      /**
       * Running node of the iterator
       *
       * Returns the running node of the iterator
       */
      constexpr Node runningNode(const IncEdgeIt& e) const noexcept { return e.direction ? this->v(e) : this->u(e); }

      template < typename _Value >
      class ArcMap: public MapExtender< DefaultMap< Graph, Arc, _Value > > {
        using Parent = MapExtender< DefaultMap< Graph, Arc, _Value > >;

        public:
        explicit ArcMap(const Graph& _g) : Parent(_g) {}
        ArcMap(const Graph& _g, const _Value& _v) : Parent(_g, _v) {}

        ArcMap& operator=(const ArcMap& cmap) { return operator=< ArcMap >(cmap); }

        template < typename CMap >
        ArcMap& operator=(const CMap& cmap) noexcept {
          Parent::operator=(cmap);
          return *this;
        }
      };

      template < typename _Value >
      class EdgeMap: public MapExtender< DefaultMap< Graph, Edge, _Value > > {
        using Parent = MapExtender< DefaultMap< Graph, Edge, _Value > >;

        public:
        explicit EdgeMap(const Graph& _g) : Parent(_g) {}

        EdgeMap(const Graph& _g, const _Value& _v) : Parent(_g, _v) {}

        EdgeMap& operator=(const EdgeMap& cmap) { return operator=< EdgeMap >(cmap); }

        template < typename CMap >
        EdgeMap& operator=(const CMap& cmap) noexcept {
          Parent::operator=(cmap);
          return *this;
        }
      };

      template < typename _Value >
      using StaticArcMap = details::StaticMapBase< Graph, Arc, nt::DynamicArray< _Value > >;

      template < typename _Value >
      using StaticEdgeMap = details::StaticMapBase< Graph, Edge, nt::DynamicArray< _Value > >;

      template < typename _Value, typename... ELEMENTS >
      using StaticGraphMap = StaticGraphMap< Graph, _Value, ELEMENTS... >;

      /** Alteration extension */

      constexpr Edge addEdge(const Node& from, const Node& to) noexcept {
        Edge edge = Parent::addEdge(from, to);
        notifier(Edge()).add(edge);
        const Arc arcs[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).add(arcs);
        return edge;
      }

      constexpr void clear() noexcept {
        notifier(Arc()).clear();
        notifier(Edge()).clear();
        Parent::clear();
      }

      constexpr void erase(const Edge& edge) noexcept {
        const Arc arcs[] = {Parent::direct(edge, true), Parent::direct(edge, false)};
        notifier(Arc()).erase(arcs);
        notifier(Edge()).erase(edge);
        Parent::erase(edge);
      }

      EdgeGraphExtender() noexcept {
        arc_notifier.setContainer(*this);
        edge_notifier.setContainer(*this);
      }

      ~EdgeGraphExtender() noexcept {
        edge_notifier.clear();
        arc_notifier.clear();
      }
    };

  }   // namespace graphs
}   // namespace nt

#endif
