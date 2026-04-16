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
 * This file incorporates work from the LEMON graph library (connectivity.h).
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
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Replaced std::stack with nt::Stack
 *   - Updated or enhanced Doxygen documentation
 *   - Updated header guard macros
 */

/**
 * @ingroup graph_properties
 * @file
 * @brief Connectivity algorithms
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CONNECTIVITY_H_
#define _NT_CONNECTIVITY_H_

#include "../../core/common.h"
#include "../../core/maps.h"
#include "../../core/stack.h"
#include "../../core/concepts/digraph.h"
#include "../../core/concepts/graph.h"
#include "../tools.h"
#include "../adaptors.h"
#include "bfs.h"
#include "dfs.h"
#include "../graph_properties.h"
#include "../graph_element_set.h"

#include <functional>

namespace nt {
  namespace graphs {

    /**
     * @brief Check whether the given subset of nodes forms a clique in the graph.
     *
     * @param graph
     * @param nodes
     */
    template < class Graph >
    // requires nt::concepts::Graph< Graph >
    bool isClique(const Graph& graph, const NodeSet< Graph >& nodes) {
      for (int u = 0; u < nodes.size() - 1; ++u)
        for (int v = u + 1; v < nodes.size(); ++v)
          if (nt::graphs::findEdge(graph, nodes[u], nodes[v]) == INVALID) return false;
      return true;
    }

    /**
     * @brief Check whether the given subset of nodes forms a clique in the graph.
     *
     * @param graph
     * @param nodes
     */
    template < class Digraph >
    // requires nt::concepts::Digraph< Digraph >
    void makeBidirectedClique(Digraph& g, const NodeSet< Digraph >& nodes) {
      makeBidirectedClique(g, nodes, [](typename Digraph::Arc arc) {});
    }

    template < class Digraph, typename F >
    // requires nt::concepts::Digraph< Digraph >
    void makeBidirectedClique(Digraph& g, const NodeSet< Digraph >& nodes, F callback) {
      for (int i = 0; i < nodes.size(); ++i)
        for (int j = 0; j < nodes.size(); ++j) {
          const typename Digraph::Node u = nodes[i];
          const typename Digraph::Node v = nodes[j];
          if (u != v && nt::graphs::findArc(g, u, v) == INVALID) {
            const typename Digraph::Arc arc = g.addArc(u, v);
            callback(arc);
          }
        }
    }

    /**
     * @brief Check whether a digraph is a tournament
     *
     * @return true if the input digraph is a tournament, false otherwise.
     *
     */
    template < class Digraph >
    // requires nt::concepts::Digraph< Digraph >
    bool isTournament(const Digraph& g) {
      return (g.nodeNum() < 2) || g.arcNum() == ((g.nodeNum() * (g.nodeNum() - 1)) >> 1);
    }

    /**
     * @brief Check whether a digraph is bidirected
     *
     * @return true is the input digraph is bidirected, false otherwise.
     */
    template < class Digraph >
    // requires nt::concepts::Digraph< Digraph >
    bool isBidirected(const Digraph& g) {
      for (typename Digraph::ArcIt arc(g); arc != INVALID; ++arc)
        if (nt::graphs::findArc(g, g.target(arc), g.source(arc)) == INVALID) return false;
      return true;
    }

    /**
     * @brief Make the input digraph bidirected. If the graph have static
     * arc maps attached to it (e.g StaticArcMap), then you have to pass
     * them to the function so that they can be updated with the new added arcs.
     *
     * TODO : There should be a way to get the list of attached maps to
     * the input graph instead of passing them as function parameters with
     * variadic template.
     *
     * @param graph
     * @param arc_map
     * @return The number of added arcs.
     *
     */
    template < class Digraph, class... ArcMap >
    // requires nt::concepts::Digraph< Digraph >
    int makeBidirected(Digraph& graph, ArcMap&... arc_map) {
      static_assert(!nt::concepts::BuildTagIndicator< Digraph >, "makeBidirected does support graphs with a BuildTag");
      using Node = typename Digraph::Node;
      using Arc = typename Digraph::Arc;
      using ArcIt = typename Digraph::ArcIt;

      const int n = graph.arcNum();

      // Naive implem. The use of the temporary array 'missing_arcs' is needed
      // since adding new arcs in the graph while looping over its arc set can
      // create undefined behavior
      nt::TrivialDynamicArray< Arc > missing_arcs;
      for (ArcIt arc(graph); arc != INVALID; ++arc) {
        const Node source = graph.source(arc);
        const Node target = graph.target(arc);
        if (nt::graphs::findArc(graph, target, source) == INVALID) missing_arcs.add(arc);
      }

      for (const Arc arc: missing_arcs) {
        const Node source = graph.source(arc);
        const Node target = graph.target(arc);
        const Arc  opposite_arc = graph.addArc(target, source);
        (arc_map.set(opposite_arc, arc_map[arc]), ...);
      }

      return graph.arcNum() - n;
    }


    /**
     * @brief Make the input digraph bidirected. If the graph have static
     * arc maps attached to it (e.g StaticArcMap), then you have to pass
     * them to the function so that they can be updated with the new added arcs.
     *
     * TODO : There should be a way to get the list of attached maps to
     * the input graph instead of passing them as function parameters with
     * variadic template.
     *
     * @param graph
     * @param arc_map
     * @return The number of added arcs.
     *
     */
    template < class Digraph, typename F >
    // requires nt::concepts::Digraph< Digraph >
    int makeBidirected(Digraph& graph, GraphProperties< Digraph >& gprops, F callback) {
      static_assert(!nt::concepts::BuildTagIndicator< Digraph >, "makeBidirected does support graphs with a BuildTag");
      using Node = typename Digraph::Node;
      using Arc = typename Digraph::Arc;
      using ArcIt = typename Digraph::ArcIt;

      const int n = graph.arcNum();

      // Naive implem. The use of the temporary array 'missing_arcs' is needed
      // since adding new arcs in the graph while looping over its arc set can
      // create undefined behavior
      nt::TrivialDynamicArray< Arc > missing_arcs;
      for (ArcIt arc(graph); arc != INVALID; ++arc) {
        const Node source = graph.source(arc);
        const Node target = graph.target(arc);
        if (nt::graphs::findArc(graph, target, source) == INVALID) missing_arcs.add(arc);
      }

      for (const Arc arc: missing_arcs) {
        const Node source = graph.source(arc);
        const Node target = graph.target(arc);
        const Arc  opposite_arc = graph.addArc(target, source);
        for (typename GraphProperties< Digraph >::ArcMaps::const_iterator it = gprops._arc_maps.begin();
             it != gprops._arc_maps.end();
             ++it) {
          assert(it->second);
          nt::details::MapStorage< Arc >& map = *(it->second);
          map.copy(opposite_arc, arc);
        }
        callback(opposite_arc);
      }

      return graph.arcNum() - n;
    }

    template < class Digraph >
    // requires nt::concepts::Digraph< Digraph >
    int makeBidirected(Digraph& graph, GraphProperties< Digraph >& gprops) {
      return makeBidirected(graph, gprops, [](typename Digraph::Arc arc) {});
    }


    /**
     * @brief Check whether a digraph is a Directed Acyclic Graph (DAG).
     *
     * @return true if there is no directed cycle in the digraph.
     *
     */
    template < typename Digraph >
    // requires nt::concepts::Digraph< Digraph >
    bool dag(const Digraph& g) {
      checkConcept< concepts::Digraph, Digraph >();
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

#if 1
      DfsVisit< Digraph > dfs(g);
      dfs.init();

      for (NodeIt it(g); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          while (!dfs.emptyQueue()) {
            const Arc  arc = dfs.nextArc();
            const Node target = g.target(arc);
            if (dfs.isIn(target)) return false;
            dfs.processNextArc();
          }
        }
      }

      return true;
#else
      IntStaticNodeMap in_degrees(g);
      int              i_head = -1;

      for (NodeIt s(g); s != INVALID; ++s) {
        const int in_deg = graphs::countInArcs(g, n);
        if (in_deg == 0) {
          in_degrees[s] = i_head;
          i_head = g.id(s);
        } else
          in_degrees[s] = in_deg;
      }

      while (i_head >= 0) {
        const Node u = g.nodeFromId(i_head);
        for (OutArcIt e(g, u); e != nt::INVALID; ++e) {
          const Node v = g.target(e);
          --in_degrees[v];
          if (in_degrees[v] == 0) {
            in_degrees[v] = i_head;
            i_head = g.id(v);
          }
        }
      }

#endif
    }


    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is connected.
     *
     * This function checks whether the given undirected graph is connected,
     * i.e. there is a path between any two nodes in the graph.
     *
     * @return \c true if the graph is connected.
     * @note By definition, the empty graph is connected.
     *
     * @see countConnectedComponents(), connectedComponents()
     * @see stronglyConnected()
     */
    template < typename Graph >
    bool connected(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;
      if (NodeIt(graph) == INVALID) return true;
      Dfs< Graph > dfs(graph);
      dfs.run(NodeIt(graph));
      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) { return false; }
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Count the number of connected components of an undirected graph
     *
     * This function counts the number of connected components of the given
     * undirected graph.
     *
     * The connected components are the classes of an equivalence relation
     * on the nodes of an undirected graph. Two nodes are in the same class
     * if they are connected with a path.
     *
     * @return The number of connected components.
     * @note By definition, the empty graph consists
     * of zero connected components.
     *
     * @see connected(), connectedComponents()
     */
    template < typename Graph >
    int countConnectedComponents(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using Node = typename Graph::Node;
      using Arc = typename Graph::Arc;

      using PredMap = NullMap< Node, Arc >;
      using DistMap = NullMap< Node, int >;

      int                                                                                           compNum = 0;
      typename Bfs< Graph >::template SetPredMap< PredMap >::template SetDistMap< DistMap >::Create bfs(graph);

      PredMap predMap;
      bfs.predMap(predMap);

      DistMap distMap;
      bfs.distMap(distMap);

      bfs.init();
      for (typename Graph::NodeIt n(graph); n != INVALID; ++n) {
        if (!bfs.reached(n)) {
          bfs.addSource(n);
          bfs.start();
          ++compNum;
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the connected components of an undirected graph
     *
     * This function finds the connected components of the given undirected
     * graph.
     *
     * The connected components are the classes of an equivalence relation
     * on the nodes of an undirected graph. Two nodes are in the same class
     * if they are connected with a path.
     *
     * \image html connected_components.png
     * \image latex connected_components.eps "Connected components"
     * width=\textwidth
     *
     * @param graph The undirected graph.
     * \retval compMap A writable node map. The values will be set from 0 to
     * the number of the connected components minus one. Each value of the map
     * will be set exactly once, and the values of a certain component will be
     * set continuously.
     * @return The number of connected components.
     * @note By definition, the empty graph consists
     * of zero connected components.
     *
     * @see connected(), countConnectedComponents()
     */
    template < class Graph, class NodeMap >
    int connectedComponents(const Graph& graph, NodeMap& compMap) {
      checkConcept< concepts::Graph, Graph >();
      using Node = typename Graph::Node;
      using Arc = typename Graph::Arc;
      checkConcept< concepts::WriteMap< Node, int >, NodeMap >();

      using PredMap = NullMap< Node, Arc >;
      using DistMap = NullMap< Node, int >;

      int                                                                                           compNum = 0;
      typename Bfs< Graph >::template SetPredMap< PredMap >::template SetDistMap< DistMap >::Create bfs(graph);

      PredMap predMap;
      bfs.predMap(predMap);

      DistMap distMap;
      bfs.distMap(distMap);

      bfs.init();
      for (typename Graph::NodeIt n(graph); n != INVALID; ++n) {
        if (!bfs.reached(n)) {
          bfs.addSource(n);
          while (!bfs.emptyQueue()) {
            compMap.set(bfs.nextNode(), compNum);
            bfs.processNextNode();
          }
          ++compNum;
        }
      }
      return compNum;
    }

    namespace _connectivity_bits {

      template < typename Digraph, typename Iterator >
      struct LeaveOrderVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        LeaveOrderVisitor(Iterator it) : _it(it) {}

        void leave(const Node& node) { *(_it++) = node; }

        private:
        Iterator _it;
      };

      template < typename Digraph, typename Map >
      struct FillMapVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Value = typename Map::Value;

        FillMapVisitor(Map& map, Value& value) : _map(map), _value(value) {}

        void reach(const Node& node) { _map.set(node, _value); }

        private:
        Map&   _map;
        Value& _value;
      };

      template < typename Digraph, typename ArcMap >
      struct StronglyConnectedCutArcsVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;

        StronglyConnectedCutArcsVisitor(const Digraph& digraph, ArcMap& cutMap, int& cutNum) :
            _digraph(digraph), _cutMap(cutMap), _cutNum(cutNum), _compMap(digraph, -1), _num(-1) {}

        void start(const Node&) { ++_num; }

        void reach(const Node& node) { _compMap.set(node, _num); }

        void examine(const Arc& arc) {
          if (_compMap[_digraph.source(arc)] != _compMap[_digraph.target(arc)]) {
            _cutMap.set(arc, true);
            ++_cutNum;
          }
        }

        private:
        const Digraph& _digraph;
        ArcMap&        _cutMap;
        int&           _cutNum;

        typename Digraph::template NodeMap< int > _compMap;
        int                                       _num;
      };

    }   // namespace _connectivity_bits

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether a directed graph is strongly connected.
     *
     * This function checks whether the given directed graph is strongly
     * connected, i.e. any two nodes of the digraph are
     * connected with directed paths in both direction.
     *
     * @return \c true if the digraph is strongly connected.
     * @note By definition, the empty digraph is strongly connected.
     *
     * @see countStronglyConnectedComponents(), stronglyConnectedComponents()
     * @see connected()
     */
    template < typename Digraph >
    bool stronglyConnected(const Digraph& digraph) {
      checkConcept< concepts::Digraph, Digraph >();

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;

      typename Digraph::Node source = NodeIt(digraph);
      if (source == INVALID) return true;

      using namespace _connectivity_bits;

      using Visitor = DefaultDfsVisitor< Digraph >;
      Visitor visitor;

      DfsVisit< Digraph, Visitor > dfs(digraph, visitor);
      dfs.init();
      dfs.addSource(source);
      dfs.start();

      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) { return false; }
      }

      using RDigraph = ReverseDigraph< const Digraph >;
      using RNodeIt = typename RDigraph::NodeIt;
      RDigraph rdigraph(digraph);

      using RVisitor = DefaultDfsVisitor< RDigraph >;
      RVisitor rvisitor;

      DfsVisit< RDigraph, RVisitor > rdfs(rdigraph, rvisitor);
      rdfs.init();
      rdfs.addSource(source);
      rdfs.start();

      for (RNodeIt it(rdigraph); it != INVALID; ++it) {
        if (!rdfs.reached(it)) { return false; }
      }

      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Count the number of strongly connected components of a
     * directed graph
     *
     * This function counts the number of strongly connected components of
     * the given directed graph.
     *
     * The strongly connected components are the classes of an
     * equivalence relation on the nodes of a digraph. Two nodes are in
     * the same class if they are connected with directed paths in both
     * direction.
     *
     * @return The number of strongly connected components.
     * @note By definition, the empty digraph has zero
     * strongly connected components.
     *
     * @see stronglyConnected(), stronglyConnectedComponents()
     */
    template < typename Digraph >
    int countStronglyConnectedComponents(const Digraph& digraph) {
      checkConcept< concepts::Digraph, Digraph >();

      using namespace _connectivity_bits;

      using Node = typename Digraph::Node;
      using Arc = typename Digraph::Arc;
      using NodeIt = typename Digraph::NodeIt;
      using ArcIt = typename Digraph::ArcIt;

      using Container = nt::TrivialDynamicArray< Node >;
      using Iterator = typename Container::iterator;

      Container nodes(nt::graphs::countNodes(digraph));
      using Visitor = LeaveOrderVisitor< Digraph, Iterator >;
      Visitor visitor(nodes.begin());

      DfsVisit< Digraph, Visitor > dfs(digraph, visitor);
      dfs.init();
      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }

      using RIterator = typename Container::reverse_iterator;
      using RDigraph = ReverseDigraph< const Digraph >;

      RDigraph rdigraph(digraph);

      using RVisitor = DefaultDfsVisitor< Digraph >;
      RVisitor rvisitor;

      DfsVisit< RDigraph, RVisitor > rdfs(rdigraph, rvisitor);

      int compNum = 0;

      rdfs.init();
      for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
        if (!rdfs.reached(*it)) {
          rdfs.addSource(*it);
          rdfs.start();
          ++compNum;
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the strongly connected components of a directed graph
     *
     * This function finds the strongly connected components of the given
     * directed graph. In addition, the numbering of the components will
     * satisfy that there is no arc going from a higher numbered component
     * to a lower one (i.e. it provides a topological order of the components).
     *
     * The strongly connected components are the classes of an
     * equivalence relation on the nodes of a digraph. Two nodes are in
     * the same class if they are connected with directed paths in both
     * direction.
     *
     * \image html strongly_connected_components.png
     * \image latex strongly_connected_components.eps "Strongly connected
     * components" width=\textwidth
     *
     * @param digraph The digraph.
     * \retval compMap A writable node map. The values will be set from 0 to
     * the number of the strongly connected components minus one. Each value
     * of the map will be set exactly once, and the values of a certain
     * component will be set continuously.
     * @return The number of strongly connected components.
     * @note By definition, the empty digraph has zero
     * strongly connected components.
     *
     * @see stronglyConnected(), countStronglyConnectedComponents()
     */
    template < typename Digraph, typename NodeMap >
    int stronglyConnectedComponents(const Digraph& digraph, NodeMap& compMap) {
      checkConcept< concepts::Digraph, Digraph >();
      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      checkConcept< concepts::WriteMap< Node, int >, NodeMap >();

      using namespace _connectivity_bits;

      using Container = nt::TrivialDynamicArray< Node >;
      using Iterator = typename Container::iterator;

      Container nodes(nt::graphs::countNodes(digraph));
      using Visitor = LeaveOrderVisitor< Digraph, Iterator >;
      Visitor visitor(nodes.begin());

      DfsVisit< Digraph, Visitor > dfs(digraph, visitor);
      dfs.init();
      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }

      using RIterator = typename Container::reverse_iterator;
      using RDigraph = ReverseDigraph< const Digraph >;

      RDigraph rdigraph(digraph);

      int compNum = 0;

      using RVisitor = FillMapVisitor< RDigraph, NodeMap >;
      RVisitor rvisitor(compMap, compNum);

      DfsVisit< RDigraph, RVisitor > rdfs(rdigraph, rvisitor);

      rdfs.init();
      for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
        if (!rdfs.reached(*it)) {
          rdfs.addSource(*it);
          rdfs.start();
          ++compNum;
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the cut arcs of the strongly connected components.
     *
     * This function finds the cut arcs of the strongly connected components
     * of the given digraph.
     *
     * The strongly connected components are the classes of an
     * equivalence relation on the nodes of a digraph. Two nodes are in
     * the same class if they are connected with directed paths in both
     * direction.
     * The strongly connected components are separated by the cut arcs.
     *
     * @param digraph The digraph.
     * \retval cutMap A writable arc map. The values will be set to \c true
     * for the cut arcs (exactly once for each cut arc), and will not be
     * changed for other arcs.
     * @return The number of cut arcs.
     *
     * @see stronglyConnected(), stronglyConnectedComponents()
     */
    template < typename Digraph, typename ArcMap >
    int stronglyConnectedCutArcs(const Digraph& digraph, ArcMap& cutMap) {
      checkConcept< concepts::Digraph, Digraph >();
      using Node = typename Digraph::Node;
      using Arc = typename Digraph::Arc;
      using NodeIt = typename Digraph::NodeIt;
      checkConcept< concepts::WriteMap< Arc, bool >, ArcMap >();

      using namespace _connectivity_bits;

      using Container = nt::TrivialDynamicArray< Node >;
      using Iterator = typename Container::iterator;

      Container nodes(nt::graphs::countNodes(digraph));
      using Visitor = LeaveOrderVisitor< Digraph, Iterator >;
      Visitor visitor(nodes.begin());

      DfsVisit< Digraph, Visitor > dfs(digraph, visitor);
      dfs.init();
      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }

      using RIterator = typename Container::reverse_iterator;
      using RDigraph = ReverseDigraph< const Digraph >;

      RDigraph rdigraph(digraph);

      int cutNum = 0;

      using RVisitor = StronglyConnectedCutArcsVisitor< RDigraph, ArcMap >;
      RVisitor rvisitor(rdigraph, cutMap, cutNum);

      DfsVisit< RDigraph, RVisitor > rdfs(rdigraph, rvisitor);

      rdfs.init();
      for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
        if (!rdfs.reached(*it)) {
          rdfs.addSource(*it);
          rdfs.start();
        }
      }
      return cutNum;
    }

    namespace _connectivity_bits {

      template < typename Digraph >
      class CountBiNodeConnectedComponentsVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        CountBiNodeConnectedComponentsVisitor(const Digraph& graph, int& compNum) :
            _graph(graph), _compNum(compNum), _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

        void start(const Node& node) { _predMap.set(node, INVALID); }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          ++_num;
        }

        void discover(const Arc& edge) { _predMap.set(_graph.target(edge), _graph.source(edge)); }

        void examine(const Arc& edge) {
          if (_graph.source(edge) == _graph.target(edge) && _graph.direction(edge)) {
            ++_compNum;
            return;
          }
          if (_predMap[_graph.source(edge)] == _graph.target(edge)) { return; }
          if (_retMap[_graph.source(edge)] > _numMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _numMap[_graph.target(edge)]);
          }
        }

        void backtrack(const Arc& edge) {
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
          if (_numMap[_graph.source(edge)] <= _retMap[_graph.target(edge)]) { ++_compNum; }
        }

        private:
        const Digraph& _graph;
        int&           _compNum;

        typename Digraph::template NodeMap< int >  _numMap;
        typename Digraph::template NodeMap< int >  _retMap;
        typename Digraph::template NodeMap< Node > _predMap;
        int                                        _num;
      };

      template < typename Digraph, typename ArcMap >
      class BiNodeConnectedComponentsVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        BiNodeConnectedComponentsVisitor(const Digraph& graph, ArcMap& compMap, int& compNum) :
            _graph(graph), _compMap(compMap), _compNum(compNum), _numMap(graph), _retMap(graph), _predMap(graph),
            _num(0) {}

        void start(const Node& node) { _predMap.set(node, INVALID); }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          ++_num;
        }

        void discover(const Arc& edge) {
          Node target = _graph.target(edge);
          _predMap.set(target, edge);
          _edgeStack.push(edge);
        }

        void examine(const Arc& edge) {
          Node source = _graph.source(edge);
          Node target = _graph.target(edge);
          if (source == target && _graph.direction(edge)) {
            _compMap.set(edge, _compNum);
            ++_compNum;
            return;
          }
          if (_numMap[target] < _numMap[source]) {
            if (_predMap[source] != _graph.oppositeArc(edge)) { _edgeStack.push(edge); }
          }
          if (_predMap[source] != INVALID && target == _graph.source(_predMap[source])) { return; }
          if (_retMap[source] > _numMap[target]) { _retMap.set(source, _numMap[target]); }
        }

        void backtrack(const Arc& edge) {
          Node source = _graph.source(edge);
          Node target = _graph.target(edge);
          if (_retMap[source] > _retMap[target]) { _retMap.set(source, _retMap[target]); }
          if (_numMap[source] <= _retMap[target]) {
            while (_edgeStack.top() != edge) {
              _compMap.set(_edgeStack.top(), _compNum);
              _edgeStack.pop();
            }
            _compMap.set(edge, _compNum);
            _edgeStack.pop();
            ++_compNum;
          }
        }

        private:
        const Digraph& _graph;
        ArcMap&        _compMap;
        int&           _compNum;

        typename Digraph::template NodeMap< int > _numMap;
        typename Digraph::template NodeMap< int > _retMap;
        typename Digraph::template NodeMap< Arc > _predMap;
        nt::Stack< Edge >                         _edgeStack;
        int                                       _num;
      };

      template < typename Digraph, typename NodeMap >
      class BiNodeConnectedCutNodesVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        BiNodeConnectedCutNodesVisitor(const Digraph& graph, NodeMap& cutMap, int& cutNum) :
            _graph(graph), _cutMap(cutMap), _cutNum(cutNum), _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

        void start(const Node& node) {
          _predMap.set(node, INVALID);
          rootCut = false;
        }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          ++_num;
        }

        void discover(const Arc& edge) { _predMap.set(_graph.target(edge), _graph.source(edge)); }

        void examine(const Arc& edge) {
          if (_graph.source(edge) == _graph.target(edge) && _graph.direction(edge)) {
            if (!_cutMap[_graph.source(edge)]) {
              _cutMap.set(_graph.source(edge), true);
              ++_cutNum;
            }
            return;
          }
          if (_predMap[_graph.source(edge)] == _graph.target(edge)) return;
          if (_retMap[_graph.source(edge)] > _numMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _numMap[_graph.target(edge)]);
          }
        }

        void backtrack(const Arc& edge) {
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
          if (_numMap[_graph.source(edge)] <= _retMap[_graph.target(edge)]) {
            if (_predMap[_graph.source(edge)] != INVALID) {
              if (!_cutMap[_graph.source(edge)]) {
                _cutMap.set(_graph.source(edge), true);
                ++_cutNum;
              }
            } else if (rootCut) {
              if (!_cutMap[_graph.source(edge)]) {
                _cutMap.set(_graph.source(edge), true);
                ++_cutNum;
              }
            } else {
              rootCut = true;
            }
          }
        }

        private:
        const Digraph& _graph;
        NodeMap&       _cutMap;
        int&           _cutNum;

        typename Digraph::template NodeMap< int >  _numMap;
        typename Digraph::template NodeMap< int >  _retMap;
        typename Digraph::template NodeMap< Node > _predMap;
        nt::Stack< Edge >                          _edgeStack;
        int                                        _num;
        bool                                       rootCut;
      };

    }   // namespace _connectivity_bits

    template < typename Graph >
    int countBiNodeConnectedComponents(const Graph& graph);

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is bi-node-connected.
     *
     * This function checks whether the given undirected graph is
     * bi-node-connected, i.e. a connected graph without articulation
     * node.
     *
     * @return \c true if the graph bi-node-connected.
     *
     * @note By definition,
     * \li a graph consisting of zero or one node is bi-node-connected,
     * \li a graph consisting of two isolated nodes
     * is \e not bi-node-connected and
     * \li a graph consisting of two nodes connected by an edge
     * is bi-node-connected.
     *
     * @see countBiNodeConnectedComponents(), biNodeConnectedComponents()
     */
    template < typename Graph >
    bool biNodeConnected(const Graph& graph) {
      bool hasNonIsolated = false, hasIsolated = false;
      for (typename Graph::NodeIt n(graph); n != INVALID; ++n) {
        if (typename Graph::OutArcIt(graph, n) == INVALID) {
          if (hasIsolated || hasNonIsolated) {
            return false;
          } else {
            hasIsolated = true;
          }
        } else {
          if (hasIsolated) {
            return false;
          } else {
            hasNonIsolated = true;
          }
        }
      }
      return countBiNodeConnectedComponents(graph) <= 1;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Count the number of bi-node-connected components of an
     * undirected graph.
     *
     * This function counts the number of bi-node-connected components of
     * the given undirected graph.
     *
     * The bi-node-connected components are the classes of an equivalence
     * relation on the edges of a undirected graph. Two edges are in the
     * same class if they are on same circle.
     *
     * @return The number of bi-node-connected components.
     *
     * @see biNodeConnected(), biNodeConnectedComponents()
     */
    template < typename Graph >
    int countBiNodeConnectedComponents(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;

      using namespace _connectivity_bits;

      using Visitor = CountBiNodeConnectedComponentsVisitor< Graph >;

      int     compNum = 0;
      Visitor visitor(graph, compNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the bi-node-connected components of an undirected graph.
     *
     * This function finds the bi-node-connected components of the given
     * undirected graph.
     *
     * The bi-node-connected components are the classes of an equivalence
     * relation on the edges of a undirected graph. Two edges are in the
     * same class if they are on same circle.
     *
     * \image html node_biconnected_components.png
     * \image latex node_biconnected_components.eps "bi-node-connected components"
     * width=\textwidth
     *
     * @param graph The undirected graph.
     * \retval compMap A writable edge map. The values will be set from 0
     * to the number of the bi-node-connected components minus one. Each
     * value of the map will be set exactly once, and the values of a
     * certain component will be set continuously.
     * @return The number of bi-node-connected components.
     *
     * @see biNodeConnected(), countBiNodeConnectedComponents()
     */
    template < typename Graph, typename EdgeMap >
    int biNodeConnectedComponents(const Graph& graph, EdgeMap& compMap) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;
      using Edge = typename Graph::Edge;
      checkConcept< concepts::WriteMap< Edge, int >, EdgeMap >();

      using namespace _connectivity_bits;

      using Visitor = BiNodeConnectedComponentsVisitor< Graph, EdgeMap >;

      int     compNum = 0;
      Visitor visitor(graph, compMap, compNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the bi-node-connected cut nodes in an undirected graph.
     *
     * This function finds the bi-node-connected cut nodes in the given
     * undirected graph.
     *
     * The bi-node-connected components are the classes of an equivalence
     * relation on the edges of a undirected graph. Two edges are in the
     * same class if they are on same circle.
     * The bi-node-connected components are separted by the cut nodes of
     * the components.
     *
     * @param graph The undirected graph.
     * \retval cutMap A writable node map. The values will be set to
     * \c true for the nodes that separate two or more components
     * (exactly once for each cut node), and will not be changed for
     * other nodes.
     * @return The number of the cut nodes.
     *
     * @see biNodeConnected(), biNodeConnectedComponents()
     */
    template < typename Graph, typename NodeMap >
    int biNodeConnectedCutNodes(const Graph& graph, NodeMap& cutMap) {
      checkConcept< concepts::Graph, Graph >();
      using Node = typename Graph::Node;
      using NodeIt = typename Graph::NodeIt;
      checkConcept< concepts::WriteMap< Node, bool >, NodeMap >();

      using namespace _connectivity_bits;

      using Visitor = BiNodeConnectedCutNodesVisitor< Graph, NodeMap >;

      int     cutNum = 0;
      Visitor visitor(graph, cutMap, cutNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return cutNum;
    }

    namespace _connectivity_bits {

      template < typename Digraph >
      class CountBiEdgeConnectedComponentsVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        CountBiEdgeConnectedComponentsVisitor(const Digraph& graph, int& compNum) :
            _graph(graph), _compNum(compNum), _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

        void start(const Node& node) { _predMap.set(node, INVALID); }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          ++_num;
        }

        void leave(const Node& node) {
          if (_numMap[node] <= _retMap[node]) { ++_compNum; }
        }

        void discover(const Arc& edge) { _predMap.set(_graph.target(edge), edge); }

        void examine(const Arc& edge) {
          if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) { return; }
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        void backtrack(const Arc& edge) {
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        private:
        const Digraph& _graph;
        int&           _compNum;

        typename Digraph::template NodeMap< int > _numMap;
        typename Digraph::template NodeMap< int > _retMap;
        typename Digraph::template NodeMap< Arc > _predMap;
        int                                       _num;
      };

      template < typename Digraph, typename NodeMap >
      class BiEdgeConnectedComponentsVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        BiEdgeConnectedComponentsVisitor(const Digraph& graph, NodeMap& compMap, int& compNum) :
            _graph(graph), _compMap(compMap), _compNum(compNum), _numMap(graph), _retMap(graph), _predMap(graph),
            _num(0) {}

        void start(const Node& node) { _predMap.set(node, INVALID); }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          _nodeStack.push(node);
          ++_num;
        }

        void leave(const Node& node) {
          if (_numMap[node] <= _retMap[node]) {
            while (_nodeStack.top() != node) {
              _compMap.set(_nodeStack.top(), _compNum);
              _nodeStack.pop();
            }
            _compMap.set(node, _compNum);
            _nodeStack.pop();
            ++_compNum;
          }
        }

        void discover(const Arc& edge) { _predMap.set(_graph.target(edge), edge); }

        void examine(const Arc& edge) {
          if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) { return; }
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        void backtrack(const Arc& edge) {
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        private:
        const Digraph& _graph;
        NodeMap&       _compMap;
        int&           _compNum;

        typename Digraph::template NodeMap< int > _numMap;
        typename Digraph::template NodeMap< int > _retMap;
        typename Digraph::template NodeMap< Arc > _predMap;
        nt::Stack< Node >                         _nodeStack;
        int                                       _num;
      };

      template < typename Digraph, typename ArcMap >
      class BiEdgeConnectedCutEdgesVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using Edge = typename Digraph::Edge;

        BiEdgeConnectedCutEdgesVisitor(const Digraph& graph, ArcMap& cutMap, int& cutNum) :
            _graph(graph), _cutMap(cutMap), _cutNum(cutNum), _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

        void start(const Node& node) { _predMap[node] = INVALID; }

        void reach(const Node& node) {
          _numMap.set(node, _num);
          _retMap.set(node, _num);
          ++_num;
        }

        void leave(const Node& node) {
          if (_numMap[node] <= _retMap[node]) {
            if (_predMap[node] != INVALID) {
              _cutMap.set(_predMap[node], true);
              ++_cutNum;
            }
          }
        }

        void discover(const Arc& edge) { _predMap.set(_graph.target(edge), edge); }

        void examine(const Arc& edge) {
          if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) { return; }
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        void backtrack(const Arc& edge) {
          if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
            _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
          }
        }

        private:
        const Digraph& _graph;
        ArcMap&        _cutMap;
        int&           _cutNum;

        typename Digraph::template NodeMap< int > _numMap;
        typename Digraph::template NodeMap< int > _retMap;
        typename Digraph::template NodeMap< Arc > _predMap;
        int                                       _num;
      };
    }   // namespace _connectivity_bits

    template < typename Graph >
    int countBiEdgeConnectedComponents(const Graph& graph);

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is bi-edge-connected.
     *
     * This function checks whether the given undirected graph is
     * bi-edge-connected, i.e. any two nodes are connected with at least
     * two edge-disjoint paths.
     *
     * @return \c true if the graph is bi-edge-connected.
     * @note By definition, the empty graph is bi-edge-connected.
     *
     * @see countBiEdgeConnectedComponents(), biEdgeConnectedComponents()
     */
    template < typename Graph >
    bool biEdgeConnected(const Graph& graph) {
      return countBiEdgeConnectedComponents(graph) <= 1;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Count the number of bi-edge-connected components of an
     * undirected graph.
     *
     * This function counts the number of bi-edge-connected components of
     * the given undirected graph.
     *
     * The bi-edge-connected components are the classes of an equivalence
     * relation on the nodes of an undirected graph. Two nodes are in the
     * same class if they are connected with at least two edge-disjoint
     * paths.
     *
     * @return The number of bi-edge-connected components.
     *
     * @see biEdgeConnected(), biEdgeConnectedComponents()
     */
    template < typename Graph >
    int countBiEdgeConnectedComponents(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;

      using namespace _connectivity_bits;

      using Visitor = CountBiEdgeConnectedComponentsVisitor< Graph >;

      int     compNum = 0;
      Visitor visitor(graph, compNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the bi-edge-connected components of an undirected graph.
     *
     * This function finds the bi-edge-connected components of the given
     * undirected graph.
     *
     * The bi-edge-connected components are the classes of an equivalence
     * relation on the nodes of an undirected graph. Two nodes are in the
     * same class if they are connected with at least two edge-disjoint
     * paths.
     *
     * \image html edge_biconnected_components.png
     * \image latex edge_biconnected_components.eps "bi-edge-connected components"
     * width=\textwidth
     *
     * @param graph The undirected graph.
     * \retval compMap A writable node map. The values will be set from 0 to
     * the number of the bi-edge-connected components minus one. Each value
     * of the map will be set exactly once, and the values of a certain
     * component will be set continuously.
     * @return The number of bi-edge-connected components.
     *
     * @see biEdgeConnected(), countBiEdgeConnectedComponents()
     */
    template < typename Graph, typename NodeMap >
    int biEdgeConnectedComponents(const Graph& graph, NodeMap& compMap) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;
      using Node = typename Graph::Node;
      checkConcept< concepts::WriteMap< Node, int >, NodeMap >();

      using namespace _connectivity_bits;

      using Visitor = BiEdgeConnectedComponentsVisitor< Graph, NodeMap >;

      int     compNum = 0;
      Visitor visitor(graph, compMap, compNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return compNum;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the bi-edge-connected cut edges in an undirected graph.
     *
     * This function finds the bi-edge-connected cut edges in the given
     * undirected graph.
     *
     * The bi-edge-connected components are the classes of an equivalence
     * relation on the nodes of an undirected graph. Two nodes are in the
     * same class if they are connected with at least two edge-disjoint
     * paths.
     * The bi-edge-connected components are separted by the cut edges of
     * the components.
     *
     * @param graph The undirected graph.
     * \retval cutMap A writable edge map. The values will be set to \c true
     * for the cut edges (exactly once for each cut edge), and will not be
     * changed for other edges.
     * @return The number of cut edges.
     *
     * @see biEdgeConnected(), biEdgeConnectedComponents()
     */
    template < typename Graph, typename EdgeMap >
    int biEdgeConnectedCutEdges(const Graph& graph, EdgeMap& cutMap) {
      checkConcept< concepts::Graph, Graph >();
      using NodeIt = typename Graph::NodeIt;
      using Edge = typename Graph::Edge;
      checkConcept< concepts::WriteMap< Edge, bool >, EdgeMap >();

      using namespace _connectivity_bits;

      using Visitor = BiEdgeConnectedCutEdgesVisitor< Graph, EdgeMap >;

      int     cutNum = 0;
      Visitor visitor(graph, cutMap, cutNum);

      DfsVisit< Graph, Visitor > dfs(graph, visitor);
      dfs.init();

      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
      return cutNum;
    }

    namespace _connectivity_bits {

      template < typename Digraph, typename IntNodeMap >
      class TopologicalSortVisitor: public DefaultDfsVisitor< Digraph > {
        public:
        using Node = typename Digraph::Node;
        using edge = typename Digraph::Arc;

        TopologicalSortVisitor(IntNodeMap& order, int num) : _order(order), _num(num) {}

        void leave(const Node& node) { _order.set(node, --_num); }

        private:
        IntNodeMap& _order;
        int         _num;
      };

    }   // namespace _connectivity_bits

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether a digraph is DAG.
     *
     * This function checks whether the given digraph is DAG, i.e.
     * \e Directed \e Acyclic \e Graph.
     * @return \c true if there is no directed cycle in the digraph.
     * @see acyclic()
     */
    // template < typename Digraph >
    // bool dag(const Digraph& digraph) {
    //   checkConcept< concepts::Digraph, Digraph >();

    //   typedef typename Digraph::Node   Node;
    //   typedef typename Digraph::NodeIt NodeIt;
    //   typedef typename Digraph::Arc    Arc;

    //   typedef typename Digraph::template NodeMap< bool > ProcessedMap;

    //   typename Dfs< Digraph >::template SetProcessedMap< ProcessedMap >::Create dfs(digraph);

    //   ProcessedMap processed(digraph);
    //   dfs.processedMap(processed);

    //   dfs.init();
    //   for (NodeIt it(digraph); it != INVALID; ++it) {
    //     if (!dfs.reached(it)) {
    //       dfs.addSource(it);
    //       while (!dfs.emptyQueue()) {
    //         Arc  arc = dfs.nextArc();
    //         Node target = digraph.target(arc);
    //         if (dfs.reached(target) && !processed[target]) { return false; }
    //         dfs.processNextArc();
    //       }
    //     }
    //   }
    //   return true;
    // }

    /**
     * @ingroup graph_properties
     *
     * @brief Sort the nodes of a DAG into topolgical order.
     *
     * This function sorts the nodes of the given acyclic digraph (DAG)
     * into topolgical order.
     *
     * @param digraph The digraph, which must be DAG.
     * \retval order A writable node map. The values will be set from 0 to
     * the number of the nodes in the digraph minus one. Each value of the
     * map will be set exactly once, and the values will be set descending
     * order.
     *
     * @see dag(), checkedTopologicalSort()
     */
    template < typename Digraph, typename NodeMap >
    void topologicalSort(const Digraph& digraph, NodeMap& order) {
      using namespace _connectivity_bits;

      checkConcept< concepts::Digraph, Digraph >();
      checkConcept< concepts::WriteMap< typename Digraph::Node, int >, NodeMap >();

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;

      TopologicalSortVisitor< Digraph, NodeMap > visitor(order, nt::graphs::countNodes(digraph));

      DfsVisit< Digraph, TopologicalSortVisitor< Digraph, NodeMap > > dfs(digraph, visitor);

      dfs.init();
      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          dfs.start();
        }
      }
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Sort the nodes of a DAG into topolgical order.
     *
     * This function sorts the nodes of the given acyclic digraph (DAG)
     * into topolgical order and also checks whether the given digraph
     * is DAG.
     *
     * @param digraph The digraph.
     * \retval order A readable and writable node map. The values will be
     * set from 0 to the number of the nodes in the digraph minus one.
     * Each value of the map will be set exactly once, and the values will
     * be set descending order.
     * @return \c false if the digraph is not DAG.
     *
     * @see dag(), topologicalSort()
     */
    template < typename Digraph, typename NodeMap >
    bool checkedTopologicalSort(const Digraph& digraph, NodeMap& order) {
      using namespace _connectivity_bits;

      checkConcept< concepts::Digraph, Digraph >();
      checkConcept< concepts::ReadWriteMap< typename Digraph::Node, int >, NodeMap >();

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;

      for (NodeIt it(digraph); it != INVALID; ++it) {
        order.set(it, -1);
      }

      TopologicalSortVisitor< Digraph, NodeMap > visitor(order, nt::graphs::countNodes(digraph));

      DfsVisit< Digraph, TopologicalSortVisitor< Digraph, NodeMap > > dfs(digraph, visitor);

      dfs.init();
      for (NodeIt it(digraph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          while (!dfs.emptyQueue()) {
            Arc  arc = dfs.nextArc();
            Node target = digraph.target(arc);
            if (dfs.reached(target) && order[target] == -1) { return false; }
            dfs.processNextArc();
          }
        }
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is acyclic.
     *
     * This function checks whether the given undirected graph is acyclic.
     * @return \c true if there is no cycle in the graph.
     * @see dag()
     */
    template < typename Graph >
    bool acyclic(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using Node = typename Graph::Node;
      using NodeIt = typename Graph::NodeIt;
      using Arc = typename Graph::Arc;
      Dfs< Graph > dfs(graph);
      dfs.init();
      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) {
          dfs.addSource(it);
          while (!dfs.emptyQueue()) {
            Arc  arc = dfs.nextArc();
            Node source = graph.source(arc);
            Node target = graph.target(arc);
            if (dfs.reached(target) && dfs.predArc(source) != graph.oppositeArc(arc)) { return false; }
            dfs.processNextArc();
          }
        }
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is tree.
     *
     * This function checks whether the given undirected graph is tree.
     * @return \c true if the graph is acyclic and connected.
     * @see acyclic(), connected()
     */
    template < typename Graph >
    bool tree(const Graph& graph) {
      checkConcept< concepts::Graph, Graph >();
      using Node = typename Graph::Node;
      using NodeIt = typename Graph::NodeIt;
      using Arc = typename Graph::Arc;
      if (NodeIt(graph) == INVALID) return true;
      Dfs< Graph > dfs(graph);
      dfs.init();
      dfs.addSource(NodeIt(graph));
      while (!dfs.emptyQueue()) {
        Arc  arc = dfs.nextArc();
        Node source = graph.source(arc);
        Node target = graph.target(arc);
        if (dfs.reached(target) && dfs.predArc(source) != graph.oppositeArc(arc)) { return false; }
        dfs.processNextArc();
      }
      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!dfs.reached(it)) { return false; }
      }
      return true;
    }

    namespace _connectivity_bits {

      template < typename Digraph >
      class BipartiteVisitor: public BfsVisitor< Digraph > {
        public:
        using Arc = typename Digraph::Arc;
        using Node = typename Digraph::Node;

        BipartiteVisitor(const Digraph& graph, bool& bipartite) : _graph(graph), _part(graph), _bipartite(bipartite) {}

        void start(const Node& node) { _part[node] = true; }
        void discover(const Arc& edge) { _part.set(_graph.target(edge), !_part[_graph.source(edge)]); }
        void examine(const Arc& edge) {
          _bipartite = _bipartite && _part[_graph.target(edge)] != _part[_graph.source(edge)];
        }

        private:
        const Digraph&                             _graph;
        typename Digraph::template NodeMap< bool > _part;
        bool&                                      _bipartite;
      };

      template < typename Digraph, typename PartMap >
      class BipartitePartitionsVisitor: public BfsVisitor< Digraph > {
        public:
        using Arc = typename Digraph::Arc;
        using Node = typename Digraph::Node;

        BipartitePartitionsVisitor(const Digraph& graph, PartMap& part, bool& bipartite) :
            _graph(graph), _part(part), _bipartite(bipartite) {}

        void start(const Node& node) { _part.set(node, true); }
        void discover(const Arc& edge) { _part.set(_graph.target(edge), !_part[_graph.source(edge)]); }
        void examine(const Arc& edge) {
          _bipartite = _bipartite && _part[_graph.target(edge)] != _part[_graph.source(edge)];
        }

        private:
        const Digraph& _graph;
        PartMap&       _part;
        bool&          _bipartite;
      };
    }   // namespace _connectivity_bits

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether an undirected graph is bipartite.
     *
     * The function checks whether the given undirected graph is bipartite.
     * @return \c true if the graph is bipartite.
     *
     * @see bipartitePartitions()
     */
    template < typename Graph >
    bool bipartite(const Graph& graph) {
      using namespace _connectivity_bits;

      checkConcept< concepts::Graph, Graph >();

      using NodeIt = typename Graph::NodeIt;
      using ArcIt = typename Graph::ArcIt;

      bool bipartite = true;

      BipartiteVisitor< Graph >                    visitor(graph, bipartite);
      BfsVisit< Graph, BipartiteVisitor< Graph > > bfs(graph, visitor);
      bfs.init();
      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!bfs.reached(it)) {
          bfs.addSource(it);
          while (!bfs.emptyQueue()) {
            bfs.processNextNode();
            if (!bipartite) return false;
          }
        }
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Find the bipartite partitions of an undirected graph.
     *
     * This function checks whether the given undirected graph is bipartite
     * and gives back the bipartite partitions.
     *
     * \image html bipartite_partitions.png
     * \image latex bipartite_partitions.eps "Bipartite partititions"
     * width=\textwidth
     *
     * @param graph The undirected graph.
     * \retval partMap A writable node map of \c bool (or convertible) value
     * type. The values will be set to \c true for one component and
     * \c false for the other one.
     * @return \c true if the graph is bipartite, \c false otherwise.
     *
     * @see bipartite()
     */
    template < typename Graph, typename NodeMap >
    bool bipartitePartitions(const Graph& graph, NodeMap& partMap) {
      using namespace _connectivity_bits;

      checkConcept< concepts::Graph, Graph >();
      checkConcept< concepts::WriteMap< typename Graph::Node, bool >, NodeMap >();

      using Node = typename Graph::Node;
      using NodeIt = typename Graph::NodeIt;
      using ArcIt = typename Graph::ArcIt;

      bool bipartite = true;

      BipartitePartitionsVisitor< Graph, NodeMap >                    visitor(graph, partMap, bipartite);
      BfsVisit< Graph, BipartitePartitionsVisitor< Graph, NodeMap > > bfs(graph, visitor);
      bfs.init();
      for (NodeIt it(graph); it != INVALID; ++it) {
        if (!bfs.reached(it)) {
          bfs.addSource(it);
          while (!bfs.emptyQueue()) {
            bfs.processNextNode();
            if (!bipartite) return false;
          }
        }
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether the given graph contains no loop arcs/edges.
     *
     * This function returns \c true if there are no loop arcs/edges in
     * the given graph. It works for both directed and undirected graphs.
     */
    template < typename Graph >
    bool loopFree(const Graph& graph) {
      for (typename Graph::ArcIt it(graph); it != INVALID; ++it) {
        if (graph.source(it) == graph.target(it)) return false;
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether the given graph contains no parallel arcs/edges.
     *
     * This function returns \c true if there are no parallel arcs/edges in
     * the given graph. It works for both directed and undirected graphs.
     */
    template < typename Graph >
    bool parallelFree(const Graph& graph) {
      typename Graph::template NodeMap< int > reached(graph, 0);
      int                                     cnt = 1;
      for (typename Graph::NodeIt n(graph); n != INVALID; ++n) {
        for (typename Graph::OutArcIt a(graph, n); a != INVALID; ++a) {
          if (reached[graph.target(a)] == cnt) return false;
          reached[graph.target(a)] = cnt;
        }
        ++cnt;
      }
      return true;
    }

    /**
     * @ingroup graph_properties
     *
     * @brief Check whether the given graph is simple.
     *
     * This function returns \c true if the given graph is simple, i.e.
     * it contains no loop arcs/edges and no parallel arcs/edges.
     * The function works for both directed and undirected graphs.
     * @see loopFree(), parallelFree()
     */
    template < typename Graph >
    bool simpleGraph(const Graph& graph) {
      typename Graph::template NodeMap< int > reached(graph, 0);
      int                                     cnt = 1;
      for (typename Graph::NodeIt n(graph); n != INVALID; ++n) {
        reached[n] = cnt;
        for (typename Graph::OutArcIt a(graph, n); a != INVALID; ++a) {
          if (reached[graph.target(a)] == cnt) return false;
          reached[graph.target(a)] = cnt;
        }
        ++cnt;
      }
      return true;
    }

  }   // namespace graphs
}   // namespace nt

#endif
