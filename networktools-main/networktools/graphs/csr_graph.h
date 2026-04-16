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
 * @file csr_graph.h
 * @brief Compressed Sparse Row (CSR) directed graph implementation.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CSR_GRAPH_H_
#define _NT_CSR_GRAPH_H_

#include "../core/common.h"
#include "../core/maps.h"
#include "../core/string.h"
#include "../core/format.h"

#include "tools.h"

#include "bits/graph_extender.h"

#include <algorithm>
#include <numeric>

namespace nt {
  namespace graphs {

    struct NoProperty {};

    namespace details {
      template < typename PROPERTY, int ID >
      struct PropertyContainer {
        nt::DynamicArray< PROPERTY > _props;

        constexpr void            add() noexcept { _props.add(); }
        constexpr void            removeAll() noexcept { _props.removeAll(); }
        constexpr void            ensure(int n) noexcept { _props.ensure(n); }
        constexpr void            ensureAndFill(int n) noexcept { _props.ensureAndFill(n); }
        constexpr PROPERTY&       operator[](int k) noexcept { return _props[k]; }
        constexpr const PROPERTY& operator[](int k) const noexcept { return _props[k]; }
        constexpr void            shiftRightAndExtend(int i_pos) noexcept { _props.shiftRightAndExtend(i_pos); }
        constexpr void            shiftLeftAndShrink(int i_pos) noexcept { _props.shiftLeftAndShrink(i_pos); }
      };

      template < int ID >
      struct PropertyContainer< NoProperty, ID > {
        constexpr void       add() const noexcept {}
        constexpr void       removeAll() const noexcept {}
        constexpr void       ensure(int) const noexcept {}
        constexpr void       ensureAndFill(int) const noexcept {}
        constexpr NoProperty operator[](int) const noexcept { return NoProperty(); }
        constexpr void       shiftRightAndExtend(int i_pos) noexcept {}
        constexpr void       shiftLeftAndShrink(int i_pos) noexcept {}
      };
    };   // namespace details

    template < class NODE_PROPERTY, class ARC_PROPERTY, bool MULTIGRAPH >
    struct CsrPropertyDigraphBase:
        details::PropertyContainer< NODE_PROPERTY, 0 >,
        details::PropertyContainer< ARC_PROPERTY, 1 > {
      using NodePropContainer = details::PropertyContainer< NODE_PROPERTY, 0 >;
      using ArcPropContainer = details::PropertyContainer< ARC_PROPERTY, 1 >;

      using NodeNumTag = nt::True;
      using ArcNumTag = nt::True;
      using BuildTag = nt::True;
      using FindArcTag = nt::True;

      using NodeIndex = int;
      using ArcIndex = int;
      using Degree = int;
      using Multiplicity = int;

      using ArcProp = ARC_PROPERTY;
      using NodeProp = NODE_PROPERTY;

      struct Node {
        NodeIndex id;
        Node() = default;
        constexpr Node(int _id) noexcept : id(_id) {}
        constexpr Node(nt::Invalid) noexcept : id(-1) {}
        constexpr bool operator==(const Node& node) const noexcept { return id == node.id; }
        constexpr bool operator!=(const Node& node) const noexcept { return id != node.id; }
        constexpr bool operator<(const Node& node) const noexcept { return id < node.id; }
      };

      struct Arc {
        ArcIndex id;
        int      i_bound;
        Arc() = default;
        constexpr Arc(int _id) noexcept : id(_id), i_bound(0) {}
        constexpr Arc(nt::Invalid) noexcept : id(-1), i_bound(-1) {}
        constexpr bool operator==(const Arc& arc) const noexcept { return id == arc.id; }
        constexpr bool operator!=(const Arc& arc) const noexcept { return id != arc.id; }
        constexpr bool operator<(const Arc& arc) const noexcept { return id < arc.id; }
      };

      // Nodes related data
      nt::TrivialDynamicArray< ArcIndex >     _offsets;
      nt::TrivialDynamicArray< Degree >       _out_degrees;
      nt::TrivialDynamicArray< Degree >       _in_degrees;
      nt::TrivialDynamicArray< Multiplicity > _out_multiplicity;   // Invariant: out_multiplicity = ⅀ arc.multiplicity
                                                                   // for arc ∊ Node.outArcs
      nt::TrivialDynamicArray< Multiplicity > _in_multiplicity;

      // Arcs related data
      nt::TrivialDynamicArray< NodeIndex >    _sources;
      nt::TrivialDynamicArray< NodeIndex >    _targets;
      nt::TrivialDynamicArray< Multiplicity > _arc_multiplicity;


      Node addNode() noexcept {
        const NodeIndex i_node = static_cast< NodeIndex >(_offsets.add());
        _offsets[i_node] = arcNum();
        _out_degrees[_out_degrees.add()] = 0;
        _in_degrees[_in_degrees.add()] = 0;
        _out_multiplicity[_out_multiplicity.add()] = 0;
        _in_multiplicity[_in_multiplicity.add()] = 0;
        NodePropContainer::add();
        return Node(i_node);
      }

      Arc addArc(Node u, Node v) noexcept {
        const NodeIndex i_source = id(u);
        const NodeIndex i_target = id(v);
        assert(i_source != i_target);

        ArcIndex i_arc;
        // For performance reasons
        // If hasArc don't find the arc (i_source, i_target)
        // It returns the index where the arc should be inserted
        // To preserve the properties of a CSR graph (sorted by i_source then by
        // i_target) Find otherwise Get Index in one action
        if (hasArc(i_source, i_target, i_arc) && !MULTIGRAPH) {
          addArcMultiplicity(i_arc, 1);
          return i_arc;
        }

        // Increment by one the offset value of vertices after i_source
        for (int k = i_source + 1; k < nodeNum(); ++k)
          ++_offsets[k];
        ++_in_degrees[i_target];
        ++_out_degrees[i_source];

        // Shift by one all elements after i_arc
        _sources.shiftRightAndExtend(i_arc);
        _targets.shiftRightAndExtend(i_arc);
        _arc_multiplicity.shiftRightAndExtend(i_arc);
        ArcPropContainer::shiftRightAndExtend(i_arc);

        // Insert
        _sources[i_arc] = i_source;
        _targets[i_arc] = i_target;
        _arc_multiplicity[i_arc] = 0;

        // As we shift arcs, some values can be set
        addArcMultiplicity(i_arc, 1);

        assert(_offsets[nodeNum() - 1] + _out_degrees[nodeNum() - 1] == arcNum());
        return i_arc;
      }

      void erase(const Node& node) noexcept {
        assert(false);   // Not Yet Implemented
      }

      void erase(const Arc& arc) noexcept {
        const ArcIndex  i_arc = id(arc);
        const NodeIndex i_source = id(source(arc));
        const NodeIndex i_target = id(target(arc));

        addArcMultiplicity(arc, -1);
        if constexpr (!MULTIGRAPH) {
          if (_arc_multiplicity[i_arc] > 0) return;
        }

        for (int k = i_source + 1; k < nodeNum(); ++k)
          --_offsets[k];
        --_out_degrees[i_source];
        --_in_degrees[i_target];

        _sources.shiftLeftAndShrink(i_arc + 1);
        _targets.shiftLeftAndShrink(i_arc + 1);
        _arc_multiplicity.shiftLeftAndShrink(i_arc + 1);
        ArcPropContainer::shiftLeftAndShrink(i_arc + 1);
      }

      /**
       * @brief Add i_delta_multiplicity to the multiplicity value of iArc.
       *
       */
      void addArcMultiplicity(Arc arc, int i_delta_multiplicity) noexcept {
        const ArcIndex i_arc = id(arc);
        _arc_multiplicity[i_arc] += i_delta_multiplicity;
        const NodeIndex i_source = _sources[i_arc];
        _out_multiplicity[i_source] += i_delta_multiplicity;
        const NodeIndex i_target = _targets[i_arc];
        _in_multiplicity[i_target] += i_delta_multiplicity;
      }

      /**
       * @brief This function removes all nodes and arcs from the digraph.
       *
       */
      void clear() noexcept {
        _offsets.removeAll();
        _out_degrees.removeAll();
        _in_degrees.removeAll();
        _out_multiplicity.removeAll();
        _in_multiplicity.removeAll();
        NodePropContainer::removeAll();
        _sources.removeAll();
        _targets.removeAll();
        _arc_multiplicity.removeAll();
        ArcPropContainer::removeAll();
      }

      /**
       * @brief Build the graph from a list of arcs.
       *
       */
      template < typename ArcListIterator >
      void build(int n, ArcListIterator&& start, ArcListIterator&& end) {
        if (n <= 0) return;

        // Step 0: Initialization
        clear();

        _offsets.ensureAndFillBytes(n, 0);
        _out_degrees.ensureAndFillBytes(n, 0);
        _in_degrees.ensureAndFillBytes(n, 0);
        _out_multiplicity.ensureAndFillBytes(n, 0);
        _in_multiplicity.ensureAndFillBytes(n, 0);
        NodePropContainer::ensureAndFill(n);

        // Step 1 : Compute the in- and out-degree of each node
        nt::NumericalMap< uint64_t, int > num_arc_duplicates;
        ArcListIterator                   iter = start;
        int                               m = 0;
        while (iter != end) {
          const int source_id = iter->first;
          const int target_id = iter->second;

          if constexpr (MULTIGRAPH) {
            ++_out_degrees[source_id];
            ++_in_degrees[target_id];
            ++m;
          } else {
            const uint64_t i_arc_hash = NT_CONCAT_INT_32(source_id, target_id);
            if (!num_arc_duplicates.hasKey(i_arc_hash)) {
              num_arc_duplicates[i_arc_hash] = 1;
              ++_out_degrees[source_id];
              ++_in_degrees[target_id];
              ++m;
            } else
              ++num_arc_duplicates[i_arc_hash];
          }

          ++_out_multiplicity[source_id];
          ++_in_multiplicity[target_id];

          ++iter;
        }

        // Step 2: Compute the arc offsets of each node
        _offsets[0] = 0;
        for (int i = 1; i < n; ++i)
          _offsets[i] = _offsets[i - 1] + _out_degrees[i - 1];

        // Step 3: Fill the arcs arrays (sources, targets, ...)
        nt::IntDynamicArray counters;
        counters.ensureAndFillBytes(m, 0);
        _sources.ensureAndFillBytes(m, 0);
        _targets.ensureAndFillBytes(m, 0);
        _arc_multiplicity.ensureAndFillBytes(m, 0);
        ArcPropContainer::ensureAndFill(m);
        while (start != end) {
          const int      source_id = start->first;
          const int      target_id = start->second;
          const int      offset = _offsets[source_id];
          int&           k = counters[offset];
          const ArcIndex arc_id = offset + k;

          if constexpr (MULTIGRAPH) {
            _sources[arc_id] = source_id;
            _targets[arc_id] = target_id;
            _arc_multiplicity[arc_id] = 1;
            ++k;
          } else {
            const uint64_t i_arc_hash = NT_CONCAT_INT_32(source_id, target_id);
            int&           num_duplicates = num_arc_duplicates[i_arc_hash];
            if (num_duplicates > 0) {
              _sources[arc_id] = source_id;
              _targets[arc_id] = target_id;
              _arc_multiplicity[arc_id] = num_duplicates;
              num_duplicates = -1;
              ++k;
            }
          }

          ++start;
        }
      }


      /**
       * @brief Build the graph from another graph.
       *
       */
      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      void build(const Digraph& g, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        using GNode = typename Digraph::Node;
        using GArc = typename Digraph::Arc;
        using GArcIt = typename Digraph::ArcIt;
        using GNodeIt = typename Digraph::NodeIt;

        nt::TrivialDynamicArray< nt::Pair< int, int > > arcs(g.maxArcId() + 1);
        for (GArcIt arc(g); arc != nt::INVALID; ++arc) {
          const GNode u = g.source(arc);
          const GNode v = g.target(arc);
          arcs[g.id(arc)] = {g.id(u), g.id(v)};
        }

        build(g.nodeNum(), arcs.begin(), arcs.end());

        for (GNodeIt n(g); n != nt::INVALID; ++n)
          nodeRef[n] = nodeFromId(g.id(n));

        for (GArcIt arc(g); arc != nt::INVALID; ++arc) {
          const GNode u = g.source(arc);
          const GNode v = g.target(arc);
          int         i_arc = -1;
          const bool  r = hasArc(nodeFromId(g.id(u)), nodeFromId(g.id(v)), i_arc);   // TODO: get rid of the hasArc()
          assert(r);
          arcRef[arc] = arcFromId(i_arc);
        }
      }

      /**
       * @brief Check whether an arc is present
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       * @param i_arc If the arc if found, arc contains its index. Otherwise, it
       * contains the index of the arc (source, v) where v is the first vertex
       * greater than target.
       *
       * @return Returns true is the arc is found, false otherwise.
       */
      bool hasArc(Node source, Node target, ArcIndex& i_arc) const noexcept {
        i_arc = 0;
        if (!arcNum()) return false;

        const NodeIndex i_source = id(source);
        i_arc = _offsets[i_source];
        const int       i_last = i_arc + _out_degrees[i_source];
        const NodeIndex i_target = id(target);

        for (; i_arc < i_last; ++i_arc)
          if (_targets[i_arc] == i_target) return true;

        return false;
      }

      /**
       * @brief Check whether an arc is present
       *
       * @param source_id Index of the arc's tail.
       * @param target_id Index of the arc's head.
       *
       * @return Returns true is the arc is found, false otherwise.
       */
      bool hasArc(Node source, Node target) const noexcept {
        ArcIndex i_arc;
        return hasArc(source, target, i_arc);
      }

      constexpr int nodeNum() const noexcept { return _offsets.size(); }
      constexpr int arcNum() const noexcept { return _sources.size(); }

      constexpr bool valid(Node n) const noexcept { return n._id >= 0 && n._id < nodeNum(); }
      constexpr bool valid(Arc a) const noexcept { return a._id >= 0 && a._id < arcNum(); }

      constexpr Node source(Arc a) const noexcept { return _sources[id(a)]; }
      constexpr Node target(Arc a) const noexcept { return _targets[id(a)]; }

      static constexpr NodeIndex id(Node n) noexcept { return n.id; }
      static constexpr ArcIndex  id(Arc a) noexcept { return a.id; }

      static constexpr Node nodeFromId(int id) noexcept { return Node(id); }
      static constexpr Arc  arcFromId(int id) noexcept { return Arc(id); }

      constexpr int maxNodeId() const noexcept { return nodeNum() - 1; }
      constexpr int maxArcId() const noexcept { return arcNum() - 1; }

      constexpr void first(Node& n) const noexcept { n.id = nodeNum() - 1; }
      constexpr void next(Node& n) const noexcept { --n.id; }

      constexpr void first(Arc& a) const noexcept { a.id = arcNum() - 1; }
      constexpr void next(Arc& a) const noexcept { --a.id; }

      constexpr void firstOut(Arc& a, const Node& n) const noexcept {
#if 1
        a = nt::INVALID;
        const int i_bound = _out_degrees[id(n)];
        if (i_bound > 0) {
          const int i_offset = _offsets[id(n)];
          a.i_bound = i_offset + i_bound;
          a.id = i_offset;
        }
#else
        a = _offsets[id(n)];   // -1 if no out arc
#endif
      }
      constexpr void nextOut(Arc& a) const noexcept {
#if 1
        ++a.id;
        if (a.id >= a.i_bound) a = nt::INVALID;
#else
        const Node prev_src = source(a);
        ++a.id;
        if (source(a) != prev_src) a = nt::INVALID;
#endif
      }

      constexpr void firstIn(Arc& a, const Node& n) const noexcept {
        first(a);
        while (a != INVALID && target(a) != n)
          next(a);
      }

      constexpr void nextIn(Arc& a) const noexcept {
        const Node arc_target = target(a);
        do {
          next(a);
        } while (a != INVALID && target(a) != arc_target);
      }
    };

    template < class NODE_PROPERTY, class ARC_PROPERTY, bool MULTIGRAPH, bool STATIC >
    using ExtendedCsrPropertyDigraphBase =
       DigraphExtender< CsrPropertyDigraphBase< NODE_PROPERTY, ARC_PROPERTY, MULTIGRAPH >, STATIC >;

    /**
     * @struct CsrPropertyDigraph
     * @headerfile csr_graph.h
     * @brief Class representing a Compressed Sparse Row Graph.
     *
     * Compressed Sparse Row Graph (definition adapted from from www.boost.org)
     *
     * CsrPropertyDigraph implements a graph representation that uses the compact Compressed
     * Sparse Row (CSR) format to store directed graphs. While
     * CSR graphs have much less overhead than many other graph formats, they do
     * not provide fast insert/remove operations. Use this format in
     * high-performance applications or for very large graphs that you do not need
     * to change.
     *
     * @tparam NODE_PROPERTY
     * @tparam ARC_PROPERTY
     * @tparam MULTIGRAPH
     * @tparam STATIC
     *
     */
    template < class NODE_PROPERTY, class ARC_PROPERTY, bool MULTIGRAPH, bool STATIC >
    struct CsrPropertyDigraph: ExtendedCsrPropertyDigraphBase< NODE_PROPERTY, ARC_PROPERTY, MULTIGRAPH, STATIC > {
      using Parent = ExtendedCsrPropertyDigraphBase< NODE_PROPERTY, ARC_PROPERTY, MULTIGRAPH, STATIC >;
      TEMPLATE_DIGRAPH_TYPEDEFS(Parent);

      using NodePropContainer = Parent::NodePropContainer;
      using ArcPropContainer = Parent::ArcPropContainer;

      using NodeProp = typename Parent::NodeProp;
      using ArcProp = typename Parent::ArcProp;
      using NodeIndex = typename Parent::NodeIndex;
      using ArcIndex = typename Parent::ArcIndex;

      template < typename V >
      struct NPropMap {
        using Value = V;
        using Key = Node;

        nt::DynamicArray< NodeProp >& _nodes;
        Value NodeProp::*_member;

        constexpr explicit NPropMap(CsrPropertyDigraph& digraph, Value NodeProp::*member) noexcept :
            _nodes(digraph.Vertices), _member(member) {}

        constexpr Value& operator[](Key key) noexcept { return static_cast< Value >(_nodes[id(key)].props.*_member); }
        constexpr const Value& operator[](Key key) const noexcept {
          return static_cast< Value >(_nodes[id(key)].props.*_member);
        }

        constexpr void set(const Key& key, const Value& val) noexcept { _nodes[id(key)].props.*_member = val; }
        constexpr void set(const Key& key, Value&& val) noexcept {
          _nodes[id(key)].props.*_member = std::forward< Value >(val);
        }
      };

      template < typename V >
      struct APropMap {
        using Value = V;
        using Key = Arc;

        nt::DynamicArray< ArcProp >& _arcs;
        Value ArcProp::*_member;

        constexpr explicit APropMap(CsrPropertyDigraph& digraph, Value ArcProp::*member) noexcept :
            _arcs(digraph.Arcs), _member(member) {}

        constexpr Value&       operator[](Key key) noexcept { return _arcs[id(key)].props.*_member; }
        constexpr const Value& operator[](Key key) const noexcept { return _arcs[id(key)].props.*_member; }

        constexpr void set(const Key& key, const Value& val) noexcept { _arcs[id(key)].props.*_member = val; }
        constexpr void set(const Key& key, Value&& val) noexcept {
          _arcs[id(key)].props.*_member = std::forward< Value >(val);
        }
      };


      /**
       * @brief Reserve memory for the given number of vertices and arcs.
       *
       * @param n
       * @param m
       *
       */
      void reserve(int n, int m) noexcept {
        reserveNode(n);
        reserveArc(m);
      }

      void reserveNode(int n) noexcept {
        Parent::_offsets.ensure(n);
        Parent::_out_degrees.ensure(n);
        Parent::_in_degrees.ensure(n);
        Parent::_out_multiplicity.ensure(n);
        Parent::_in_multiplicity.ensure(n);
        NodePropContainer::ensure(n);
      }

      void reserveArc(int m) noexcept {
        Parent::_sources.ensure(m);
        Parent::_targets.ensure(m);
        Parent::_arc_multiplicity.ensure(m);
        ArcPropContainer::ensure(m);
      }

      /**
       * @brief
       *
       * @return
       */
      constexpr bool isMultiGraph() const noexcept { return MULTIGRAPH == true; }


      /**
       * @brief Returns the multiplicity of iArc.
       *
       * @param arc Index of the arc.
       *
       * @return The multiplicity of iArc.
       */
      int arcMultiplicity(Arc arc) const noexcept { return Parent::_arc_multiplicity[Parent::id(arc)]; }


      /**
       * @brief Build the digraph from an arc list. Prefer calling this method instead of
       * repeated calls to addArc() to construct a CsrPropertyDigraph with a large amount of arcs.
       *
       * @tparam ArcListIterator
       *
       * @param n
       * @param begin
       * @param end
       */
      template < typename ArcListIterator >
      void build(int n, ArcListIterator&& begin, ArcListIterator&& end) noexcept {
        CsrPropertyDigraphBase< NODE_PROPERTY, ARC_PROPERTY, MULTIGRAPH >::build(
           n, std::forward< ArcListIterator >(begin), std::forward< ArcListIterator >(end));
        Parent::notifier(Node()).build();
        Parent::notifier(Arc()).build();
      }

      /**
       * @brief Build the graph from another graph.
       *
       */
      template < typename Digraph, typename NodeRefMap, typename ArcRefMap >
      void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) noexcept {
        Parent::build(digraph, nodeRef, arcRef);
      }

      /**
       * @brief Build the graph from another graph.
       *
       */
      template < typename Digraph >
      void build(const Digraph& g) noexcept {
        using GNode = typename Digraph::Node;
        using GArc = typename Digraph::Arc;
        auto node_map = details::StaticMapBase< Digraph, GNode, nt::DynamicArray< Node > >(g);
        auto arc_map = nt::NullMap< GArc, Arc >();
        build(g, node_map, arc_map);
      }

      /**
       * @brief
       *
       * @param arc
       */
      void erase(Arc arc) noexcept { Parent::erase(arc); }

      /**
       * @brief Remove an arc in the graph.
       *
       * @param source_id Index of the arc's tail.
       * @param target_id Index of the arc's head.
       *
       * @return Index of the removed arc
       *
       */
      void erase(Node source, Node target) noexcept {
        ArcIndex i_arc;
        if (Parent::hasArc(source, target, i_arc)) Parent::erase(Arc(i_arc));
      }

      /**
       * @brief
       *
       * @return Node
       */
      Node addNode() noexcept { return Parent::addNode(); }

      /**
       * @brief Add i_num vertices in the graph.
       *
       * @param i_num Number of vertices to add.
       *
       */
      constexpr void addNodes(int i_num) noexcept {
        while (i_num--)
          Parent::addNode();
      }

      /**
       * @brief Insert a vertex in the graph.
       *
       * @param NodeProperty Properties of the vertex to be inserted.
       *
       * @return Index of the inserted vertex
       *
       */
      constexpr Node addNode(NodeProp&& node_prop) noexcept {
        const Node         node = addNode();
        NodePropContainer::operator[](Parent::id(node)) = std::forward< NodeProp >(node_prop);
        return node;
      }

      /**
       * @brief Insert the arc (source, target) in the graph.
       *
       * Observe that the function does not return an Arc as usually done in other graph classes.
       * This is because the index where the newly arc is added is not stable, there is a high chance that
       * adding/erasing arcs in the graph will have the effect of shifting the arc to another position
       * in the arc array and thus changing its index.
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       *
       */
      void addArc(Node source, Node target) noexcept {
        Arc arc;
        addArc(source, target, arc);
      }

      /**
       * @brief Insert the arc (source, target) in the graph.
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       * @param arc Store the index value where the arc is inserted. This value will
       *            likely change if subsequent arc are added in the graph and thus
       *            should NOT be taken as the arc id.
       */
      void addArc(Node source, Node target, Arc& arc) noexcept { arc = Parent::addArc(source, target); }

      /**
       * @brief Insert the arc (source, target) in the graph along with properties
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       * @param ArcProperty Properties of the arc to be inserted.
       *
       */
      void addArc(Node source, Node target, ArcProp&& arc_prop) noexcept {
        Arc arc;
        addArc(source, target, std::forward< ArcProp >(arc_prop), arc);
      }

      /**
       * @brief Insert the arc (source, target) in the graph along with properties
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       * @param arc_prop Properties of the arc to be inserted.
       * @param arc Store the index value where the arc is inserted. This value will
       *            likely change if subsequent arc are added in the graph and thus
       *            should NOT be taken as the arc id.
       *
       */
      void addArc(Node source, Node target, ArcProp&& arc_prop, Arc& arc) noexcept {
        addArc(source, target, arc);
        ArcPropContainer::operator[](Parent::id(arc)) = std::forward< ArcProp >(arc_prop);
      }

      /**
       * @brief The node with the given index.
       *
       * This function returns the node with the given index.
       */
      static Node node(int id) { return Parent::nodeFromId(id); }

      /**
       * @brief The arc with the given index.
       *
       * This function returns the arc with the given index.
       */
      static Arc arc(int id) { return Parent::arcFromId(id); }

      /**
       * @brief The index of the given node.
       *
       * This function returns the index of the the given node.
       */
      static int index(Node node) { return Parent::id(node); }

      /**
       * @brief The index of the given arc.
       *
       * This function returns the index of the the given arc.
       */
      static int index(Arc arc) { return Parent::id(arc); }

      /**
       * @brief Returns the index of the arc (source_id, target_id)
       *
       * @param source_id Index of the arc's tail.
       * @param target_id Index of the arc's head.
       * @param prev Does not apply for CsrDigraph.
       * @return Returns the arc's index if it exists, the number of arcs
       * otherwise.
       */
      Arc findArc(Node source, Node target, Arc prev = INVALID) const noexcept {
        assert(prev == INVALID);
        ArcIndex i_arc;
        if (Parent::hasArc(source, target, i_arc)) return i_arc;
        return INVALID;
      }

      /**
       * @brief Returns the out multiplicity of iNode.
       * OutMultiplicity = Σ multiplicity(arc) for arc ∊ {out_arcs(iNode)}
       *
       * @param node Index of the vertex.
       *
       * @return The out multiplicity of iNode.
       */
      int outMultiplicity(Node node) const noexcept {
        const CsrPropertyDigraph& m = *this;
        const NodeIndex           i_node = Parent::id(node);
        assert(std::accumulate(m._arc_multiplicity.data() + m._offsets[i_node],
                               m._arc_multiplicity.data() + m._offsets[i_node] + m._out_degrees[i_node],
                               0,
                               [](int acc, Parent::Multiplicity multiplicity) { return acc + multiplicity; })
               == m._out_multiplicity[i_node]);
        return m._out_multiplicity[i_node];
      }

      /**
       * @brief Returns the out-degree of iNode.
       *        OutDegree = deg⁺(iNode) = |{out_arcs(iNode)}|
       *
       * @param iNode Index of the vertex.
       *
       * @return The number of iNode's outgoing arcs.
       */
      int outDegree(Node node) const noexcept {
        const CsrPropertyDigraph& m = *this;
        return m._out_degrees[Parent::id(node)];
      }

      /**
       * @brief Returns the k-th out neighbor of the vertex iNode.
       *
       * @param iNode Index of the vertex.
       * @param k An index of a iNode's out-neighbor.
       *
       * @return The index of k-th out-neighbor of iNode.
       */
      Node kthOutNeighbor(Node node, int k) const noexcept {
        const CsrPropertyDigraph& m = *this;
        return m._targets[m._offsets[Parent::id(node)] + k];
      }

      /**
       * @brief Returns the k-th out arc of the vertex iNode.
       *
       * @param iNode Index of the vertex.
       * @param k An index of a iNode's out-arc.
       *
       * @return The index of k-th out-arc of iNode.
       */
      Arc kthOutArc(Node node, int k) const noexcept {
        const CsrPropertyDigraph& m = *this;
        return m._offsets[Parent::id(node)] + k;
      }

      /**
       * @brief Returns the in-degree of iNode.
       *
       * @param iNode Index of the vertex.
       *
       * @return The number of iNode's ingoing arcs.
       */
      int inDegree(Node node) const noexcept {
        const CsrPropertyDigraph& m = *this;
        return m._in_degrees[Parent::id(node)];
      }

      /**
       * @brief Returns the properties of vertex iNode.
       *
       * @param iNode Index of the vertex.
       *
       * @return Properties of vertex iNode.
       */
      const NodeProp& NProp(Node node) const noexcept { return NodePropContainer::operator[](Parent::id(node)); }

      NodeProp& NProp(Node node) noexcept { return NodePropContainer::operator[](Parent::id(node)); }

      /**
       * @brief Returns the properties of the arc (source, target).
       *
       * @param source Index of the arc's tail.
       * @param target Index of the arc's head.
       *
       * @return Properties of the arc (source, target).
       */
      const ArcProp& AProp(Node source, Node target) const noexcept {
        return ArcPropContainer::operator[](Parent::id(findArc(source, target)));
      }

      ArcProp& AProp(Node source, Node target) noexcept {
        return ArcPropContainer::operator[](Parent::id(findArc(source, target)));
      }

      /**
       * @brief Returns the properties of the arc arc.
       *
       * @param arc Index of the arc.
       *
       * @return Properties of the arc arc.
       */
      const ArcProp& AProp(Arc arc) const noexcept { return ArcPropContainer::operator[](Parent::id(arc)); }

      ArcProp& AProp(Arc arc) noexcept { return ArcPropContainer::operator[](Parent::id(arc)); }
    };

    /**
     * @brief Specialization of a CsrPropertyDigraph with no properties attached to the nodes and
     * arcs.
     *
     */
    using CsrDigraph = CsrPropertyDigraph< NoProperty, NoProperty, true, false >;

    /**
     * @brief Same as CsrDigraph with the additional property that the graph contains no arc
     * duplication. Instead, adding an arc uv that already exists will only increase the
     * multiplicity value of that arc by one.
     *
     */
    using CsrSimpleDigraph = CsrPropertyDigraph< NoProperty, NoProperty, false, false >;

  }   // namespace graphs
}   // namespace nt

#endif