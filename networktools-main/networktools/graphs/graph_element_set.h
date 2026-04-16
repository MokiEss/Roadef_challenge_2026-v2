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
 * @file
 * @brief
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_GRAPH_ELEMENT_SET_H_
#define _NT_GRAPH_ELEMENT_SET_H_

#include "tools.h"
#include "../core/arrays.h"

namespace nt {
  namespace graphs {

    /**
     * @class GraphElementSetBase
     * @brief This is the base class that implements the graph element sets : Node/Arc/EdgeSet. This
     * class should not be used directly, use rather NodeSet, ArcSet or EdgeSet.
     *
     * @tparam GR The type of the directed graph
     * @tparam Item The type of the element : Node, Arc or Edge
     */
    template < class GR, class Item >
    struct GraphElementSetBase {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using Array = nt::TrivialDynamicArray< Item >;
      using iterator = typename Array::iterator;
      using const_iterator = typename Array::const_iterator;
      using reference = typename Array::reference;
      using const_reference = typename Array::const_reference;

      const Digraph* _p_digraph;   // Not clear if reference are movable, use a pointer instead
      Array          _array;
      nt::BitArray   _marked;

      explicit GraphElementSetBase(const Digraph& digraph) : _p_digraph(&digraph) { build(); }

      GraphElementSetBase(const GraphElementSetBase&) = delete;
      GraphElementSetBase& operator=(const GraphElementSetBase&) = delete;

      GraphElementSetBase(GraphElementSetBase&& other) noexcept :
          _p_digraph(other._p_digraph), _array(std::move(other._array)), _marked(std::move(other._marked)) {
        assert(other._p_digraph);
        other._p_digraph = nullptr;
      }
      GraphElementSetBase& operator=(GraphElementSetBase&& other) noexcept {
        assert(other._p_digraph);
        if (this == &other) return *this;
        _p_digraph = other._p_digraph;
        other._p_digraph = nullptr;
        _array = std::move(other._array);
        _marked = std::move(other._marked);
        return *this;
      }

      /**
       * @brief
       *
       */
      void build() noexcept {
        assert(_p_digraph);
        const int i_num_element = countItems< Digraph, Item >(*_p_digraph);
        _marked.extendByBits(i_num_element);
        clear();
      }


      /**
       * @brief Insert a new element in the set
       *
       * @param u The element to be inserted
       */
      void insert(Item u) noexcept {
        if (!_isMarked(u)) {
          _array.add(u);
          _mark(u);
        }
      }

      /**
       * @brief Remove the last inserted element
       *
       * @return The element that has been removed
       */
      Item pop_back() noexcept {
        const Item u = _array.back();
        _array.removeLast();
        _unmark(u);
        return u;
      }

      /**
       * @brief Remove all elements from the set
       *
       */
      void clear() noexcept {
        _array.removeAll();
        _resetMarkers();
      }

      /**
       * @brief Check whether an element is present in the set
       *
       * @param u The element to be tested
       *
       * @return true if present, false otherwise
       */
      constexpr bool contains(Item u) const noexcept { return u != nt::INVALID && _isMarked(u); }


      /**
       * @brief Find the position of an element
       *
       * @param u The element to be found
       *
       * @return The position of the element or the size of the set if no such element is found.
       */
      constexpr int find(Item u) const noexcept { return _array.find(u); }

      /**
       * @brief Given a set 'from' of graph elements, this function returns the intersection 'this' ∩ 'from'
       * Complexity : O(min(|this|, |from|))
       *
       * @param from A set of graph elements
       *
       * @return The intersection 'this' ∩ 'from'
       */
      template < class ElementSet >
      ElementSet intersectionSet(const ElementSet& from) const noexcept {
        ElementSet inter_set(*_p_digraph);

        if (empty() || from.empty()) return inter_set;

        inter_set._marked = _marked & from._marked;

        if (from.size() < size()) {
          for (int k = 0; k < from.size(); ++k) {
            const Node& v = from[k];
            if (contains(v)) inter_set._array.add(v);
          }
        } else {
          for (int k = 0; k < size(); ++k) {
            const Node& v = _array[k];
            if (from.contains(v)) inter_set._array.add(v);
          }
        }

        return inter_set;
      }

      /**
       * @brief Given a set 'from' of graph elements, this function returns the union 'this' ∪ 'from'
       * Complexity : O(|this| + |from|)
       *
       * @param from A set of graph elements
       *
       * @return The union 'this' ∪ 'from'
       */
      template < class ElementSet >
      ElementSet unionSet(const ElementSet& from) const noexcept {
        ElementSet union_set(*_p_digraph);

        if (empty()) {
          union_set.copyFrom(from);
          return union_set;
        }

        if (from.empty()) {
          union_set.copyFrom(*this);
          return union_set;
        }

        union_set._marked = _marked | from._marked;
        union_set._array.copyFrom(_array);

        for (int k = 0; k < from.size(); ++k) {
          const Node& v = from._array[k];
          if (!contains(v)) union_set._array.add(v);
        }

        return union_set;
      }

      /**
       * @brief Given a set 'from' of graph elements, this function returns the symmetric difference 'this' ∆ 'from'.
       * Complexity : O(|this| + |from|)
       *
       * @param from A set of graph elements
       *
       * @return The symmetric difference 'this' ∆ 'from'
       */
      template < class ElementSet >
      ElementSet differenceSet(const ElementSet& from) const noexcept {
        ElementSet diff_set(*_p_digraph);

        if (empty()) {
          diff_set.copyFrom(from);
          return diff_set;
        }

        if (from.empty()) {
          diff_set.copyFrom(*this);
          return diff_set;
        }

        diff_set._marked = _marked ^ from._marked;

        for (int k = 0; k < size(); ++k) {
          const Node& x = _array[k];
          if (!from.contains(x)) diff_set._array.add(x);
        }

        for (int k = 0; k < from.size(); ++k) {
          const Node& y = from[k];
          if (!contains(y)) diff_set._array.add(y);
        }

        return diff_set;
      }

      /**
       * @brief Return the number of elements in the set
       *
       */
      constexpr int size() const noexcept { return _array.size(); }

      /**
       * @brief Check whether the set is empty
       *
       */
      constexpr bool empty() const noexcept { return _array.empty(); }

      /**
       * @brief Replaces the contents with a copy of the contents of from.
       *
       */
      constexpr void copyFrom(const GraphElementSetBase& from) noexcept {
        _array.copyFrom(from._array);
        _marked.copyFrom(from._marked);
      }

      constexpr iterator       begin() noexcept { return _array.begin(); }
      constexpr const_iterator begin() const noexcept { return _array.begin(); }

      constexpr iterator       end() noexcept { return _array.end(); }
      constexpr const_iterator end() const noexcept { return _array.end(); }

      constexpr reference       operator[](int index) noexcept { return _array[index]; }
      constexpr const_reference operator[](int index) const noexcept { return _array[index]; }

      //---------------------------------------------------------------------------
      // Auxiliary methods
      //---------------------------------------------------------------------------

      constexpr bool _isMarked(Item u) const noexcept {
        assert(_p_digraph);
        return _marked.isOne(_p_digraph->id(u));
      }
      constexpr void _mark(Item u) noexcept {
        assert(_p_digraph);
        _marked.setOneAt(_p_digraph->id(u));
      }
      constexpr void _unmark(Item u) noexcept {
        assert(_p_digraph);
        _marked.setZeroAt(_p_digraph->id(u));
      }
      constexpr void _resetMarkers() noexcept { _marked.setBitsToZero(); }
    };


    /**
     * @brief This class implements an efficient representation of a set of nodes.
     *
     * @tparam GR The type of the graph to which the nodes belong to
     */
    template < class GR >
    struct NodeSet: GraphElementSetBase< GR, typename GR::Node > {
      using Digraph = GR;
      using Parent = GraphElementSetBase< Digraph, typename GR::Node >;
      NodeSet(const Digraph& digraph) : Parent(digraph) {}
    };


    /**
     * @brief This class implements an efficient representation of a set of arcs.
     *
     * @tparam GR The type of the digraph to which the arcs belong to
     */
    template < class GR >
    struct ArcSet: GraphElementSetBase< GR, typename GR::Arc > {
      using Digraph = GR;
      using Parent = GraphElementSetBase< Digraph, typename GR::Arc >;
      ArcSet(const Digraph& digraph) : Parent(digraph) {}
    };

    /**
     * @brief This class implements an efficient representation of a set of edges.
     *
     * @tparam GR The type of the graph to which the edges belong to
     */
    template < class GR >
    struct EdgeSet: GraphElementSetBase< GR, typename GR::Edge > {
      using Graph = GR;
      using Parent = GraphElementSetBase< Graph, typename GR::Edge >;
      EdgeSet(const Graph& graph) : Parent(graph) {}
    };
  }   // namespace graphs
}   // namespace nt

#endif