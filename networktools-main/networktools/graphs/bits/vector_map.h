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
 * This file incorporates work from the LEMON graph library (vector_map.h).
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
 * Modifications made by Orange:
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Replaced std::vector with nt::DynamicArray for container storage
 *   - Updated include paths to networktools structure
 *   - Replaced exception-based error handling with error codes (ObserverReturnCode)
 *   - Converted typedef declarations to C++11 using declarations
 *   - Updated header guard macros
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Added constructor for const char* (with C++20 requires clause for nt::String type)
 *   - Added move constructor
 *   - Replaced std::vector<Key> with nt::ArrayView<Key> in observer interface
 *   - Changed virtual function signatures to return ObserverReturnCode instead of void
 *   - Added override keyword to virtual functions
 *   - Added setBitsToZero() and setBitsToOne() methods with static_assert checks
 *   - Used C++20 requires clause for compile-time type checking
 *   - Added container() accessor methods
 *   - Added size() method
 *   - Added reserve() notification override for pre-allocation
 *   - Used resizeEmplace() for efficient string construction
 */

/**
 * @ingroup graphbits
 * @file
 * @brief Vector based graph maps.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_VECTOR_MAP_H_
#define _NT_VECTOR_MAP_H_

#include "../../core/arrays.h"
#include "../../core/string.h"
#include "../../core/traits.h"
#include "alteration_notifier.h"

namespace nt {
  namespace graphs {

    /**
     * @ingroup graphbits
     *
     * @brief Graph map based on the nt::DynamicArray storage.
     *
     * The VectorMap template class is graph map structure that automatically
     * updates the map when a key is added to or erased from the graph.
     * This map type uses nt::DynamicArray to store the values.
     *
     * @tparam _Graph The graph this map is attached to.
     * @tparam _Item The item type of the graph items.
     * @tparam _Value The value type of the map.
     */
    template < typename _Graph, typename _Item, typename _Value >
    class VectorMap: public ItemSetTraits< _Graph, _Item >::ItemNotifier::ObserverBase {
      private:
      /** The container type of the map. */
      using Container = nt::DynamicArray< _Value >;

      public:
      /** The graph type of the map. */
      using GraphType = _Graph;
      /** The item type of the map. */
      using Item = _Item;
      /** The reference map tag. */
      using ReferenceMapTag = nt::True;

      /** The key type of the map. */
      using Key = _Item;
      /** The value type of the map. */
      using Value = _Value;

      /** The notifier type. */
      using Notifier = typename ItemSetTraits< _Graph, _Item >::ItemNotifier;

      /** The map type. */
      using Map = VectorMap;

      /** The reference type of the map; */
      using Reference = typename Container::reference;
      /** The const reference type of the map; */
      using ConstReference = typename Container::const_reference;

      private:
      /** The base class of the map. */
      using Parent = typename Notifier::ObserverBase;

      public:
      /**
       * @brief Constructor to attach the new map into the notifier.
       *
       * It constructs a map and attachs it into the notifier.
       * It adds all the items of the graph to the map.
       */
      VectorMap(const GraphType& graph) noexcept {
        Parent::attach(graph.notifier(Item()));
        _container.resize(Parent::notifier()->maxId() + 1);
      }

      /**
       * @brief Constructor uses given value to initialize the map.
       *
       * It constructs a map uses a given value to initialize the map.
       * It adds all the items of the graph to the map.
       */
      VectorMap(const GraphType& graph, const Value& value) noexcept {
        Parent::attach(graph.notifier(Item()));
        _container.resize(Parent::notifier()->maxId() + 1, value);
      }

      /**
       * @brief
       *
       * @param graph
       * @param map
       */
      VectorMap(const GraphType& graph, const VectorMap& map) noexcept {
        Parent::attach(graph.notifier(Item()));
        copyFrom(map);
      }

      /**
       * Constructor for const char*, only if Value is of type nt::String
       */
      VectorMap(const GraphType& graph, const char* value) noexcept requires(std::is_same< Value, nt::String >::value) {
        Parent::attach(graph.notifier(Item()));
        _container.resizeEmplace(Parent::notifier()->maxId() + 1, value);
      }

      /** Move constructor */
      VectorMap(VectorMap&& other) noexcept : _container(std::move(other._container)) {}

      /**
       * @brief Move assign operator.
       */
      constexpr VectorMap& operator=(VectorMap&& cmap) noexcept {
        if (this != &cmap) _container = std::move(cmap._container);
        return *this;
      }

      private:
      /**
       * @brief Copy constructor
       *
       * Copy constructor.
       */
      VectorMap(const VectorMap& _copy) noexcept : Parent() {
        if (_copy.attached()) {
          Parent::attach(*_copy.notifier());
          _container = _copy._container;
        }
      }

      /**
       * @brief Assign operator.
       *
       * This operator assigns for each item in the map the
       * value mapped to the same item in the copied map.
       * The parameter map should be indiced with the same
       * itemset because this assign operator does not change
       * the container of the map.
       */
      constexpr VectorMap& operator=(const VectorMap& cmap) noexcept { return operator=< VectorMap >(cmap); }

      /**
       * @brief Template assign operator.
       *
       * The given parameter should conform to the ReadMap
       * concept and could be indiced by the current item set of
       * the NodeMap. In this case the value for each item
       * is assigned by the value of the given ReadMap.
       */
      template < typename CMap >
      constexpr VectorMap& operator=(const CMap& cmap) noexcept {
        const typename Parent::Notifier* nf = Parent::notifier();
        Item                             it;
        for (nf->first(it); it != INVALID; nf->next(it)) {
          set(it, cmap[it]);
        }
        return *this;
      }

      public:
      /**
       * @brief The subcript operator.
       *
       * The subscript operator. The map can be subscripted by the
       * actual items of the graph.
       */
      constexpr Reference operator[](const Key& key) noexcept { return _container[Parent::notifier()->id(key)]; }

      /**
       * @brief The const subcript operator.
       *
       * The const subscript operator. The map can be subscripted by the
       * actual items of the graph.
       */
      constexpr ConstReference operator[](const Key& key) const noexcept {
        return _container[Parent::notifier()->id(key)];
      }

      /**
       * @brief The setter function of the map.
       *
       * It the same as operator[](key) = value expression.
       */
      constexpr void set(const Key& key, const Value& value) noexcept { (*this)[key] = value; }

      /**
       * @brief Return the number of elements stored in the map
       *
       */
      constexpr int size() const noexcept { return _container.size(); }

      /**
       * @brief Set all bits of the map elements to zero
       *
       * If the Value type is trivial, it sets all bits of the map elements to zero.
       */
      constexpr void setBitsToZero() noexcept {
        if constexpr (std::is_trivial< Value >::value)
          _container.setBitsToZero();
        else
          static_assert(true, "setBitsToZero is not supported for non-trivial types");
      }

      /**
       * @brief Set all bits of the map elements to one
       *
       * If the Value type is trivial, it sets all bits of the map elements to one.
       */
      constexpr void setBitsToOne() noexcept {
        if constexpr (std::is_trivial< Value >::value)
          _container.setBitsToOne();
        else
          static_assert(true, "setBitsToOne is not supported for non-trivial types");
      }

      /**
       * @brief Copy the contents from another VectorMap.
       *
       */
      constexpr void copyFrom(const VectorMap& other) noexcept {
        assert(Parent::notifier()->maxId() + 1 == other._container.size());
        _container.copyFrom(other._container);
      }

      /**
       * @brief Return a reference to the container
       *
       */
      constexpr Container&       container() noexcept { return _container; }
      constexpr const Container& container() const noexcept { return _container; }

      protected:
      /**
       * @brief Adds a new key to the map.
       *
       * It adds a new key to the map. It is called by the observer notifier
       * and it overrides the add() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode add(const Key& key) noexcept override {
        int id = Parent::notifier()->id(key);
        if (id >= int(_container.size())) { _container.resize(id + 1); }
        return ObserverReturnCode::Ok;
      }

      /**
       * @brief Adds more new keys to the map.
       *
       * It adds more new keys to the map. It is called by the observer notifier
       * and it overrides the add() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode add(const nt::ArrayView< Key >& keys) noexcept override {
        int max = _container.size() - 1;
        for (int i = 0; i < int(keys.size()); ++i) {
          int id = Parent::notifier()->id(keys[i]);
          if (id >= max) { max = id; }
        }
        _container.resize(max + 1);
        return ObserverReturnCode::Ok;
      }

      /**
       * @brief Erase a key from the map.
       *
       * Erase a key from the map. It is called by the observer notifier
       * and it overrides the erase() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode erase(const Key& key) noexcept override {
        _container[Parent::notifier()->id(key)] = Value();
        return ObserverReturnCode::Ok;
      }

      /**
       * @brief Erase more keys from the map.
       *
       * It erases more keys from the map. It is called by the observer notifier
       * and it overrides the erase() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode erase(const nt::ArrayView< Key >& keys) noexcept override {
        for (int i = 0; i < int(keys.size()); ++i)
          _container[Parent::notifier()->id(keys[i])] = Value();
        return ObserverReturnCode::Ok;
      }

      /**
       * @brief Build the map.
       *
       * It builds the map. It is called by the observer notifier
       * and it overrides the build() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode build() noexcept override {
        const int size = Parent::notifier()->maxId() + 1;
        _container.resize(size);
        return ObserverReturnCode::Ok;
      }

      constexpr virtual ObserverReturnCode reserve(int n) noexcept override {
        _container.reserve(n);
        return ObserverReturnCode::Ok;
      }

      /**
       * @brief Clear the map.
       *
       * It erases all items from the map. It is called by the observer notifier
       * and it overrides the clear() member function of the observer base.
       */
      constexpr virtual ObserverReturnCode clear() noexcept override {
        _container.clear();
        return ObserverReturnCode::Ok;
      }

      private:
      Container _container;
    };

  }   // namespace graphs
}   // namespace nt

#endif
