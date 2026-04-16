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
 * This file incorporates work from the LEMON graph library (maps.h).
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
 *   - Replacing std::vector with nt::TrivialDynamicArray
 */

/**
 * @file
 * @brief Provides an immutable and unique id for each item in a graph.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_GRAPH_MAPS_H_
#define _NT_GRAPH_MAPS_H_

#include "../core/maps.h"

namespace nt {
  namespace graphs {

    /**
     * \addtogroup graph_maps
     * @{
     */

    /**
     * @brief Provides an immutable and unique id for each item in a graph.
     *
     * IdMap provides a unique and immutable id for each item of the
     * same type (\c Node, \c Arc or \c Edge) in a graph. This id is
     * - \b unique: different items get different ids,
     * - \b immutable: the id of an item does not change (even if you
     * delete other nodes).
     *
     * Using this map you get access (i.e. can read) the inner id values of
     * the items stored in the graph, which is returned by the \c id()
     * function of the graph. This map can be inverted with its member
     * class \c InverseMap or with the \c operator()() member.
     *
     * @tparam GR The graph type.
     * @tparam K The key type of the map (\c GR::Node, \c GR::Arc or
     * \c GR::Edge).
     *
     * @see RangeIdMap
     */
    template < typename GR, typename K >
    class IdMap: public MapBase< K, int > {
      public:
      /** The graph type of IdMap. */
      using Graph = GR;
      using Digraph = GR;
      /** The key type of IdMap (\c Node, \c Arc or \c Edge). */
      using Item = K;
      /** The key type of IdMap (\c Node, \c Arc or \c Edge). */
      using Key = K;
      /** The value type of IdMap. */
      using Value = int;

      /**
       * @brief Constructor.
       *
       * Constructor of the map.
       */
      explicit IdMap(const Graph& graph) : _graph(&graph) {}

      /**
       * @brief Gives back the \e id of the item.
       *
       * Gives back the immutable and unique \e id of the item.
       */
      int operator[](const Item& item) const { return _graph->id(item); }

      /**
       * @brief Gives back the \e item by its id.
       *
       * Gives back the \e item by its id.
       */
      Item operator()(int id) { return _graph->fromId(id, Item()); }

      private:
      const Graph* _graph;

      public:
      /**
       * @brief The inverse map type of IdMap.
       *
       * The inverse map type of IdMap. The subscript operator gives back
       * an item by its id.
       * This type conforms to the @ref concepts::ReadMap "ReadMap" concept.
       * @see inverse()
       */
      class InverseMap {
        public:
        /**
         * @brief Constructor.
         *
         * Constructor for creating an id-to-item map.
         */
        explicit InverseMap(const Graph& graph) : _graph(&graph) {}

        /**
         * @brief Constructor.
         *
         * Constructor for creating an id-to-item map.
         */
        explicit InverseMap(const IdMap& map) : _graph(map._graph) {}

        /**
         * @brief Gives back an item by its id.
         *
         * Gives back an item by its id.
         */
        Item operator[](int id) const { return _graph->fromId(id, Item()); }

        private:
        const Graph* _graph;
      };

      /**
       * @brief Gives back the inverse of the map.
       *
       * Gives back the inverse of the IdMap.
       */
      InverseMap inverse() const { return InverseMap(*_graph); }
    };

    /**
     * @brief Returns an \c IdMap class.
     *
     * This function just returns an \c IdMap class.
     * \relates IdMap
     */
    template < typename K, typename GR >
    inline IdMap< GR, K > idMap(const GR& graph) {
      return IdMap< GR, K >(graph);
    }

    /**
     * @brief Provides continuous and unique id for the
     * items of a graph.
     *
     * RangeIdMap provides a unique and continuous
     * id for each item of a given type (\c Node, \c Arc or
     * \c Edge) in a graph. This id is
     * - \b unique: different items get different ids,
     * - \b continuous: the range of the ids is the set of integers
     * between 0 and \c n-1, where \c n is the number of the items of
     * this type (\c Node, \c Arc or \c Edge).
     * - So, the ids can change when deleting an item of the same type.
     *
     * Thus this id is not (necessarily) the same as what can get using
     * the \c id() function of the graph or @ref IdMap.
     * This map can be inverted with its member class \c InverseMap,
     * or with the \c operator()() member.
     *
     * @tparam GR The graph type.
     * @tparam K The key type of the map (\c GR::Node, \c GR::Arc or
     * \c GR::Edge).
     *
     * @see IdMap
     */
    template < typename GR, typename K >
    class RangeIdMap: protected ItemSetTraits< GR, K >::template Map< int >::Type {
      using Map = typename ItemSetTraits< GR, K >::template Map< int >::Type;

      public:
      /** The graph type of RangeIdMap. */
      using Graph = GR;
      using Digraph = GR;
      /** The key type of RangeIdMap (\c Node, \c Arc or \c Edge). */
      using Item = K;
      /** The key type of RangeIdMap (\c Node, \c Arc or \c Edge). */
      using Key = K;
      /** The value type of RangeIdMap. */
      using Value = int;

      /**
       * @brief Constructor.
       *
       * Constructor.
       */
      explicit RangeIdMap(const Graph& gr) : Map(gr) {
        Item                          it;
        const typename Map::Notifier* nf = Map::notifier();
        for (nf->first(it); it != INVALID; nf->next(it)) {
          Map::set(it, _inv_map.size());
          _inv_map.push_back(it);
        }
      }

      protected:
      /**
       * @brief Adds a new key to the map.
       *
       * Add a new key to the map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode add(const Item& item) noexcept override {
        Map::add(item);
        Map::set(item, _inv_map.size());
        _inv_map.push_back(item);
        return graphs::ObserverReturnCode::Ok;
      }

      /**
       * @brief Add more new keys to the map.
       *
       * Add more new keys to the map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Item >& items) noexcept override {
        Map::add(items);
        for (int i = 0; i < int(items.size()); ++i) {
          Map::set(items[i], _inv_map.size());
          _inv_map.push_back(items[i]);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      /**
       * @brief Erase the key from the map.
       *
       * Erase the key from the map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode erase(const Item& item) noexcept override {
        Map::set(_inv_map.back(), Map::operator[](item));
        _inv_map[Map::operator[](item)] = _inv_map.back();
        _inv_map.pop_back();
        return Map::erase(item);
      }

      /**
       * @brief Erase more keys from the map.
       *
       * Erase more keys from the map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Item >& items) noexcept override {
        for (int i = 0; i < int(items.size()); ++i) {
          Map::set(_inv_map.back(), Map::operator[](items[i]));
          _inv_map[Map::operator[](items[i])] = _inv_map.back();
          _inv_map.pop_back();
        }
        return Map::erase(items);
      }

      /**
       * @brief Build the unique map.
       *
       * Build the unique map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode build() noexcept override {
        const graphs::ObserverReturnCode r = Map::build();
        if (r != graphs::ObserverReturnCode::Ok) return r;
        Item                          it;
        const typename Map::Notifier* nf = Map::notifier();
        for (nf->first(it); it != INVALID; nf->next(it)) {
          Map::set(it, _inv_map.size());
          _inv_map.push_back(it);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      /**
       * @brief Clear the keys from the map.
       *
       * Clear the keys from the map. It is called by the
       * \c AlterationNotifier.
       */
      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        _inv_map.clear();
        return Map::clear();
      }

      /**
       * @brief Reserve the memory to accomodate n items
       *
       * \c AlterationNotifier.
       */
      // constexpr virtual graphs::ObserverReturnCode reserve(int n) noexcept override {
      //   _inv_map.reserve(n);
      //   return Map::reserve(n);
      // }

      public:
      /**
       * @brief Returns the maximal value plus one.
       *
       * Returns the maximal value plus one in the map.
       */
      unsigned int size() const { return _inv_map.size(); }

      /**
       * @brief Swaps the position of the two items in the map.
       *
       * Swaps the position of the two items in the map.
       */
      void swap(const Item& p, const Item& q) {
        int pi = Map::operator[](p);
        int qi = Map::operator[](q);
        Map::set(p, qi);
        _inv_map[qi] = p;
        Map::set(q, pi);
        _inv_map[pi] = q;
      }

      /**
       * @brief Gives back the \e range \e id of the item
       *
       * Gives back the \e range \e id of the item.
       */
      int operator[](const Item& item) const { return Map::operator[](item); }

      /**
       * @brief Gives back the item belonging to a \e range \e id
       *
       * Gives back the item belonging to the given \e range \e id.
       */
      Item operator()(int id) const { return _inv_map[id]; }

      private:
      using Container = nt::DynamicArray< Item >;
      Container _inv_map;

      public:
      /**
       * @brief The inverse map type of RangeIdMap.
       *
       * The inverse map type of RangeIdMap. The subscript operator gives
       * back an item by its \e range \e id.
       * This type conforms to the @ref concepts::ReadMap "ReadMap" concept.
       */
      class InverseMap {
        public:
        /**
         * @brief Constructor
         *
         * Constructor of the InverseMap.
         */
        explicit InverseMap(const RangeIdMap& inverted) : _inverted(inverted) {}

        /** The value type of the InverseMap. */
        using Value = typename RangeIdMap::Key;
        /** The key type of the InverseMap. */
        using Key = typename RangeIdMap::Value;

        /**
         * @brief Subscript operator.
         *
         * Subscript operator. It gives back the item
         * that the given \e range \e id currently belongs to.
         */
        Value operator[](const Key& key) const { return _inverted(key); }

        /**
         * @brief Size of the map.
         *
         * Returns the size of the map.
         */
        unsigned int size() const { return _inverted.size(); }

        private:
        const RangeIdMap& _inverted;
      };

      /**
       * @brief Gives back the inverse of the map.
       *
       * Gives back the inverse of the RangeIdMap.
       */
      const InverseMap inverse() const { return InverseMap(*this); }
    };

    /**
     * @brief Returns a \c RangeIdMap class.
     *
     * This function just returns an \c RangeIdMap class.
     * \relates RangeIdMap
     */
    template < typename K, typename GR >
    inline RangeIdMap< GR, K > rangeIdMap(const GR& graph) {
      return RangeIdMap< GR, K >(graph);
    }

    /**
     * @brief Dynamic iterable \c bool map.
     *
     * This class provides a special graph map type which can store a
     * \c bool value for graph items (\c Node, \c Arc or \c Edge).
     * For both \c true and \c false values it is possible to iterate on
     * the keys mapped to the value.
     *
     * This type is a reference map, so it can be modified with the
     * subscript operator.
     *
     * @tparam GR The graph type.
     * @tparam K The key type of the map (\c GR::Node, \c GR::Arc or
     * \c GR::Edge).
     *
     * @see IterableIntMap, IterableValueMap
     * @see CrossRefMap
     */
    template < typename GR, typename K >
    class IterableBoolMap: protected ItemSetTraits< GR, K >::template Map< int >::Type {
      private:
      using Graph = GR;

      using KeyIt = typename ItemSetTraits< GR, K >::ItemIt;
      using Parent = typename ItemSetTraits< GR, K >::template Map< int >::Type;

      nt::DynamicArray< K > _array;
      int                   _sep;

      public:
      /** Indicates that the map is reference map. */
      using ReferenceMapTag = True;

      /** The key type */
      using Key = K;
      /** The value type */
      using Value = bool;
      /** The const reference type. */
      using ConstReference = const Value&;

      private:
      int position(const Key& key) const { return Parent::operator[](key); }

      public:
      /**
       * @brief Reference to the value of the map.
       *
       * This class is similar to the \c bool type. It can be converted to
       * \c bool and it provides the same operators.
       */
      class Reference {
        friend class IterableBoolMap;

        private:
        Reference(IterableBoolMap& map, const Key& key) : _key(key), _map(map) {}

        public:
        Reference& operator=(const Reference& value) {
          _map.set(_key, static_cast< bool >(value));
          return *this;
        }

        operator bool() const { return static_cast< const IterableBoolMap& >(_map)[_key]; }

        Reference& operator=(bool value) {
          _map.set(_key, value);
          return *this;
        }
        Reference& operator&=(bool value) {
          _map.set(_key, _map[_key] & value);
          return *this;
        }
        Reference& operator|=(bool value) {
          _map.set(_key, _map[_key] | value);
          return *this;
        }
        Reference& operator^=(bool value) {
          _map.set(_key, _map[_key] ^ value);
          return *this;
        }

        private:
        Key              _key;
        IterableBoolMap& _map;
      };

      /**
       * @brief Constructor of the map with a default value.
       *
       * Constructor of the map with a default value.
       */
      explicit IterableBoolMap(const Graph& graph, bool def = false) : Parent(graph) {
        typename Parent::Notifier* nf = Parent::notifier();
        Key                        it;
        for (nf->first(it); it != INVALID; nf->next(it)) {
          Parent::set(it, _array.size());
          _array.push_back(it);
        }
        _sep = (def ? _array.size() : 0);
      }

      /**
       * @brief Const subscript operator of the map.
       *
       * Const subscript operator of the map.
       */
      bool operator[](const Key& key) const { return position(key) < _sep; }

      /**
       * @brief Subscript operator of the map.
       *
       * Subscript operator of the map.
       */
      Reference operator[](const Key& key) { return Reference(*this, key); }

      /**
       * @brief Set operation of the map.
       *
       * Set operation of the map.
       */
      void set(const Key& key, bool value) {
        int pos = position(key);
        if (value) {
          if (pos < _sep) return;
          Key tmp = _array[_sep];
          _array[_sep] = key;
          Parent::set(key, _sep);
          _array[pos] = tmp;
          Parent::set(tmp, pos);
          ++_sep;
        } else {
          if (pos >= _sep) return;
          --_sep;
          Key tmp = _array[_sep];
          _array[_sep] = key;
          Parent::set(key, _sep);
          _array[pos] = tmp;
          Parent::set(tmp, pos);
        }
      }

      /**
       * @brief Set all items.
       *
       * Set all items in the map.
       * @note Constant time operation.
       */
      void setAll(bool value) { _sep = (value ? _array.size() : 0); }

      /**
       * @brief Returns the number of the keys mapped to \c true.
       *
       * Returns the number of the keys mapped to \c true.
       */
      int trueNum() const { return _sep; }

      /**
       * @brief Returns the number of the keys mapped to \c false.
       *
       * Returns the number of the keys mapped to \c false.
       */
      int falseNum() const { return _array.size() - _sep; }

      /**
       * @brief Iterator for the keys mapped to \c true.
       *
       * Iterator for the keys mapped to \c true. It works
       * like a graph item iterator, it can be converted to
       * the key type of the map, incremented with \c ++ operator, and
       * if the iterator leaves the last valid key, it will be equal to
       * \c INVALID.
       */
      class TrueIt: public Key {
        public:
        using Parent = Key;

        /**
         * @brief Creates an iterator.
         *
         * Creates an iterator. It iterates on the
         * keys mapped to \c true.
         * @param map The IterableBoolMap.
         */
        explicit TrueIt(const IterableBoolMap& map) :
            Parent(map._sep > 0 ? map._array[map._sep - 1] : INVALID), _map(&map) {}

        /**
         * @brief Invalid constructor \& conversion.
         *
         * This constructor initializes the iterator to be invalid.
         * \sa Invalid for more details.
         */
        TrueIt(Invalid) : Parent(INVALID), _map(0) {}

        /**
         * @brief Increment operator.
         *
         * Increment operator.
         */
        TrueIt& operator++() {
          int     pos = _map->position(*this);
          Parent::operator=(pos > 0 ? _map->_array[pos - 1] : INVALID);
          return *this;
        }

        private:
        const IterableBoolMap* _map;
      };

      /**
       * @brief Iterator for the keys mapped to \c false.
       *
       * Iterator for the keys mapped to \c false. It works
       * like a graph item iterator, it can be converted to
       * the key type of the map, incremented with \c ++ operator, and
       * if the iterator leaves the last valid key, it will be equal to
       * \c INVALID.
       */
      class FalseIt: public Key {
        public:
        using Parent = Key;

        /**
         * @brief Creates an iterator.
         *
         * Creates an iterator. It iterates on the
         * keys mapped to \c false.
         * @param map The IterableBoolMap.
         */
        explicit FalseIt(const IterableBoolMap& map) :
            Parent(map._sep < int(map._array.size()) ? map._array.back() : INVALID), _map(&map) {}

        /**
         * @brief Invalid constructor \& conversion.
         *
         * This constructor initializes the iterator to be invalid.
         * \sa Invalid for more details.
         */
        FalseIt(Invalid) : Parent(INVALID), _map(0) {}

        /**
         * @brief Increment operator.
         *
         * Increment operator.
         */
        FalseIt& operator++() {
          int     pos = _map->position(*this);
          Parent::operator=(pos > _map->_sep ? _map->_array[pos - 1] : INVALID);
          return *this;
        }

        private:
        const IterableBoolMap* _map;
      };

      /**
       * @brief Iterator for the keys mapped to a given value.
       *
       * Iterator for the keys mapped to a given value. It works
       * like a graph item iterator, it can be converted to
       * the key type of the map, incremented with \c ++ operator, and
       * if the iterator leaves the last valid key, it will be equal to
       * \c INVALID.
       */
      class ItemIt: public Key {
        public:
        using Parent = Key;

        /**
         * @brief Creates an iterator with a value.
         *
         * Creates an iterator with a value. It iterates on the
         * keys mapped to the given value.
         * @param map The IterableBoolMap.
         * @param value The value.
         */
        ItemIt(const IterableBoolMap& map, bool value) :
            Parent(value ? (map._sep > 0 ? map._array[map._sep - 1] : INVALID)
                         : (map._sep < int(map._array.size()) ? map._array.back() : INVALID)),
            _map(&map) {}

        /**
         * @brief Invalid constructor \& conversion.
         *
         * This constructor initializes the iterator to be invalid.
         * \sa Invalid for more details.
         */
        ItemIt(Invalid) : Parent(INVALID), _map(0) {}

        /**
         * @brief Increment operator.
         *
         * Increment operator.
         */
        ItemIt& operator++() {
          int     pos = _map->position(*this);
          int     _sep = pos >= _map->_sep ? _map->_sep : 0;
          Parent::operator=(pos > _sep ? _map->_array[pos - 1] : INVALID);
          return *this;
        }

        private:
        const IterableBoolMap* _map;
      };

      protected:
      constexpr virtual graphs::ObserverReturnCode add(const Key& key) noexcept override {
        Parent::add(key);
        Parent::set(key, _array.size());
        _array.push_back(key);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Key >& keys) noexcept override {
        Parent::add(keys);
        for (int i = 0; i < int(keys.size()); ++i) {
          Parent::set(keys[i], _array.size());
          _array.push_back(keys[i]);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const Key& key) noexcept override {
        int pos = position(key);
        if (pos < _sep) {
          --_sep;
          Parent::set(_array[_sep], pos);
          _array[pos] = _array[_sep];
          Parent::set(_array.back(), _sep);
          _array[_sep] = _array.back();
          _array.pop_back();
        } else {
          Parent::set(_array.back(), pos);
          _array[pos] = _array.back();
          _array.pop_back();
        }
        Parent::erase(key);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Key >& keys) noexcept override {
        for (int i = 0; i < int(keys.size()); ++i) {
          int pos = position(keys[i]);
          if (pos < _sep) {
            --_sep;
            Parent::set(_array[_sep], pos);
            _array[pos] = _array[_sep];
            Parent::set(_array.back(), _sep);
            _array[_sep] = _array.back();
            _array.pop_back();
          } else {
            Parent::set(_array.back(), pos);
            _array[pos] = _array.back();
            _array.pop_back();
          }
        }
        Parent::erase(keys);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode build() noexcept override {
        Parent::build();
        typename Parent::Notifier* nf = Parent::notifier();
        Key                        it;
        for (nf->first(it); it != INVALID; nf->next(it)) {
          Parent::set(it, _array.size());
          _array.push_back(it);
        }
        _sep = 0;
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        _array.clear();
        _sep = 0;
        Parent::clear();
        return graphs::ObserverReturnCode::Ok;
      }
    };

    namespace _maps_bits {
      template < typename Item >
      struct IterableIntMapNode {
        IterableIntMapNode() : value(-1) {}
        IterableIntMapNode(int _value) : value(_value) {}
        Item prev, next;
        int  value;
      };
    }   // namespace _maps_bits

    /**
     * @brief Dynamic iterable integer map.
     *
     * This class provides a special graph map type which can store an
     * integer value for graph items (\c Node, \c Arc or \c Edge).
     * For each non-negative value it is possible to iterate on the keys
     * mapped to the value.
     *
     * This map is intended to be used with small integer values, for which
     * it is efficient, and supports iteration only for non-negative values.
     * If you need large values and/or iteration for negative integers,
     * consider to use @ref IterableValueMap instead.
     *
     * This type is a reference map, so it can be modified with the
     * subscript operator.
     *
     * @note The size of the data structure depends on the largest
     * value in the map.
     *
     * @tparam GR The graph type.
     * @tparam K The key type of the map (\c GR::Node, \c GR::Arc or
     * \c GR::Edge).
     *
     * @see IterableBoolMap, IterableValueMap
     * @see CrossRefMap
     */
    template < typename GR, typename K >
    class IterableIntMap: protected ItemSetTraits< GR, K >::template Map< _maps_bits::IterableIntMapNode< K > >::Type {
      public:
      using Parent = typename ItemSetTraits< GR, K >::template Map< _maps_bits::IterableIntMapNode< K > >::Type;

      /** The key type */
      using Key = K;
      /** The value type */
      using Value = int;
      /** The graph type */
      using Graph = GR;

      /**
       * @brief Constructor of the map.
       *
       * Constructor of the map. It sets all values to -1.
       */
      explicit IterableIntMap(const Graph& graph) : Parent(graph) {}

      /**
       * @brief Constructor of the map with a given value.
       *
       * Constructor of the map with a given value.
       */
      explicit IterableIntMap(const Graph& graph, int value) :
          Parent(graph, _maps_bits::IterableIntMapNode< K >(value)) {
        if (value >= 0) {
          for (typename Parent::ItemIt it(*this); it != INVALID; ++it) {
            lace(it);
          }
        }
      }

      private:
      void unlace(const Key& key) {
        typename Parent::Value& node = Parent::operator[](key);
        if (node.value < 0) return;
        if (node.prev != INVALID) {
          Parent::operator[](node.prev).next = node.next;
        } else {
          _first[node.value] = node.next;
        }
        if (node.next != INVALID) { Parent::operator[](node.next).prev = node.prev; }
        while (!_first.empty() && _first.back() == INVALID) {
          _first.pop_back();
        }
      }

      void lace(const Key& key) {
        typename Parent::Value& node = Parent::operator[](key);
        if (node.value < 0) return;
        if (node.value >= int(_first.size())) { _first.resize(node.value + 1, INVALID); }
        node.prev = INVALID;
        node.next = _first[node.value];
        if (node.next != INVALID) { Parent::operator[](node.next).prev = key; }
        _first[node.value] = key;
      }

      public:
      /** Indicates that the map is reference map. */
      using ReferenceMapTag = True;

      /**
       * @brief Reference to the value of the map.
       *
       * This class is similar to the \c int type. It can
       * be converted to \c int and it has the same operators.
       */
      class Reference {
        friend class IterableIntMap;

        private:
        Reference(IterableIntMap& map, const Key& key) : _key(key), _map(map) {}

        public:
        Reference& operator=(const Reference& value) {
          _map.set(_key, static_cast< const int& >(value));
          return *this;
        }

        operator const int&() const { return static_cast< const IterableIntMap& >(_map)[_key]; }

        Reference& operator=(int value) {
          _map.set(_key, value);
          return *this;
        }
        Reference& operator++() {
          _map.set(_key, _map[_key] + 1);
          return *this;
        }
        int operator++(int) {
          int value = _map[_key];
          _map.set(_key, value + 1);
          return value;
        }
        Reference& operator--() {
          _map.set(_key, _map[_key] - 1);
          return *this;
        }
        int operator--(int) {
          int value = _map[_key];
          _map.set(_key, value - 1);
          return value;
        }
        Reference& operator+=(int value) {
          _map.set(_key, _map[_key] + value);
          return *this;
        }
        Reference& operator-=(int value) {
          _map.set(_key, _map[_key] - value);
          return *this;
        }
        Reference& operator*=(int value) {
          _map.set(_key, _map[_key] * value);
          return *this;
        }
        Reference& operator/=(int value) {
          _map.set(_key, _map[_key] / value);
          return *this;
        }
        Reference& operator%=(int value) {
          _map.set(_key, _map[_key] % value);
          return *this;
        }
        Reference& operator&=(int value) {
          _map.set(_key, _map[_key] & value);
          return *this;
        }
        Reference& operator|=(int value) {
          _map.set(_key, _map[_key] | value);
          return *this;
        }
        Reference& operator^=(int value) {
          _map.set(_key, _map[_key] ^ value);
          return *this;
        }
        Reference& operator<<=(int value) {
          _map.set(_key, _map[_key] << value);
          return *this;
        }
        Reference& operator>>=(int value) {
          _map.set(_key, _map[_key] >> value);
          return *this;
        }

        private:
        Key             _key;
        IterableIntMap& _map;
      };

      /** The const reference type. */
      using ConstReference = const Value&;

      /**
       * @brief Gives back the maximal value plus one.
       *
       * Gives back the maximal value plus one.
       */
      int size() const { return _first.size(); }

      /**
       * @brief Set operation of the map.
       *
       * Set operation of the map.
       */
      void set(const Key& key, const Value& value) {
        unlace(key);
        Parent::operator[](key).value = value;
        lace(key);
      }

      /**
       * @brief Const subscript operator of the map.
       *
       * Const subscript operator of the map.
       */
      const Value& operator[](const Key& key) const { return Parent::operator[](key).value; }

      /**
       * @brief Subscript operator of the map.
       *
       * Subscript operator of the map.
       */
      Reference operator[](const Key& key) { return Reference(*this, key); }

      /**
       * @brief Iterator for the keys with the same value.
       *
       * Iterator for the keys with the same value. It works
       * like a graph item iterator, it can be converted to
       * the item type of the map, incremented with \c ++ operator, and
       * if the iterator leaves the last valid item, it will be equal to
       * \c INVALID.
       */
      class ItemIt: public Key {
        public:
        using Parent = Key;

        /**
         * @brief Invalid constructor \& conversion.
         *
         * This constructor initializes the iterator to be invalid.
         * \sa Invalid for more details.
         */
        ItemIt(Invalid) : Parent(INVALID), _map(0) {}

        /**
         * @brief Creates an iterator with a value.
         *
         * Creates an iterator with a value. It iterates on the
         * keys mapped to the given value.
         * @param map The IterableIntMap.
         * @param value The value.
         */
        ItemIt(const IterableIntMap& map, int value) : _map(&map) {
          if (value < 0 || value >= int(_map->_first.size())) {
            Parent::operator=(INVALID);
          } else {
            Parent::operator=(_map->_first[value]);
          }
        }

        /**
         * @brief Increment operator.
         *
         * Increment operator.
         */
        ItemIt& operator++() {
          Parent::operator=(_map->IterableIntMap::Parent::operator[](static_cast< Parent& >(*this)).next);
          return *this;
        }

        private:
        const IterableIntMap* _map;
      };

      protected:
      constexpr virtual graphs::ObserverReturnCode erase(const Key& key) noexcept override {
        unlace(key);
        Parent::erase(key);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Key >& keys) noexcept override {
        for (int i = 0; i < int(keys.size()); ++i) {
          unlace(keys[i]);
        }
        Parent::erase(keys);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        _first.clear();
        Parent::clear();
        return graphs::ObserverReturnCode::Ok;
      }

      private:
      nt::DynamicArray< Key > _first;
    };

    namespace _maps_bits {
      template < typename Item, typename Value >
      struct IterableValueMapNode {
        IterableValueMapNode(Value _value = Value()) : value(_value) {}
        Item  prev, next;
        Value value;
      };
    }   // namespace _maps_bits

    /**
     * @brief Dynamic iterable map for comparable values.
     *
     * This class provides a special graph map type which can store a
     * comparable value for graph items (\c Node, \c Arc or \c Edge).
     * For each value it is possible to iterate on the keys mapped to
     * the value (\c ItemIt), and the values of the map can be accessed
     * with an STL compatible forward iterator (\c ValueIt).
     * The map stores a linked list for each value, which contains
     * the items mapped to the value, and the used values are stored
     * in balanced binary tree (\c std::map).
     *
     * @ref IterableBoolMap and @ref IterableIntMap are similar classes
     * specialized for \c bool and \c int values, respectively.
     *
     * This type is not reference map, so it cannot be modified with
     * the subscript operator.
     *
     * @tparam GR The graph type.
     * @tparam K The key type of the map (\c GR::Node, \c GR::Arc or
     * \c GR::Edge).
     * @tparam V The value type of the map. It can be any comparable
     * value type.
     *
     * @see IterableBoolMap, IterableIntMap
     * @see CrossRefMap
     */
    template < typename GR, typename K, typename V >
    class IterableValueMap:
        protected ItemSetTraits< GR, K >::template Map< _maps_bits::IterableValueMapNode< K, V > >::Type {
      public:
      using Parent = typename ItemSetTraits< GR, K >::template Map< _maps_bits::IterableValueMapNode< K, V > >::Type;

      /** The key type */
      using Key = K;
      /** The value type */
      using Value = V;
      /** The graph type */
      using Graph = GR;

      public:
      /**
       * @brief Constructor of the map with a given value.
       *
       * Constructor of the map with a given value.
       */
      explicit IterableValueMap(const Graph& graph, const Value& value = Value()) :
          Parent(graph, _maps_bits::IterableValueMapNode< K, V >(value)) {
        for (typename Parent::ItemIt it(*this); it != INVALID; ++it) {
          lace(it);
        }
      }

      protected:
      void unlace(const Key& key) {
        typename Parent::Value& node = Parent::operator[](key);
        if (node.prev != INVALID) {
          Parent::operator[](node.prev).next = node.next;
        } else {
          if (node.next != INVALID) {
            _first[node.value] = node.next;
          } else {
            _first.erase(node.value);
          }
        }
        if (node.next != INVALID) { Parent::operator[](node.next).prev = node.prev; }
      }

      void lace(const Key& key) {
        typename Parent::Value& node = Parent::   operator[](key);
        typename std::map< Value, Key >::iterator it = _first.find(node.value);
        if (it == _first.end()) {
          node.prev = node.next = INVALID;
          _first.insert(std::make_pair(node.value, key));
        } else {
          node.prev = INVALID;
          node.next = it->second;
          if (node.next != INVALID) { Parent::operator[](node.next).prev = key; }
          it->second = key;
        }
      }

      public:
      /**
       * @brief Forward iterator for values.
       *
       * This iterator is an STL compatible forward
       * iterator on the values of the map. The values can
       * be accessed in the <tt>[beginValue, endValue)</tt> range.
       */
      class ValueIt: public std::iterator< std::forward_iterator_tag, Value > {
        friend class IterableValueMap;

        private:
        ValueIt(typename std::map< Value, Key >::const_iterator _it) : it(_it) {}

        public:
        /** Constructor */
        ValueIt() {}

        
        ValueIt& operator++() {
          ++it;
          return *this;
        }
        
        ValueIt operator++(int) {
          ValueIt tmp(*this);
                  operator++();
          return tmp;
        }

        
        const Value& operator*() const { return it->first; }
        
        const Value* operator->() const { return &(it->first); }

        
        bool operator==(ValueIt jt) const { return it == jt.it; }
        
        bool operator!=(ValueIt jt) const { return it != jt.it; }

        private:
        typename std::map< Value, Key >::const_iterator it;
      };

      /**
       * @brief Returns an iterator to the first value.
       *
       * Returns an STL compatible iterator to the
       * first value of the map. The values of the
       * map can be accessed in the <tt>[beginValue, endValue)</tt>
       * range.
       */
      ValueIt beginValue() const { return ValueIt(_first.begin()); }

      /**
       * @brief Returns an iterator after the last value.
       *
       * Returns an STL compatible iterator after the
       * last value of the map. The values of the
       * map can be accessed in the <tt>[beginValue, endValue)</tt>
       * range.
       */
      ValueIt endValue() const { return ValueIt(_first.end()); }

      /**
       * @brief Set operation of the map.
       *
       * Set operation of the map.
       */
      void set(const Key& key, const Value& value) {
        unlace(key);
        Parent::operator[](key).value = value;
        lace(key);
      }

      /**
       * @brief Const subscript operator of the map.
       *
       * Const subscript operator of the map.
       */
      const Value& operator[](const Key& key) const { return Parent::operator[](key).value; }

      /**
       * @brief Iterator for the keys with the same value.
       *
       * Iterator for the keys with the same value. It works
       * like a graph item iterator, it can be converted to
       * the item type of the map, incremented with \c ++ operator, and
       * if the iterator leaves the last valid item, it will be equal to
       * \c INVALID.
       */
      class ItemIt: public Key {
        public:
        using Parent = Key;

        /**
         * @brief Invalid constructor \& conversion.
         *
         * This constructor initializes the iterator to be invalid.
         * \sa Invalid for more details.
         */
        ItemIt(Invalid) : Parent(INVALID), _map(0) {}

        /**
         * @brief Creates an iterator with a value.
         *
         * Creates an iterator with a value. It iterates on the
         * keys which have the given value.
         * @param map The IterableValueMap
         * @param value The value
         */
        ItemIt(const IterableValueMap& map, const Value& value) : _map(&map) {
          typename std::map< Value, Key >::const_iterator it = map._first.find(value);
          if (it == map._first.end()) {
            Parent::operator=(INVALID);
          } else {
            Parent::operator=(it->second);
          }
        }

        /**
         * @brief Increment operator.
         *
         * Increment Operator.
         */
        ItemIt& operator++() {
          Parent::operator=(_map->IterableValueMap::Parent::operator[](static_cast< Parent& >(*this)).next);
          return *this;
        }

        private:
        const IterableValueMap* _map;
      };

      protected:
      constexpr virtual graphs::ObserverReturnCode add(const Key& key) noexcept override {
        const graphs::ObserverReturnCode r = Parent::add(key);
        if (r != graphs::ObserverReturnCode::Ok) return r;
        lace(key);
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Key >& keys) noexcept override {
        const graphs::ObserverReturnCode r = Parent::add(keys);
        if (r != graphs::ObserverReturnCode::Ok) return r;
        for (int i = 0; i < int(keys.size()); ++i) {
          lace(keys[i]);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const Key& key) noexcept override {
        unlace(key);
        return Parent::erase(key);
      }

      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Key >& keys) noexcept override {
        for (int i = 0; i < int(keys.size()); ++i) {
          unlace(keys[i]);
        }
        return Parent::erase(keys);
      }

      constexpr virtual graphs::ObserverReturnCode build() noexcept override {
        const graphs::ObserverReturnCode r = Parent::build();
        if (r != graphs::ObserverReturnCode::Ok) return r;
        for (typename Parent::ItemIt it(*this); it != INVALID; ++it) {
          lace(it);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        _first.clear();
        return Parent::clear();
      }

      private:
      std::map< Value, Key > _first;
    };

    /**
     * @brief Map of the source nodes of arcs in a digraph.
     *
     * SourceMap provides access for the source node of each arc in a digraph,
     * which is returned by the \c source() function of the digraph.
     * @tparam GR The digraph type.
     * @see TargetMap
     */
    template < typename GR >
    class SourceMap {
      public:
      /** The key type (the \c Arc type of the digraph). */
      using Key = typename GR::Arc;
      /** The value type (the \c Node type of the digraph). */
      using Value = typename GR::Node;

      /**
       * @brief Constructor
       *
       * Constructor.
       * @param digraph The digraph that the map belongs to.
       */
      explicit SourceMap(const GR& digraph) : _graph(digraph) {}

      /**
       * @brief Returns the source node of the given arc.
       *
       * Returns the source node of the given arc.
       */
      Value operator[](const Key& arc) const { return _graph.source(arc); }

      private:
      const GR& _graph;
    };

    /**
     * @brief Returns a \c SourceMap class.
     *
     * This function just returns an \c SourceMap class.
     * \relates SourceMap
     */
    template < typename GR >
    inline SourceMap< GR > sourceMap(const GR& graph) {
      return SourceMap< GR >(graph);
    }

    /**
     * @brief Map of the target nodes of arcs in a digraph.
     *
     * TargetMap provides access for the target node of each arc in a digraph,
     * which is returned by the \c target() function of the digraph.
     * @tparam GR The digraph type.
     * @see SourceMap
     */
    template < typename GR >
    class TargetMap {
      public:
      /** The key type (the \c Arc type of the digraph). */
      using Key = typename GR::Arc;
      /** The value type (the \c Node type of the digraph). */
      using Value = typename GR::Node;

      /**
       * @brief Constructor
       *
       * Constructor.
       * @param digraph The digraph that the map belongs to.
       */
      explicit TargetMap(const GR& digraph) : _graph(digraph) {}

      /**
       * @brief Returns the target node of the given arc.
       *
       * Returns the target node of the given arc.
       */
      Value operator[](const Key& e) const { return _graph.target(e); }

      private:
      const GR& _graph;
    };

    /**
     * @brief Returns a \c TargetMap class.
     *
     * This function just returns a \c TargetMap class.
     * \relates TargetMap
     */
    template < typename GR >
    inline TargetMap< GR > targetMap(const GR& graph) {
      return TargetMap< GR >(graph);
    }

    /**
     * @brief Map of the "forward" directed arc view of edges in a graph.
     *
     * ForwardMap provides access for the "forward" directed arc view of
     * each edge in a graph, which is returned by the \c direct() function
     * of the graph with \c true parameter.
     * @tparam GR The graph type.
     * @see BackwardMap
     */
    template < typename GR >
    class ForwardMap {
      public:
      /** The key type (the \c Edge type of the digraph). */
      using Key = typename GR::Edge;
      /** The value type (the \c Arc type of the digraph). */
      using Value = typename GR::Arc;

      /**
       * @brief Constructor
       *
       * Constructor.
       * @param graph The graph that the map belongs to.
       */
      explicit ForwardMap(const GR& graph) : _graph(graph) {}

      /**
       * @brief Returns the "forward" directed arc view of the given edge.
       *
       * Returns the "forward" directed arc view of the given edge.
       */
      Value operator[](const Key& key) const { return _graph.direct(key, true); }

      private:
      const GR& _graph;
    };

    /**
     * @brief Returns a \c ForwardMap class.
     *
     * This function just returns an \c ForwardMap class.
     * \relates ForwardMap
     */
    template < typename GR >
    inline ForwardMap< GR > forwardMap(const GR& graph) {
      return ForwardMap< GR >(graph);
    }

    /**
     * @brief Map of the "backward" directed arc view of edges in a graph.
     *
     * BackwardMap provides access for the "backward" directed arc view of
     * each edge in a graph, which is returned by the \c direct() function
     * of the graph with \c false parameter.
     * @tparam GR The graph type.
     * @see ForwardMap
     */
    template < typename GR >
    class BackwardMap {
      public:
      /** The key type (the \c Edge type of the digraph). */
      using Key = typename GR::Edge;
      /** The value type (the \c Arc type of the digraph). */
      using Value = typename GR::Arc;

      /**
       * @brief Constructor
       *
       * Constructor.
       * @param graph The graph that the map belongs to.
       */
      explicit BackwardMap(const GR& graph) : _graph(graph) {}

      /**
       * @brief Returns the "backward" directed arc view of the given edge.
       *
       * Returns the "backward" directed arc view of the given edge.
       */
      Value operator[](const Key& key) const { return _graph.direct(key, false); }

      private:
      const GR& _graph;
    };

    /** @brief Returns a \c BackwardMap class */

    /**
     * This function just returns a \c BackwardMap class.
     * \relates BackwardMap
     */
    template < typename GR >
    inline BackwardMap< GR > backwardMap(const GR& graph) {
      return BackwardMap< GR >(graph);
    }

    /**
     * @brief Map of the in-degrees of nodes in a digraph.
     *
     * This map returns the in-degree of a node. Once it is constructed,
     * the degrees are stored in a standard \c NodeMap, so each query is done
     * in constant time. On the other hand, the values are updated automatically
     * whenever the digraph changes.
     *
     * @warning Besides \c addNode() and \c addArc(), a digraph structure
     * may provide alternative ways to modify the digraph.
     * The correct behavior of InDegMap is not guarantied if these additional
     * features are used. For example, the functions
     * @ref ListDigraph::changeSource() "changeSource()",
     * @ref ListDigraph::changeTarget() "changeTarget()" and
     * @ref ListDigraph::reverseArc() "reverseArc()"
     * of @ref ListDigraph will \e not update the degree values correctly.
     *
     * \sa OutDegMap
     */
    template < typename GR >
    class InDegMap: protected ItemSetTraits< GR, typename GR::Arc >::ItemNotifier::ObserverBase {
      public:
      /** The graph type of InDegMap */
      using Graph = GR;
      using Digraph = GR;
      /** The key type */
      using Key = typename Digraph::Node;
      /** The value type */
      using Value = int;

      using Parent = typename ItemSetTraits< Digraph, typename Digraph::Arc >::ItemNotifier::ObserverBase;

      private:
      class AutoNodeMap: public ItemSetTraits< Digraph, Key >::template Map< int >::Type {
        public:
        using Parent = typename ItemSetTraits< Digraph, Key >::template Map< int >::Type;

        AutoNodeMap(const Digraph& digraph) : Parent(digraph, 0) {}

        constexpr virtual graphs::ObserverReturnCode add(const Key& key) noexcept override {
          const graphs::ObserverReturnCode r = Parent::add(key);
          if (r != graphs::ObserverReturnCode::Ok) return r;
          Parent::set(key, 0);
          return graphs::ObserverReturnCode::Ok;
        }

        constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Key >& keys) noexcept override {
          const graphs::ObserverReturnCode r = Parent::add(keys);
          if (r != graphs::ObserverReturnCode::Ok) return r;
          for (int i = 0; i < int(keys.size()); ++i) {
            Parent::set(keys[i], 0);
          }
          return graphs::ObserverReturnCode::Ok;
        }

        constexpr virtual graphs::ObserverReturnCode build() noexcept override {
          const graphs::ObserverReturnCode r = Parent::build();
          if (r != graphs::ObserverReturnCode::Ok) return r;
          Key                        it;
          typename Parent::Notifier* nf = Parent::notifier();
          for (nf->first(it); it != INVALID; nf->next(it)) {
            Parent::set(it, 0);
          }
          return graphs::ObserverReturnCode::Ok;
        }
      };

      public:
      /**
       * @brief Constructor.
       *
       * Constructor for creating an in-degree map.
       */
      explicit InDegMap(const Digraph& graph) : _digraph(graph), _deg(graph) {
        Parent::attach(_digraph.notifier(typename Digraph::Arc()));

        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = countInArcs(_digraph, it);
        }
      }

      /**
       * @brief Gives back the in-degree of a Node.
       *
       * Gives back the in-degree of a Node.
       */
      int operator[](const Key& key) const { return _deg[key]; }

      protected:
      using Arc = typename Digraph::Arc;

      constexpr virtual graphs::ObserverReturnCode add(const Arc& arc) noexcept override {
        ++_deg[_digraph.target(arc)];
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Arc >& arcs) noexcept override {
        for (int i = 0; i < int(arcs.size()); ++i) {
          ++_deg[_digraph.target(arcs[i])];
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const Arc& arc) noexcept override {
        --_deg[_digraph.target(arc)];
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Arc >& arcs) noexcept override {
        for (int i = 0; i < int(arcs.size()); ++i) {
          --_deg[_digraph.target(arcs[i])];
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode build() noexcept override {
        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = countInArcs(_digraph, it);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = 0;
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode reserve(int n) noexcept override {
        NT_ASSERT(false, "Should not be called.");
        return graphs::ObserverReturnCode::Ok;
        //  return _deg.reserve(n);
      }


      private:
      const Digraph& _digraph;
      AutoNodeMap    _deg;
    };

    /**
     * @brief Map of the out-degrees of nodes in a digraph.
     *
     * This map returns the out-degree of a node. Once it is constructed,
     * the degrees are stored in a standard \c NodeMap, so each query is done
     * in constant time. On the other hand, the values are updated automatically
     * whenever the digraph changes.
     *
     * @warning Besides \c addNode() and \c addArc(), a digraph structure
     * may provide alternative ways to modify the digraph.
     * The correct behavior of OutDegMap is not guarantied if these additional
     * features are used. For example, the functions
     * @ref ListDigraph::changeSource() "changeSource()",
     * @ref ListDigraph::changeTarget() "changeTarget()" and
     * @ref ListDigraph::reverseArc() "reverseArc()"
     * of @ref ListDigraph will \e not update the degree values correctly.
     *
     * \sa InDegMap
     */
    template < typename GR >
    class OutDegMap: protected ItemSetTraits< GR, typename GR::Arc >::ItemNotifier::ObserverBase {
      public:
      /** The graph type of OutDegMap */
      using Graph = GR;
      using Digraph = GR;
      /** The key type */
      using Key = typename Digraph::Node;
      /** The value type */
      using Value = int;

      using Parent = typename ItemSetTraits< Digraph, typename Digraph::Arc >::ItemNotifier::ObserverBase;

      private:
      class AutoNodeMap: public ItemSetTraits< Digraph, Key >::template Map< int >::Type {
        public:
        using Parent = typename ItemSetTraits< Digraph, Key >::template Map< int >::Type;

        AutoNodeMap(const Digraph& digraph) : Parent(digraph, 0) {}

        constexpr virtual graphs::ObserverReturnCode add(const Key& key) noexcept override {
          const graphs::ObserverReturnCode r = Parent::add(key);
          if (r != graphs::ObserverReturnCode::Ok) return r;
          Parent::set(key, 0);
          return graphs::ObserverReturnCode::Ok;
        }
        constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Key >& keys) noexcept override {
          const graphs::ObserverReturnCode r = Parent::add(keys);
          if (r != graphs::ObserverReturnCode::Ok) return r;
          for (int i = 0; i < int(keys.size()); ++i) {
            Parent::set(keys[i], 0);
          }
          return graphs::ObserverReturnCode::Ok;
        }
        constexpr virtual graphs::ObserverReturnCode build() noexcept override {
          const graphs::ObserverReturnCode r = Parent::build();
          if (r != graphs::ObserverReturnCode::Ok) return r;
          Key                        it;
          typename Parent::Notifier* nf = Parent::notifier();
          for (nf->first(it); it != INVALID; nf->next(it)) {
            Parent::set(it, 0);
          }
          return graphs::ObserverReturnCode::Ok;
        }
      };

      public:
      /**
       * @brief Constructor.
       *
       * Constructor for creating an out-degree map.
       */
      explicit OutDegMap(const Digraph& graph) : _digraph(graph), _deg(graph) {
        Parent::attach(_digraph.notifier(typename Digraph::Arc()));

        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = countOutArcs(_digraph, it);
        }
      }

      /**
       * @brief Gives back the out-degree of a Node.
       *
       * Gives back the out-degree of a Node.
       */
      int operator[](const Key& key) const { return _deg[key]; }

      protected:
      using Arc = typename Digraph::Arc;

      constexpr virtual graphs::ObserverReturnCode add(const Arc& arc) noexcept override {
        ++_deg[_digraph.source(arc)];
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode add(const nt::ArrayView< Arc >& arcs) noexcept override {
        for (int i = 0; i < int(arcs.size()); ++i) {
          ++_deg[_digraph.source(arcs[i])];
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const Arc& arc) noexcept override {
        --_deg[_digraph.source(arc)];
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode erase(const nt::ArrayView< Arc >& arcs) noexcept override {
        for (int i = 0; i < int(arcs.size()); ++i) {
          --_deg[_digraph.source(arcs[i])];
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode build() noexcept override {
        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = countOutArcs(_digraph, it);
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode clear() noexcept override {
        for (typename Digraph::NodeIt it(_digraph); it != INVALID; ++it) {
          _deg[it] = 0;
        }
        return graphs::ObserverReturnCode::Ok;
      }

      constexpr virtual graphs::ObserverReturnCode reserve(int n) noexcept override {
        NT_ASSERT(false, "Should not be called.");
        return graphs::ObserverReturnCode::Ok;
        //  return _deg.reserve(n);
      }

      private:
      const Digraph& _digraph;
      AutoNodeMap    _deg;
    };

    /**
     * @brief Potential difference map
     *
     * PotentialDifferenceMap returns the difference between the potentials of
     * the source and target nodes of each arc in a digraph, i.e. it returns
     * @code
     * potential[gr.target(arc)] - potential[gr.source(arc)].
     * @endcode
     * @tparam GR The digraph type.
     * @tparam POT A node map storing the potentials.
     */
    template < typename GR, typename POT >
    class PotentialDifferenceMap {
      public:
      /** Key type */
      using Key = typename GR::Arc;
      /** Value type */
      using Value = typename POT::Value;

      /**
       * @brief Constructor
       *
       * Contructor of the map.
       */
      explicit PotentialDifferenceMap(const GR& gr, const POT& potential) : _digraph(gr), _potential(potential) {}

      /**
       * @brief Returns the potential difference for the given arc.
       *
       * Returns the potential difference for the given arc, i.e.
       * @code
       * potential[gr.target(arc)] - potential[gr.source(arc)].
       * @endcode
       */
      Value operator[](const Key& arc) const {
        return _potential[_digraph.target(arc)] - _potential[_digraph.source(arc)];
      }

      private:
      const GR&  _digraph;
      const POT& _potential;
    };

    /**
     * @brief Returns a PotentialDifferenceMap.
     *
     * This function just returns a PotentialDifferenceMap.
     * \relates PotentialDifferenceMap
     */
    template < typename GR, typename POT >
    PotentialDifferenceMap< GR, POT > potentialDifferenceMap(const GR& gr, const POT& potential) {
      return PotentialDifferenceMap< GR, POT >(gr, potential);
    }

    /**
     * @brief Copy the values of a graph map to another map.
     *
     * This function copies the values of a graph map to another graph map.
     * \c To::Key must be equal or convertible to \c From::Key and
     * \c From::Value must be equal or convertible to \c To::Value.
     *
     * For example, an edge map of \c int value type can be copied to
     * an arc map of \c double value type in an undirected graph, but
     * an arc map cannot be copied to an edge map.
     * Note that even a @ref ConstMap can be copied to a standard graph map,
     * but @ref mapFill() can also be used for this purpose.
     *
     * @param gr The graph for which the maps are defined.
     * @param from The map from which the values have to be copied.
     * It must conform to the @ref concepts::ReadMap "ReadMap" concept.
     * @param to The map to which the values have to be copied.
     * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
     */
    template < typename GR, typename From, typename To >
    void mapCopy(const GR& gr, const From& from, To& to) {
      using Item = typename To::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      for (ItemIt it(gr); it != INVALID; ++it) {
        to.set(it, from[it]);
      }
    }

    /**
     * @brief Compare two graph maps.
     *
     * This function compares the values of two graph maps. It returns
     * \c true if the maps assign the same value for all items in the graph.
     * The \c Key type of the maps (\c Node, \c Arc or \c Edge) must be equal
     * and their \c Value types must be comparable using \c %operator==().
     *
     * @param gr The graph for which the maps are defined.
     * @param map1 The first map.
     * @param map2 The second map.
     */
    template < typename GR, typename Map1, typename Map2 >
    bool mapCompare(const GR& gr, const Map1& map1, const Map2& map2) {
      using Item = typename Map2::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      for (ItemIt it(gr); it != INVALID; ++it) {
        if (!(map1[it] == map2[it])) return false;
      }
      return true;
    }

    /**
     * @brief Return an item having minimum value of a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * minimum value of the given graph map.
     * If the item set is empty, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param comp Comparison function object.
     */
    template < typename GR, typename Map, typename Comp >
    typename Map::Key mapMin(const GR& gr, const Map& map, const Comp& comp) {
      using Item = typename Map::Key;
      using Value = typename Map::Value;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      ItemIt min_item(gr);
      if (min_item == INVALID) return INVALID;
      Value min = map[min_item];
      for (ItemIt it(gr); it != INVALID; ++it) {
        if (comp(map[it], min)) {
          min = map[it];
          min_item = it;
        }
      }
      return min_item;
    }

    /**
     * @brief Return an item having minimum value of a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * minimum value of the given graph map.
     * If the item set is empty, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     */
    template < typename GR, typename Map >
    typename Map::Key mapMin(const GR& gr, const Map& map) {
      return mapMin(gr, map, std::less< typename Map::Value >());
    }

    /**
     * @brief Return an item having maximum value of a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * maximum value of the given graph map.
     * If the item set is empty, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param comp Comparison function object.
     */
    template < typename GR, typename Map, typename Comp >
    typename Map::Key mapMax(const GR& gr, const Map& map, const Comp& comp) {
      using Item = typename Map::Key;
      using Value = typename Map::Value;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      ItemIt max_item(gr);
      if (max_item == INVALID) return INVALID;
      Value max = map[max_item];
      for (ItemIt it(gr); it != INVALID; ++it) {
        if (comp(max, map[it])) {
          max = map[it];
          max_item = it;
        }
      }
      return max_item;
    }

    /**
     * @brief Return an item having maximum value of a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * maximum value of the given graph map.
     * If the item set is empty, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     */
    template < typename GR, typename Map >
    typename Map::Key mapMax(const GR& gr, const Map& map) {
      return mapMax(gr, map, std::less< typename Map::Value >());
    }

    /**
     * @brief Return the minimum value of a graph map.
     *
     * This function returns the minimum value of the given graph map.
     * The corresponding item set of the graph must not be empty.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     */
    template < typename GR, typename Map >
    typename Map::Value mapMinValue(const GR& gr, const Map& map) {
      return map[mapMin(gr, map, std::less< typename Map::Value >())];
    }

    /**
     * @brief Return the minimum value of a graph map.
     *
     * This function returns the minimum value of the given graph map.
     * The corresponding item set of the graph must not be empty.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param comp Comparison function object.
     */
    template < typename GR, typename Map, typename Comp >
    typename Map::Value mapMinValue(const GR& gr, const Map& map, const Comp& comp) {
      return map[mapMin(gr, map, comp)];
    }

    /**
     * @brief Return the maximum value of a graph map.
     *
     * This function returns the maximum value of the given graph map.
     * The corresponding item set of the graph must not be empty.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     */
    template < typename GR, typename Map >
    typename Map::Value mapMaxValue(const GR& gr, const Map& map) {
      return map[mapMax(gr, map, std::less< typename Map::Value >())];
    }

    /**
     * @brief Return the maximum value of a graph map.
     *
     * This function returns the maximum value of the given graph map.
     * The corresponding item set of the graph must not be empty.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param comp Comparison function object.
     */
    template < typename GR, typename Map, typename Comp >
    typename Map::Value mapMaxValue(const GR& gr, const Map& map, const Comp& comp) {
      return map[mapMax(gr, map, comp)];
    }

    /**
     * @brief Return an item having a specified value in a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * the specified assigned value in the given graph map.
     * If no such item exists, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param val The value that have to be found.
     */
    template < typename GR, typename Map >
    typename Map::Key mapFind(const GR& gr, const Map& map, const typename Map::Value& val) {
      using Item = typename Map::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      for (ItemIt it(gr); it != INVALID; ++it) {
        if (map[it] == val) return it;
      }
      return INVALID;
    }

    /**
     * @brief Return an item having value for which a certain predicate is
     * true in a graph map.
     *
     * This function returns an item (\c Node, \c Arc or \c Edge) having
     * such assigned value for which the specified predicate is true
     * in the given graph map.
     * If no such item exists, it returns \c INVALID.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param pred The predicate function object.
     */
    template < typename GR, typename Map, typename Pred >
    typename Map::Key mapFindIf(const GR& gr, const Map& map, const Pred& pred) {
      using Item = typename Map::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      for (ItemIt it(gr); it != INVALID; ++it) {
        if (pred(map[it])) return it;
      }
      return INVALID;
    }

    /**
     * @brief Return the number of items having a specified value in a
     * graph map.
     *
     * This function returns the number of items (\c Node, \c Arc or \c Edge)
     * having the specified assigned value in the given graph map.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param val The value that have to be counted.
     */
    template < typename GR, typename Map >
    int mapCount(const GR& gr, const Map& map, const typename Map::Value& val) {
      using Item = typename Map::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      int cnt = 0;
      for (ItemIt it(gr); it != INVALID; ++it) {
        if (map[it] == val) ++cnt;
      }
      return cnt;
    }

    /**
     * @brief Return the number of items having values for which a certain
     * predicate is true in a graph map.
     *
     * This function returns the number of items (\c Node, \c Arc or \c Edge)
     * having such assigned values for which the specified predicate is true
     * in the given graph map.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map.
     * @param pred The predicate function object.
     */
    template < typename GR, typename Map, typename Pred >
    int mapCountIf(const GR& gr, const Map& map, const Pred& pred) {
      using Item = typename Map::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      int cnt = 0;
      for (ItemIt it(gr); it != INVALID; ++it) {
        if (pred(map[it])) ++cnt;
      }
      return cnt;
    }

    /**
     * @brief Fill a graph map with a certain value.
     *
     * This function sets the specified value for all items (\c Node,
     * \c Arc or \c Edge) in the given graph map.
     *
     * @param gr The graph for which the map is defined.
     * @param map The graph map. It must conform to the
     * @ref concepts::WriteMap "WriteMap" concept.
     * @param val The value.
     */
    template < typename GR, typename Map >
    void mapFill(const GR& gr, Map& map, const typename Map::Value& val) {
      using Item = typename Map::Key;
      using ItemIt = typename ItemSetTraits< GR, Item >::ItemIt;

      for (ItemIt it(gr); it != INVALID; ++it) {
        map.set(it, val);
      }
    }

  }   // namespace graphs
}   // namespace nt

#endif   // _NT_GRAPH_MAPS_H_