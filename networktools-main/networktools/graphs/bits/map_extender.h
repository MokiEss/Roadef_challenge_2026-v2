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
 * This file incorporates work from the LEMON graph library (map_extender.h).
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
 *   - Add constexpr, noexcept specifiers
 *   - Replace typedef with using
 *   - Replace NULL with nullptr
 *   - Add MapExtender(const GraphType& graph, const char* str)
 */

/**
 * @file
 * @brief Extenders for iterable maps.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_MAP_EXTENDER_H_
#define _NT_MAP_EXTENDER_H_

namespace nt {
  namespace graphs {

    /**
     * @ingroup graphbits
     *
     * @brief Extender for maps
     */
    template < typename _Map >
    class MapExtender: public _Map {
      using Parent = _Map;
      using GraphType = typename Parent::GraphType;

      public:
      using Map = MapExtender;
      using Item = typename Parent::Key;

      using Key = typename Parent::Key;
      using Value = typename Parent::Value;
      using Reference = typename Parent::Reference;
      using ConstReference = typename Parent::ConstReference;

      using ReferenceMapTag = typename Parent::ReferenceMapTag;

      class MapIt;
      class ConstMapIt;

      friend class MapIt;
      friend class ConstMapIt;

      public:
      MapExtender(const GraphType& graph) noexcept : Parent(graph) {}
      MapExtender(const GraphType& graph, const Value& value) noexcept : Parent(graph, value) {}
      MapExtender(const GraphType& graph, const MapExtender& map) noexcept : Parent(graph, map) {}
      MapExtender(const GraphType& graph, const char* str) noexcept requires(std::is_same< Value, nt::String >::value) :
          Parent(graph, str) {}
      MapExtender(MapExtender&& other) noexcept : Parent(std::move(other)) {}

      public:
      MapExtender& operator=(MapExtender&& cmap) noexcept {
        static_cast<Parent&>(*this) = std::move(static_cast<Parent&>(cmap));
        return *this;
      }

      private:
      MapExtender& operator=(const MapExtender& cmap) noexcept { return operator=< MapExtender >(cmap); }

      template < typename CMap >
      constexpr MapExtender& operator=(const CMap& cmap) noexcept {
        Parent::operator=(cmap);
        return *this;
      }

      public:
      class MapIt: public Item {
        using Parent = Item;

        public:
        using Value = typename Map::Value;

        MapIt() : map(nullptr) {}
        MapIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit MapIt(Map& _map) : map(&_map) { map->notifier()->first(*this); }
        MapIt(const Map& _map, const Item& item) : Parent(item), map(&_map) {}

        constexpr MapIt& operator++() noexcept {
          map->notifier()->next(*this);
          return *this;
        }

        constexpr typename MapTraits< Map >::ConstReturnValue operator*() const noexcept { return (*map)[*this]; }
        constexpr typename MapTraits< Map >::ReturnValue      operator*() noexcept { return (*map)[*this]; }

        constexpr void set(const Value& value) noexcept { map->set(*this, value); }

        protected:
        Map* map;
      };

      class ConstMapIt: public Item {
        using Parent = Item;

        public:
        using Value = typename Map::Value;

        ConstMapIt() : map(nullptr) {}
        ConstMapIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit ConstMapIt(Map& _map) : map(&_map) { map->notifier()->first(*this); }
        ConstMapIt(const Map& _map, const Item& item) : Parent(item), map(_map) {}

        constexpr ConstMapIt& operator++() noexcept {
          map->notifier()->next(*this);
          return *this;
        }

        constexpr typename MapTraits< Map >::ConstReturnValue operator*() const noexcept { return map[*this]; }

        protected:
        const Map* map;
      };

      class ItemIt: public Item {
        using Parent = Item;

        public:
        ItemIt() : map(nullptr) {}
        ItemIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit ItemIt(Map& _map) : map(&_map) { map->notifier()->first(*this); }
        ItemIt(const Map& _map, const Item& item) : Parent(item), map(&_map) {}

        constexpr ItemIt& operator++() noexcept {
          map->notifier()->next(*this);
          return *this;
        }

        protected:
        const Map* map;
      };
    };

    /**
     * @ingroup graphbits
     *
     * @brief Extender for maps which use a subset of the items.
     */
    template < typename _Graph, typename _Map >
    class SubMapExtender: public _Map {
      using Parent = _Map;
      using GraphType = _Graph;

      public:
      using Map = SubMapExtender;
      using Item = typename Parent::Key;

      using Key = typename Parent::Key;
      using Value = typename Parent::Value;
      using Reference = typename Parent::Reference;
      using ConstReference = typename Parent::ConstReference;

      using ReferenceMapTag = typename Parent::ReferenceMapTag;

      class MapIt;
      class ConstMapIt;

      friend class MapIt;
      friend class ConstMapIt;

      public:
      SubMapExtender(const GraphType& _graph) : Parent(_graph), graph(_graph) {}
      SubMapExtender(const GraphType& _graph, const Value& _value) : Parent(_graph, _value), graph(_graph) {}

      private:
      SubMapExtender& operator=(const SubMapExtender& cmap) { return operator=< MapExtender >(cmap); }

      template < typename CMap >
      constexpr SubMapExtender& operator=(const CMap& cmap) noexcept {
        Item it;
        for (graph.first(it); it != INVALID; graph.next(it)) {
          Parent::set(it, cmap[it]);
        }
        return *this;
      }

      public:
      class MapIt: public Item {
        using Parent = Item;

        public:
        using Value = typename Map::Value;

        MapIt() : map(nullptr) {}
        MapIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit MapIt(Map& _map) : map(&_map) { map->graph.first(*this); }
        MapIt(const Map& _map, const Item& item) : Parent(item), map(&_map) {}

        constexpr MapIt& operator++() noexcept {
          map->graph.next(*this);
          return *this;
        }

        constexpr typename MapTraits< Map >::ConstReturnValue operator*() const noexcept { return (*map)[*this]; }
        constexpr typename MapTraits< Map >::ReturnValue      operator*() noexcept { return (*map)[*this]; }
        constexpr void set(const Value& value) noexcept { map->set(*this, value); }

        protected:
        Map* map;
      };

      class ConstMapIt: public Item {
        using Parent = Item;

        public:
        using Value = typename Map::Value;

        ConstMapIt() : map(nullptr) {}
        ConstMapIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit ConstMapIt(Map& _map) : map(&_map) { map->graph.first(*this); }
        ConstMapIt(const Map& _map, const Item& item) : Parent(item), map(&_map) {}

        constexpr ConstMapIt& operator++() noexcept {
          map->graph.next(*this);
          return *this;
        }

        constexpr typename MapTraits< Map >::ConstReturnValue operator*() const { return (*map)[*this]; }

        protected:
        const Map* map;
      };

      class ItemIt: public Item {
        using Parent = Item;

        public:
        ItemIt() : map(nullptr) {}
        ItemIt(Invalid i) : Parent(i), map(nullptr) {}
        explicit ItemIt(Map& _map) : map(&_map) { map->graph.first(*this); }
        ItemIt(const Map& _map, const Item& item) : Parent(item), map(&_map) {}

        constexpr ItemIt& operator++() noexcept {
          map->graph.next(*this);
          return *this;
        }

        protected:
        const Map* map;
      };

      private:
      const GraphType& graph;
    };

  }   // namespace graphs
}   // namespace nt

#endif
