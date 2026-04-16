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
 * This file incorporates work from the LEMON graph library (default_map.h).
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
 *   - Updated include paths to networktools structure
 *   - Converted typedef declarations to C++11 using declarations
 *   - Updated header guard macros
 *   - Added constexpr qualifiers for compile-time optimization
 *   - Added noexcept specifiers for better exception safety
 *   - Added constructor for const char* (with C++20 requires clause for String type)
 *   - Added move constructor
 *   - Added setBitsToZero() and setBitsToOne() methods
 *   - Used C++20 requires clause for compile-time type checking
 */

/**
 * @ingroup graphbits
 * @file
 * @brief Graph maps that construct and destruct their elements dynamically.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DEFAULT_MAP_H_
#define _NT_DEFAULT_MAP_H_

#include "vector_map.h"

namespace nt {
  namespace graphs {

    template < typename _Graph, typename _Item, typename _Value >
    struct DefaultMapSelector {
      using Map = VectorMap< _Graph, _Item, _Value >;
    };

    /**
     * @brief DefaultMap class
     */
    template < typename _Graph, typename _Item, typename _Value >
    class DefaultMap: public DefaultMapSelector< _Graph, _Item, _Value >::Map {
      using Parent = typename DefaultMapSelector< _Graph, _Item, _Value >::Map;

      public:
      using Map = DefaultMap< _Graph, _Item, _Value >;

      using GraphType = typename Parent::GraphType;
      using Value = typename Parent::Value;

      explicit DefaultMap(const GraphType& graph) noexcept : Parent(graph) {}
      DefaultMap(const GraphType& graph, const Value& value) noexcept : Parent(graph, value) {}
      DefaultMap(const GraphType& graph, const DefaultMap& map) noexcept : Parent(graph, map) {}
      DefaultMap(const GraphType& graph, const char* str) noexcept requires(std::is_same< Value, nt::String >::value) :
          Parent(graph, str) {}

      constexpr DefaultMap& operator=(const DefaultMap& cmap) noexcept { return operator=< DefaultMap >(cmap); }

      template < typename CMap >
      constexpr DefaultMap& operator=(const CMap& cmap) noexcept {
        Parent::operator=(cmap);
        return *this;
      }

      DefaultMap(DefaultMap&& other) noexcept : Parent(std::move(other)) {}

      constexpr DefaultMap& operator=(DefaultMap&& cmap) noexcept {
        static_cast<Parent&>(*this) = std::move(static_cast<Parent&>(cmap));
        return *this;
      }

      constexpr void setBitsToZero() noexcept { Parent::setBitsToZero(); }
      constexpr void setBitsToOne() noexcept { Parent::setBitsToOne(); }
    };

  }   // namespace graphs
}   // namespace nt

#endif
