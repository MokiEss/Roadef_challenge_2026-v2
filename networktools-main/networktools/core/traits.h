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
 * This file incorporates work from the LEMON graph library (traits.h).
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
 *   - Replace typedef with using
 */

/**
 * @file
 * @brief Traits for graphs and maps
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TRAITS_H_
#define _NT_TRAITS_H_

#include "concepts/tags.h"
#include "common.h"

namespace nt {

  struct InvalidType {};

  template < typename GR, typename _Item >
  class ItemSetTraits {};

  template < typename GR, typename Enable = void >
  struct NodeNotifierSelector {
    using Type = InvalidType;
  };
  template < typename GR >
  struct NodeNotifierSelector< GR, typename std::enable_if< nt::concepts::NodeNotifierIndicator< GR >, void >::type > {
    using Type = typename GR::NodeNotifier;
  };

  template < typename GR >
  struct ItemSetTraits< GR, typename GR::Node > {
    using Graph = GR;
    using Digraph = GR;

    using Item = typename GR::Node;
    using ItemIt = typename GR::NodeIt;

    using ItemNotifier = typename NodeNotifierSelector< GR >::Type;

    template < typename V >
    class Map: public GR::template NodeMap< V > {
      using Parent = typename GR::template NodeMap< V >;

      public:
      using Type = typename GR::template NodeMap< V >;
      using Value = typename Parent::Value;

      Map(const GR& _digraph) : Parent(_digraph) {}
      Map(const GR& _digraph, const Value& _value) : Parent(_digraph, _value) {}
    };
  };

  template < typename GR, typename Enable = void >
  struct ArcNotifierSelector {
    using Type = InvalidType;
  };
  template < typename GR >
  struct ArcNotifierSelector< GR, typename std::enable_if< nt::concepts::ArcNotifierIndicator< GR >, void >::type > {
    using Type = typename GR::ArcNotifier;
  };

  template < typename GR >
  struct ItemSetTraits< GR, typename GR::Arc > {
    using Graph = GR;
    using Digraph = GR;

    using Item = typename GR::Arc;
    using ItemIt = typename GR::ArcIt;

    using ItemNotifier = typename ArcNotifierSelector< GR >::Type;

    template < typename V >
    class Map: public GR::template ArcMap< V > {
      using Parent = typename GR::template ArcMap< V >;

      public:
      using Type = typename GR::template ArcMap< V >;
      using Value = typename Parent::Value;

      Map(const GR& _digraph) : Parent(_digraph) {}
      Map(const GR& _digraph, const Value& _value) : Parent(_digraph, _value) {}
    };
  };

  template < typename GR, typename Enable = void >
  struct EdgeNotifierSelector {
    using Type = InvalidType;
  };
  template < typename GR >
  struct EdgeNotifierSelector< GR, typename std::enable_if< nt::concepts::EdgeNotifierIndicator< GR >, void >::type > {
    using Type = typename GR::EdgeNotifier;
  };

  template < typename GR >
  struct ItemSetTraits< GR, typename GR::Edge > {
    using Graph = GR;
    using Digraph = GR;

    using Item = typename GR::Edge;
    using ItemIt = typename GR::EdgeIt;

    using ItemNotifier = typename EdgeNotifierSelector< GR >::Type;

    template < typename V >
    class Map: public GR::template EdgeMap< V > {
      using Parent = typename GR::template EdgeMap< V >;

      public:
      using Type = typename GR::template EdgeMap< V >;
      using Value = typename Parent::Value;

      Map(const GR& _digraph) : Parent(_digraph) {}
      Map(const GR& _digraph, const Value& _value) : Parent(_digraph, _value) {}
    };
  };

  template < typename GR, typename Enable = void >
  struct RedNodeNotifierSelector {
    using Type = InvalidType;
  };
  template < typename GR >
  struct RedNodeNotifierSelector<
     GR,
     typename std::enable_if< nt::concepts::RedNodeNotifierIndicator< GR >, void >::type > {
    using Type = typename GR::RedNodeNotifier;
  };

  template < typename GR >
  struct ItemSetTraits< GR, typename GR::RedNode > {
    using BpGraph = GR;
    using Graph = GR;
    using Digraph = GR;

    using Item = typename GR::RedNode;
    using ItemIt = typename GR::RedNodeIt;

    using ItemNotifier = typename RedNodeNotifierSelector< GR >::Type;

    template < typename V >
    class Map: public GR::template RedNodeMap< V > {
      using Parent = typename GR::template RedNodeMap< V >;

      public:
      using Type = typename GR::template RedNodeMap< V >;
      using Value = typename Parent::Value;

      Map(const GR& _bpgraph) : Parent(_bpgraph) {}
      Map(const GR& _bpgraph, const Value& _value) : Parent(_bpgraph, _value) {}
    };
  };

  template < typename GR, typename Enable = void >
  struct BlueNodeNotifierSelector {
    using Type = InvalidType;
  };
  template < typename GR >
  struct BlueNodeNotifierSelector<
     GR,
     typename std::enable_if< nt::concepts::BlueNodeNotifierIndicator< GR >, void >::type > {
    using Type = typename GR::BlueNodeNotifier;
  };

  template < typename GR >
  struct ItemSetTraits< GR, typename GR::BlueNode > {
    using BpGraph = GR;
    using Graph = GR;
    using Digraph = GR;

    using Item = typename GR::BlueNode;
    using ItemIt = typename GR::BlueNodeIt;

    using ItemNotifier = typename BlueNodeNotifierSelector< GR >::Type;

    template < typename V >
    class Map: public GR::template BlueNodeMap< V > {
      using Parent = typename GR::template BlueNodeMap< V >;

      public:
      using Type = typename GR::template BlueNodeMap< V >;
      using Value = typename Parent::Value;

      Map(const GR& _bpgraph) : Parent(_bpgraph) {}
      Map(const GR& _bpgraph, const Value& _value) : Parent(_bpgraph, _value) {}
    };
  };

  template < typename Map, typename Enable = void >
  struct MapTraits {
    using ReferenceMapTag = nt::False;

    using Key = typename Map::Key;
    using Value = typename Map::Value;

    using ConstReturnValue = Value;
    using ReturnValue = Value;
  };

  template < typename Map >
  struct MapTraits< Map, typename std::enable_if< nt::concepts::ReferenceMapIndicator< Map >, void >::type > {
    using ReferenceMapTag = nt::True;

    using Key = typename Map::Key;
    using Value = typename Map::Value;

    using ConstReturnValue = typename Map::ConstReference;
    using ReturnValue = typename Map::Reference;

    using ConstReference = typename Map::ConstReference;
    using Reference = typename Map::Reference;
  };

  template < typename MatrixMap, typename Enable = void >
  struct MatrixMapTraits {
    using FirstKey = typename MatrixMap::FirstKey;
    using SecondKey = typename MatrixMap::SecondKey;
    using Value = typename MatrixMap::Value;

    using ConstReturnValue = Value;
    using ReturnValue = Value;
  };

  template < typename MatrixMap >
  struct MatrixMapTraits< MatrixMap,
                          typename std::enable_if< nt::concepts::ReferenceMapIndicator< MatrixMap >, void >::type > {
    using FirstKey = typename MatrixMap::FirstKey;
    using SecondKey = typename MatrixMap::SecondKey;
    using Value = typename MatrixMap::Value;

    using ConstReturnValue = typename MatrixMap::ConstReference;
    using ReturnValue = typename MatrixMap::Reference;

    using ConstReference = typename MatrixMap::ConstReference;
    using Reference = typename MatrixMap::Reference;
  };

}   // namespace nt

#endif
