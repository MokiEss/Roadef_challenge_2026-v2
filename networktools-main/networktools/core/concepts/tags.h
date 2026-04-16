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
 * @brief tags.h
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_CONCEPTS_H_
#define _NT_CONCEPTS_H_

#include "../common.h"

namespace nt {
  namespace concepts {

    template < typename GR >
    concept NodeNumTagIndicator = requires(GR g) {
      { g.nodeNum() } -> nt::same_as< int >;
    };

    template < typename GR >
    concept ArcNumTagIndicator = requires(GR g) {
      { g.arcNum() } -> nt::same_as< int >;
    };

    template < typename GR >
    concept EdgeNumTagIndicator = requires(GR g) {
      { g.edgeNum() } -> nt::same_as< int >;
    };

    template < typename GR >
    concept FindArcTagIndicator = requires(GR g) {
      {
        g.findArc(
           nt::declval< typename GR::Node& >(), nt::declval< typename GR::Node& >(), nt::declval< typename GR::Arc& >())
        } -> nt::same_as< int >;
    };

    template < typename GR >
    concept FindEdgeTagIndicator = requires(GR g) {
      {
        g.findEdge(nt::declval< typename GR::Node& >(),
                   nt::declval< typename GR::Node& >(),
                   nt::declval< typename GR::Edge& >())
        } -> nt::same_as< int >;
    };

    template < typename GR >
    concept UndirectedTagIndicator = requires {
      typename GR::Edge;
    };

    template < typename Map >
    concept ReferenceMapIndicator = requires {
      typename Map::ConstReference;
      typename Map::Reference;
    };

    template < typename GR >
    concept NodeNotifierIndicator = requires {
      typename GR::NodeNotifier;
    };

    template < typename GR >
    concept RedNodeNotifierIndicator = requires {
      typename GR::RedNodeNotifier;
    };

    template < typename GR >
    concept BlueNodeNotifierIndicator = requires {
      typename GR::BlueNodeNotifier;
    };

    template < typename GR >
    concept ArcNotifierIndicator = requires {
      typename GR::ArcNotifier;
    };

    template < typename GR >
    concept EdgeNotifierIndicator = requires {
      typename GR::EdgeNotifier;
    };

    template < typename GR >
    concept BuildTagIndicator = requires {
      typename GR::BuildTag;
    };

    template < typename Path >
    concept RevPathTagIndicator = requires {
      typename Path::RevPathTag;
    };

  }   // namespace concepts
}   // namespace nt
#endif
