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

#ifndef _NT_DEMAND_GRAPH_H_
#define _NT_DEMAND_GRAPH_H_

#include "tools.h"
#include "edge_graph.h"

namespace nt {
  namespace graphs {

    /**
     * @class DemandGraph
     *
     * @brief A demand graph DG associated to a digraph G is a directed graph that uses the same
     * node set as G and posseses its own arc set. Each arc uv in DG corresponds to a demand in G
     * with source u and destination v.
     *
     * @tparam GR The type of the digraph to which the demand graph is associated.
     */
    template < class GR >
    struct DemandGraph: SmartArcGraph< GR > {
      using Digraph = GR;
      using Parent = SmartArcGraph< Digraph >;
      TEMPLATE_DIGRAPH_TYPEDEFS(Parent);

      DemandGraph(const Digraph& g) : Parent(g) {}
      DemandGraph(DemandGraph&& other) : Parent(std::move(other)) {}

      /**
       * @brief Return all vertices that are targets of at least one demand
       *
       * @return
       */
      constexpr TrivialDynamicArray< Node > targets() const noexcept {
        TrivialDynamicArray< Node > targets;

        for (NodeIt n(*this); n != INVALID; ++n)
          if (InArcIt(*this, n) != INVALID) targets.add(n);

        return targets;
      }

      /**
       * @brief Return all vertices that are sources of at least one demand
       *
       * @return TrivialDynamicArray< Node >
       */
      constexpr TrivialDynamicArray< Node > sources() const noexcept {
        TrivialDynamicArray< Node > sources;

        for (NodeIt n(*this); n != INVALID; ++n)
          if (OutArcIt(*this, n) != INVALID) sources.add(n);

        return sources;
      }
    };

  }   // namespace graphs
}   // namespace nt


#endif