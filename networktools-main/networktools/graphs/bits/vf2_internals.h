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
 * This file incorporates work from the LEMON graph library (vf2_internals.h).
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
 *   -
 */

/**
 * @ingroup graph_properties
 * @file
 * @brief Mapping types for graph matching algorithms.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_VF2_INTERNALS_H_
#define _NT_VF2_INTERNALS_H_

namespace nt {
  namespace graphs {
    /**
     * @ingroup graph_isomorphism
     * The @ref Vf2 "VF2" algorithm is capable of finding different kind of
     * graph embeddings, this enum specifies their types.
     *
     * See @ref graph_isomorphism for a more detailed description.
     */
    enum class MappingType {
      /** Subgraph isomorphism */
      SUBGRAPH = 0,
      /** Induced subgraph isomorphism */
      INDUCED = 1,
      /**
       * Graph isomorphism
       *
       * If the two graphs have the same number of nodes, then it is
       * equivalent to @ref INDUCED, and if they also have the same
       * number of edges, then it is also equivalent to @ref SUBGRAPH.
       *
       * However, using this setting is faster than the other two options.
       */
      ISOMORPH = 2
    };
  }   // namespace graphs
}   // namespace nt


#endif   // _NT_VF2_INTERNALS_H_