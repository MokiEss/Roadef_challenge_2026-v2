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
 * This file incorporates work from the LEMON graph library (path_dump.h).
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
 *   - Add constexpr, noexcept whenever possible
 */

/**
 * @file
 * @brief
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_BITS_PATH_DUMP_H_
#define _NT_BITS_PATH_DUMP_H_

namespace nt {
  namespace graphs {

    template < typename _Digraph, typename _PredMap >
    class PredMapPath {
      public:
      using RevPathTag = True;

      using Digraph = _Digraph;
      using Arc = typename Digraph::Arc;
      using PredMap = _PredMap;

      PredMapPath(const Digraph& _digraph, const PredMap& _predMap, typename Digraph::Node _target) :
          digraph(_digraph), predMap(_predMap), target(_target) {}

      constexpr int length() const noexcept {
        int                    len = 0;
        typename Digraph::Node node = target;
        typename Digraph::Arc  arc;
        while ((arc = predMap[node]) != INVALID) {
          node = digraph.source(arc);
          ++len;
        }
        return len;
      }

      constexpr bool empty() const noexcept { return predMap[target] == INVALID; }

      class RevArcIt {
        public:
        RevArcIt() {}
        RevArcIt(Invalid) : path(0), current(INVALID) {}
        RevArcIt(const PredMapPath& _path) : path(&_path), current(_path.target) {
          if (path->predMap[current] == INVALID) current = INVALID;
        }

        constexpr operator const typename Digraph::Arc() const noexcept { return path->predMap[current]; }

        constexpr RevArcIt& operator++() noexcept {
          current = path->digraph.source(path->predMap[current]);
          if (path->predMap[current] == INVALID) current = INVALID;
          return *this;
        }

        constexpr bool operator==(const RevArcIt& e) const noexcept { return current == e.current; }

        constexpr bool operator!=(const RevArcIt& e) const noexcept { return current != e.current; }

        constexpr bool operator<(const RevArcIt& e) const noexcept { return current < e.current; }

        private:
        const PredMapPath*     path;
        typename Digraph::Node current;
      };

      private:
      const Digraph&         digraph;
      const PredMap&         predMap;
      typename Digraph::Node target;
    };

    template < typename _Digraph, typename _PredMatrixMap >
    class PredMatrixMapPath {
      public:
      using RevPathTag = True;

      using Digraph = _Digraph;
      using Arc = typename Digraph::Arc;
      using PredMatrixMap = _PredMatrixMap;

      PredMatrixMapPath(const Digraph&         _digraph,
                        const PredMatrixMap&   _predMatrixMap,
                        typename Digraph::Node _source,
                        typename Digraph::Node _target) :
          digraph(_digraph),
          predMatrixMap(_predMatrixMap), source(_source), target(_target) {}

      constexpr int length() const noexcept {
        int                    len = 0;
        typename Digraph::Node node = target;
        typename Digraph::Arc  arc;
        while ((arc = predMatrixMap(source, node)) != INVALID) {
          node = digraph.source(arc);
          ++len;
        }
        return len;
      }

      constexpr bool empty() const noexcept { return predMatrixMap(source, target) == INVALID; }

      class RevArcIt {
        public:
        RevArcIt() {}
        RevArcIt(Invalid) : path(0), current(INVALID) {}
        RevArcIt(const PredMatrixMapPath& _path) : path(&_path), current(_path.target) {
          if (path->predMatrixMap(path->source, current) == INVALID) current = INVALID;
        }

        constexpr operator const typename Digraph::Arc() const noexcept {
          return path->predMatrixMap(path->source, current);
        }

        constexpr RevArcIt& operator++() noexcept {
          current = path->digraph.source(path->predMatrixMap(path->source, current));
          if (path->predMatrixMap(path->source, current) == INVALID) current = INVALID;
          return *this;
        }

        constexpr bool operator==(const RevArcIt& e) const noexcept { return current == e.current; }

        constexpr bool operator!=(const RevArcIt& e) const noexcept { return current != e.current; }

        constexpr bool operator<(const RevArcIt& e) const noexcept { return current < e.current; }

        private:
        const PredMatrixMapPath* path;
        typename Digraph::Node   current;
      };

      private:
      const Digraph&         digraph;
      const PredMatrixMap&   predMatrixMap;
      typename Digraph::Node source;
      typename Digraph::Node target;
    };

  }   // namespace graphs
}   // namespace nt

#endif
