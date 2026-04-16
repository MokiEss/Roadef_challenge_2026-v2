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
 * This file incorporates work from the LEMON graph library (floyd_warshall.h).
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
 * @ingroup shortest_path
 * @file
 * @brief FloydWarshall algorithm.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_FLOYD_WARSHALL_H_
#define _NT_FLOYD_WARSHALL_H_

#include "../bits/path_dump.h"
#include "../../core/maps.h"
#include "../matrix_maps.h"

#include <limits>

namespace nt {
  namespace graphs {

    /**
     * @brief Default OperationTraits for the FloydWarshall algorithm class.
     *
     * It defines all computational operations and constants which are
     * used in the Floyd-Warshall algorithm. The default implementation
     * is based on the numeric_limits class. If the numeric type does not
     * have infinity value then the maximum value is used as extremal
     * infinity value.
     */
    template < typename Value, bool has_infinity = std::numeric_limits< Value >::has_infinity >
    struct FloydWarshallDefaultOperationTraits {
      /** @brief Gives back the zero value of the type. */
      static Value zero() { return static_cast< Value >(0); }
      /** @brief Gives back the positive infinity value of the type. */
      static Value infinity() { return std::numeric_limits< Value >::infinity(); }
      /** @brief Gives back the sum of the given two elements. */
      static Value plus(const Value& left, const Value& right) { return left + right; }
      /**
       * @brief Gives back true only if the first value less than the second.
       */
      static bool less(const Value& left, const Value& right) { return left < right; }
    };

    template < typename Value >
    struct FloydWarshallDefaultOperationTraits< Value, false > {
      static Value zero() { return static_cast< Value >(0); }
      static Value infinity() { return std::numeric_limits< Value >::max(); }
      static Value plus(const Value& left, const Value& right) {
        if (left == infinity() || right == infinity()) return infinity();
        return left + right;
      }
      static bool less(const Value& left, const Value& right) { return left < right; }
    };

    /**
     * @brief Default traits class of FloydWarshall class.
     *
     * Default traits class of FloydWarshall class.
     * @param GR Graph type.
     * @param LEN Type of length map.
     */
    template < typename GR, typename LEN >
    struct FloydWarshallDefaultTraits {
      /** The digraph type the algorithm runs on. */
      using Digraph = GR;

      /**
       * @brief The type of the map that stores the arcs lengths.
       *
       * The type of the map that stores the arcs lengths.
       * It must meet the @ref concepts::ReadMap "ReadMap" concept.
       */
      using LengthMap = LEN;

      // The type of the length of the arcss.
      using Value = typename LengthMap::Value;

      /**
       * @brief Operation traits for floyd-warshall algorithm.
       *
       * It defines the infinity type on the given Value type
       * and the used operation.
       * @see FloydWarshallDefaultOperationTraits
       */
      using OperationTraits = FloydWarshallDefaultOperationTraits< Value >;

      /**
       * @brief The type of the matrix map that stores the last arcss of the
       * shortest paths.
       *
       * The type of the map that stores the last arcss of the shortest paths.
       * It must be a matrix map with \c Digraph::Arc value type.
       *
       */
      using PredMap = DynamicMatrixMap< Digraph, typename Digraph::Node, typename Digraph::Arc >;

      /**
       * @brief Instantiates a PredMap.
       *
       * This function instantiates a @ref PredMap.
       * @param graph is the graph,
       * to which we would like to define the PredMap.
       * \todo The graph alone may be insufficient for the initialization
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * @brief The type of the map that stores the dists of the nodes.
       *
       * The type of the map that stores the dists of the nodes.
       * It must meet the @ref concepts::WriteMatrixMap "WriteMatrixMap" concept.
       *
       */
      using DistMap = DynamicMatrixMap< Digraph, typename Digraph::Node, Value >;

      /**
       * @brief Instantiates a DistMap.
       *
       * This function instantiates a @ref DistMap.
       * @param graph is the graph, to which we would like to define the
       * @ref DistMap
       */
      static DistMap* createDistMap(const Digraph& g) { return new DistMap(g); }
    };

/**
 * @brief %FloydWarshall algorithm class.
 *
 * @ingroup shortest_path
 * This class provides an efficient implementation of \c Floyd-Warshall
 * algorithm. The arcs lengths are passed to the algorithm using a
 * @ref concepts::ReadMap "ReadMap", so it is easy to change it to any
 * kind of length.
 *
 * The algorithm solves the shortest path problem for each pair
 * of node when the arcss can have negative length but the graph should
 * not contain cycles with negative sum of length. If we can assume
 * that all arcs is non-negative in the graph then the dijkstra algorithm
 * should be used from each node rather and if the graph is sparse and
 * there are negative circles then the johnson algorithm.
 *
 * The complexity of this algorithm is \f$ O(n^3+e) \f$.
 *
 * The type of the length is determined by the
 * @ref concepts::ReadMap::Value "Value" of the length map.
 *
 * @param GR The digraph type the algorithm runs on. The default value
 * is @ref ListGraph. The value of _Graph is not used directly by
 * FloydWarshall, it is only passed to @ref FloydWarshallDefaultTraits.
 * @param LEN This read-only ArcMap determines the lengths of the
 * arcss. It is read once for each arcs, so the map may involve in
 * relatively time consuming process to compute the arcs length if
 * it is necessary. The default map type is @ref
 * concepts::Graph::ArcMap "Graph::ArcMap<int>".  The value
 * of _LengthMap is not used directly by FloydWarshall, it is only passed
 * to @ref FloydWarshallDefaultTraits.
 * @param TR Traits class to set
 * various data types used by the algorithm.  The default traits
 * class is @ref FloydWarshallDefaultTraits
 * "FloydWarshallDefaultTraits<_Graph,_LengthMap>".  See @ref
 * FloydWarshallDefaultTraits for the documentation of a FloydWarshall
 * traits class.
 *
 * @author Balazs Dezso
 * \todo A function type interface would be nice.
 * \todo Implement \c nextNode() and \c nextArc()
 */
#ifdef DOXYGEN
    template < typename GR, typename LEN, typename TR >
#else
    template < typename GR,
               typename LEN = typename GR::template ArcMap< int >,
               typename TR = FloydWarshallDefaultTraits< GR, LEN > >
#endif
    class FloydWarshall {
      public:
      using Traits = TR;
      /** The type of the underlying graph. */
      using Digraph = typename Traits::Digraph;

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using ArcIt = typename Digraph::ArcIt;

      /** @brief The type of the length of the arcss. */
      using Value = typename Traits::LengthMap::Value;
      /** @brief The type of the map that stores the arcs lengths. */
      using LengthMap = typename Traits::LengthMap;
      /**
       * @brief The type of the map that stores the last
       * arcss of the shortest paths. The type of the PredMap
       * is a matrix map for Arcs
       */
      using PredMap = typename Traits::PredMap;
      /**
       * @brief The type of the map that stores the dists of the nodes.
       * The type of the DistMap is a matrix map for Values
       *
       * \todo It should rather be
       * called \c DistMatrix
       */
      using DistMap = typename Traits::DistMap;
      /** @brief The operation traits. */
      using OperationTraits = typename Traits::OperationTraits;

      private:
      /** Pointer to the underlying graph. */
      const Digraph* _digraph;
      /** Pointer to the length map */
      const LengthMap* _length;
      /** Pointer to the map of predecessors arcss. */
      PredMap* _pred;
      /**
       * Indicates if @ref _pred is locally allocated (\c true) or not.
       */
      bool local_pred;
      /** Pointer to the map of distances. */
      DistMap* _dist;
      /**
       * Indicates if @ref _dist is locally allocated (\c true) or not.
       */
      bool local_dist;

      /** Creates the maps if necessary. */
      void create_maps() {
        if (!_pred) {
          local_pred = true;
          _pred = Traits::createPredMap(*_digraph);
        }
        if (!_dist) {
          local_dist = true;
          _dist = Traits::createDistMap(*_digraph);
        }
      }

      public:
      /** \name Named template parameters */

      /** @{ */

      template < class T >
      struct SetPredMapTraits: public Traits {
        using PredMap = T;
        static PredMap* createPredMap(const Digraph&) {
          NT_ASSERT(false, "PredMap is not initialized");
          return 0;   // ignore warnings
        }
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting PredMap
       * type
       * @ref named-templ-param "Named parameter" for setting PredMap type
       *
       */
      template < class T >
      struct SetPredMap: public FloydWarshall< Digraph, LengthMap, SetPredMapTraits< T > > {
        using Create = FloydWarshall< Digraph, LengthMap, SetPredMapTraits< T > >;
      };

      template < class T >
      struct SetDistMapTraits: public Traits {
        using DistMap = T;
        static DistMap* createDistMap(const Digraph&) {
          NT_ASSERT(false, "DistMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting DistMap
       * type
       *
       * @ref named-templ-param "Named parameter" for setting DistMap type
       *
       */
      template < class T >
      struct SetDistMap: public FloydWarshall< Digraph, LengthMap, SetDistMapTraits< T > > {
        using Create = FloydWarshall< Digraph, LengthMap, SetDistMapTraits< T > >;
      };

      template < class T >
      struct SetOperationTraitsTraits: public Traits {
        using OperationTraits = T;
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * OperationTraits type
       *
       * @ref named-templ-param "Named parameter" for setting PredMap type
       */
      template < class T >
      struct SetOperationTraits: public FloydWarshall< Digraph, LengthMap, SetOperationTraitsTraits< T > > {
        using Create = FloydWarshall< Digraph, LengthMap, SetOperationTraitsTraits< T > >;
      };

      /** @} */

      protected:
      FloydWarshall() {}

      public:
      using Create = FloydWarshall;

      /**
       * @brief Constructor.
       *
       * @param _graph the graph the algorithm will run on.
       * @param _length the length map used by the algorithm.
       */
      FloydWarshall(const Digraph& digraph, const LengthMap& length) :
          _digraph(&digraph), _length(&length), _pred(0), local_pred(false), _dist(0), local_dist(false) {}

      /** Destructor. */
      ~FloydWarshall() noexcept {
        if (local_pred) delete _pred;
        if (local_dist) delete _dist;
      }

      /**
       * @brief Sets the length map.
       *
       * Sets the length map.
       * @return \c (*this)
       */
      FloydWarshall& lengthMap(const LengthMap& m) {
        _length = &m;
        return *this;
      }

      /**
       * @brief Sets the map storing the predecessor arcss.
       *
       * Sets the map storing the predecessor arcss.
       * If you don't use this function before calling @ref run(),
       * it will allocate one. The destuctor deallocates this
       * automatically allocated map, of course.
       * @return \c (*this)
       */
      FloydWarshall& predMap(PredMap& m) {
        if (local_pred) {
          delete _pred;
          local_pred = false;
        }
        _pred = &m;
        return *this;
      }

      /**
       * @brief Sets the map storing the distances calculated by the algorithm.
       *
       * Sets the map storing the distances calculated by the algorithm.
       * If you don't use this function before calling @ref run(),
       * it will allocate one. The destuctor deallocates this
       * automatically allocated map, of course.
       * @return \c (*this)
       */
      FloydWarshall& distMap(DistMap& m) {
        if (local_dist) {
          delete _dist;
          local_dist = false;
        }
        _dist = &m;
        return *this;
      }

      /**
       * \name Execution control
       * The simplest way to execute the algorithm is to use
       * one of the member functions called \c run(...).
       * \n
       * If you need more control on the execution,
       * Finally @ref start() will perform the actual path
       * computation.
       */

      /** @{ */

      /**
       * @brief Initializes the internal data structures.
       *
       * Initializes the internal data structures.
       */
      void init() {
        create_maps();
        for (NodeIt it(*_digraph); it != INVALID; ++it) {
          for (NodeIt jt(*_digraph); jt != INVALID; ++jt) {
            _pred->set(it, jt, INVALID);
            _dist->set(it, jt, OperationTraits::infinity());
          }
          _dist->set(it, it, OperationTraits::zero());
        }
        for (ArcIt it(*_digraph); it != INVALID; ++it) {
          Node source = _digraph->source(it);
          Node target = _digraph->target(it);
          if (OperationTraits::less((*_length)[it], (*_dist)(source, target))) {
            _dist->set(source, target, (*_length)[it]);
            _pred->set(source, target, it);
          }
        }
      }

      /**
       * @brief Executes the algorithm.
       *
       * This method runs the %FloydWarshall algorithm in order to compute
       * the shortest path to each node pairs. The algorithm
       * computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       */
      void start() {
        for (NodeIt kt(*_digraph); kt != INVALID; ++kt) {
          for (NodeIt it(*_digraph); it != INVALID; ++it) {
            for (NodeIt jt(*_digraph); jt != INVALID; ++jt) {
              Value relaxed = OperationTraits::plus((*_dist)(it, kt), (*_dist)(kt, jt));
              if (OperationTraits::less(relaxed, (*_dist)(it, jt))) {
                _dist->set(it, jt, relaxed);
                _pred->set(it, jt, (*_pred)(kt, jt));
              }
            }
          }
        }
      }

      /**
       * @brief Executes the algorithm and checks the negative cycles.
       *
       * This method runs the %FloydWarshall algorithm in order to compute
       * the shortest path to each node pairs. If there is a negative cycle
       * in the graph it gives back false.
       * The algorithm computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       */
      bool checkedStart() {
        start();
        for (NodeIt it(*_digraph); it != INVALID; ++it) {
          if (OperationTraits::less((*_dist)(it, it), OperationTraits::zero())) { return false; }
        }
        return true;
      }

      /**
       * @brief Runs %FloydWarshall algorithm.
       *
       * This method runs the %FloydWarshall algorithm from a each node
       * in order to compute the shortest path to each node pairs.
       * The algorithm computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       *
       * @note d.run(s) is just a shortcut of the following code.
       * @code
       * d.init();
       * d.start();
       * @endcode
       */
      void run() {
        init();
        start();
      }

      /** @} */

      /**
       * \name Query Functions
       * The result of the %FloydWarshall algorithm can be obtained using these
       * functions.\n
       * Before the use of these functions,
       * either run() or start() must be called.
       */

      /** @{ */

      using Path = PredMatrixMapPath< Digraph, PredMap >;

      /** Gives back the shortest path. */

      /**
       * Gives back the shortest path.
       * @pre The \c t should be reachable from the \c t.
       */
      Path path(Node s, Node t) { return Path(*_digraph, *_pred, s, t); }

      /**
       * @brief The distance between two nodes.
       *
       * Returns the distance between two nodes.
       * @pre @ref run() must be called before using this function.
       * @warning If node \c v in unreachable from the root the return value
       * of this funcion is undefined.
       */
      Value dist(Node source, Node target) const { return (*_dist)(source, target); }

      /**
       * @brief Returns the 'previous arcs' of the shortest path tree.
       *
       * For the node \c node it returns the 'previous arcs' of the shortest
       * path tree to direction of the node \c root
       * i.e. it returns the last arcs of a shortest path from the node \c root
       * to \c node. It is @ref INVALID if \c node is unreachable from the root
       * or if \c node=root. The shortest path tree used here is equal to the
       * shortest path tree used in @ref predNode().
       * @pre @ref run() must be called before using this function.
       */
      Arc predArc(Node root, Node node) const { return (*_pred)(root, node); }

      /**
       * @brief Returns the 'previous node' of the shortest path tree.
       *
       * For a node \c node it returns the 'previous node' of the shortest path
       * tree to direction of the node \c root, i.e. it returns the last but
       * one node from a shortest path from the \c root to \c node. It is
       * INVALID if \c node is unreachable from the root or if \c node=root.
       * The shortest path tree used here is equal to the
       * shortest path tree used in @ref predArc().
       * @pre @ref run() must be called before using this function.
       */
      Node predNode(Node root, Node node) const {
        return (*_pred)(root, node) == INVALID ? INVALID : _digraph->source((*_pred)(root, node));
      }

      /**
       * @brief Returns a reference to the matrix node map of distances.
       *
       * Returns a reference to the matrix node map of distances.
       *
       * @pre @ref run() must be called before using this function.
       */
      const DistMap& distMap() const { return *_dist; }

      /**
       * @brief Returns a reference to the shortest path tree map.
       *
       * Returns a reference to the matrix node map of the arcss of the
       * shortest path tree.
       * @pre @ref run() must be called before using this function.
       */
      const PredMap& predMap() const { return *_pred; }

      /**
       * @brief Checks if a node is reachable from the root.
       *
       * Returns \c true if \c v is reachable from the root.
       * @pre @ref run() must be called before using this function.
       *
       */
      bool connected(Node source, Node target) { return (*_dist)(source, target) != OperationTraits::infinity(); }

      /** @} */
    };

  }   // namespace graphs
}   // namespace nt
#endif
