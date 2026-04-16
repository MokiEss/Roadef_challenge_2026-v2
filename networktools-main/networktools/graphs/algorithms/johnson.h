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
 * This file incorporates work from the LEMON graph library (johnson.h).
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
 * @brief Johnson algorithm.
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_JOHNSON_H_
#define _NT_JOHNSON_H_

#include "bellman_ford.h"
#include "../bits/path_dump.h"
#include "dijkstra.h"
#include "../../core/maps.h"
#include "../../core/bin_heap.h"
#include "../matrix_maps.h"

#include <limits>

namespace nt {
  namespace graphs {

    /**
     * @brief Default OperationTraits for the Johnson algorithm class.
     *
     * It defines all computational operations and constants which are
     * used in the Floyd-Warshall algorithm. The default implementation
     * is based on the numeric_limits class. If the numeric type does not
     * have infinity value then the maximum value is used as extremal
     * infinity value.
     */
    template < typename Value, bool has_infinity = std::numeric_limits< Value >::has_infinity >
    struct JohnsonDefaultOperationTraits {
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
    struct JohnsonDefaultOperationTraits< Value, false > {
      static Value zero() { return static_cast< Value >(0); }
      static Value infinity() { return std::numeric_limits< Value >::max(); }
      static Value plus(const Value& left, const Value& right) {
        if (left == infinity() || right == infinity()) return infinity();
        return left + right;
      }
      static bool less(const Value& left, const Value& right) { return left < right; }
    };

    /**
     * @brief Default traits class of Johnson class.
     *
     * Default traits class of Johnson class.
     * @param GR Graph type.
     * @param LEN Type of length map.
     */
    template < class GR, class LEN >
    struct JohnsonDefaultTraits {
      /** The graph type the algorithm runs on. */
      using Digraph = GR;

      /**
       * @brief The type of the map that stores the arc lengths.
       *
       * The type of the map that stores the arc lengths.
       * It must meet the @ref concepts::ReadMap "ReadMap" concept.
       */
      using LengthMap = LEN;

      // The type of the length of the arcs.
      using Value = typename LengthMap::Value;

      /**
       * @brief Operation traits for bellman-ford algorithm.
       *
       * It defines the infinity type on the given Value type
       * and the used operation.
       * @see JohnsonDefaultOperationTraits
       */
      using OperationTraits = JohnsonDefaultOperationTraits< Value >;

      /** The cross reference type used by heap. */

      /**
       * The cross reference type used by heap.
       * Usually it is \c Graph::NodeMap<int>.
       */
      using HeapCrossRef = typename Digraph::template NodeMap< int >;

      /** Instantiates a HeapCrossRef. */

      /**
       * This function instantiates a @ref HeapCrossRef.
       * @param graph is the graph, to which we would like to define the
       * HeapCrossRef.
       */
      static HeapCrossRef* createHeapCrossRef(const Digraph& g) { return new HeapCrossRef(g); }

      /** The heap type used by Dijkstra algorithm. */

      /**
       * The heap type used by Dijkstra algorithm.
       *
       * \sa BinHeap
       * \sa Dijkstra
       */
      using Heap = BinHeap< typename LengthMap::Value, HeapCrossRef, std::less< Value > >;

      /** Instantiates a Heap. */

      /**
       * This function instantiates a @ref Heap.
       * @param crossRef The cross reference for the heap.
       */
      static Heap* createHeap(HeapCrossRef& crossRef) { return new Heap(crossRef); }

      /**
       * @brief The type of the matrix map that stores the last arcs of the
       * shortest paths.
       *
       * The type of the map that stores the last arcs of the shortest paths.
       * It must be a matrix map with \c Graph::Arc value type.
       *
       */
      using PredMap = DynamicMatrixMap< Digraph, typename Digraph::Node, typename Digraph::Arc >;

      /**
       * @brief Instantiates a PredMap.
       *
       * This function instantiates a @ref PredMap.
       * @param graph is the graph, to which we would like to define the PredMap.
       * \todo The graph alone may be insufficient for the initialization
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * @brief The type of the matrix map that stores the dists of the nodes.
       *
       * The type of the matrix map that stores the dists of the nodes.
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
     * @brief %Johnson algorithm class.
     *
     * @ingroup shortest_path
     * This class provides an efficient implementation of \c %Johnson
     * algorithm. The arc lengths are passed to the algorithm using a
     * @ref concepts::ReadMap "ReadMap", so it is easy to change it to any
     * kind of length.
     *
     * The algorithm solves the shortest path problem for each pair
     * of node when the arcs can have negative length but the graph should
     * not contain cycles with negative sum of length. If we can assume
     * that all arc is non-negative in the graph then the dijkstra algorithm
     * should be used from each node.
     *
     * The complexity of this algorithm is \f$ O(n^2\log(n)+n\log(n)e) \f$ or
     * with fibonacci heap \f$ O(n^2\log(n)+ne) \f$. Usually the fibonacci heap
     * implementation is slower than either binary heap implementation or the
     * Floyd-Warshall algorithm.
     *
     * The type of the length is determined by the
     * @ref concepts::ReadMap::Value "Value" of the length map.
     *
     * @param GR The graph type the algorithm runs on. The default value
     * is @ref ListGraph. The value of _Graph is not used directly by
     * Johnson, it is only passed to @ref JohnsonDefaultTraits.
     * @param LEN This read-only ArcMap determines the lengths of the
     * arcs. It is read once for each arc, so the map may involve in
     * relatively time consuming process to compute the arc length if
     * it is necessary. The default map type is @ref
     * concepts::Graph::ArcMap "Graph::ArcMap<int>".  The value
     * of _LengthMap is not used directly by Johnson, it is only passed
     * to @ref JohnsonDefaultTraits.
     * @param TR Traits class to set
     * various data types used by the algorithm.  The default traits
     * class is @ref JohnsonDefaultTraits
     * "JohnsonDefaultTraits<_Graph,_LengthMap>".  See @ref
     * JohnsonDefaultTraits for the documentation of a Johnson traits
     * class.
     *
     * @author Balazs Dezso
     */

#ifdef DOXYGEN
    template < typename GR, typename LEN, typename TR >
#else
    template < typename GR,
               typename LEN = typename GR::template ArcMap< int >,
               typename TR = JohnsonDefaultTraits< GR, LEN > >
#endif
    class Johnson {
      public:
      using Traits = TR;
      /** The type of the underlying graph. */
      using Digraph = typename Traits::Digraph;

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using ArcIt = typename Digraph::ArcIt;
      using OutArcIt = typename Digraph::OutArcIt;

      /** @brief The type of the length of the arcs. */
      using Value = typename Traits::LengthMap::Value;
      /** @brief The type of the map that stores the arc lengths. */
      using LengthMap = typename Traits::LengthMap;
      /**
       * @brief The type of the map that stores the last
       * arcs of the shortest paths. The type of the PredMap
       * is a matrix map for Arcs
       */
      using PredMap = typename Traits::PredMap;
      /**
       * @brief The type of the map that stores the dists of the nodes.
       * The type of the DistMap is a matrix map for Values
       */
      using DistMap = typename Traits::DistMap;
      /** @brief The operation traits. */
      using OperationTraits = typename Traits::OperationTraits;
      /** The cross reference type used for the current heap. */
      using HeapCrossRef = typename Traits::HeapCrossRef;
      /** The heap type used by the dijkstra algorithm. */
      using Heap = typename Traits::Heap;

      private:
      /** Pointer to the underlying graph. */
      const Digraph* _digraph;
      /** Pointer to the length map */
      const LengthMap* _length;
      /** Pointer to the map of predecessors arcs. */
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
      /** Pointer to the heap cross references. */
      HeapCrossRef* _heap_cross_ref;
      /**
       * Indicates if @ref _heap_cross_ref is locally allocated (\c true) or not.
       */
      bool local_heap_cross_ref;
      /** Pointer to the heap. */
      Heap* _heap;
      /**
       * Indicates if @ref _heap is locally allocated (\c true) or not.
       */
      bool local_heap;

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
        if (!_heap_cross_ref) {
          local_heap_cross_ref = true;
          _heap_cross_ref = Traits::createHeapCrossRef(*_digraph);
        }
        if (!_heap) {
          local_heap = true;
          _heap = Traits::createHeap(*_heap_cross_ref);
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
      struct SetPredMap: public Johnson< Digraph, LengthMap, SetPredMapTraits< T > > {
        using Create = Johnson< Digraph, LengthMap, SetPredMapTraits< T > >;
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
      struct SetDistMap: public Johnson< Digraph, LengthMap, SetDistMapTraits< T > > {
        using Create = Johnson< Digraph, LengthMap, SetDistMapTraits< T > >;
      };

      template < class T >
      struct SetOperationTraitsTraits: public Traits {
        using OperationTraits = T;
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * OperationTraits type
       *
       * @ref named-templ-param "Named parameter" for setting
       * OperationTraits type
       */
      template < class T >
      struct DefOperationTraits: public Johnson< Digraph, LengthMap, SetOperationTraitsTraits< T > > {
        using Create = Johnson< Digraph, LengthMap, SetOperationTraitsTraits< T > >;
      };

      template < class H, class CR >
      struct SetHeapTraits: public Traits {
        using HeapCrossRef = CR;
        using Heap = H;
        static HeapCrossRef* createHeapCrossRef(const Digraph&) {
          NT_ASSERT(false, "HeapCrossRef is not initialized");
          return 0;   // ignore warnings
        }
        static Heap* createHeap(HeapCrossRef&) {
          NT_ASSERT(false, "Heap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting heap and
       * cross reference type
       *
       * @ref named-templ-param "Named parameter" for setting heap and cross
       * reference type
       *
       */
      template < class H, class CR = typename Digraph::template NodeMap< int > >
      struct SetHeap: public Johnson< Digraph, LengthMap, SetHeapTraits< H, CR > > {
        using Create = Johnson< Digraph, LengthMap, SetHeapTraits< H, CR > >;
      };

      template < class H, class CR >
      struct SetStandardHeapTraits: public Traits {
        using HeapCrossRef = CR;
        using Heap = H;
        static HeapCrossRef* createHeapCrossRef(const Digraph& G) { return new HeapCrossRef(G); }
        static Heap*         createHeap(HeapCrossRef& R) { return new Heap(R); }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * heap and cross reference type with automatic allocation
       *
       * @ref named-templ-param "Named parameter" for setting heap and cross
       * reference type. It can allocate the heap and the cross reference
       * object if the cross reference's constructor waits for the graph as
       * parameter and the heap's constructor waits for the cross reference.
       */
      template < class H, class CR = typename Digraph::template NodeMap< int > >
      struct SetStandardHeap: public Johnson< Digraph, LengthMap, SetStandardHeapTraits< H, CR > > {
        using Create = Johnson< Digraph, LengthMap, SetStandardHeapTraits< H, CR > >;
      };

      /** @} */

      protected:
      Johnson() {}

      public:
      using Create = Johnson;

      /**
       * @brief Constructor.
       *
       * @param _graph the graph the algorithm will run on.
       * @param _length the length map used by the algorithm.
       */
      Johnson(const Digraph& digraph, const LengthMap& length) :
          _digraph(&digraph), _length(&length), _pred(0), local_pred(false), _dist(0), local_dist(false),
          _heap_cross_ref(0), local_heap_cross_ref(false), _heap(0), local_heap(false) {}

      /** Destructor. */
      ~Johnson() noexcept {
        if (local_pred) delete _pred;
        if (local_dist) delete _dist;
        if (local_heap_cross_ref) delete _heap_cross_ref;
        if (local_heap) delete _heap;
      }

      /**
       * @brief Sets the length map.
       *
       * Sets the length map.
       * @return \c (*this)
       */
      Johnson& lengthMap(const LengthMap& m) {
        _length = &m;
        return *this;
      }

      /**
       * @brief Sets the map storing the predecessor arcs.
       *
       * Sets the map storing the predecessor arcs.
       * If you don't use this function before calling @ref run(),
       * it will allocate one. The destuctor deallocates this
       * automatically allocated map, of course.
       * @return \c (*this)
       */
      Johnson& predMap(PredMap& m) {
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
      Johnson& distMap(DistMap& m) {
        if (local_dist) {
          delete _dist;
          local_dist = false;
        }
        _dist = &m;
        return *this;
      }

      public:
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
      void init() { create_maps(); }

      /**
       * @brief Executes the algorithm with own potential map.
       *
       * This method runs the %Johnson algorithm in order to compute
       * the shortest path to each node pairs. The potential map
       * can be given for this algorithm which usually calculated
       * by the Bellman-Ford algorithm. If the graph does not have
       * negative length arc then this start function can be used
       * with constMap<Node, int>(0) parameter to omit the running time of
       * the Bellman-Ford.
       * The algorithm computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       */
      template < typename PotentialMap >
      void shiftedStart(const PotentialMap& potential) {
        typename Digraph::template ArcMap< Value > shiftlen(*_digraph);
        for (ArcIt it(*_digraph); it != INVALID; ++it) {
          shiftlen[it] = (*_length)[it] + potential[_digraph->source(it)] - potential[_digraph->target(it)];
        }

        typename DijkstraLegacy< Digraph, typename Digraph::template ArcMap< Value > >::
           template SetHeap< Heap, HeapCrossRef >::Create dijkstra(*_digraph, shiftlen);

        dijkstra.heap(*_heap, *_heap_cross_ref);

        for (NodeIt it(*_digraph); it != INVALID; ++it) {
          dijkstra.run(it);
          for (NodeIt jt(*_digraph); jt != INVALID; ++jt) {
            if (dijkstra.reached(jt)) {
              _dist->set(it, jt, dijkstra.dist(jt) + potential[jt] - potential[it]);
              _pred->set(it, jt, dijkstra.predArc(jt));
            } else {
              _dist->set(it, jt, OperationTraits::infinity());
              _pred->set(it, jt, INVALID);
            }
          }
        }
      }

      /**
       * @brief Executes the algorithm.
       *
       * This method runs the %Johnson algorithm in order to compute
       * the shortest path to each node pairs. The algorithm
       * computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       */
      void start() {
        using BellmanFordType = typename BellmanFord< Digraph, LengthMap >::template SetOperationTraits<
           OperationTraits >::template SetPredMap< NullMap< Node, Arc > >::Create;

        BellmanFordType bellmanford(*_digraph, *_length);

        NullMap< Node, Arc > pm;

        bellmanford.predMap(pm);

        bellmanford.init(OperationTraits::zero());
        bellmanford.start();

        shiftedStart(bellmanford.distMap());
      }

      /**
       * @brief Executes the algorithm and checks the negatvie cycles.
       *
       * This method runs the %Johnson algorithm in order to compute
       * the shortest path to each node pairs. If the graph contains
       * negative cycle it gives back false. The algorithm
       * computes
       * - The shortest path tree for each node.
       * - The distance between each node pairs.
       */
      bool checkedStart() {
        using BellmanFordType = typename BellmanFord< Digraph, LengthMap >::template DefOperationTraits<
           OperationTraits >::template DefPredMap< NullMap< Node, Arc > >::Create;

        BellmanFordType bellmanford(*_digraph, *_length);

        NullMap< Node, Arc > pm;

        bellmanford.predMap(pm);

        bellmanford.init(OperationTraits::zero());
        if (!bellmanford.checkedStart()) return false;

        shiftedStart(bellmanford.distMap());
        return true;
      }

      /**
       * @brief Runs %Johnson algorithm.
       *
       * This method runs the %Johnson algorithm from a each node
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
       * The result of the %Johnson algorithm can be obtained using these
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
       * @brief Returns the 'previous arc' of the shortest path tree.
       *
       * For the node \c node it returns the 'previous arc' of the shortest
       * path tree to direction of the node \c root
       * i.e. it returns the last arc of a shortest path from the node \c root
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
       * Returns a reference to the matrix node map of the arcs of the
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
