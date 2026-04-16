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
 * This file incorporates work from the LEMON graph library (dijkstra.h).
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
 *   - Renamed lemon::Dijkstra to nt::graphs::DijkstraLegacy for clarity
 *   - Added modern flag-based implementation (DijkstraBase, Dijkstra, Dijkstra16Bits)
 *     with compile-time configuration, SIMD optimization support, and multiple
 *     shortest path enumeration
 *   - Changed namespace from 'lemon' to 'nt'
 *   - Updated include paths to networktools structure
 *   - Changed LEMON_ASSERT to NT_ASSERT
 *   - Adapted LEMON concept checking to networktools
 *   - Converted typedef declarations to C++11 using declarations
 *   - Adapted LEMON INVALID sentinel value handling
 *   - Updated header guard macros
 */

/**
 * @file
 * @brief Dijkstra's algorithm
 *
 * @author LEMON contributors (see original source) and Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DIJKSTRA_H_
#define _NT_DIJKSTRA_H_

#include "../../core/quick_heap.h"
#include "../../core/bin_heap.h"
#include "../../core/stack.h"
#include "../../core/queue.h"
#include "../../core/json.h"
#include "../../core/tolerance.h"
#include "../bits/path_dump.h"
#include "../tools.h"
#include "../edge_graph.h"
#include "../path.h"

namespace nt {
  namespace graphs {

    enum DijkstraFlags : int {
      NONE = 0,
      LOCAL_HEAP = 1 << 0,
      PRED_MAP = 1 << 1,
      SINGLE_PATH = 1 << 3,   // Has no effect if PRED_MAP is not set
      BACKWARD = 1 << 4,

      // Common combinations
      STANDARD = LOCAL_HEAP | PRED_MAP,
      SINGLE_PRED = LOCAL_HEAP | PRED_MAP | SINGLE_PATH,
      MINIMAL = LOCAL_HEAP
    };

    namespace details {
      template < typename HEAP >
      struct DijkstraLocalHeap {
        using Heap = HEAP;
        Heap _heap;

        constexpr Heap* getLocalHeap() noexcept { return &_heap; }
      };

      template < typename HEAP >
      struct NoDijkstraLocalHeap {
        using Heap = HEAP;
        constexpr Heap* getLocalHeap() const noexcept { return nullptr; }
      };

      template < typename HEAP, int FLAGS >
      using LocalHeapSelector = std::conditional_t< FLAGS & DijkstraFlags::LOCAL_HEAP,   //
                                                    DijkstraLocalHeap< HEAP >,
                                                    NoDijkstraLocalHeap< HEAP > >;

      /**
       * @brief Iterator type for forward direction shortest path computation.
       *
       * Computes shortest distances FROM a source node to all other nodes.
       * When using Forward, dist[v] represents the distance from source to v.
       */
      template < typename GR >
      struct DijkstraForward {
        using Digraph = GR;
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using SuccIterator = typename Digraph::OutArcIt;
        using PredIterator = typename Digraph::InArcIt;
        constexpr static Node running(const Digraph& g, Arc arc) noexcept { return g.source(arc); }
        constexpr static Node opposite(const Digraph& g, Arc arc) noexcept { return g.target(arc); }
      };

      /**
       * @brief Iterator type for backward direction shortest path computation.
       *
       * Computes shortest distances TO a target node from all other nodes.
       * When using Backward, dist[v] represents the distance from v to target.
       */
      template < typename GR >
      struct DijkstraBackward {
        using Digraph = GR;
        using Node = typename Digraph::Node;
        using Arc = typename Digraph::Arc;
        using SuccIterator = typename Digraph::InArcIt;
        using PredIterator = typename Digraph::OutArcIt;
        constexpr static Node running(const Digraph& g, Arc arc) noexcept { return g.target(arc); }
        constexpr static Node opposite(const Digraph& g, Arc arc) noexcept { return g.source(arc); }
      };

      template < typename GR, int FLAGS >
      using DirectionSelector = std::conditional_t< FLAGS & DijkstraFlags::BACKWARD,   //
                                                    DijkstraBackward< GR >,
                                                    DijkstraForward< GR > >;

      template < typename GR, int FLAGS >
      struct DijkstraPredMap {
        using Digraph = GR;
        TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
        using PredArray = TrivialDynamicArray< Arc, 4 >;
        using PredMap = typename Digraph::template NodeMap< PredArray >;
        using SuccArray = TrivialDynamicArray< Arc, 4 >;
        using SuccMap = typename Digraph::template NodeMap< SuccArray >;
        using Direction = details::DirectionSelector< GR, FLAGS >;

        PredMap _pred_map;   // < Given a vertex v, _pred_map[v] is the predecessor of v

        DijkstraPredMap(const Digraph& g) : _pred_map(g) {}

        constexpr PredMap&       predMap() noexcept { return _pred_map; }
        constexpr const PredMap& predMap() const noexcept { return _pred_map; }
        constexpr int            numPred(Node v) const noexcept { return _pred_map[v].size(); }
        constexpr Arc            predArc(Node v, int k = 0) const noexcept { return _pred_map[v][k]; }
        constexpr PredArray      predArcs(Node v) const noexcept { return _pred_map[v]; }
      };

      template < typename GR, int FLAGS >
      struct DijkstraSinglePredMap {
        using Digraph = GR;
        TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
        using PredMap = typename Digraph::template NodeMap< Arc >;
        using SuccMap = typename Digraph::template NodeMap< Arc >;
        using Direction = details::DirectionSelector< GR, FLAGS >;

        PredMap _pred_map;   // < Given a vertex v, _pred_map[v] is an array containing the predecessors of v

        DijkstraSinglePredMap(const Digraph& g) : _pred_map(g) {}

        constexpr PredMap&       predMap() noexcept { return _pred_map; }
        constexpr const PredMap& predMap() const noexcept { return _pred_map; }
        constexpr int            numPred(Node v) const noexcept { return _pred_map[v] != INVALID ? 1 : 0; }
        constexpr Arc            predArc(Node v) const noexcept { return _pred_map[v]; }
      };

      template < typename GR >
      struct NoDijkstraPredMap {
        using Digraph = GR;
        TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
        using PredMap = typename Digraph::template NodeMap< Arc >;
        using SuccMap = typename Digraph::template NodeMap< Arc >;
        NoDijkstraPredMap(const GR& g) {}
      };

      template < typename GR, int FLAGS >
      using PredMapSelector = std::conditional_t< FLAGS & DijkstraFlags::PRED_MAP,                          //
                                                  std::conditional_t< FLAGS & DijkstraFlags::SINGLE_PATH,   //
                                                                      DijkstraSinglePredMap< GR, FLAGS >,
                                                                      DijkstraPredMap< GR, FLAGS > >,
                                                  NoDijkstraPredMap< GR > >;

      template < int FLAGS >
      struct FlagValidator {
        static constexpr bool has_single_path = FLAGS & DijkstraFlags::SINGLE_PATH;
        static constexpr bool has_pred = (FLAGS & DijkstraFlags::PRED_MAP);
        static_assert(!has_single_path || has_pred, "SINGLE_PATH flag requires PRED_MAP");
      };
    }   // namespace details

    /**
     * @class DijkstraDefaultOperations
     * @brief This class defines the default length operations performed during Dijkstra's
     * algorithm.
     *
     * @tparam LENGTH The type used to represent lengths (lengths) of the arcs.
     */
    template < typename LENGTH >
    struct DijkstraDefaultOperations {
      static_assert(std::is_arithmetic_v< LENGTH >, "DijkstraDefaultOperations:Non-arithmetic length type.");
      using Length = LENGTH;
      constexpr static Length zero() noexcept { return static_cast< Length >(0); }
      constexpr static Length plus(const Length& left, const Length& right) noexcept {
#if 0
        if constexpr (std::is_same< Length, std::uint16_t >::value) {
          Length result;
          if(__builtin_add_overflow(left, right, &result)) {
            LOG_F(ERROR, "Overflow in length addition: {} + {} exceeds maximum value for uint16_t", left, right);
            return infinity();
          }
          return result;
        } else
          return left + right;
#else
        if constexpr (std::is_same< Length, std::uint16_t >::value) {
          assert(static_cast< uint32_t >(left) + static_cast< uint32_t >(right)
                 <= std::numeric_limits< Length >::max());
        }
        return left + right;
#endif
      }
      constexpr static bool   less(const Length& left, const Length& right) noexcept { return left < right; }
      constexpr static bool   equal(const Length& left, const Length& right) noexcept { return left == right; }
      constexpr static Length infinity() noexcept { return std::numeric_limits< Length >::max(); }
    };

    /**
     * @brief
     *
     * @tparam GR
     */
    template < typename GR >
    struct DijkstraDefaultVisitor {
      using Digraph = GR;
      using Node = typename Digraph::Node;
      constexpr void onNextNode(const Node node) noexcept {}
    };

    /**
     * @class DijkstraBase
     * @brief Base class that implements Dijkstra's algorithm with customizable heap management.
     *
     * This is the base class for Dijkstra's algorithm implementation. Use it only if you need
     * to run a customized version with ad-hoc length operations or custom heap management.
     * For standard use cases, prefer the Dijkstra or Dijkstra16Bits template aliases.
     *
     * **Heap Management:**
     * - If `FLAGS & DijkstraFlags::LOCAL_HEAP` is true: A heap is automatically allocated as a member variable.
     *   The algorithm is ready to run immediately after construction.
     * - If `FLAGS & DijkstraFlags::LOCAL_HEAP` is false: No heap is allocated. You MUST call `heap(Heap&)`
     *   to provide an external heap before calling `run()`. This avoids unnecessary heap
     *   allocation when sharing a heap across multiple Dijkstra instances.
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling the behavior of the algorithm, such as heap allocation.
     * @tparam HEAP The type of the heap data structure used for priority queue operations.
     * @tparam OPERATIONS A type defining operations performed on lengths during computation
     *         (addition, comparison, zero, infinity).
     * @tparam SPGRAPH The type of the shortest path graph.
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see Dijkstra
     * @see Dijkstra16Bits
     */
    template < typename GR, int FLAGS, typename HEAP, typename OPERATIONS, typename SPGRAPH, typename LM >
    struct DijkstraBase: details::LocalHeapSelector< HEAP, FLAGS >, details::PredMapSelector< GR, FLAGS > {
      using FlagValidator = details::FlagValidator< FLAGS >;

      using LocalHeapSelector = details::LocalHeapSelector< HEAP, FLAGS >;
      using PredMapSelector = details::PredMapSelector< GR, FLAGS >;
      using Direction = details::DirectionSelector< GR, FLAGS >;

      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using Operations = OPERATIONS;
      using Length = typename Operations::Length;
      using LengthMap = LM;

      using DistMap = typename Digraph::template NodeMap< Length >;
      using Heap = LocalHeapSelector::Heap;
      using PredMap = PredMapSelector::PredMap;
      using SuccMap = PredMapSelector::SuccMap;
      using SPGraph = SPGRAPH;

      struct PredHandler {
        constexpr static void preHeapHook(DijkstraBase& dijkstra, Node u, Node v, Arc arc) noexcept {
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
              dijkstra._pred_map[v] = arc;
            } else {
              dijkstra._pred_map[v].add(arc);
            }
          }
        }

        constexpr static void inHeapLessHook(DijkstraBase& dijkstra, Node u, Node v, Arc arc) noexcept {
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
              dijkstra._pred_map[v] = arc;
            } else {
              dijkstra._pred_map[v].removeAll();
              dijkstra._pred_map[v].add(arc);
            }
          }
        }

        constexpr static void inHeapEqualHook(DijkstraBase& dijkstra, Node u, Node v, Arc arc) noexcept {
          if constexpr ((FLAGS & DijkstraFlags::PRED_MAP) && !(FLAGS & DijkstraFlags::SINGLE_PATH))
            dijkstra._pred_map[v].add(arc);
        }

        constexpr static void postHeapEqualHook(DijkstraBase& dijkstra, Node u, Node v, Arc arc) noexcept {
          if constexpr ((FLAGS & DijkstraFlags::PRED_MAP) && !(FLAGS & DijkstraFlags::SINGLE_PATH))
            dijkstra._pred_map[v].add(arc);
        }
      };

      struct PathIterator {
        using iterator_category = std::input_iterator_tag;
        using value_type = TrivialDynamicArray< Arc >;
        using difference_type = std::ptrdiff_t;
        using pointer = const value_type*;
        using reference = const value_type&;

        using StackElt = Pair< Arc, int >;
        using Stack = nt::Stack< StackElt >;

        const DijkstraBase& _dijkstra;
        Node                _target;
        Stack               _stack;
        value_type          _current_path;
        bool                _has_path;

        PathIterator(const DijkstraBase& dijkstra, Node target) :
            _dijkstra(dijkstra), _target(target), _has_path(true) {
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
              // Single path case: _pred_map[v] is a single Arc
              Arc pred_arc = _dijkstra._pred_map[target];
              if (pred_arc != INVALID) _stack.push({pred_arc, 0});
            } else {
              // Multi-path case: _pred_map[v] is an array of Arcs
              const auto& pred_array = _dijkstra._pred_map[target];
              for (const Arc arc: pred_array)
                _stack.push({arc, 0});
            }
            _nextPath();
          } else {
            NT_ASSERT(false, "PathIterator requires PRED_MAP flag");
          }
        }

        PathIterator(const DijkstraBase& dijkstra, Node target, bool) :
            _dijkstra(dijkstra), _target(target), _has_path(false) {}

        constexpr reference operator*() const noexcept { return _current_path; }
        constexpr pointer   operator->() const noexcept { return &_current_path; }

        constexpr PathIterator& operator++() noexcept {
          _nextPath();
          return *this;
        }

        constexpr bool operator==(const PathIterator& other) const noexcept { return _has_path == other._has_path; }
        constexpr bool operator!=(const PathIterator& other) const noexcept { return !(*this == other); }

        constexpr void _nextPath() noexcept {
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            _current_path.clear();
            _has_path = false;

            while (!_stack.empty()) {
              StackElt&  elt = _stack.top();
              const Node u = _dijkstra._digraph.source(elt.first);

              if (u == _dijkstra._source) {
                int k = _stack.size();
                do {
                  --k;
                  _current_path.add(_stack[k].first);
                } while (k > 0 && _dijkstra._digraph.target(_stack[k].first) != _target);
                _has_path = true;
              }

              // Check whether there is another path arriving to u.
              if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
                // Single path: no alternative paths to explore
                _stack.pop();
              } else {
                // Multi-path: check for alternative predecessors
                if (elt.second < _dijkstra._pred_map[u]._i_count) {
                  const Arc next_pred = _dijkstra._pred_map[u][elt.second];
                  ++elt.second;
                  _stack.push({next_pred, 0});
                } else {
                  _stack.pop();
                }
              }

              if (_has_path) return;   //-- We found a path => yield
            }
          }
        }
      };

      struct PathRange {
        const DijkstraBase& _dijkstra;
        Node                _target;

        PathRange(const DijkstraBase& dijkstra, Node target) : _dijkstra(dijkstra), _target(target) {}

        PathIterator begin() const { return PathIterator(_dijkstra, _target); }
        PathIterator end() const { return PathIterator(_dijkstra, _target, true); }
      };

      struct ShortestPathGraphTraverser {
        const Digraph& _digraph;
        Queue< Node >  _queue;
        BitArray       _marked;

        ShortestPathGraphTraverser(const Digraph& digraph) :
            _digraph(digraph), _queue(digraph.maxNodeId() + 1), _marked(digraph.maxNodeId() + 1) {}

        /**
         * @brief Traverse both nodes and arcs in the shortest path graph
         * 
         * @param pred_map Predecessor map defining the shortest path graph structure
         * @param source Source node
         * @param target Target node
         * @param onVisitNode Called for each node (return false to stop early)
         * @param onVisitArc Called for each arc (return false to stop early)
         * @return true if traversal completed, false if stopped early or empty graph
         */
        template < typename NodeVisitor, typename ArcVisitor >
        bool traverse(const PredMap& pred_map,
                      const Node     source,
                      const Node     target,
                      NodeVisitor    onVisitNode,
                      ArcVisitor     onVisitArc) noexcept {
          if (target == source) return false;

          _queue.clear();
          _queue.push(source);
          _resetMarkers();
          _markNode(source);
          if (!onVisitNode(source)) return false;
          while (!_queue.empty()) {
            const Node v = _queue.front();
            _queue.pop();
            if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
              const Arc pred_arc = pred_map[v];
              if (pred_arc != INVALID) {
                if (!onVisitArc(pred_arc)) return false;
                const Node w = Direction::running(_digraph, pred_arc);
                if (!_isMarked(w)) {
                  if (!onVisitNode(w)) return false;
                  _markNode(w);
                  _queue.push(w);
                }
              }
            } else {
              const auto& preds = pred_map[v];
              for (int k = 0; k < preds.size(); ++k) {
                const Arc pred_arc = preds[k];
                if (!onVisitArc(pred_arc)) return false;
                const Node w = Direction::running(_digraph, pred_arc);
                if (!_isMarked(w)) {
                  if (!onVisitNode(w)) return false;
                  _markNode(w);
                  _queue.push(w);
                }
              }
            }
          }

          return true;
        }

        template < typename NodeVisitor >
        bool forEachNode(const PredMap& pred_map,
                         const Node     source,
                         const Node     target,
                         NodeVisitor    onVisitNode) noexcept {
          return traverse(pred_map, source, target, onVisitNode, [](Arc) { return true; });
        }

        template < typename ArcVisitor >
        bool forEachArc(const PredMap& pred_map, const Node source, const Node target, ArcVisitor onVisitArc) noexcept {
          return traverse(
             pred_map, source, target, [](Node) { return true; }, onVisitArc);
        }

        // Markers manipulation methods
        constexpr bool _isMarked(Node u) const noexcept { return _marked.isOne(_digraph.id(u)); }
        constexpr void _markNode(Node u) noexcept { _marked.setOneAt(_digraph.id(u)); }
        constexpr void _unmarkNode(Node u) noexcept { _marked.setZeroAt(_digraph.id(u)); }
        constexpr void _resetMarkers() noexcept { _marked.setBitsToZero(); }
      };

      const Digraph&   _digraph;
      const LengthMap* _p_length_map;
      Node             _source;
      Heap*            _p_heap;
      DistMap          _dist_map;   // < Given a vertex v, _dist_map[v] gives the shortest distance from source to v.
                                    // It is equal to 'Operations::infinity()' if there is no path toward v

      /**
       * @brief Construct a DijkstraBase object for a given directed graph.
       *
       * Initializes the Dijkstra algorithm instance for the given graph. Internal data structures
       * (_dist_map and _pred_map) are sized according to the graph's maximum node ID.
       *
       * **Heap initialization:**
       * - If `use_local_heap = true`: Internal heap is automatically allocated and ready to use.
       * - If `use_local_heap = false`: No heap is allocated. You **MUST** call `heap(Heap&)`
       *   before calling `run()`.
       *
       * @param digraph The directed graph on which to perform Dijkstra's algorithm.
       * @param length_map A map providing lengths associated to the arcs in the graph.
       *
       * @note The graph reference is stored internally, so the graph must remain valid
       *       for the lifetime of this Dijkstra instance.
       */
      explicit DijkstraBase(const Digraph& digraph, const LengthMap& length_map) :
          PredMapSelector(digraph), _digraph(digraph), _p_length_map(&length_map), _source(INVALID),
          _p_heap(LocalHeapSelector::getLocalHeap()), _dist_map(digraph) {}

      /**
       * @brief Construct a DijkstraBase object with an external heap.
       *
       * This constructor is useful when you want to provide an external heap that can be
       * shared across multiple Dijkstra instances, avoiding repeated heap allocations.
       * This is particularly beneficial when running Dijkstra multiple times on the same
       * graph with different source nodes.
       *
       * **Usage pattern:**
       * - Create a single heap instance
       * - Pass it to multiple Dijkstra instances via this constructor
       * - Each Dijkstra.run() call will reuse the same heap (avoiding allocation overhead)
       *
       * @param digraph The directed graph on which to perform Dijkstra's algorithm.
       * @param length_map A map providing lengths associated to the arcs in the graph.
       * @param heap External heap instance to use for priority queue operations. The heap
       *        must remain valid for the lifetime of this Dijkstra instance.
       *
       * @note This constructor ignores the `use_local_heap` template parameter and always
       *       uses the provided external heap.
       * @note The graph reference is stored internally, so the graph must remain valid
       *       for the lifetime of this Dijkstra instance.
       *
       * @code
       *  SmartDigraph graph;
       *  // ... add nodes and arcs ...
       *
       *  SmartDigraph::ArcMap<uint16_t> lengths(graph);
       *  // ... set lengths ...
       *
       *  // Create shared heap
       *  using Heap = OptimalHeap<uint16_t, SmartDigraph::StaticNodeMap<int>>;
       *  Heap shared_heap;
       *
       *  // Run Dijkstra from multiple sources with shared heap
       *  Dijkstra16Bits<SmartDigraph> dijkstra1(graph, shared_heap);
       *  dijkstra1.run(source1, lengths);
       *
       *  Dijkstra16Bits<SmartDigraph> dijkstra2(graph, shared_heap);
       *  dijkstra2.run(source2, lengths);
       * @endcode
       */
      explicit DijkstraBase(const Digraph& digraph, const LengthMap& length_map, Heap& heap) :
          PredMapSelector(digraph), _digraph(digraph), _p_length_map(&length_map), _source(INVALID), _p_heap(&heap),
          _dist_map(digraph) {}

      // Check if configuration supports path reconstruction
      static constexpr bool canReconstructPaths() noexcept { return FLAGS & DijkstraFlags::PRED_MAP; }

      // Check if using single-path mode
      static constexpr bool isSinglePathMode() noexcept { return FLAGS & DijkstraFlags::SINGLE_PATH; }

      // Check if using backward traversal
      static constexpr bool isBackward() noexcept { return FLAGS & DijkstraFlags::BACKWARD; }

      /**
       * @brief Compute the shortest paths from a source vertex to all other vertices in the graph.
       *
       * @param source The source vertex.
       *
       */
      void run(Node source) noexcept {
        ZoneScoped;
        run(source, DijkstraDefaultVisitor< Digraph >());
      }

      /**
       * @brief Compute the shortest paths from a source vertex to all other vertices in the graph.
       *
       * @param source The source vertex.
       * @param vs The visitor object to handle events during the algorithm execution.
       *
       */
      template < typename Visitor >
      void run(Node source, Visitor&& vs) noexcept {
        ZoneScoped;
        assert(_p_heap);

        _source = source;
        clear();

        _p_heap->push(source, Operations::zero());
        while (!_p_heap->empty())
          _processNextNode(vs);
      }

      /**
       * @brief Compute the shortest path from a source vertex to a target vertex in the graph.
       *
       * @param source The source vertex.
       * @param target The target vertex.
       *
       */
      void run(Node source, Node target) noexcept { run(source, target, DijkstraDefaultVisitor< Digraph >()); }

      /**
       * @brief Compute the shortest path from a source vertex to a target vertex in the graph.
       *
       * @param source The source vertex.
       * @param target The target vertex.
       * @param vs The visitor object to handle events during the algorithm execution.
       *
       */
      template < typename Visitor >
      void run(Node source, Node target, Visitor&& vs) noexcept {
        ZoneScoped;
        assert(_p_heap);

        _source = source;
        clear();

        _p_heap->push(source, Operations::zero());
        while (!_p_heap->empty() && target != _p_heap->top())
          _processNextNode(vs);

        if (!_p_heap->empty()) {
          const Node   u = _p_heap->top();
          const Length DistU = _p_heap->prio();
          _dist_map[u] = DistU;
          _p_heap->pop();
        }
      }

      /**
       * @brief Return a shortest path toward target
       *
       * @note This method is only available when PRED_MAP flag is set.
       *
       * @param target The target vertex.
       * @return The first shortest path, or an empty array if no path exists.
       *
       */
      TrivialDynamicArray< Arc > path(Node target) const noexcept {
        if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
          // Optimized path for SINGLE_PATH - no iterator overhead
          if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
            TrivialDynamicArray< Arc > result;
            Node                       current = target;
            while (current != _source) {
              Arc pred_arc = this->_pred_map[current];
              if (pred_arc == INVALID) break;
              result.add(pred_arc);
              current = _digraph.source(pred_arc);
            }
            // Reverse to get source-to-target order
            std::reverse(result.data(), result.data() + result.size());
            return result;
          } else {
            return PathIterator(*this, target)._current_path;
          }
        } else {
          NT_ASSERT(false, "path() requires PRED_MAP flag");
          return {};
        }
      }

      /**
       * @brief Get a range of shortest paths toward the target vertex.
       *
       * @note This method is only available when PRED_MAP flag is set.
       *
       * @param target The target vertex.
       * @return A range of shortest paths toward the target vertex.
       */
      PathRange paths(Node target) const noexcept {
        if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
          return PathRange(*this, target);
        } else {
          NT_ASSERT(false, "paths() requires PRED_MAP flag");
          return {};
        }
      }

      /**
       * @brief Get a reference to the underlying directed graph.
       *
       * @return A reference to the underlying directed graph.
       *
       */
      constexpr const Digraph& digraph() const noexcept { return _digraph; }

      /**
       * @brief Get a reference to the underlying heap data structure.
       *
       * @return A reference to the underlying heap.
       *
       */
      constexpr Heap& heap() noexcept {
        assert(_p_heap);
        return *_p_heap;
      }
      constexpr const Heap& heap() const noexcept {
        assert(_p_heap);
        return *_p_heap;
      }

      /**
       * @brief Set an external heap for the algorithm.
       *
       * This method allows you to provide a custom external heap instead of using the
       * internal local heap. This is useful for:
       * - Sharing a single heap across multiple Dijkstra instances (saves memory)
       * - Avoiding heap allocation overhead when `use_local_heap = false`
       * - Using a pre-initialized heap with specific configuration
       *
       * **Important:**
       * - If `use_local_heap = false`, this method **MUST** be called before `run()`,
       *   otherwise the algorithm will assert/crash.
       * - If `use_local_heap = true`, calling this method will override the local heap
       *   with the provided external heap.
       * - The provided heap must remain valid for the lifetime of this Dijkstra instance
       *   or until another heap is set.
       *
       * @param heap Reference to an external heap to use for priority queue operations.
       * @return Reference to this DijkstraBase instance (for method chaining).
       *
       * @code
       *  // Example: Share heap across multiple Dijkstra instances
       *  using Heap = OptimalHeap<double, SmartDigraph::NodeMap<int>>;
       *  Heap shared_heap;
       *
       *  Dijkstra<SmartDigraph, double, false> dijkstra1(graph);
       *  dijkstra1.heap(shared_heap).run(source1, lengths);
       *
       *  Dijkstra<SmartDigraph, double, false> dijkstra2(graph);
       *  dijkstra2.heap(shared_heap).run(source2, lengths);
       * @endcode
       */
      DijkstraBase& heap(Heap& heap) noexcept {
        _p_heap = &heap;
        return *this;
      }

      /**
       * @brief Sets the length map.
       *
       * @param m The length map to set.
       * @return Reference to this DijkstraBase instance (for method chaining).
       */
      constexpr DijkstraBase& lengthMap(const LengthMap& m) noexcept {
        _p_length_map = &m;
        return *this;
      }

      /**
       * @brief Get a reference to the length map used for arc lengths.
       *
       * @return A reference to the length map.
       */
      constexpr const LengthMap& lengthMap() const noexcept {
        assert(_p_length_map);
        return *_p_length_map;
      }

      /**
       * @brief Get a reference to the node map that stores the shortest distances.
       *
       * @return A reference to the distance map.
       *
       */
      constexpr DistMap&       distMap() noexcept { return _dist_map; }
      constexpr const DistMap& distMap() const noexcept { return _dist_map; }

      /**
       * @brief Get the shortest distance of the given node from the source.
       *
       * @param v The node for which to get the distance.
       *
       * @return The distance from the source to the given node.
       *
       */
      constexpr Length dist(Node v) const noexcept { return _dist_map[v]; }

      /**
       * @brief Check if a node is reached from the source.
       *
       * @param v The node to check.
       *
       * @return True if the node is reached from the root, false otherwise.
       *
       */
      constexpr bool reached(Node v) const noexcept { return _dist_map[v] != Operations::infinity(); }

      /**
       * @brief Get the source node from which the shortest paths are computed.
       *
       */
      constexpr Node source() const noexcept { return _source; }

      /**
       * @brief Get the flags used to configure this Dijkstra instance.
       *
       * @return The configuration flags.
       *
       */
      static constexpr int flags() noexcept { return FLAGS; }

      /**
       * @brief Reset the internal data structures and state.
       *
       */
      constexpr void clear() noexcept {
        // ZoneScoped;  // generate a compile time error when building with tracy: ‘__tracy_source_location416’ declared
        // ‘static’ in ‘constexpr’ function
        assert(_p_heap);

        _p_heap->init(_digraph.maxNodeId() + 1);
        for (NodeIt node(_digraph); node != INVALID; ++node) {
          _dist_map[node] = Operations::infinity();
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
              this->_pred_map[node] = INVALID;
            } else {
              this->_pred_map[node].removeAll();
            }
          }
        }
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node
       *
       * @param v Node toward which we search for a unique path
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      constexpr bool hasUniqueShortestPath(Node v) const noexcept {
        return hasUniqueShortestPath(v, [](const Arc) {});
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node and
       * stores the arcs belonging to that path if it exists.
       *
       * @param v Node toward which we search for a unique path
       * @param path An array storing the arcs of the unique path if it exists
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      constexpr bool hasUniqueShortestPath(Node v, TrivialDynamicArray< Arc >& path) const noexcept {
        return hasUniqueShortestPath(v, [&](const Arc a) { path.add(a); });
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node and
       * applies the function doVisitArc on every arc belonging to that path if it exists.
       *
       * @param v Node toward which we search for a unique path
       * @param doVisitArc A function called on every arc of the unique path if it exists
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      template < typename F >
      constexpr bool hasUniqueShortestPath(Node v, F doVisitArc) const noexcept {
        if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
          while (this->numPred(v) == 1) {
            const Arc a = this->predArc(v);
            v = _digraph.oppositeNode(v, a);
            doVisitArc(a);
          }
          return v == source();
        } else {
          NT_ASSERT(false, "hasUniqueShortestPath() requires PRED_MAP flag");
          return false;
        }
      }

      /**
       * @brief Build the shortest path graph based on the current distance map
       *
       */
      constexpr void shortestPathGraph(SPGraph& spg) const noexcept {
        for (ArcIt arc(_digraph); arc != INVALID; ++arc)
          if (isShortestPathArc(arc)) spg.addArc(_digraph.source(arc), _digraph.target(arc));
      }

      /**
       * @brief Check whether an arc belongs to the shortest path graph
       *
       */
      constexpr bool isShortestPathArc(Arc arc) const noexcept {
        assert(_p_length_map);
        const Node u = Direction::running(_digraph, arc);
        const Node v = Direction::opposite(_digraph, arc);
        // If slack ≈ 0, then arc is in the shortest path graph
        const Length slack = _dist_map[v] - _dist_map[u] - (*_p_length_map)[arc];
        return Tolerance< Length >().isZero(slack);
      }

      /**
       * @brief Build a successor map from the predecessor map.
       *
       * This method constructs a successor map by inverting the predecessor relationships.
       * For each node v with predecessor arc (u, v), the arc is added to the successor list of node u.
       *
       * @param succ_map Reference to a NodeMap that will be filled with successors.
       *
       */
      void buildSuccMap(SuccMap& succ_map) const noexcept {
        if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
          if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
            succ_map.setBitsToOne();   // This set the map values to -1
            for (NodeIt v(_digraph); v != INVALID; ++v) {
              const Arc pred_arc = this->_pred_map[v];
              if (pred_arc == INVALID) continue;
              const Node u = Direction::running(_digraph, pred_arc);
              succ_map[u] = pred_arc;
            }
          } else {
            for (NodeIt v(_digraph); v != INVALID; ++v)
              succ_map[v].removeAll();
            for (NodeIt v(_digraph); v != INVALID; ++v) {
              for (const Arc pred_arc: this->_pred_map[v]) {
                const Node u = Direction::running(_digraph, pred_arc);
                succ_map[u].add(pred_arc);
              }
            }
          }
        } else {
          NT_ASSERT(false, "buildSuccMap() requires PRED_MAP flag");
        }
      }

      /**
       * @brief Export the solution to a JSON document.
       *
       * @tparam Hooks The type of hooks used in the algorithm.
       * @tparam LengthMap The type of the length map.
       *
       * @return A JSON document representing the solution.
       *
       */
      JSONDocument exportSolutionToJSON() const noexcept {
        ZoneScoped;
        JSONDocument json;
        json.SetObject();

        JSONDocument::AllocatorType& allocator = json.GetAllocator();
        json.AddMember("source", JSONValue(_digraph.id(_source)), allocator);

        JSONValue targets_array_json(JSONValueType::ARRAY_TYPE);
        for (NodeIt v(_digraph); v != INVALID; ++v) {
          JSONValue node_object(JSONValueType::OBJECT_TYPE);
          node_object.AddMember("id", JSONValue(_digraph.id(v)), allocator);
          node_object.AddMember("min_distance", JSONValue(_dist_map[v]), allocator);
          if constexpr (FLAGS & DijkstraFlags::PRED_MAP) {
            JSONValue preds_array_json(JSONValueType::ARRAY_TYPE);
            for (const Arc pred: this->_pred_map[v])
              preds_array_json.PushBack(JSONValue(_digraph.id(pred)), allocator);
            node_object.AddMember("predecessors", std::move(preds_array_json), allocator);
          }
          targets_array_json.PushBack(std::move(node_object), allocator);
        }

        json.AddMember("targets", std::move(targets_array_json), allocator);
        return json;
      }

      /**
       * @brief Process the next node in the internal loop of Dijkstra
       *
       */
      template < typename Visitor >
      void _processNextNode(Visitor&& vs) noexcept {
        ZoneScoped;
        assert(_p_heap);
        assert(_p_length_map);

        const Node   u = _p_heap->top();
        const Length dist_u = _p_heap->prio();
        _p_heap->pop();
        _dist_map[u] = dist_u;
        vs.onNextNode(u);

        for (typename Direction::SuccIterator arc(_digraph, u); arc != INVALID; ++arc) {
          const Length uv_length = (*_p_length_map)[arc];

          // We use ">=" rather than "==" to include the case where uv_length is a floating number
          // with a 'inf' value.
          if (uv_length >= Operations::infinity()) [[unlikely]]
            continue;

          const Node   v = Direction::opposite(_digraph, arc);
          const Length new_dist = Operations::plus(dist_u, uv_length);

          switch (_p_heap->state(v)) {
            case Heap::PRE_HEAP: {
              PredHandler::preHeapHook(*this, u, v, arc);
              _p_heap->push(v, new_dist);
            } break;

            case Heap::IN_HEAP: {
              if (Operations::less(new_dist, (*_p_heap)[v])) {
                PredHandler::inHeapLessHook(*this, u, v, arc);
                _p_heap->decrease(v, new_dist);
              } else if (Operations::equal(new_dist, (*_p_heap)[v])) {
                PredHandler::inHeapEqualHook(*this, u, v, arc);
              }
            } break;

            case Heap::POST_HEAP: {
              if (Operations::equal(new_dist, _dist_map[v])) PredHandler::postHeapEqualHook(*this, u, v, arc);
            } break;

            default: assert(false);
          }
        }
      }
    };

    /**
     * @brief Default operation traits for the Dijkstra algorithm class.
     *
     * This operation traits class defines all computational operations and
     * constants which are used in the Dijkstra algorithm.
     */
    template < typename V >
    struct DijkstraDefaultOperationTraits {
      using Value = V;
      /** @brief Gives back the zero value of the type. */
      static Value zero() { return static_cast< Value >(0); }
      /** @brief Gives back the sum of the given two elements. */
      static Value plus(const Value& left, const Value& right) { return left + right; }
      /**
       * @brief Gives back true only if the first value is less than the second.
       */
      static bool less(const Value& left, const Value& right) { return left < right; }
    };

    /** Default traits class of Dijkstra class. */

    /**
     * Default traits class of Dijkstra class.
     * @tparam GR The type of the digraph.
     * @tparam LEN The type of the length map.
     */
    template < typename GR, typename LEN >
    struct DijkstraDefaultTraits {
      /** The type of the digraph the algorithm runs on. */
      using Digraph = GR;

      /** The type of the map that stores the arc lengths. */

      /**
       * The type of the map that stores the arc lengths.
       * It must conform to the @ref concepts::ReadMap "ReadMap" concept.
       */
      using LengthMap = LEN;
      /** The type of the arc lengths. */
      using Value = typename LEN::Value;

      /** Operation traits for %Dijkstra algorithm. */

      /**
       * This class defines the operations that are used in the algorithm.
       * @see DijkstraDefaultOperationTraits
       */
      using OperationTraits = DijkstraDefaultOperationTraits< Value >;

      /** The cross reference type used by the heap. */

      /**
       * The cross reference type used by the heap.
       * Usually it is \c Digraph::NodeMap<int>.
       */
      using HeapCrossRef = typename Digraph::template NodeMap< int >;
      /** Instantiates a \c HeapCrossRef. */

      /**
       * This function instantiates a @ref HeapCrossRef.
       * @param g is the digraph, to which we would like to define the
       * @ref HeapCrossRef.
       */
      static HeapCrossRef* createHeapCrossRef(const Digraph& g) { return new HeapCrossRef(g); }

      /** The heap type used by the %Dijkstra algorithm. */

      /**
       * The heap type used by the Dijkstra algorithm.
       *
       * \sa BinHeap
       * \sa Dijkstra
       */
      using Heap = BinHeap< typename LEN::Value, HeapCrossRef, std::less< Value > >;
      /** Instantiates a \c Heap. */

      /** This function instantiates a @ref Heap. */
      static Heap* createHeap(HeapCrossRef& r) { return new Heap(r); }

      /**
       * @brief The type of the map that stores the predecessor
       * arcs of the shortest paths.
       *
       * The type of the map that stores the predecessor
       * arcs of the shortest paths.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using PredMap = typename Digraph::template NodeMap< typename Digraph::Arc >;
      /** Instantiates a \c PredMap. */

      /**
       * This function instantiates a @ref PredMap.
       * @param g is the digraph, to which we would like to define the
       * @ref PredMap.
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * The type of the map that indicates which nodes are processed.
       */

      /**
       * The type of the map that indicates which nodes are processed.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       * By default, it is a NullMap.
       */
      using ProcessedMap = NullMap< typename Digraph::Node, bool >;
      /** Instantiates a \c ProcessedMap. */

      /**
       * This function instantiates a @ref ProcessedMap.
       * @param g is the digraph, to which
       * we would like to define the @ref ProcessedMap.
       */
#ifdef DOXYGEN
      static ProcessedMap* createProcessedMap(const Digraph& g)
#else
      static ProcessedMap* createProcessedMap(const Digraph&)
#endif
      {
        return new ProcessedMap();
      }

      /** The type of the map that stores the distances of the nodes. */

      /**
       * The type of the map that stores the distances of the nodes.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using DistMap = typename Digraph::template NodeMap< typename LEN::Value >;
      /** Instantiates a \c DistMap. */

      /**
       * This function instantiates a @ref DistMap.
       * @param g is the digraph, to which we would like to define
       * the @ref DistMap.
       */
      static DistMap* createDistMap(const Digraph& g) { return new DistMap(g); }
    };

/** %Dijkstra algorithm class. */

/**
 * @ingroup shortest_path
 * This class provides an efficient implementation of the %Dijkstra algorithm.
 *
 * The %Dijkstra algorithm solves the single-source shortest path problem
 * when all arc lengths are non-negative. If there are negative lengths,
 * the BellmanFord algorithm should be used instead.
 *
 * The arc lengths are passed to the algorithm using a
 * @ref concepts::ReadMap "ReadMap",
 * so it is easy to change it to any kind of length.
 * The type of the length is determined by the
 * @ref concepts::ReadMap::Value "Value" of the length map.
 * It is also possible to change the underlying priority heap.
 *
 * There is also a @ref dijkstra() "function-type interface" for the
 * %Dijkstra algorithm, which is convenient in the simplier cases and
 * it can be used easier.
 *
 * @tparam GR The type of the digraph the algorithm runs on.
 * @tparam LEN A @ref concepts::ReadMap "readable" arc map that specifies
 * the lengths of the arcs.
 * It is read once for each arc, so the map may involve in
 * relatively time consuming process to compute the arc lengths if
 * it is necessary. The default map type is @ref
 * concepts::Digraph::ArcMap "GR::ArcMap<int>".
 * @tparam TR The traits class that defines various types used by the
 * algorithm. By default, it is @ref DijkstraDefaultTraits
 * "DijkstraDefaultTraits<GR, LEN>".
 * In most cases, this parameter should not be set directly,
 * consider to use the named template parameters instead.
 */
#ifdef DOXYGEN
    template < typename GR, typename LEN, typename TR >
#else
    template < typename GR,
               typename LEN = typename GR::template ArcMap< int >,
               typename TR = DijkstraDefaultTraits< GR, LEN > >
#endif
    class DijkstraLegacy {
      public:
      /** The type of the digraph the algorithm runs on. */
      using Digraph = typename TR::Digraph;

      /** The type of the arc lengths. */
      using Value = typename TR::Value;
      /** The type of the map that stores the arc lengths. */
      using LengthMap = typename TR::LengthMap;
      /**
       * @brief The type of the map that stores the predecessor arcs of the
       * shortest paths.
       */
      using PredMap = typename TR::PredMap;
      /** The type of the map that stores the distances of the nodes. */
      using DistMap = typename TR::DistMap;
      /**
       * The type of the map that indicates which nodes are processed.
       */
      using ProcessedMap = typename TR::ProcessedMap;
      /** The type of the paths. */
      using Path = PredMapPath< Digraph, PredMap >;
      /** The cross reference type used for the current heap. */
      using HeapCrossRef = typename TR::HeapCrossRef;
      /** The heap type used by the algorithm. */
      using Heap = typename TR::Heap;
      /**
       * @brief The @ref nt::graphs::DijkstraDefaultOperationTraits
       * "operation traits class" of the algorithm.
       */
      using OperationTraits = typename TR::OperationTraits;

      /**
       * The @ref nt::graphs::DijkstraDefaultTraits "traits class" of the algorithm.
       */
      using Traits = TR;

      private:
      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using OutArcIt = typename Digraph::OutArcIt;

      // Pointer to the underlying digraph.
      const Digraph* G;
      // Pointer to the length map.
      const LengthMap* _length;
      // Pointer to the map of predecessors arcs.
      PredMap* _pred;
      // Indicates if _pred is locally allocated (true) or not.
      bool local_pred;
      // Pointer to the map of distances.
      DistMap* _dist;
      // Indicates if _dist is locally allocated (true) or not.
      bool local_dist;
      // Pointer to the map of processed status of the nodes.
      ProcessedMap* _processed;
      // Indicates if _processed is locally allocated (true) or not.
      bool local_processed;
      // Pointer to the heap cross references.
      HeapCrossRef* _heap_cross_ref;
      // Indicates if _heap_cross_ref is locally allocated (true) or not.
      bool local_heap_cross_ref;
      // Pointer to the heap.
      Heap* _heap;
      // Indicates if _heap is locally allocated (true) or not.
      bool local_heap;

      // Creates the maps if necessary.
      void create_maps() {
        if (!_pred) {
          local_pred = true;
          _pred = Traits::createPredMap(*G);
        }
        if (!_dist) {
          local_dist = true;
          _dist = Traits::createDistMap(*G);
        }
        if (!_processed) {
          local_processed = true;
          _processed = Traits::createProcessedMap(*G);
        }
        if (!_heap_cross_ref) {
          local_heap_cross_ref = true;
          _heap_cross_ref = Traits::createHeapCrossRef(*G);
        }
        if (!_heap) {
          local_heap = true;
          _heap = Traits::createHeap(*_heap_cross_ref);
        }
      }

      public:
      using Create = DijkstraLegacy;

      /** \name Named Template Parameters */

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
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c PredMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c PredMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetPredMap: public DijkstraLegacy< Digraph, LengthMap, SetPredMapTraits< T > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetPredMapTraits< T > >;
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
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c DistMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c DistMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetDistMap: public DijkstraLegacy< Digraph, LengthMap, SetDistMapTraits< T > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetDistMapTraits< T > >;
      };

      template < class T >
      struct SetProcessedMapTraits: public Traits {
        using ProcessedMap = T;
        static ProcessedMap* createProcessedMap(const Digraph&) {
          NT_ASSERT(false, "ProcessedMap is not initialized");
          return 0;   // ignore warnings
        }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      template < class T >
      struct SetProcessedMap: public DijkstraLegacy< Digraph, LengthMap, SetProcessedMapTraits< T > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetProcessedMapTraits< T > >;
      };

      struct SetStandardProcessedMapTraits: public Traits {
        using ProcessedMap = typename Digraph::template NodeMap< bool >;
        static ProcessedMap* createProcessedMap(const Digraph& g) { return new ProcessedMap(g); }
      };
      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
       * If you don't set it explicitly, it will be automatically allocated.
       */
      struct SetStandardProcessedMap: public DijkstraLegacy< Digraph, LengthMap, SetStandardProcessedMapTraits > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetStandardProcessedMapTraits >;
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
       * @brief @ref named-templ-param "Named parameter" for setting
       * heap and cross reference types
       *
       * @ref named-templ-param "Named parameter" for setting heap and cross
       * reference types. If this named parameter is used, then external
       * heap and cross reference objects must be passed to the algorithm
       * using the @ref heap() function before calling @ref run(Node) "run()"
       * or @ref init().
       * \sa SetStandardHeap
       */
      template < class H, class CR = typename Digraph::template NodeMap< int > >
      struct SetHeap: public DijkstraLegacy< Digraph, LengthMap, SetHeapTraits< H, CR > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetHeapTraits< H, CR > >;
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
       * heap and cross reference types with automatic allocation
       *
       * @ref named-templ-param "Named parameter" for setting heap and cross
       * reference types with automatic allocation.
       * They should have standard constructor interfaces to be able to
       * automatically created by the algorithm (i.e. the digraph should be
       * passed to the constructor of the cross reference and the cross
       * reference should be passed to the constructor of the heap).
       * However, external heap and cross reference objects could also be
       * passed to the algorithm using the @ref heap() function before
       * calling @ref run(Node) "run()" or @ref init().
       * \sa SetHeap
       */
      template < class H, class CR = typename Digraph::template NodeMap< int > >
      struct SetStandardHeap: public DijkstraLegacy< Digraph, LengthMap, SetStandardHeapTraits< H, CR > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetStandardHeapTraits< H, CR > >;
      };

      template < class T >
      struct SetOperationTraitsTraits: public Traits {
        using OperationTraits = T;
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * \c OperationTraits type
       *
       * @ref named-templ-param "Named parameter" for setting
       * \c OperationTraits type.
       * For more information, see @ref DijkstraDefaultOperationTraits.
       */
      template < class T >
      struct SetOperationTraits: public DijkstraLegacy< Digraph, LengthMap, SetOperationTraitsTraits< T > > {
        using Create = DijkstraLegacy< Digraph, LengthMap, SetOperationTraitsTraits< T > >;
      };

      /** @} */

      protected:
      DijkstraLegacy() {}

      public:
      /** Constructor. */

      /**
       * Constructor.
       * @param g The digraph the algorithm runs on.
       * @param length The length map used by the algorithm.
       */
      DijkstraLegacy(const Digraph& g, const LengthMap& length) :
          G(&g), _length(&length), _pred(NULL), local_pred(false), _dist(NULL), local_dist(false), _processed(NULL),
          local_processed(false), _heap_cross_ref(NULL), local_heap_cross_ref(false), _heap(NULL), local_heap(false) {}

      /** Destructor. */
      ~DijkstraLegacy() noexcept {
        if (local_pred) delete _pred;
        if (local_dist) delete _dist;
        if (local_processed) delete _processed;
        if (local_heap_cross_ref) delete _heap_cross_ref;
        if (local_heap) delete _heap;
      }

      /** Sets the length map. */

      /**
       * Sets the length map.
       * @return <tt> (*this) </tt>
       */
      DijkstraLegacy& lengthMap(const LengthMap& m) {
        _length = &m;
        return *this;
      }

      /** Sets the map that stores the predecessor arcs. */

      /**
       * Sets the map that stores the predecessor arcs.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      DijkstraLegacy& predMap(PredMap& m) {
        if (local_pred) {
          delete _pred;
          local_pred = false;
        }
        _pred = &m;
        return *this;
      }

      /** Sets the map that indicates which nodes are processed. */

      /**
       * Sets the map that indicates which nodes are processed.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      DijkstraLegacy& processedMap(ProcessedMap& m) {
        if (local_processed) {
          delete _processed;
          local_processed = false;
        }
        _processed = &m;
        return *this;
      }

      /** Sets the map that stores the distances of the nodes. */

      /**
       * Sets the map that stores the distances of the nodes calculated by the
       * algorithm.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), an instance will be allocated automatically.
       * The destructor deallocates this automatically allocated map,
       * of course.
       * @return <tt> (*this) </tt>
       */
      DijkstraLegacy& distMap(DistMap& m) {
        if (local_dist) {
          delete _dist;
          local_dist = false;
        }
        _dist = &m;
        return *this;
      }

      /** Sets the heap and the cross reference used by algorithm. */

      /**
       * Sets the heap and the cross reference used by algorithm.
       * If you don't use this function before calling @ref run(Node) "run()"
       * or @ref init(), heap and cross reference instances will be
       * allocated automatically.
       * The destructor deallocates these automatically allocated objects,
       * of course.
       * @return <tt> (*this) </tt>
       */
      DijkstraLegacy& heap(Heap& hp, HeapCrossRef& cr) {
        if (local_heap_cross_ref) {
          delete _heap_cross_ref;
          local_heap_cross_ref = false;
        }
        _heap_cross_ref = &cr;
        if (local_heap) {
          delete _heap;
          local_heap = false;
        }
        _heap = &hp;
        return *this;
      }

      private:
      void finalizeNodeData(Node v, Value dst) {
        _processed->set(v, true);
        _dist->set(v, dst);
      }

      public:
      /**
       * \name Execution Control
       * The simplest way to execute the %Dijkstra algorithm is to use
       * one of the member functions called @ref run(Node) "run()".\n
       * If you need better control on the execution, you have to call
       * @ref init() first, then you can add several source nodes with
       * @ref addSource(). Finally the actual path computation can be
       * performed with one of the @ref start() functions.
       */

      /** @{ */

      /**
       * @brief Initializes the internal data structures.
       *
       * Initializes the internal data structures.
       */
      void init() {
        create_maps();
        _heap->clear();
        for (NodeIt u(*G); u != INVALID; ++u) {
          _pred->set(u, INVALID);
          _processed->set(u, false);
          _heap_cross_ref->set(u, Heap::PRE_HEAP);
        }
      }

      /** Adds a new source node. */

      /**
       * Adds a new source node to the priority heap.
       * The optional second parameter is the initial distance of the node.
       *
       * The function checks if the node has already been added to the heap and
       * it is pushed to the heap only if either it was not in the heap
       * or the shortest path found till then is shorter than \c dst.
       */
      void addSource(Node s, Value dst = OperationTraits::zero()) {
        if (_heap->state(s) != Heap::IN_HEAP) {
          _heap->push(s, dst);
        } else if (OperationTraits::less((*_heap)[s], dst)) {
          _heap->set(s, dst);
          _pred->set(s, INVALID);
        }
      }

      /** Processes the next node in the priority heap */

      /**
       * Processes the next node in the priority heap.
       *
       * @return The processed node.
       *
       * @warning The priority heap must not be empty.
       */
      Node processNextNode() {
        Node  v = _heap->top();
        Value oldvalue = _heap->prio();
        _heap->pop();
        finalizeNodeData(v, oldvalue);

        for (OutArcIt e(*G, v); e != INVALID; ++e) {
          Node w = G->target(e);
          switch (_heap->state(w)) {
            case Heap::PRE_HEAP:
              _heap->push(w, OperationTraits::plus(oldvalue, (*_length)[e]));
              _pred->set(w, e);
              break;
            case Heap::IN_HEAP: {
              Value newvalue = OperationTraits::plus(oldvalue, (*_length)[e]);
              if (OperationTraits::less(newvalue, (*_heap)[w])) {
                _heap->decrease(w, newvalue);
                _pred->set(w, e);
              }
            } break;
            case Heap::POST_HEAP: break;
          }
        }
        return v;
      }

      /** The next node to be processed. */

      /**
       * Returns the next node to be processed or \c INVALID if the
       * priority heap is empty.
       */
      Node nextNode() const { return !_heap->empty() ? _heap->top() : INVALID; }

      /** Returns \c false if there are nodes to be processed. */

      /**
       * Returns \c false if there are nodes to be processed
       * in the priority heap.
       */
      bool emptyQueue() const { return _heap->empty(); }

      /** Returns the number of the nodes to be processed. */

      /**
       * Returns the number of the nodes to be processed
       * in the priority heap.
       */
      int queueSize() const { return _heap->size(); }

      /** Executes the algorithm. */

      /**
       * Executes the algorithm.
       *
       * This method runs the %Dijkstra algorithm from the root node(s)
       * in order to compute the shortest path to each node.
       *
       * The algorithm computes
       * - the shortest path tree (forest),
       * - the distance of each node from the root(s).
       *
       * @pre init() must be called and at least one root node should be
       * added with addSource() before using this function.
       *
       * @note <tt>d.start()</tt> is just a shortcut of the following code.
       * @code
       * while ( !d.emptyQueue() ) {
       * d.processNextNode();
       * }
       * @endcode
       */
      void start() {
        while (!emptyQueue())
          processNextNode();
      }

      /**
       * Executes the algorithm until the given target node is processed.
       */

      /**
       * Executes the algorithm until the given target node is processed.
       *
       * This method runs the %Dijkstra algorithm from the root node(s)
       * in order to compute the shortest path to \c t.
       *
       * The algorithm computes
       * - the shortest path to \c t,
       * - the distance of \c t from the root(s).
       *
       * @pre init() must be called and at least one root node should be
       * added with addSource() before using this function.
       */
      void start(Node t) {
        while (!_heap->empty() && _heap->top() != t)
          processNextNode();
        if (!_heap->empty()) {
          finalizeNodeData(_heap->top(), _heap->prio());
          _heap->pop();
        }
      }

      /** Executes the algorithm until a condition is met. */

      /**
       * Executes the algorithm until a condition is met.
       *
       * This method runs the %Dijkstra algorithm from the root node(s) in
       * order to compute the shortest path to a node \c v with
       * <tt>nm[v]</tt> true, if such a node can be found.
       *
       * @param nm A \c bool (or convertible) node map. The algorithm
       * will stop when it reaches a node \c v with <tt>nm[v]</tt> true.
       *
       * @return The reached node \c v with <tt>nm[v]</tt> true or
       * \c INVALID if no such node was found.
       *
       * @pre init() must be called and at least one root node should be
       * added with addSource() before using this function.
       */
      template < class NodeBoolMap >
      Node start(const NodeBoolMap& nm) {
        while (!_heap->empty() && !nm[_heap->top()])
          processNextNode();
        if (_heap->empty()) return INVALID;
        finalizeNodeData(_heap->top(), _heap->prio());
        return _heap->top();
      }

      /** Runs the algorithm from the given source node. */

      /**
       * This method runs the %Dijkstra algorithm from node \c s
       * in order to compute the shortest path to each node.
       *
       * The algorithm computes
       * - the shortest path tree,
       * - the distance of each node from the root.
       *
       * @note <tt>d.run(s)</tt> is just a shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start();
       * @endcode
       */
      void run(Node s) {
        init();
        addSource(s);
        start();
      }

      /** Finds the shortest path between \c s and \c t. */

      /**
       * This method runs the %Dijkstra algorithm from node \c s
       * in order to compute the shortest path to node \c t
       * (it stops searching when \c t is processed).
       *
       * @return \c true if \c t is reachable form \c s.
       *
       * @note Apart from the return value, <tt>d.run(s,t)</tt> is just a
       * shortcut of the following code.
       * @code
       * d.init();
       * d.addSource(s);
       * d.start(t);
       * @endcode
       */
      bool run(Node s, Node t) {
        init();
        addSource(s);
        start(t);
        return (*_heap_cross_ref)[t] == Heap::POST_HEAP;
      }

      /** @} */

      /**
       * \name Query Functions
       * The results of the %Dijkstra algorithm can be obtained using these
       * functions.\n
       * Either @ref run(Node) "run()" or @ref init() should be called
       * before using them.
       */

      /** @{ */

      /** The shortest path to the given node. */

      /**
       * Returns the shortest path to the given node from the root(s).
       *
       * @warning \c t should be reached from the root(s).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Path path(Node t) const { return Path(*G, *_pred, t); }

      /** The distance of the given node from the root(s). */

      /**
       * Returns the distance of the given node from the root(s).
       *
       * @warning If node \c v is not reached from the root(s), then
       * the return value of this function is undefined.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Value dist(Node v) const { return (*_dist)[v]; }

      /**
       * @brief Returns the 'previous arc' of the shortest path tree for
       * the given node.
       *
       * This function returns the 'previous arc' of the shortest path
       * tree for the node \c v, i.e. it returns the last arc of a
       * shortest path from a root to \c v. It is \c INVALID if \c v
       * is not reached from the root(s) or if \c v is a root.
       *
       * The shortest path tree used here is equal to the shortest path
       * tree used in @ref predNode() and @ref predMap().
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Arc predArc(Node v) const { return (*_pred)[v]; }

      /**
       * @brief Returns the 'previous node' of the shortest path tree for
       * the given node.
       *
       * This function returns the 'previous node' of the shortest path
       * tree for the node \c v, i.e. it returns the last but one node
       * of a shortest path from a root to \c v. It is \c INVALID
       * if \c v is not reached from the root(s) or if \c v is a root.
       *
       * The shortest path tree used here is equal to the shortest path
       * tree used in @ref predArc() and @ref predMap().
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      Node predNode(Node v) const { return (*_pred)[v] == INVALID ? INVALID : G->source((*_pred)[v]); }

      /**
       * @brief Returns a const reference to the node map that stores the
       * distances of the nodes.
       *
       * Returns a const reference to the node map that stores the distances
       * of the nodes calculated by the algorithm.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      const DistMap& distMap() const { return *_dist; }

      /**
       * @brief Returns a const reference to the node map that stores the
       * predecessor arcs.
       *
       * Returns a const reference to the node map that stores the predecessor
       * arcs, which form the shortest path tree (forest).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      const PredMap& predMap() const { return *_pred; }

      /** Checks if the given node is reached from the root(s). */

      /**
       * Returns \c true if \c v is reached from the root(s).
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      bool reached(Node v) const { return (*_heap_cross_ref)[v] != Heap::PRE_HEAP; }

      /** Checks if a node is processed. */

      /**
       * Returns \c true if \c v is processed, i.e. the shortest
       * path to \c v has already found.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function.
       */
      bool processed(Node v) const { return (*_heap_cross_ref)[v] == Heap::POST_HEAP; }

      /** The current distance of the given node from the root(s). */

      /**
       * Returns the current distance of the given node from the root(s).
       * It may be decreased in the following processes.
       *
       * @pre Either @ref run(Node) "run()" or @ref init()
       * must be called before using this function and
       * node \c v must be reached but not necessarily processed.
       */
      Value currentDist(Node v) const { return processed(v) ? (*_dist)[v] : (*_heap)[v]; }

      /** @} */
    };

    /** Default traits class of dijkstra() function. */

    /**
     * Default traits class of dijkstra() function.
     * @tparam GR The type of the digraph.
     * @tparam LEN The type of the length map.
     */
    template < class GR, class LEN >
    struct DijkstraWizardDefaultTraits {
      /** The type of the digraph the algorithm runs on. */
      using Digraph = GR;
      /** The type of the map that stores the arc lengths. */

      /**
       * The type of the map that stores the arc lengths.
       * It must conform to the @ref concepts::ReadMap "ReadMap" concept.
       */
      using LengthMap = LEN;
      /** The type of the arc lengths. */
      using Value = typename LEN::Value;

      /** Operation traits for Dijkstra algorithm. */

      /**
       * This class defines the operations that are used in the algorithm.
       * @see DijkstraDefaultOperationTraits
       */
      using OperationTraits = DijkstraDefaultOperationTraits< Value >;

      /** The cross reference type used by the heap. */

      /**
       * The cross reference type used by the heap.
       * Usually it is \c Digraph::NodeMap<int>.
       */
      using HeapCrossRef = typename Digraph::template NodeMap< int >;
      /** Instantiates a @ref HeapCrossRef. */

      /**
       * This function instantiates a @ref HeapCrossRef.
       * @param g is the digraph, to which we would like to define the
       * HeapCrossRef.
       */
      static HeapCrossRef* createHeapCrossRef(const Digraph& g) { return new HeapCrossRef(g); }

      /** The heap type used by the Dijkstra algorithm. */

      /**
       * The heap type used by the Dijkstra algorithm.
       *
       * \sa BinHeap
       * \sa Dijkstra
       */
      using Heap = BinHeap< Value, typename Digraph::template NodeMap< int >, std::less< Value > >;

      /** Instantiates a @ref Heap. */

      /**
       * This function instantiates a @ref Heap.
       * @param r is the HeapCrossRef which is used.
       */
      static Heap* createHeap(HeapCrossRef& r) { return new Heap(r); }

      /**
       * @brief The type of the map that stores the predecessor
       * arcs of the shortest paths.
       *
       * The type of the map that stores the predecessor
       * arcs of the shortest paths.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using PredMap = typename Digraph::template NodeMap< typename Digraph::Arc >;
      /** Instantiates a PredMap. */

      /**
       * This function instantiates a PredMap.
       * @param g is the digraph, to which we would like to define the
       * PredMap.
       */
      static PredMap* createPredMap(const Digraph& g) { return new PredMap(g); }

      /**
       * The type of the map that indicates which nodes are processed.
       */

      /**
       * The type of the map that indicates which nodes are processed.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       * By default, it is a NullMap.
       */
      using ProcessedMap = NullMap< typename Digraph::Node, bool >;
      /** Instantiates a ProcessedMap. */

      /**
       * This function instantiates a ProcessedMap.
       * @param g is the digraph, to which
       * we would like to define the ProcessedMap.
       */
#ifdef DOXYGEN
      static ProcessedMap* createProcessedMap(const Digraph& g)
#else
      static ProcessedMap* createProcessedMap(const Digraph&)
#endif
      {
        return new ProcessedMap();
      }

      /** The type of the map that stores the distances of the nodes. */

      /**
       * The type of the map that stores the distances of the nodes.
       * It must conform to the @ref concepts::WriteMap "WriteMap" concept.
       */
      using DistMap = typename Digraph::template NodeMap< typename LEN::Value >;
      /** Instantiates a DistMap. */

      /**
       * This function instantiates a DistMap.
       * @param g is the digraph, to which we would like to define
       * the DistMap
       */
      static DistMap* createDistMap(const Digraph& g) { return new DistMap(g); }

      /** The type of the shortest paths. */

      /**
       * The type of the shortest paths.
       * It must conform to the @ref concepts::Path "Path" concept.
       */
      using Path = nt::graphs::Path< Digraph >;
    };

    /** Default traits class used by DijkstraWizard */

    /**
     * Default traits class used by DijkstraWizard.
     * @tparam GR The type of the digraph.
     * @tparam LEN The type of the length map.
     */
    template < typename GR, typename LEN >
    class DijkstraWizardBase: public DijkstraWizardDefaultTraits< GR, LEN > {
      using Base = DijkstraWizardDefaultTraits< GR, LEN >;

      protected:
      // The type of the nodes in the digraph.
      using Node = typename Base::Digraph::Node;

      // Pointer to the digraph the algorithm runs on.
      void* _g;
      // Pointer to the length map.
      void* _length;
      // Pointer to the map of processed nodes.
      void* _processed;
      // Pointer to the map of predecessors arcs.
      void* _pred;
      // Pointer to the map of distances.
      void* _dist;
      // Pointer to the shortest path to the target node.
      void* _path;
      // Pointer to the distance of the target node.
      void* _di;

      public:
      /** Constructor. */

      /**
       * This constructor does not require parameters, therefore it initiates
       * all of the attributes to \c 0.
       */
      DijkstraWizardBase() : _g(0), _length(0), _processed(0), _pred(0), _dist(0), _path(0), _di(0) {}

      /** Constructor. */

      /**
       * This constructor requires two parameters,
       * others are initiated to \c 0.
       * @param g The digraph the algorithm runs on.
       * @param l The length map.
       */
      DijkstraWizardBase(const GR& g, const LEN& l) :
          _g(reinterpret_cast< void* >(const_cast< GR* >(&g))),
          _length(reinterpret_cast< void* >(const_cast< LEN* >(&l))), _processed(0), _pred(0), _dist(0), _path(0),
          _di(0) {}
    };

    /**
     * Auxiliary class for the function-type interface of Dijkstra algorithm.
     */

    /**
     * This auxiliary class is created to implement the
     * @ref dijkstra() "function-type interface" of @ref Dijkstra algorithm.
     * It does not have own @ref run(Node) "run()" method, it uses the
     * functions and features of the plain @ref Dijkstra.
     *
     * This class should only be used through the @ref dijkstra() function,
     * which makes it easier to use the algorithm.
     *
     * @tparam TR The traits class that defines various types used by the
     * algorithm.
     */
    template < class TR >
    class DijkstraWizard: public TR {
      using Base = TR;

      using Digraph = typename TR::Digraph;

      using Node = typename Digraph::Node;
      using NodeIt = typename Digraph::NodeIt;
      using Arc = typename Digraph::Arc;
      using OutArcIt = typename Digraph::OutArcIt;

      using LengthMap = typename TR::LengthMap;
      using Value = typename LengthMap::Value;
      using PredMap = typename TR::PredMap;
      using DistMap = typename TR::DistMap;
      using ProcessedMap = typename TR::ProcessedMap;
      using Path = typename TR::Path;
      using Heap = typename TR::Heap;

      public:
      /** Constructor. */
      DijkstraWizard() : TR() {}

      /** Constructor that requires parameters. */

      /**
       * Constructor that requires parameters.
       * These parameters will be the default values for the traits class.
       * @param g The digraph the algorithm runs on.
       * @param l The length map.
       */
      DijkstraWizard(const Digraph& g, const LengthMap& l) : TR(g, l) {}

      /** Copy constructor */
      DijkstraWizard(const TR& b) : TR(b) {}

      ~DijkstraWizard() noexcept {}

      /** Runs Dijkstra algorithm from the given source node. */

      /**
       * This method runs %Dijkstra algorithm from the given source node
       * in order to compute the shortest path to each node.
       */
      void run(Node s) {
        DijkstraLegacy< Digraph, LengthMap, TR > dijk(*reinterpret_cast< const Digraph* >(Base::_g),
                                                      *reinterpret_cast< const LengthMap* >(Base::_length));
        if (Base::_pred) dijk.predMap(*reinterpret_cast< PredMap* >(Base::_pred));
        if (Base::_dist) dijk.distMap(*reinterpret_cast< DistMap* >(Base::_dist));
        if (Base::_processed) dijk.processedMap(*reinterpret_cast< ProcessedMap* >(Base::_processed));
        dijk.run(s);
      }

      /** Finds the shortest path between \c s and \c t. */

      /**
       * This method runs the %Dijkstra algorithm from node \c s
       * in order to compute the shortest path to node \c t
       * (it stops searching when \c t is processed).
       *
       * @return \c true if \c t is reachable form \c s.
       */
      bool run(Node s, Node t) {
        DijkstraLegacy< Digraph, LengthMap, TR > dijk(*reinterpret_cast< const Digraph* >(Base::_g),
                                                      *reinterpret_cast< const LengthMap* >(Base::_length));
        if (Base::_pred) dijk.predMap(*reinterpret_cast< PredMap* >(Base::_pred));
        if (Base::_dist) dijk.distMap(*reinterpret_cast< DistMap* >(Base::_dist));
        if (Base::_processed) dijk.processedMap(*reinterpret_cast< ProcessedMap* >(Base::_processed));
        dijk.run(s, t);
        if (Base::_path) *reinterpret_cast< Path* >(Base::_path) = dijk.path(t);
        if (Base::_di) *reinterpret_cast< Value* >(Base::_di) = dijk.dist(t);
        return dijk.reached(t);
      }

      template < class T >
      struct SetPredMapBase: public Base {
        using PredMap = T;
        static PredMap* createPredMap(const Digraph&) { return 0; };
        SetPredMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the predecessor map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that stores the predecessor arcs of the nodes.
       */
      template < class T >
      DijkstraWizard< SetPredMapBase< T > > predMap(const T& t) {
        Base::_pred = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DijkstraWizard< SetPredMapBase< T > >(*this);
      }

      template < class T >
      struct SetDistMapBase: public Base {
        using DistMap = T;
        static DistMap* createDistMap(const Digraph&) { return 0; };
        SetDistMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-templ-param "Named parameter" for setting
       * the distance map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that stores the distances of the nodes calculated
       * by the algorithm.
       */
      template < class T >
      DijkstraWizard< SetDistMapBase< T > > distMap(const T& t) {
        Base::_dist = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DijkstraWizard< SetDistMapBase< T > >(*this);
      }

      template < class T >
      struct SetProcessedMapBase: public Base {
        using ProcessedMap = T;
        static ProcessedMap* createProcessedMap(const Digraph&) { return 0; };
        SetProcessedMapBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-func-param "Named parameter" for setting
       * the processed map.
       *
       * @ref named-templ-param "Named parameter" function for setting
       * the map that indicates which nodes are processed.
       */
      template < class T >
      DijkstraWizard< SetProcessedMapBase< T > > processedMap(const T& t) {
        Base::_processed = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DijkstraWizard< SetProcessedMapBase< T > >(*this);
      }

      template < class T >
      struct SetPathBase: public Base {
        using Path = T;
        SetPathBase(const TR& b) : TR(b) {}
      };

      /**
       * @brief @ref named-func-param "Named parameter"
       * for getting the shortest path to the target node.
       *
       * @ref named-func-param "Named parameter"
       * for getting the shortest path to the target node.
       */
      template < class T >
      DijkstraWizard< SetPathBase< T > > path(const T& t) {
        Base::_path = reinterpret_cast< void* >(const_cast< T* >(&t));
        return DijkstraWizard< SetPathBase< T > >(*this);
      }

      /**
       * @brief @ref named-func-param "Named parameter"
       * for getting the distance of the target node.
       *
       * @ref named-func-param "Named parameter"
       * for getting the distance of the target node.
       */
      DijkstraWizard dist(const Value& d) {
        Base::_di = reinterpret_cast< void* >(const_cast< Value* >(&d));
        return *this;
      }
    };

    /** Function-type interface for Dijkstra algorithm. */

    /**
     * @ingroup shortest_path
     * Function-type interface for Dijkstra algorithm.
     *
     * This function also has several @ref named-func-param "named parameters",
     * they are declared as the members of class @ref DijkstraWizard.
     * The following examples show how to use these parameters.
     * @code
     * // Compute shortest path from node s to each node
     * dijkstra(g,length).predMap(preds).distMap(dists).run(s);
     *
     * // Compute shortest path from s to t
     * bool reached = dijkstra(g,length).path(p).dist(d).run(s,t);
     * @endcode
     * @warning Don't forget to put the @ref DijkstraWizard::run(Node) "run()"
     * to the end of the parameter list.
     * \sa DijkstraWizard
     * \sa Dijkstra
     */
    template < typename GR, typename LEN >
    DijkstraWizard< DijkstraWizardBase< GR, LEN > > dijkstra(const GR& digraph, const LEN& length) {
      return DijkstraWizard< DijkstraWizardBase< GR, LEN > >(digraph, length);
    }

    /**
     * @brief Standard Dijkstra's algorithm for computing shortest paths in a directed graph.
     *
     * This template alias provides a convenient interface to DijkstraBase with sensible defaults
     * for computing shortest paths from a source vertex to all other vertices, or to a specific
     * target vertex in a directed graph.
     *
     * **Features:**
     * - Supports single-source shortest paths to all vertices
     * - Supports single-source single-target shortest path with early termination
     * - Enumerates all shortest paths to a target using iterator-based API
     * - Configurable heap management (local or external)
     *
     * **Heap Management:**
     * - `use_local_heap = true` (default): Heap is automatically managed internally
     * - `use_local_heap = false`: Must provide external heap via `heap(Heap&)` before `run()`
     *
     * **Example usage:**
     *
     * @code
     *  // Create a directed graph
     *  SmartDigraph graph;
     *  // ... add nodes and arcs ...
     *
     *  // Create arc length map
     *  SmartDigraph::ArcMap<double> lengths(graph);
     *  // ... set lengths ...
     *
     *  // Instantiate Dijkstra with local heap (default)
     *  Dijkstra<SmartDigraph, double> dijkstra(graph);
     *
     *  // Compute shortest paths from source to all nodes
     *  dijkstra.run(source_node, lengths);
     *
     *  // Check if target is reachable and get distance
     *  if (dijkstra.reached(target_node)) {
     *    double distance = dijkstra.dist(target_node);
     *  }
     *
     *  // Get a single shortest path
     *  auto path = dijkstra.path(target_node);
     *
     *  // Iterate over all shortest paths to target
     *  for (const auto& path : dijkstra.paths(target_node)) {
     *    // Process each shortest path
     *    for (Arc arc : path) {
     *      // Process arc
     *    }
     *  }
     *
     *  // Example with external heap (avoid heap allocation when sharing)
     *  using Heap = OptimalHeap<double, SmartDigraph::StaticNodeMap<int>>;
     *  Heap shared_heap;
     *  Dijkstra<SmartDigraph, double, false> dijkstra_no_heap(graph);
     *  dijkstra_no_heap.heap(shared_heap).run(source_node, lengths);
     * @endcode
     *
     * @tparam GR The type of the directed graph.
     * @tparam LENGTH The type of the arc lengths (default: double).
     * @tparam FLAGS Flags controlling the behavior of the algorithm, such as heap allocation.
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see DijkstraBase
     * @see Dijkstra16Bits
     */
    template < typename GR,
               typename LENGTH = double,
               int FLAGS = DijkstraFlags::STANDARD,
               typename LM = typename GR::template ArcMap< LENGTH > >
    using Dijkstra = DijkstraBase< GR,
                                   FLAGS,
                                   OptimalHeap< LENGTH, typename GR::template StaticNodeMap< int > >,
                                   DijkstraDefaultOperations< LENGTH >,
                                   SmartArcGraph< GR >,
                                   LM >;

    /**
     * @brief Optimized Dijkstra's algorithm for 16-bit integer lengths with SIMD support.
     *
     * This template alias provides a specialized version of Dijkstra's algorithm optimized
     * for 16-bit unsigned integer lengths. When SIMD instructions are enabled (see ntconfig.h),
     * this implementation runs substantially faster than the standard Dijkstra with double
     * or float lengths due to:
     * - Smaller memory footprint (16-bit vs 32/64-bit)
     * - Better cache locality
     * - SIMD vectorization opportunities in OptimalHeap operations
     *
     * **Use this when:**
     * - Arc lengths are integers that fit in the range [0, 65535]
     * - Performance is critical (e.g., repeated shortest path computations)
     * - Memory efficiency is important (e.g., large graphs with many Dijkstra instances)
     *
     * **Heap Management:**
     * - `use_local_heap = true` (default): Heap is automatically managed internally
     * - `use_local_heap = false`: Must provide external heap via `heap(Heap&)` before `run()`
     *
     * **Example usage:**
     * @code
     *  SmartDigraph graph;
     *  // ... add nodes and arcs ...
     *
     *  // Use 16-bit lengths (must fit in [0, 65535])
     *  SmartDigraph::ArcMap<uint16_t> lengths(graph);
     *  // ... set lengths ...
     *
     *  Dijkstra16Bits<SmartDigraph> dijkstra(graph);
     *  dijkstra.run(source_node, lengths);
     *
     *  // Access results same as regular Dijkstra
     *  auto distance = dijkstra.dist(target_node);
     *  auto shortest_path = dijkstra.path(target_node);
     * @endcode
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling the behavior of the algorithm, such as heap allocation.
     *
     * @see Dijkstra
     * @see DijkstraBase
     */
    template < typename GR, int FLAGS = DijkstraFlags::STANDARD >
    using Dijkstra16Bits = DijkstraBase< GR,
                                         FLAGS,
                                         QuickHeap< std::uint16_t, typename GR::template StaticNodeMap< int > >,
                                         DijkstraDefaultOperations< std::uint16_t >,
                                         SmartArcGraph< GR >,
                                         typename GR::template ArcMap< std::uint16_t > >;

    // Other common variants
    template < typename GR,
               typename LENGTH = double,
               int FLAGS = DijkstraFlags::STANDARD | DijkstraFlags::BACKWARD,
               typename LM = typename GR::template ArcMap< LENGTH > >
    using DijkstraBackward = DijkstraBase< GR,
                                           FLAGS,
                                           OptimalHeap< LENGTH, typename GR::template StaticNodeMap< int > >,
                                           DijkstraDefaultOperations< LENGTH >,
                                           SmartArcGraph< GR >,
                                           LM >;

    template < typename GR, int FLAGS = DijkstraFlags::STANDARD | DijkstraFlags::BACKWARD >
    using Dijkstra16BitsBackward = DijkstraBase< GR,
                                                 FLAGS,
                                                 QuickHeap< std::uint16_t, typename GR::template StaticNodeMap< int > >,
                                                 DijkstraDefaultOperations< std::uint16_t >,
                                                 SmartArcGraph< GR >,
                                                 typename GR::template ArcMap< std::uint16_t > >;
  }   // namespace graphs
}   // namespace nt

#endif
