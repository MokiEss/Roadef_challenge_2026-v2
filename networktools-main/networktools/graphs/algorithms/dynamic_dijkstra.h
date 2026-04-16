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
 * @brief Dynamic Dijkstra's algorithm with incremental length updates
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DYNAMIC_DIJKSTRA_H_
#define _NT_DYNAMIC_DIJKSTRA_H_

#include "dijkstra.h"

namespace nt {
  namespace graphs {

    /**
     * @class DynamicDijkstraBase
     * @headerfile dynamic_dijkstra.h
     * @brief Dynamic Dijkstra's algorithm supporting incremental arc length updates Based on
     *        the Ramalingam-Reps (RR) algorithm described in [1]
     *
     * This class extends DijkstraBase to support efficient updates to the shortest path tree
     * when arc lengths change. Instead of recomputing all shortest paths from scratch, this
     * implementation uses an incremental algorithm that only recomputes the affected portions
     * of the shortest path tree.
     *
     * [1] Luciana S. Buriol, Mauricio G. C. Resende, Mikkel Thorup:
     *     "Speeding Up Dynamic Shortest-Path Algorithms."
     *     INFORMS J. Comput. 20(2): 191-204 (2008)
     *
     * **Usage Example:**
     * @code
     *  SmartDigraph graph;
     *  // ... add nodes and arcs ...
     *
     *  SmartDigraph::ArcMap<double> lengths(graph);
     *  // ... set initial lengths ...
     *
     *  DynamicDijkstra<SmartDigraph> dyn_dijkstra(graph, lengths);
     *  dyn_dijkstra.run(source);
     *
     *  // Get initial shortest distance
     *  double dist_before = dyn_dijkstra.dist(target);
     *
     *  // Update arc length incrementally
     *  dyn_dijkstra.updateArcLength(some_arc, new_length);
     *
     *  // Shortest paths are updated automatically
     *  double dist_after = dyn_dijkstra.dist(target);
     * @endcode
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling algorithm behavior (same as DijkstraBase).
     * @tparam HEAP The type of the heap data structure used for priority queue operations.
     * @tparam OPERATIONS A type defining operations performed on lengths.
     * @tparam SPGRAPH The type of the shortest path graph.
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see DijkstraBase
     * @see DynamicDijkstra
     * @see DynamicDijkstra16Bits
     */
    template < typename GR, int FLAGS, typename HEAP, typename OPERATIONS, typename SPGRAPH, typename LM >
    struct DynamicDijkstraBase: DijkstraBase< GR, FLAGS, HEAP, OPERATIONS, SPGRAPH, LM > {
      using Parent = DijkstraBase< GR, FLAGS, HEAP, OPERATIONS, SPGRAPH, LM >;
      using Digraph = Parent::Digraph;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using PredMapSelector = Parent::PredMapSelector;
      using PredMap = Parent::PredMap;

      using Operations = Parent::Operations;
      using Length = Parent::Length;
      using LengthMap = Parent::LengthMap;

      using DistMap = Parent::DistMap;
      using Heap = Parent::Heap;

      using PathIterator = Parent::PathIterator;
      using PathRange = Parent::PathRange;

      using Direction = Parent::Direction;

      using ShortestPathGraphTraverser = Parent::ShortestPathGraphTraverser;

      template < bool b_save_length >
      struct SnapshotBase {
        DynamicDijkstraBase&   _dd;
        BoolDynamicArray       _sp_arcs_filter;
        IntDynamicArray        _sp_out_deg;
        DynamicArray< Length > _dist_map;
        DynamicArray< Length > _length_map;
        PredMap                _pred_map;

        /**
         * @brief Construct a snapshot for the given DynamicDijkstraBase instance.
         *
         * @param spr Reference to the DynamicDijkstraBase instance to snapshot.
         */
        SnapshotBase(DynamicDijkstraBase& spr) noexcept : _dd(spr), _pred_map(_dd.digraph()) {}

        /**
         * @brief Save the current state of the algorithm.
         *
         * Captures the current shortest path graph structure and distance map.
         */
        void save() noexcept {
          assert(_dd._checkSpGraphConsistency());
          _sp_arcs_filter.copyFrom(_dd._sp_arcs_filter);
          _sp_out_deg.copyFrom(_dd._sp_out_deg);
          _dist_map.copyFrom(_dd._dist_map.container());
          if constexpr (b_save_length) _length_map.copyFrom(_dd.lengthMap().container());

          if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
            _pred_map.container().copyFrom(_dd._pred_map.container());
          } else {
            for (NodeIt v(_dd.digraph()); v != INVALID; ++v)
              _pred_map[v].copyFrom(_dd._pred_map[v]);
          }
        }

        /**
         * @brief Restore the algorithm to the previously saved state.
         *
         * Reverts all changes made to the shortest path graph and distance map
         * since the last call to save().
         */
        void restore() noexcept {
          _dd._sp_arcs_filter.copyFrom(_sp_arcs_filter);
          _dd._sp_out_deg.copyFrom(_sp_out_deg);
          _dd._dist_map.container().copyFrom(_dist_map);

          if constexpr (b_save_length) {
            assert(_dd._p_length_map);
            const_cast< LengthMap* >(_dd._p_length_map)->container().copyFrom(_length_map);
          }

          if constexpr (FLAGS & DijkstraFlags::SINGLE_PATH) {
            _dd._pred_map.container().copyFrom(_pred_map.container());
          } else {
            for (NodeIt v(_dd.digraph()); v != INVALID; ++v)
              _dd._pred_map[v].copyFrom(_pred_map[v]);
          }
          assert(_dd._checkSpGraphConsistency());
        }
      };

      /**
       * @brief Snapshot for saving and restoring algorithm state.
       *
       * This struct allows you to save the current state of the dynamic Dijkstra algorithm
       * and restore it later. This is useful for implementing rollback functionality when
       * testing different length modifications.
       *
       * **Example usage:**
       * @code
       *  DynamicDijkstra<SmartDigraph> dd(graph, lengths);
       *  dd.run(source);
       *
       *  // Create and save snapshot
       *  DynamicDijkstra<SmartDigraph>::Snapshot snapshot(dd);
       *  snapshot.save();
       *
       *  // Modify lengths
       *  dd.updateArcLength(arc1, new_length1);
       *  dd.updateArcLength(arc2, new_length2);
       *
       *  // Restore to previous state
       *  snapshot.restore();
       * @endcode
       */
      using Snapshot = SnapshotBase< true >;

      // Data structures for maintaining the shortest path graph (SPGraph)
      BoolDynamicArray            _sp_arcs_filter;   ///< Filter marking arcs belonging to shortest path graph
      IntDynamicArray             _sp_out_deg;       ///< Number of successor arcs in SPG for each node
      TrivialDynamicArray< Node > _affected_nodes;   ///< Set Q in RR algorithm - nodes affected by length changes

      /**
       * @brief Construct a DynamicDijkstraBase object for a given directed graph.
       *
       * Initializes the dynamic Dijkstra algorithm instance with automatic heap management.
       * Internal data structures are sized according to the graph's maximum node and arc IDs.
       *
       * @param digraph The directed graph on which to perform dynamic shortest path computations.
       * @param length_map A map providing lengths associated to the arcs in the graph.
       *
       * @note The graph and length map references are stored internally and must remain valid
       *       for the lifetime of this instance.
       */
      explicit DynamicDijkstraBase(const Digraph& digraph, const LengthMap& length_map) noexcept :
          Parent(digraph, length_map), _sp_arcs_filter(digraph.maxArcId() + 1, false),
          _sp_out_deg(digraph.maxNodeId() + 1, 0) {}

      /**
       * @brief Construct a DynamicDijkstraBase object with an external heap.
       *
       * This constructor allows sharing a single heap instance across multiple algorithm
       * instances to reduce memory allocation overhead.
       *
       * @param digraph The directed graph on which to perform dynamic shortest path computations.
       * @param length_map A map providing lengths associated to the arcs in the graph.
       * @param heap External heap instance to use for priority queue operations.
       *
       * @note The graph, length map, and heap references must remain valid for the lifetime
       *       of this instance.
       */
      explicit DynamicDijkstraBase(const Digraph& digraph, const LengthMap& length_map, Heap& heap) noexcept :
          Parent(digraph, length_map, heap), _sp_arcs_filter(digraph.maxArcId() + 1, false),
          _sp_out_deg(digraph.maxNodeId() + 1, 0) {}

      /**
       * @brief Compute initial shortest paths from a source vertex and build the shortest path graph.
       *
       * Performs the initial shortest path computation using Dijkstra's algorithm, then constructs
       * the shortest path graph (SPG) which will be maintained incrementally during subsequent
       * length updates.
       *
       * @param source The source vertex from which to compute shortest paths.
       *
       * @note This must be called before any length update operations.
       */
      void run(Node source) noexcept {
        Parent::run(source);
        _buildSPGraph();
      }

      /**
       * @brief Compute initial shortest paths from source (target parameter ignored).
       *
       * This overload exists for API compatibility but ignores the target parameter.
       * Dynamic Dijkstra always computes shortest paths to all reachable vertices.
       *
       * @param source The source vertex from which to compute shortest paths.
       * @param target Ignored (for API compatibility).
       */
      void run(Node source, Node) noexcept { run(source); }

      /**
       * @brief Update the length of an arc and incrementally update affected shortest paths.
       *
       * Changes the length of the specified arc and efficiently updates only the portions of
       * the shortest path tree that are affected by this change. This is much faster than
       * recomputing all shortest paths from scratch.
       *
       * @param arc The arc whose length is being updated.
       * @param new_length The new length value for the arc.
       * @param doBeforeSPGraphChange A callback function invoked before modifying the shortest path graph.
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note If new_length <= 0, the operation is skipped.
       * @note If new_length == current length, no operation is performed.
       * @note If new_length >= infinity, it's treated as an infinite length increase.
       */
      template < typename F >
      bool updateArcLength(Arc arc, Length new_length, F doBeforeSPGraphChange) noexcept {
        const Length old_metric = (*Parent::_p_length_map)[arc];
        if (new_length <= 0 || new_length == old_metric) return false;
        if (new_length >= Operations::infinity())
          return increaseArcLength(arc, Operations::infinity(), doBeforeSPGraphChange);
        assert(old_metric < Operations::infinity());
        const Length delta = new_length - old_metric;
        return delta < 0 ? decreaseArcLength(arc, -delta, doBeforeSPGraphChange)
                         : increaseArcLength(arc, delta, doBeforeSPGraphChange);
      }

      /**
       * @brief Update the length of an arc and incrementally update affected shortest paths.
       *
       * Changes the length of the specified arc and efficiently updates only the portions of
       * the shortest path tree that are affected by this change. This is much faster than
       * recomputing all shortest paths from scratch.
       *
       * @param arc The arc whose length is being updated.
       * @param new_length The new length value for the arc.
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note If new_length <= 0, the operation is skipped.
       * @note If new_length == current length, no operation is performed.
       * @note If new_length >= infinity, it's treated as an infinite length increase.
       */
      bool updateArcLength(Arc arc, Length new_length) noexcept {
        return updateArcLength(arc, new_length, []() {});
      }

      /**
       * @brief Decrease the length of an arc by the specified delta and update shortest paths.
       *
       * Reduces the arc length and efficiently updates the shortest path tree. When an arc
       * length decreases, it may create new or improved shortest paths. The algorithm identifies
       * and updates only the affected nodes.
       *
       * @param arc The arc whose length is being decreased.
       * @param delta The amount to decrease the length (must be >= 0).
       * @param doBeforeSPGraphChange A callback function invoked before modifying the shortest path graph.
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note The actual length is modified in-place in the length map.
       * @note If delta <= 0, the operation is skipped.
       * @note Resulting lengths are clamped to a minimum of 1 to avoid non-positive lengths.
       */
      template < typename F >
      bool decreaseArcLength(const Arc arc, const Length delta, F doBeforeSPGraphChange) noexcept {
        assert(delta >= 0);
        if (delta <= 0) return false;

        assert(Parent::_p_length_map);
        LengthMap& length_map = *const_cast< LengthMap* >(Parent::_p_length_map);
        length_map[arc] -= delta;
        if (delta >= Operations::infinity() || length_map[arc] <= 0) length_map[arc] = 1;

        return updateDecrease(arc, doBeforeSPGraphChange);
      }

      /**
       * @brief Decrease the length of an arc by the specified delta and update shortest paths.
       *
       * Reduces the arc length and efficiently updates the shortest path tree. When an arc
       * length decreases, it may create new or improved shortest paths. The algorithm identifies
       * and updates only the affected nodes.
       *
       * @param arc The arc whose length is being decreased.
       * @param delta The amount to decrease the length (must be >= 0).
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note The actual length is modified in-place in the length map.
       * @note If delta <= 0, the operation is skipped.
       * @note Resulting lengths are clamped to a minimum of 1 to avoid non-positive lengths.
       */
      bool decreaseArcLength(const Arc arc, const Length delta) noexcept {
        return decreaseArcLength(arc, delta, []() {});
      }

      /**
       * @brief Update shortest paths after decreasing the length of an arc.
       *
       * @param arc The arc whose length has been decreased.
       * @param doBeforeSPGraphChange A callback function invoked before modifying the shortest path graph.
       * @return True if the shortest path tree was modified, false otherwise.
       */
      template < typename F >
      bool updateDecrease(const Arc arc, F doBeforeSPGraphChange) noexcept {
        using SuccIterator = typename Direction::SuccIterator;
        using PredIterator = typename Direction::PredIterator;

        assert(Parent::_p_length_map);
        const LengthMap& length_map = *const_cast< LengthMap* >(Parent::_p_length_map);
        const Node       u = Direction::running(Parent::_digraph, arc);
        const Node       v = Direction::opposite(Parent::_digraph, arc);
        const Length     sum = Parent::_dist_map[u] + length_map[arc];

        // The distances do not change w.r.t. to this decrease, nothing to do
        if (Parent::_dist_map[v] < sum) return false;

        // If there is an alternative shortest path traversing uv, then add uv in SPGraph
        if (Parent::_dist_map[v] == sum) {
          doBeforeSPGraphChange();
          _addSPArc(arc);
          return true;
        }

        // Shortest path to v improved => update distance
        Parent::_dist_map[v] = sum;
        doBeforeSPGraphChange();

        Parent::heap().init(Parent::_digraph.maxNodeId() + 1);
        Parent::heap().push(v, Parent::_dist_map[v]);

        while (!Parent::heap().empty()) {
          const Node u = Parent::heap().top();
          Parent::heap().pop();

          for (PredIterator pred_arc(Parent::_digraph, u); pred_arc != INVALID; ++pred_arc) {
            const Node v = Direction::running(Parent::_digraph, pred_arc);
            const bool b_is_sp_arc = Parent::_dist_map[u] == Parent::_dist_map[v] + length_map[pred_arc];
            const bool b_has_sp_arc = _hasSPArc(pred_arc);
            if (b_is_sp_arc && !b_has_sp_arc) {
              // New shortest path to u found via pred_arc, add to SPGraph
              _addSPArc(pred_arc);
            } else if (!b_is_sp_arc && b_has_sp_arc) {
              // Remove pred_arc from SPGraph
              _removeSPArc(pred_arc);
            }
          }

          assert(_checkSpGraphConsistency());

          for (SuccIterator succ_arc(Parent::_digraph, u); succ_arc != INVALID; ++succ_arc) {
            const Node v = Direction::opposite(Parent::_digraph, succ_arc);
            if (_relaxArc(Parent::_dist_map, succ_arc, length_map[succ_arc]))
              Parent::heap().set(v, Parent::_dist_map[v]);
            else if (!_hasSPArc(succ_arc) && Parent::_dist_map[v] == Parent::_dist_map[u] + length_map[succ_arc])
              _addSPArc(succ_arc);
          }

          assert(_checkSpGraphConsistency());
        }

        return true;
      }

      /**
       * @brief Update shortest paths after decreasing the length of an arc.
       *
       * @param arc The arc whose length has been decreased.
       * @return True if the shortest path tree was modified, false otherwise.
       */
      bool updateDecrease(const Arc arc) noexcept {
        return updateDecrease(arc, []() {});
      }

      /**
       * @brief Increase the length of an arc by the specified delta and update shortest paths.
       *
       * Increases the arc length and efficiently updates the shortest path tree. When an arc
       * length increases, existing shortest paths through that arc may become invalid.
       *
       * @param arc The arc whose length is being increased.
       * @param delta The amount to increase the length (must be >= 0).
       * @param doBeforeSPGraphChange A callback function invoked before modifying the shortest path graph.
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note The actual length is modified in-place in the length map.
       * @note If delta <= 0, the operation is skipped.
       * @note If delta >= infinity, the arc length is set to infinity.
       * @note If the arc is not part of any shortest path, this operation is very fast (O(1)).
       */
      template < typename F >
      bool increaseArcLength(Arc arc, Length delta, F doBeforeSPGraphChange) noexcept {
        assert(delta >= 0);
        if (delta <= 0) return false;

        // Update the length of arc uv
        assert(Parent::_p_length_map);
        LengthMap& length_map = *const_cast< LengthMap* >(Parent::_p_length_map);
        if (delta < Operations::infinity())
          length_map[arc] += delta;
        else
          length_map[arc] = Operations::infinity();

        return updateIncrease(arc, doBeforeSPGraphChange);
      }

      /**
       * @brief Increase the length of an arc by the specified delta and update shortest paths.
       *
       * Increases the arc length and efficiently updates the shortest path tree. When an arc
       * length increases, existing shortest paths through that arc may become invalid.
       *
       * @param arc The arc whose length is being increased.
       * @param delta The amount to increase the length (must be >= 0).
       *
       * @return True if the shortest path tree was modified, false otherwise.
       *
       * @note The actual length is modified in-place in the length map.
       * @note If delta <= 0, the operation is skipped.
       * @note If delta >= infinity, the arc length is set to infinity.
       * @note If the arc is not part of any shortest path, this operation is very fast (O(1)).
       */
      bool increaseArcLength(Arc arc, Length delta) noexcept {
        return increaseArcLength(arc, delta, []() {});
      }

      /**
       * @brief Update shortest paths after increasing the length of an arc.
       *
       * @param arc The arc whose length has been increased.
       * @param doBeforeSPGraphChange A callback function invoked before modifying the shortest path graph.
       * @return True if the shortest path tree was modified, false otherwise.
       */
      template < typename F >
      bool updateIncrease(Arc arc, F doBeforeSPGraphChange) noexcept {
        using SuccIterator = typename Direction::SuccIterator;
        using PredIterator = typename Direction::PredIterator;

        // If the arc is not part of SPGraph then skip
        if (!_hasSPArc(arc)) return false;

        assert(_checkSpGraphConsistency());
        doBeforeSPGraphChange();
        _removeSPArc(arc);

        // if w has an alternative shortest path (i.e can still be reached from s) then nothing to be done
        const Node w = Direction::opposite(Parent::_digraph, arc);
        if (_predNum(w)) return true;

        // Phase 1: Identify the affected vertices and remove the affected arcs from SPGraph
        _affected_nodes.removeAll();
        _affected_nodes.add(w);
        Parent::_dist_map[w] = Operations::infinity();

        for (int k = 0; k < _affected_nodes.size(); ++k) {
          const Node u = _affected_nodes[k];
          for (SuccIterator succ_arc(Parent::_digraph, u); succ_arc != INVALID; ++succ_arc) {
            if (!_hasSPArc(succ_arc)) continue;
            _removeSPArc(succ_arc);

            const Node v = Direction::opposite(Parent::_digraph, succ_arc);
            if (!_spOutDegree(v)) {
              _affected_nodes.add(v);
              Parent::_dist_map[v] = Operations::infinity();
            }
          }
        }

        assert(Parent::_p_length_map);
        const LengthMap& length_map = *const_cast< LengthMap* >(Parent::_p_length_map);

        // Phase 2: Determine new distances from affected vertices to s and update SPGraph
        Parent::heap().init(Parent::_digraph.maxNodeId() + 1);
        for (const Node u: _affected_nodes) {
          // Check all predecessor arcs of u to update dist_map[u] with the shortest
          for (PredIterator pred_arc(Parent::_digraph, u); pred_arc != INVALID; ++pred_arc)
            _relaxArc(Parent::_dist_map, pred_arc, length_map[pred_arc]);
          if (Parent::_dist_map[u] < Operations::infinity()) Parent::heap().push(u, Parent::_dist_map[u]);
        }

        // The nodes u from the heap are removed one by one.
        while (!Parent::heap().empty()) {
          const Node u = Parent::heap().top();
          Parent::heap().pop();

          // All arcs e = (u,v) successor of node u are traversed. In case node v has a
          // shortest path traversing node u (i.e _relaxArc() returns true), its distance is
          // updated and heap is adjusted.
          for (SuccIterator succ_arc(Parent::_digraph, u); succ_arc != INVALID; ++succ_arc) {
            if (_relaxArc(Parent::_dist_map, succ_arc, length_map[succ_arc])) {
              const Node v = Direction::opposite(Parent::_digraph, succ_arc);
              Parent::heap().set(v, Parent::_dist_map[v]);
            }
          }

          // Add the missing arcs in SPGraph in the shortest paths from nodes u ∈ Q

          // In case that the updated shortest distance from u is inf then there is no
          // need to add uv arcs in SPGraph since every path starting from u will be
          // a shortest path with inf length => shortest path has no meaning in this case.
          if (Parent::_dist_map[u] == Operations::infinity()) continue;
          for (PredIterator pred_arc(Parent::_digraph, u); pred_arc != INVALID; ++pred_arc) {
            const Node v = Direction::running(Parent::_digraph, pred_arc);
            if (Parent::_dist_map[u] == Parent::_dist_map[v] + length_map[pred_arc]) _addSPArc(pred_arc);
          }
        }

        assert(_checkSpGraphConsistency());

        return true;
      }

      /**
       * @brief Update shortest paths after increasing the length of an arc.
       *
       * @param arc The arc whose length has been increased.
       * @return True if the shortest path tree was modified, false otherwise.
       */
      bool updateIncrease(const Arc arc) noexcept {
        return updateIncrease(arc, []() {});
      }

      /**
       * @brief Reset the internal data structures and state.
       *
       * Clears all distance information, shortest path graph structure, and affected nodes list.
       * After calling clear(), you must call run() again before performing length updates.
       */
      constexpr void clear() noexcept {
        Parent::clear();
        _affected_nodes.removeAll();
      }

      // Internal methods for shortest path graph manipulation

      /**
       * @brief Build the initial shortest path graph from current distance map.
       */
      constexpr void _buildSPGraph() noexcept {
        _clearSPGraph();
#if 0   // More straightforward but numerically less stable method
        LengthMap&          length_map = *const_cast< LengthMap* >(Parent::_p_length_map);
        Tolerance< Length > tol;
        for (ArcIt arc(Parent::_digraph); arc != INVALID; ++arc) {
          const Node u = Direction::running(Parent::_digraph, arc);
          const Node v = Direction::opposite(Parent::_digraph, arc);

          // If slack ≈ 0, then arc is in the shortest path graph
          const Length slack = Parent::_dist_map[v] - Parent::_dist_map[u] - length_map[arc];
          if (tol.isZero(slack)) _enableSPArc(arc);
        }
#else
        Queue< Node >    queue;
        BoolDynamicArray marked(Parent::_digraph.maxNodeId() + 1, false);

        marked.setTrue(Parent::_digraph.id(Parent::_source));
        for (NodeIt s(Parent::_digraph); s != nt::INVALID; ++s) {
          if (Parent::_source == s || marked.isTrue(Parent::_digraph.id(s))) continue;
          queue.clear();
          queue.push(s);
          marked.setTrue(Parent::_digraph.id(s));

          while (!queue.empty()) {
            const Node v = queue.front();
            queue.pop();
            for (int k = 0; k < Parent::numPred(v); ++k) {
              const Arc  in_arc = Parent::predArc(v, k);
              const Node w = Parent::_digraph.target(in_arc);
              _enableSPArc(in_arc);
              if (!marked.isTrue(Parent::_digraph.id(w))) {
                marked.setTrue(Parent::_digraph.id(w));
                queue.push(w);
              }
            }
          }
        }
#endif
      }

      /**
       * @brief Clear the shortest path graph data structures.
       */
      constexpr void _clearSPGraph() noexcept {
        _sp_arcs_filter.ensureAndFill(Parent::_digraph.maxArcId() + 1);
        _sp_arcs_filter.setFalse();
        _sp_out_deg.ensureAndFill(Parent::_digraph.maxNodeId() + 1);
        _sp_out_deg.setBitsToZero();
      }

      /**
       * @brief Check if an arc belongs to the shortest path graph.
       * @param arc The arc to check.
       * @return True if the arc is in the shortest path graph.
       */
      constexpr bool _hasSPArc(Arc arc) const noexcept { return _sp_arcs_filter.isTrue(Parent::_digraph.id(arc)); }

      /**
       * @brief Mark an arc as belonging to the shortest path graph (without updating predecessor map).
       * @param arc The arc to enable in the SPG.
       */
      constexpr void _enableSPArc(Arc arc) noexcept {
        assert(!_hasSPArc(arc));
        const int i_arc = Parent::_digraph.id(arc);
        _sp_arcs_filter.setTrue(i_arc);

        const Node u = Parent::_digraph.source(arc);
        ++_sp_out_deg[Parent::_digraph.id(u)];
      }

      /**
       * @brief Add an arc to the shortest path graph and update predecessor map.
       * @param arc The arc to add to the SPG.
       */
      constexpr void _addSPArc(Arc arc) noexcept {
        _enableSPArc(arc);
        const Node v = Direction::opposite(Parent::_digraph, arc);
        Parent::_pred_map[v].add(arc);
      }

      /**
       * @brief Remove an arc from the shortest path graph and update predecessor map.
       * @param arc The arc to remove from the SPG.
       */
      constexpr void _removeSPArc(Arc arc) noexcept {
        if (!_hasSPArc(arc)) return;   // if the arc is not present then return

        const int i_arc = Parent::_digraph.id(arc);
        _sp_arcs_filter.setFalse(i_arc);

        const Node u = Parent::_digraph.source(arc);
        assert(_sp_out_deg[Parent::_digraph.id(u)] > 0);
        --_sp_out_deg[Parent::_digraph.id(u)];

        const Node v = Direction::opposite(Parent::_digraph, arc);
        const int  i_arc_pos = Parent::_pred_map[v].find(arc);   // find() should be O(1) most of the time.
        assert(i_arc_pos < Parent::_pred_map[v].size());
        Parent::_pred_map[v].remove(i_arc_pos);
      }

      /**
       * @brief Remove all predecessor arcs of a node from the shortest path graph.
       * @param v The node whose predecessors should be removed.
       */
      constexpr void _removeAllPred(Node v) noexcept {
        for (const Arc pred_arc: Parent::_pred_map[v]) {
          const int i_arc = Parent::_digraph.id(pred_arc);
          _sp_arcs_filter.setFalse(i_arc);
          const Node u = Parent::_digraph.source(pred_arc);
          assert(_sp_out_deg[Parent::_digraph.id(u)] > 0);
          --_sp_out_deg[Parent::_digraph.id(u)];
        }
        Parent::_pred_map[v].removeAll();
      }

      /**
       * @brief Get the number of successor arcs in the shortest path graph for a node.
       * @param v The node to query.
       * @return Number of outgoing arcs in the SPG from node v.
       */
      constexpr int _spOutDegree(Node v) const noexcept { return _sp_out_deg[Parent::_digraph.id(v)]; }

      /**
       * @brief Get the number of predecessor arcs in the shortest path graph for a node.
       * @internal
       * @param v The node to query.
       * @return Number of incoming arcs in the SPG to node v.
       */
      constexpr int _predNum(Node v) const noexcept { return Parent::_pred_map[v].size(); }

      /**
       * @brief Attempt to relax an arc and update distance if a shorter path is found.
       * @param dist_map The distance map to update.
       * @param arc The arc to relax.
       * @param arc_length The length of the arc.
       * @return True if the distance was improved, false otherwise.
       */
      constexpr bool _relaxArc(DistMap& dist_map, Arc arc, Length arc_length) noexcept {
        const Node u = Direction::running(Parent::_digraph, arc);
        const Node v = Direction::opposite(Parent::_digraph, arc);

        if (dist_map[u] >= Operations::infinity() || arc_length >= Operations::infinity()) return false;
        const Length d = arc_length + dist_map[u];
        if (d < dist_map[v]) {
          dist_map[v] = d;
          return true;
        }

        return false;
      }

      constexpr bool _checkSpGraphConsistency() const noexcept {
        IntDynamicArray out_deg(Parent::_digraph.maxNodeId() + 1, 0);
        IntDynamicArray in_deg(Parent::_digraph.maxNodeId() + 1, 0);
        for (ArcIt arc(Parent::_digraph); arc != nt::INVALID; ++arc) {
          if (_hasSPArc(arc)) {
            const Node u = Parent::_digraph.source(arc);
            ++out_deg[Parent::_digraph.id(u)];
            const Node v = Parent::_digraph.target(arc);
            ++in_deg[Parent::_digraph.id(v)];
          }
        }

        for (NodeIt n(Parent::_digraph); n != nt::INVALID; ++n) {
          if (out_deg[Parent::_digraph.id(n)] != _spOutDegree(n)) {
            LOG_F(ERROR,
                  "Inconsistent out degree for node {}: expected {}, found {}",
                  Parent::_digraph.id(n),
                  out_deg[Parent::_digraph.id(n)],
                  _spOutDegree(n));
            return false;
          }
        }

        for (ArcIt arc(Parent::_digraph); arc != nt::INVALID; ++arc) {
          if (!_hasSPArc(arc)) continue;
          if constexpr (FLAGS & DijkstraFlags::BACKWARD) {
            const Node u = Parent::_digraph.source(arc);
            if (_predNum(u) != out_deg[Parent::_digraph.id(u)]) {
              LOG_F(ERROR,
                    "Inconsistent pred num for node {}: expected {}, found {}",
                    Parent::_digraph.id(u),
                    out_deg[Parent::_digraph.id(u)],
                    _predNum(u));
              return false;
            }
          } else {
            const Node v = Parent::_digraph.target(arc);
            if (_predNum(v) != in_deg[Parent::_digraph.id(v)]) {
              LOG_F(ERROR,
                    "Inconsistent pred num for node {}: expected {}, found {}",
                    Parent::_digraph.id(v),
                    in_deg[Parent::_digraph.id(v)],
                    _predNum(v));
              return false;
            }
          }
        }

        return true;
      }
    };

    /**
     * @brief Dynamic Dijkstra with forward direction (source to targets).
     *
     * Template alias for DynamicDijkstraBase configured for forward shortest path computation
     * (from source to all other nodes). Supports incremental length updates.
     *
     * @tparam GR The type of the directed graph.
     * @tparam LENGTH The type of the arc lengths (default: double).
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra
     */
    template < typename GR,
               typename LENGTH = double,
               int FLAGS = DijkstraFlags::STANDARD,
               typename LM = typename GR::template ArcMap< LENGTH > >
    using DynamicDijkstraForward =
       DynamicDijkstraBase< GR,
                            FLAGS,
                            OptimalHeap< LENGTH, typename GR::template StaticNodeMap< int > >,
                            DijkstraDefaultOperations< LENGTH >,
                            SmartArcGraph< GR >,
                            LM >;

    /**
     * @brief Dynamic Dijkstra with backward direction (targets to source).
     *
     * Template alias for DynamicDijkstraBase configured for backward shortest path computation
     * (from all nodes to a target). Supports incremental length updates.
     *
     * @tparam GR The type of the directed graph.
     * @tparam LENGTH The type of the arc lengths (default: double).
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra
     */
    template < typename GR,
               typename LENGTH = double,
               int FLAGS = DijkstraFlags::STANDARD,
               typename LM = typename GR::template ArcMap< LENGTH > >
    using DynamicDijkstraBackward =
       DynamicDijkstraBase< GR,
                            FLAGS,
                            OptimalHeap< LENGTH, typename GR::template StaticNodeMap< int > >,
                            DijkstraDefaultOperations< LENGTH >,
                            SmartArcGraph< GR >,
                            LM >;

    /**
     * @brief Optimized dynamic Dijkstra for 16-bit lengths with forward direction.
     *
     * Specialized version of DynamicDijkstraBase optimized for 16-bit unsigned integer lengths.
     * Provides better performance and memory efficiency for lengths in range [0, 65535].
     * Configured for forward shortest path computation.
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra16Bits
     */
    template < typename GR, int FLAGS = DijkstraFlags::STANDARD >
    using DynamicDijkstraForward16Bits =
       DynamicDijkstraBase< GR,
                            FLAGS,
                            OptimalHeap< std::uint16_t, typename GR::template StaticNodeMap< int > >,
                            DijkstraDefaultOperations< std::uint16_t >,
                            SmartArcGraph< GR >,
                            typename GR::template ArcMap< std::uint16_t > >;

    /**
     * @brief Optimized dynamic Dijkstra for 16-bit lengths with backward direction.
     *
     * Specialized version of DynamicDijkstraBase optimized for 16-bit unsigned integer lengths.
     * Provides better performance and memory efficiency for lengths in range [0, 65535].
     * Configured for backward shortest path computation.
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra16Bits
     */
    template < typename GR, int FLAGS = DijkstraFlags::STANDARD >
    using DynamicDijkstraBackward16Bits =
       DynamicDijkstraBase< GR,
                            FLAGS,
                            OptimalHeap< std::uint16_t, typename GR::template StaticNodeMap< int > >,
                            DijkstraDefaultOperations< std::uint16_t >,
                            SmartArcGraph< GR >,
                            typename GR::template ArcMap< std::uint16_t > >;

    /**
     * @brief Standard dynamic Dijkstra's algorithm for incremental shortest path updates.
     *
     * This is the primary template alias for dynamic shortest path computation with
     * incremental length updates. It provides a convenient interface with sensible defaults.
     *
     * **Features:**
     * - Incremental length increase/decrease operations
     * - Efficient updates to shortest path tree (faster than full recomputation)
     * - Snapshot/restore functionality
     * - All features from standard Dijkstra (path reconstruction, multi-path support)
     *
     * **Example usage:**
     * @code
     *  SmartDigraph graph;
     *  // ... build graph ...
     *
     *  SmartDigraph::ArcMap<double> lengths(graph);
     *  // ... set lengths ...
     *
     *  DynamicDijkstra<SmartDigraph> dyn_dijkstra(graph, lengths);
     *  dyn_dijkstra.run(source);
     *
     *  // Incrementally update lengths
     *  dyn_dijkstra.updateArcLength(arc1, new_length1);
     *  dyn_dijkstra.updateArcLength(arc2, new_length2);
     *
     *  // Shortest paths are automatically updated
     *  auto path = dyn_dijkstra.path(target);
     *  double dist = dyn_dijkstra.dist(target);
     * @endcode
     *
     * @tparam GR The type of the directed graph.
     * @tparam LENGTH The type of the arc lengths (default: double).
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     * @tparam LM A concepts::ReadMap "readable" arc map storing the length values.
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra16Bits
     * @see Dijkstra
     */
    template < typename GR,
               typename LENGTH = double,
               int FLAGS = DijkstraFlags::STANDARD,
               typename LM = typename GR::template ArcMap< LENGTH > >
    using DynamicDijkstra = DynamicDijkstraBase< GR,
                                                 FLAGS,
                                                 OptimalHeap< LENGTH, typename GR::template StaticNodeMap< int > >,
                                                 DijkstraDefaultOperations< LENGTH >,
                                                 SmartArcGraph< GR >,
                                                 LM >;

    /**
     * @brief Optimized dynamic Dijkstra's algorithm for 16-bit integer lengths.
     *
     * Specialized version of DynamicDijkstra optimized for 16-bit unsigned integer lengths.
     * When SIMD instructions are enabled, this provides significantly better performance
     * than DynamicDijkstra with double or float lengths.
     *
     * **Use this when:**
     * - Arc lengths are integers in range [0, 65535]
     * - Performance is critical (repeated updates)
     * - Memory efficiency is important
     *
     * **Example usage:**
     * @code
     *  SmartDigraph graph;
     *  // ... build graph ...
     *
     *  SmartDigraph::ArcMap<uint16_t> lengths(graph);
     *  // ... set lengths (must be in [0, 65535]) ...
     *
     *  DynamicDijkstra16Bits<SmartDigraph> dyn_dijkstra(graph, lengths);
     *  dyn_dijkstra.run(source);
     *
     *  // Fast incremental updates
     *  dyn_dijkstra.updateArcLength(arc, new_length);
     * @endcode
     *
     * @tparam GR The type of the directed graph.
     * @tparam FLAGS Flags controlling algorithm behavior (default: STANDARD).
     *
     * @see DynamicDijkstraBase
     * @see DynamicDijkstra
     * @see Dijkstra16Bits
     */
    template < typename GR, int FLAGS = DijkstraFlags::STANDARD >
    using DynamicDijkstra16Bits =
       DynamicDijkstraBase< GR,
                            FLAGS,
                            OptimalHeap< std::uint16_t, typename GR::template StaticNodeMap< int > >,
                            DijkstraDefaultOperations< std::uint16_t >,
                            SmartArcGraph< GR >,
                            typename GR::template ArcMap< std::uint16_t > >;

  }   // namespace graphs
}   // namespace nt

#endif
