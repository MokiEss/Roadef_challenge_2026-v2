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
 * @brief Implementation of the Dynamic Shortest Path Routing (SPR) protocol.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_DYNAMIC_SPR_H_
#define _NT_DYNAMIC_SPR_H_

#include "../../core/algebra.h"
#include "../../core/arrays.h"
#include "../../core/queue.h"

#include "../../graphs/algorithms/dynamic_dijkstra.h"
#include "../../graphs/demand_graph.h"
#include "../../graphs/graph_element_set.h"
#include "../../graphs/edge_graph.h"

#include "ecmp.h"

namespace nt {
  namespace te {

    /**
     * @class DynamicShortestPathRouting
     * @headerfile dynamic_spr.h
     * @brief Dynamic Shortest Path Routing with efficient metric updates and ECMP.
     *
     * This class extends shortest path routing with **efficient dynamic updates** when link metrics
     * change. Unlike static routing that requires full recomputation, this implementation uses
     * incremental algorithms to update only affected paths, making it ideal for:
     * - **Network failure simulation**: Model link/node failures and traffic rerouting
     * - **Traffic engineering**: Test "what-if" scenarios with metric changes
     * - **Real-time optimization**: Adapt routing to changing network conditions
     * - **Resilience analysis**: Evaluate network robustness under failures
     *
     * Key features:
     * - **Incremental updates**: Only recomputes affected shortest paths after metric changes
     * - **Snapshot/restore**: Save routing state and rollback changes efficiently
     * - **ECMP support**: Automatic equal-cost multi-path load balancing
     * - **Performance**: Much faster than full recomputation for sparse metric changes
     *
     * The class maintains a Dynamic Dijkstra instance for each node, enabling efficient updates
     * when metrics change. Uses the algorithm from:
     * "A Dynamic Algorithm for Shortest Paths" by Ramalingam and Reps (1996)
     *
     * Example usage:
     * @code
     * using Digraph = nt::graphs::SmartDigraph;
     * using DemandGraph = nt::graphs::DemandGraph<Digraph>;
     *
     * // Setup network
     * Digraph network;
     * Digraph::ArcMap<double> metrics(network);
     * Digraph::ArcMap<double> capacities(network);
     * Digraph::Node n1 = network.addNode();
     * Digraph::Node n2 = network.addNode();
     * Digraph::Node n3 = network.addNode();
     * Digraph::Arc a12 = network.addArc(n1, n2); metrics[a12] = 10; capacities[a12] = 100;
     * Digraph::Arc a23 = network.addArc(n2, n3); metrics[a23] = 10; capacities[a23] = 100;
     * Digraph::Arc a13 = network.addArc(n1, n3); metrics[a13] = 20; capacities[a13] = 100;
     *
     * // Create demand graph
     * DemandGraph demands(network);
     * DemandGraph::ArcMap<double> volumes(demands);
     * DemandGraph::Arc d = demands.addArc(n1, n3); volumes[d] = 50.0;
     *
     * // Initialize dynamic routing
     * using DynamicShortestPathRouting = nt::te::DynamicShortestPathRouting<Digraph, double, double>;
     * DynamicShortestPathRouting dspr(network, metrics);
     * dspr.run(demands, volumes);
     *
     * std::cout << "Initial MLU: " << dspr.maxSaturation(capacities) << std::endl;
     *
     * // Save current state using snapshot
     * DynamicShortestPathRouting::Snapshot snapshot(dspr);
     * snapshot.save();
     *
     * // Simulate link failure by increasing metric to infinity
     * dspr.updateArcWeight(a23, DynamicShortestPathRouting::infinity);
     *
     * std::cout << "After failure MLU: " << dspr.maxSaturation(capacities) << std::endl;
     *
     * // Rollback changes using snapshot
     * snapshot.restore();
     * std::cout << "After restore MLU: " << dspr.maxSaturation(capacities) << std::endl;
     * @endcode
     *
     * @tparam GR The digraph type representing the network.
     * @tparam FPTYPE Floating-point type for metrics and flows (float, double, long double).
     * @tparam VEC Vector type for multi-period or multi-commodity flows (default: Vec<FPTYPE, 1>).
     * @tparam MM The metric map type (default: GR::ArcMap<FPTYPE>).
     */
    template < typename GR,
               typename FPTYPE = double,
               typename VEC = nt::Vec< FPTYPE, 1 >,
               typename MM = typename GR::template ArcMap< FPTYPE > >
    struct DynamicShortestPathRouting {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);
      using Float = FPTYPE;

      using MetricMap = MM;
      using Metric = typename MetricMap::Value;

      using DynamicDijkstra = graphs::DynamicDijkstra< Digraph,
                                                       Metric,
                                                       graphs::DijkstraFlags::BACKWARD |   //
                                                          graphs::DijkstraFlags::PRED_MAP,
                                                       MetricMap >;
      using Vector = VEC;
      using VecTraits = VectorTraits< VEC >;
      using DemandVector = Vector;
      using FlowValue = Vector;
      using ArcFlowMap = typename Digraph::template ArcMap< FlowValue >;
      using NodeFlowMap = typename Digraph::template NodeMap< FlowValue >;
      using DijkstraMap = typename Digraph::template NodeMap< DynamicDijkstra >;
      using Ecmp = te::EcmpBase< DynamicDijkstra, Vector, ArcFlowMap, NodeFlowMap >;

      using PredMap = typename DynamicDijkstra::PredMap;
      using SuccMap = typename DynamicDijkstra::SuccMap;
      using DistMap = typename DynamicDijkstra::DistMap;
      using Heap = typename DynamicDijkstra::Heap;
      using SPGraph = typename DynamicDijkstra::SPGraph;

      using ShortestPathGraphTraverser = typename DynamicDijkstra::ShortestPathGraphTraverser;

      /**
       * @brief Class to make a snapshot of the current shortest path routing and restore it later.
       *
       * Any weight increase/decrease modifications can be undone using the restore() function.
       *
       */
      struct Snapshot {
        using SnapshotDijkstra = typename DynamicDijkstra::template SnapshotBase< false >;
        DynamicShortestPathRouting& _dspr;

        DynamicArray< SnapshotDijkstra >     _dijkstras_snapshots;
        TrivialDynamicArray< FlowValue >     _arc_flows;
        TrivialMultiDimArray< FlowValue, 2 > _partial_arc_flows;
        DynamicArray< Metric >               _metric_map;

        Snapshot(DynamicShortestPathRouting& spr) noexcept : _dspr(spr) {}

        /**
         * \brief Make a snapshot.
         *
         * This function makes a snapshot of the current shortest path routing.
         * It can be called more than once. In case of a repeated call, the previous snapshot gets
         * lost.
         */
        void save() noexcept {
          _dspr._affected_spgraphs.clear();
          _dspr._affected_partial_flows.removeAll();

          _dijkstras_snapshots.removeAll();
          // Add snapshots in node id order to ensure correct indexing in restore()
          for (int i = 0; i <= _dspr.digraph().maxNodeId(); ++i) {
            SnapshotDijkstra dd_snapshot(_dspr._dijkstras[i]);
            dd_snapshot.save();
            _dijkstras_snapshots.add(std::move(dd_snapshot));
          }

          _partial_arc_flows.copyFrom(_dspr._partial_arc_flows);
          _arc_flows.copyFrom(_dspr._arc_flows.container());
          _metric_map.copyFrom(_dspr.metricMap().container());
        }

        /**
         * \brief Undo the changes until the last snapshot.
         *
         * This function undos the changes until the last snapshot created by save().
         */
        void restore() noexcept {
          const_cast< MetricMap& >(_dspr.metricMap()).container().copyFrom(_metric_map);

          if (_dspr._affected_spgraphs.empty()) {
            assert(_dspr._affected_partial_flows.empty());
            return;
          }

          for (const Node v: _dspr._affected_spgraphs)
            _dijkstras_snapshots[_dspr.digraph().id(v)].restore();

          for (const uint64_t concat: _dspr._affected_partial_flows) {
            Node t;
            Arc  arc;
            _dspr._getAffectedPartialFlows(concat, t, arc);
            _dspr._partial_arc_flows(_dspr.digraph().id(t), _dspr.digraph().id(arc)) =
               _partial_arc_flows(_dspr.digraph().id(t), _dspr.digraph().id(arc));
          }

          _dspr._arc_flows.container().copyFrom(_arc_flows);
          _dspr._affected_spgraphs.clear();
          _dspr._affected_partial_flows.removeAll();
        }
      };

      FixedSizeArray< DynamicDijkstra > _dijkstras;
      ArcFlowMap                        _arc_flows;
      NodeFlowMap                       _node_flows;
      Ecmp                              _ecmp;
      Heap                              _heap;

      BoolDynamicArray                        _sp_arcs_filter_backup;
      TrivialMultiDimArray< FlowValue, 2 >    _partial_arc_flows;
      TrivialMultiDimArray< DemandVector, 2 > _demand_matrix;
      graphs::NodeSet< Digraph >              _targets;

      // These attributes are used by snapshots to restore only affected spgraphs and partial flows
      // since last call to save()
      graphs::NodeSet< Digraph > _affected_spgraphs;
      Uint64DynamicArray         _affected_partial_flows;

      constexpr static Metric infinity = DynamicDijkstra::Operations::infinity();

      explicit DynamicShortestPathRouting(const Digraph& g, const MetricMap& mm) :
          _dijkstras(g.maxNodeId() + 1, g, mm, _heap), _arc_flows(g), _node_flows(g),
          _partial_arc_flows(g.maxNodeId() + 1, g.maxArcId() + 1), _demand_matrix(g.maxNodeId() + 1, g.maxNodeId() + 1),
          _targets(g), _affected_spgraphs(g) {
        clear();
      }

      /**
       * @brief Run the SPR algorithm i.e route all the demands of the provided demand graphs.
       *
       */
      template < typename DemandGraph, typename DemandValueMap >
      int run(const DemandGraph& dg, const DemandValueMap& dvm) noexcept {
        int i_num_routed = 0;
        for (typename DemandGraph::NodeIt target(dg); target != INVALID; ++target) {
          for (typename DemandGraph::InArcIt demand_arc(dg, target); demand_arc != INVALID; ++demand_arc) {
            const Node source = dg.source(demand_arc);
            i_num_routed += routeFlow(source, target, dvm[demand_arc]);
          }
        }
        return i_num_routed;
      }

      /**
       * @brief Run the SPR algorithm i.e route all the demands of the provided demand graphs.
       * Furthermore, the set of demands routing over each arc is stored in the passed arc map 'dpa'
       * i.e dpa[a] is an array containing the demands using arc 'a' (dpa : demands per arc)
       *
       * WARNING: dpa is NOT updated after a call to increaseArcWeight()/decreaseArcWeight().
       *
       */
      template < typename DemandGraph, typename DemandValueMap, typename DemandsPerArc >
      int run(const DemandGraph& dg, const DemandValueMap& dvm, DemandsPerArc& dpa) noexcept {
        int i_num_routed = 0;
        for (typename DemandGraph::NodeIt target(dg); target != INVALID; ++target) {
          for (typename DemandGraph::InArcIt demand_arc(dg, target); demand_arc != INVALID; ++demand_arc) {
            const Node source = dg.source(demand_arc);
            i_num_routed += routeFlow(source, target, dvm[demand_arc], demand_arc, dpa);
          }
        }
        return i_num_routed;
      }

      /**
       * @brief Route a given volume of flow along the shortest paths from source to target with the
       * ECMP split rule.
       *
       */
      bool routeFlow(const Node source, const Node target, const FlowValue flow) noexcept {
        ZoneScoped;
        const bool r = ecmpTraversal(source, target, flow, [this, target](const Arc arc, const FlowValue f_split) {
          this->_addPartialFlow(target, arc, f_split);
          this->_addFlow(arc, f_split);
        });
        if (r) _demand_matrix(digraph().id(source), digraph().id(target)) += flow;
        return r;
      }

      /**
       * @brief Route a given volume of flow along the shortest paths from source to target with the
       * ECMP split rule.
       * Furthermore, the set of demands routing over each arc is stored in the passed arc map 'dpa'
       * i.e dpa[a] is an array containing the demands using arc 'a' (dpa : demands per arc)
       *
       * TODO: The arguments list should be simply :
       *          routeFlow(const DemandArc demand_arc, const FlowValue flow, DemandsPerArc& dpa)
       *       but we need to get the source and target of demand_arc, which would require to pass a ref to
       *       the demand graph...
       *
       */
      template < typename DemandArc, typename DemandsPerArc >
      bool routeFlow(const Node      source,
                     const Node      target,
                     const FlowValue flow,
                     const DemandArc demand_arc,
                     DemandsPerArc&  dpa) noexcept {
        ZoneScoped;
        const bool r = ecmpTraversal(
           source, target, flow, [this, target, &dpa, demand_arc](const Arc arc, const FlowValue f_split) {
             this->_addPartialFlow(target, arc, f_split);
             this->_addFlow(arc, f_split);
             dpa[arc].add(demand_arc);
           });
        if (r) _demand_matrix(digraph().id(source), digraph().id(target)) += flow;
        return r;
      }

      /**
       * @brief Add a 'static' amount of flow on an arc. This flow is not routed through shortest path and thus will not
       * be impacted by metric changes.
       *
       */
      constexpr void routeStaticFlow(const Arc arc, const FlowValue flow) noexcept {
        // ZoneScoped; // Generate compile time error when building with tracy : error: ‘__tracy_source_location448’
        // declared ‘static’ in ‘constexpr’ function
        this->_addFlow(arc, flow);
      }

      /**
       * @brief Remove all the flow from source to target
       *
       */
      bool removeAllFlow(const Node source, const Node target) noexcept {
        ZoneScoped;
        FlowValue& flow = _demand_matrix(digraph().id(source), digraph().id(target));
        const bool r = routeFlow(source, target, -flow);
        if (r) flow = 0;
        return r;
      }

      /**
       * @brief Remove a given amount of flow from source to target
       *
       */
      bool removeFlow(const Node source, const Node target, const FlowValue delta) noexcept {
        ZoneScoped;
        FlowValue& flow = _demand_matrix(digraph().id(source), digraph().id(target));
        if (delta >= flow) removeAllFlow(source, target);
        const bool r = routeFlow(source, target, -delta);
        if (r) flow -= delta;
        return r;
      }

      /**
       * @brief Function that iterates over the nodes and arcs of an SPGraph and apply 'doVisitNode' & 'doVisitArc'
       * callbacks along the way
       *
       * @param source The source node
       * @param target The target node
       * @param doVisitNode Function to be called every time a vertex is traversed for the first time
       * @param doVisitArc Function to be called every time an arc is traversed for the first time
       * @param traverser The SPGraph traverser to use
       *
       */
      template < typename F, typename G >
      bool traversal(const Node                  source,
                     const Node                  target,
                     F                           doVisitNode,
                     G                           doVisitArc,
                     ShortestPathGraphTraverser& traverser) noexcept {
        if (target == source) return false;
        return traverser.traverse(_runDijkstraIfNecessary(target).predMap(), source, target, doVisitNode, doVisitArc);
      }

      template < typename F, typename G >
      bool traversal(const Node source, const Node target, F doVisitNode, G doVisitArc) noexcept {
        ShortestPathGraphTraverser traverser(digraph());
        return traversal(source, target, doVisitNode, doVisitArc, traverser);
      }

      /**
       * @brief Function that iterates over the nodes of an SPGraph and apply 'doVisitNode' callbacks along the way
       *
       * @param source The source node
       * @param target The target node
       * @param doVisitNode Function to be called every time a vertex is traversed for the first time
       * @param traverser The SPGraph traverser to use
       * @param arcOk Optional arc filter; an arc is traversed only if arcOk(arc) returns true
       *
       */
      template < typename F, typename G = nt::True >
      bool forEachNode(const Node                  source,
                       const Node                  target,
                       F                           doVisitNode,
                       ShortestPathGraphTraverser& traverser,
                       G                           arcOk = {}) noexcept {
        return traversal(source, target, doVisitNode, arcOk, traverser);
      }

      /**
       * @brief Function that iterates over the nodes of an SPGraph and apply 'doVisitNode' callbacks along the way
       *
       * @param source The source node
       * @param target The target node
       * @param doVisitNode Function to be called every time a vertex is traversed for the first time
       * @param arcOk Optional arc filter; an arc is traversed only if arcOk(arc) returns true
       *
       */
      template < typename F, typename G = nt::True >
      bool forEachNode(const Node source, const Node target, F doVisitNode, G arcOk = {}) noexcept {
        ShortestPathGraphTraverser traverser(digraph());
        return forEachNode(source, target, doVisitNode, traverser, arcOk);
      }

      /**
       * @brief Function that iterates over the arcs of an SPGraph and apply 'doVisitArc' callbacks along the way
       *
       * @param source The source node
       * @param target The target node
       * @param doVisitArc Function to be called every time an arc is traversed for the first time
       * @param traverser The SPGraph traverser to use
       * @param arcOk Optional arc filter; an arc is traversed only if arcOk(arc) returns true
       *
       */
      template < typename G >
      bool forEachArc(const Node                  source,
                      const Node                  target,
                      G                           doVisitArc,
                      ShortestPathGraphTraverser& traverser) noexcept {
        return traversal(
           source, target, [](Digraph::Node) { return true; }, doVisitArc, traverser);
      }

      /**
       * @brief Function that iterates over the arcs of an SPGraph and apply 'doVisitArc' callbacks along the way
       *
       * @param source The source node
       * @param target The target node
       * @param doVisitArc Function to be called every time an arc is traversed for the first time
       *
       */
      template < typename G >
      bool forEachArc(const Node source, const Node target, G doVisitArc) noexcept {
        ShortestPathGraphTraverser traverser(digraph());
        return forEachArc(source, target, doVisitArc, traverser);
      }

      /**
       * @brief Compute the split factor (or ratio) of every arc that belongs to the SPGraph from 'source' to 'target'
       *
       * @param source The source node
       * @param target The target node
       * @param ratio_map Map that stores the computed ratios
       *
       * @return true if there exists a path from source to target, false otherwise
       *
       */
      template < typename MAP >
      bool computeRatios(const Node source, const Node target, MAP& ratio_map) noexcept {
        ratio_map.setBitsToZero();
        return _ecmp.run(_runDijkstraIfNecessary(target),
                         source,
                         target,
                         1,
                         _node_flows,
                         [&ratio_map](const Arc arc, const FlowValue f_split) { ratio_map[arc] += f_split; })
               != Ecmp::E_NO_PATH;
      }

      /**
       * @brief Compute the split factor (or ratio) of every arc that belongs to the SPGraph from 'source' to 'target'
       * Furthermore, 'sp_arcs' contains the arcs belonging to that SPGraph.
       *
       * @param source The source node
       * @param target The target node
       * @param ratio_map Map that stores the computed ratios
       * @param sp_arcs Arc array that contains the arcs of the SP graphs between 'source' and 'target'
       *
       * @return true if there exists a path from source to target, false otherwise
       *
       */
      template < typename MAP >
      bool computeRatios(const Node                  source,
                         const Node                  target,
                         MAP&                        ratio_map,
                         TrivialDynamicArray< Arc >& sp_arcs) noexcept {
        ratio_map.setBitsToZero();
        return _ecmp.run(_runDijkstraIfNecessary(target),
                         source,
                         target,
                         1,
                         _node_flows,
                         [&ratio_map, &sp_arcs](const Arc arc, const FlowValue f_split) {
                           ratio_map[arc] += f_split;
                           sp_arcs.add(arc);
                         })
               != Ecmp::E_NO_PATH;
      }

      /**
       * @brief Flow traversal with the ECMP split rule.
       *
       * @param source The source node
       * @param target The target node
       * @param flow The amount of flow to be routed from source to target
       * @param doVisitArc Callback function called every that time a new arc a is traversed. It
       * takes as input the arc and the total amount of flow routed along that arc. This function
       * returns nothing.
       *
       * @return true if the flow from source to target could be routed, false otherwise
       *
       */
      template < typename F >
      bool ecmpTraversal(const Node source, const Node target, const FlowValue flow, F doVisitArc) noexcept {
        return _ecmp.run(_runDijkstraIfNecessary(target), source, target, flow, _node_flows, doVisitArc)
               != Ecmp::E_NO_PATH;
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node
       *
       * @param u Source node
       * @param v Node toward which we search for a unique path
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      constexpr bool hasUniqueShortestPath(Node u, Node v) noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u);
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node and
       * stores the arcs belonging to that path if it exists.
       *
       * @param u Source node
       * @param v Node toward which we search for a unique path
       * @param path An array storing the arcs of the unique path if it exists
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      constexpr bool hasUniqueShortestPath(Node u, Node v, TrivialDynamicArray< Arc >& path) noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u, [&](const Arc a) { path.add(a); });
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node and
       * applies the function doVisitArc on every arc belonging to that path if it exists.
       *
       * @param u Source node
       * @param v Node toward which we search for a unique path
       * @param doVisitArc A function called on every arc of the unique path if it exists
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      template < typename F >
      constexpr bool hasUniqueShortestPath(Node u, Node v, F doVisitArc) noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u, doVisitArc);
      }

      /**
       *  @brief Compute the SPGraph(t)
       *
       */
      constexpr void spGraph(Node t, SPGraph& spg) const noexcept {
        for (ArcIt arc(digraph()); arc != nt::INVALID; ++arc) {
          if (!_hasSPArc(t, arc)) continue;
          spg.addArc(digraph().source(arc), digraph().target(arc));
        }
      }

      /**
       *@brief Set the weight of an arc to new_metric and update the shortest
       * path graphs and flows accordingly.
       *
       * @param arc Arc whose weight is updated
       * @param new_metric A positive metric value.
       *
       */
      bool updateArcWeight(Arc arc, Metric new_metric) noexcept {
        const MetricMap& weight_map = metricMap();
        const Metric     old_metric = weight_map[arc];
        if (new_metric <= 0 || new_metric == old_metric) return false;
        if (new_metric >= infinity) return increaseArcWeight(arc, infinity);
        assert(old_metric < infinity);
        const Metric delta = new_metric - old_metric;
        return delta < 0 ? decreaseArcWeight(arc, -delta) : increaseArcWeight(arc, delta);
      }

      /**
       * @brief Decrease the weight of an arc by delta and update the shortest
       * path graphs and flows accordingly.
       *
       * @param arc Arc whose weight is decreased
       * @param delta A positive real value  by which the weight is decreased. Setting delta to infinity will
       * set the weight of arc to 1. Similarly, if the provided delta decreases
       * the weight below zero it is set to 1.
       *
       */
      bool decreaseArcWeight(Arc arc, Metric delta) noexcept {
        if (delta <= 0) return false;
        MetricMap& weight_map = const_cast< MetricMap& >(metricMap());
        weight_map[arc] -= delta;
        if (delta >= infinity || weight_map[arc] <= 0) weight_map[arc] = 1;
        bool b_modified = false;
        for (const Node t: _targets) {
          if (_dijkstras[digraph().id(t)].updateDecrease(arc, [this, t]() {
                this->_saveSPGraph(t);
                this->_affected_spgraphs.insert(t);
              })) {
            _updateFlows(t);
            b_modified = true;
          }
        }
        return b_modified;
      }

      /**
       * @brief Increase the weight of an arc by delta and update the shortest
       * path graphs and flows accordingly.
       *
       * @param arc Arc whose weight is increased
       * @param delta A positive real value by which the weight is increased.
       *
       */
      bool increaseArcWeight(Arc arc, Metric delta) noexcept {
        if (delta <= 0) return false;

        MetricMap& weight_map = const_cast< MetricMap& >(metricMap());
        if (delta < infinity)
          weight_map[arc] += delta;
        else
          weight_map[arc] = infinity;

        bool b_modified = false;
        for (const Node t: _targets) {
          if (_dijkstras[digraph().id(t)].updateIncrease(arc, [this, t]() {
                this->_saveSPGraph(t);
                this->_affected_spgraphs.insert(t);
              })) {
            _updateFlows(t);
            b_modified = true;
          }
        }
        return b_modified;
      }

      /**
       *  @brief Returns the digraph
       *
       */
      constexpr const Digraph& digraph() const noexcept { return _dijkstras[0].digraph(); }

      /**
       * @brief Sets the metric map.
       *
       * @param m The metric map to set.
       * @return Reference to this DynamicShortestPathRouting instance (for method chaining).
       */
      constexpr DynamicShortestPathRouting& metricMap(const MetricMap& mm) noexcept {
        for (int i = 0; i < _dijkstras.size(); ++i)
          _dijkstras[i].lengthMap(mm);
        return *this;
      }

      /**
       *  @brief Returns the metric map
       *
       */
      constexpr const MetricMap& metricMap() const noexcept { return _dijkstras[0].lengthMap(); }
      constexpr MetricMap&       metricMap() noexcept { return const_cast< MetricMap& >(_dijkstras[0].lengthMap()); }

      /**
       * @brief Returns the maximum link utilization (MLU) value
       *
       * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
       *
       */
      template < typename CM >
      constexpr Vector maxSaturation(const CM& cm) const noexcept {
        Vector f_mlu = 0.;
        for (ArcIt arc(digraph()); arc != nt::INVALID; ++arc)
          f_mlu = VecTraits::maxVector(f_mlu, saturation(arc, cm[arc]));
        return f_mlu;
      }

      /**
       * @brief Returns the maximum loaded arc
       *
       * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
       *
       */
      template < typename CM >
      constexpr nt::Pair< Arc, Float > mostLoadedArc(const CM& cm) const noexcept {
        Float f_mlu = 0.;
        Arc   most_loaded_arc = nt::INVALID;
        for (ArcIt arc(digraph()); arc != nt::INVALID; ++arc) {
          const Float max_sat = VecTraits::max(saturation(arc, cm[arc]));
          if (max_sat > f_mlu) {
            f_mlu = max_sat;
            most_loaded_arc = arc;
          }
        }
        return {most_loaded_arc, f_mlu};
      }

      /**
       *  @brief Returns the saturation value in [0, 1] of an arc.
       *
       */
      constexpr Vector saturation(Arc arc, Float f_capacity) const noexcept { return flow(arc) / f_capacity; }

      /**
       *  @brief Returns the flow value
       *
       */
      constexpr const ArcFlowMap& flows() const noexcept { return _arc_flows; }

      /**
       *  @brief Returns the saturations
       *
       */
      template < typename CM >
      constexpr ArcFlowMap saturations(const CM& cm) const noexcept {
        ArcFlowMap saturations(digraph());
        for (ArcIt arc(digraph()); arc != nt::INVALID; ++arc)
          saturations[arc] = saturation(arc, cm[arc]);
        return saturations;
      }

      /**
       *  @brief Returns the flow value of a given arc.
       *
       */
      constexpr FlowValue flow(Arc arc) const noexcept { return _flow(arc); }

      /**
       *  @brief Returns the flow value traversing the given arc toward destination t.
       *
       */
      constexpr FlowValue partialFlow(Node t, Arc arc) const noexcept { return _partialFlow(t, arc); }

      /**
       *  @brief Returns the distance map toward a given node.
       *
       */
      constexpr const DistMap& distMap(Node t) const noexcept { return _dijkstras[Digraph::id(t)].distMap(); }

      /**
       * @brief Clears the dynamic shortest path routing data structures.
       *
       */
      void clear() noexcept {
        _arc_flows.setBitsToZero();
        _node_flows.setBitsToZero();
        _partial_arc_flows.setBitsToZero();
        _demand_matrix.setBitsToZero();
        _affected_partial_flows.removeAll();
      }

      /**
       * @brief Callback function invoked when the digraph changes (e.g., arcs/nodes are added/removed).
       *
       */
      void onDigraphChange() noexcept {
        _dijkstras = FixedSizeArray< DynamicDijkstra >(digraph().maxNodeId() + 1, digraph(), metricMap(), _heap);
        _partial_arc_flows.setDimensions(digraph().maxNodeId() + 1, digraph().maxArcId() + 1);
        _demand_matrix.setDimensions(digraph().maxNodeId() + 1, digraph().maxNodeId() + 1);
        _targets.build(digraph());
        clear();
      }

      // Private methods

      void _resetHeap() noexcept { _heap.init(digraph().maxNodeId() + 1); }

      // Ensures that the Dijkstra toward 'target' has been run and returns it
      DynamicDijkstra& _runDijkstraIfNecessary(const Node target) noexcept {
        if (!_targets.contains(target)) {
          _targets.insert(target);
          _dijkstras[Digraph::id(target)].run(target);
        }
        return _dijkstras[Digraph::id(target)];
      }

      // Update the flows toward destination t. _saveSPGraph() must be called before using this
      // function.
      void _updateFlows(const Node t) noexcept {
        using SuccIterator = typename DynamicDijkstra::Direction::SuccIterator;
        using PredIterator = typename DynamicDijkstra::Direction::PredIterator;

        _resetHeap();

        const DistMap& dist_map = _dijkstras[Digraph::id(t)].distMap();

        for (ArcIt arc(digraph()); arc != nt::INVALID; ++arc) {   // TODO : Iterate over the set of updated arcs only
          if (!_isUpdatedSPGraph(t, arc)) continue;
          const Node u = digraph().source(arc);
          const Node v = digraph().target(arc);
          if (_heap.state(u) == Heap::PRE_HEAP) _heap.push(u, -dist_map[u]);
          if (_heap.state(v) == Heap::PRE_HEAP) _heap.push(v, -dist_map[v]);
          _removeFlow(arc, _partialFlow(t, arc));
          _setPartialFlow(t, arc, 0);
          _addAffectedPartialFlows(t, arc);
        }

        while (!_heap.empty()) {
          const Node u = _heap.top();
          _heap.pop();

          FlowValue f_flow = _demand(u, t);

          if (_outDegreeSPNode(t, u)) {
            for (InArcIt in_arc(digraph(), u); in_arc != nt::INVALID; ++in_arc) {
              if (!_hasSPArc(t, in_arc)) continue;
              f_flow += _partialFlow(t, in_arc);
            }
            f_flow = f_flow / _outDegreeSPNode(t, u);
          }

          // TODO: replace with for(Arc out_arc : pred_map[u])
          for (OutArcIt out_arc(digraph(), u); out_arc != nt::INVALID; ++out_arc) {
            if (!_hasSPArc(t, out_arc)) continue;
            const bool is_arc_added = _isAddedSPGraph(t, out_arc);
            const bool has_flow_changed = f_flow != _partialFlow(t, out_arc);
            if (is_arc_added || has_flow_changed) {
              if (!is_arc_added) _removeFlow(out_arc, _partialFlow(t, out_arc));
              _addFlow(out_arc, f_flow);
              _setPartialFlow(t, out_arc, f_flow);
              _addAffectedPartialFlows(t, out_arc);
              const Node v = digraph().target(out_arc);
              if (_heap.state(v) == Heap::PRE_HEAP) _heap.push(v, -dist_map[v]);
            }
          }
        }
      }

      // Methods for manipulating the SP graphs
      constexpr bool _hasSPArc(Node t, Arc arc) const noexcept { return _dijkstras[Digraph::id(t)]._hasSPArc(arc); }
      constexpr void _enableSPArc(Node t, Arc arc) noexcept { _dijkstras[Digraph::id(t)]._enableSPArc(arc); }
      constexpr void _addSPArc(Node t, Arc arc) noexcept { _dijkstras[Digraph::id(t)]._addSPArc(arc); }
      constexpr void _removeSPArc(Node t, Arc arc) noexcept { _dijkstras[Digraph::id(t)]._removeSPArc(arc); }
      constexpr int  _outDegreeSPNode(Node t, Node v) const noexcept {
         return _dijkstras[Digraph::id(t)]._spOutDegree(v);
      }
      constexpr void _saveSPGraph(Node t) noexcept {
        _sp_arcs_filter_backup.copyFrom(_dijkstras[Digraph::id(t)]._sp_arcs_filter);
      }
      constexpr bool _isAddedSPGraph(Node t, Arc arc) const noexcept {
        return !_sp_arcs_filter_backup.isTrue(digraph().id(arc))
               && _dijkstras[Digraph::id(t)]._sp_arcs_filter.isTrue(digraph().id(arc));
      }
      constexpr bool _isRemovedSPGraph(Node t, Arc arc) const noexcept {
        return _sp_arcs_filter_backup.isTrue(digraph().id(arc))
               && !_dijkstras[Digraph::id(t)]._sp_arcs_filter.isTrue(digraph().id(arc));
      }
      constexpr bool _isUpdatedSPGraph(Node t, Arc arc) const noexcept {
        return _sp_arcs_filter_backup.isTrue(digraph().id(arc))
               != _dijkstras[Digraph::id(t)]._sp_arcs_filter.isTrue(digraph().id(arc));
      }

      // Demands
      constexpr DemandVector _demand(Node s, Node t) const noexcept {
        return _demand_matrix(digraph().id(s), digraph().id(t));
      }

      // Methods for setting/getting flows
      constexpr void _addFlow(Arc arc, FlowValue f_flow) noexcept { _arc_flows[arc] += f_flow; }
      constexpr void _removeFlow(Arc arc, FlowValue f_flow) noexcept {
        Vector& arc_flows = _arc_flows[arc];
        arc_flows -= f_flow;
        // TODO : Dirty fix to avoid numerical errors...use Kahan compensated sum ?
        if constexpr (std::is_floating_point_v< Vector >) {
          if (std::abs(arc_flows) < 0.001) arc_flows = 0.;
        } else {
          for (int k = 0; k < arc_flows.size(); ++k) {
            if (std::abs(arc_flows[k]) < 0.001) arc_flows[k] = 0.;
          }
        }
      }
      constexpr FlowValue _flow(Arc arc) const noexcept { return _arc_flows[arc]; }

      // Methods for setting/getting the partial flows
      constexpr void _setPartialFlow(Node t, Arc arc, FlowValue f_flow) noexcept {
        _partial_arc_flows(digraph().id(t), digraph().id(arc)) = f_flow;
      }
      constexpr void _addPartialFlow(Node t, Arc arc, FlowValue f_flow) noexcept {
        _partial_arc_flows(digraph().id(t), digraph().id(arc)) += f_flow;
      }
      constexpr FlowValue _partialFlow(Node t, Arc arc) const noexcept {
        return _partial_arc_flows(digraph().id(t), digraph().id(arc));
      }
      constexpr void _addAffectedPartialFlows(Node t, Arc arc) noexcept {
        _affected_partial_flows.add(NT_CONCAT_INT_32(digraph().id(t), digraph().id(arc)));
      }
      constexpr void _getAffectedPartialFlows(uint64_t concat, Node& t, Arc& arc) noexcept {
        t = digraph().nodeFromId(static_cast< int >(concat >> 32));
        arc = digraph().arcFromId(static_cast< int >((concat << 32) >> 32));
      }
    };

  }   // namespace te
}   // namespace nt

#endif