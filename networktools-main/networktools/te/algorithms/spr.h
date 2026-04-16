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
 * @brief Implementation of the Shortest Path Routing (SPR) protocol.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SPR_H_
#define _NT_SPR_H_

#include "../../core/algebra.h"
#include "../../core/arrays.h"
#include "../../core/queue.h"

#include "../../graphs/algorithms/dijkstra.h"
#include "../../graphs/demand_graph.h"
#include "../../graphs/graph_element_set.h"
#include "../../graphs/edge_graph.h"

#include "../../core/quick_heap.h"

#include "ecmp.h"

namespace nt {
  namespace te {

    /**
     * @class ShortestPathRouting
     * @headerfile spr.h
     * @brief Shortest Path Routing protocol with Equal-Cost Multi-Path (ECMP) load balancing.
     *
     * This class simulates shortest path routing protocols (like OSPF, IS-IS) by routing traffic
     * along shortest paths in a network. When multiple equal-cost shortest paths exist between
     * source and destination, traffic is split equally among them (ECMP).
     *
     * Key features:
     * - **ECMP support**: Automatic equal-cost multi-path load balancing
     * - **Flow accumulation**: Tracks total flow on each link from all routed demands
     * - **Demand graph routing**: Routes entire traffic matrices efficiently
     * - **Saturation monitoring**: Calculate link utilization and maximum link utilization (MLU)
     * - **Flexible traversal**: Iterate over shortest path graphs with custom callbacks
     *
     * This implementation uses backward Dijkstra from each target, caching results for efficiency
     * when routing multiple demands to the same destination.
     *
     * Example usage:
     * @code
     * using Digraph = nt::graphs::SmartDigraph;
     * using DemandGraph = nt::graphs::DemandGraph<Digraph>;
     * 
     * // Create network with IGP metrics
     * Digraph network;
     * Digraph::ArcMap<double> metrics(network);
     * Digraph::ArcMap<double> capacities(network);
     * // ... populate network ...
     *
     * // Create demand graph
     * DemandGraph demands(network);
     * DemandGraph::ArcMap<double> demand_volumes(demands);
     * // ... add demands ...
     *
     * // Initialize shortest path routing
     * nt::te::ShortestPathRouting<Digraph, double, double> spr(network, metrics);
     *
     * // Route all demands using ECMP
     * int routed = spr.run(demands, demand_volumes);
     *
     * // Analyze results
     * double mlu = spr.maxSaturation(capacities);
     * std::cout << "Maximum Link Utilization: " << mlu << std::endl;
     *
     * // Get flow on specific link
     * for (auto arc : network.arcs()) {
     *   double flow = spr.flow(arc);
     *   double util = spr.saturation(arc, capacities[arc]);
     *   std::cout << "Arc flow: " << flow << ", utilization: " << util << std::endl;
     * }
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
    struct ShortestPathRouting {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS_EX(Digraph);
      using Float = FPTYPE;

      using MetricMap = MM;
      using Metric = typename MetricMap::Value;

      using Dijkstra = graphs::Dijkstra< Digraph,
                                         Metric,
                                         graphs::DijkstraFlags::BACKWARD |        //
                                            graphs::DijkstraFlags::LOCAL_HEAP |   //
                                            graphs::DijkstraFlags::PRED_MAP,
                                         MetricMap >;

      using Vector = VEC;
      using VecTraits = VectorTraits< VEC >;
      using DemandVector = Vector;
      using FlowValue = Vector;
      using ArcFlowMap = typename Digraph::template ArcMap< FlowValue >;
      using NodeFlowMap = typename Digraph::template NodeMap< FlowValue >;

      using Ecmp = te::EcmpBase< Dijkstra, Vector, ArcFlowMap, NodeFlowMap >;

      using PredMap = typename Dijkstra::PredMap;
      using SuccMap = typename Dijkstra::SuccMap;
      using DistMap = typename Dijkstra::DistMap;
      using Heap = typename Dijkstra::Heap;
      using SPGraph = typename Dijkstra::SPGraph;

      using ShortestPathGraphTraverser = typename Dijkstra::ShortestPathGraphTraverser;

      Dijkstra    _dijkstra;
      ArcFlowMap  _arc_flows;
      NodeFlowMap _node_flows;
      Ecmp        _ecmp;

      constexpr static Metric infinity = Dijkstra::Operations::infinity();

      explicit ShortestPathRouting(const Digraph& g, const MetricMap& mm) :
          _dijkstra(g, mm), _arc_flows(g), _node_flows(g) {
        clear();
      }

      /**
       * @brief Run the SPR algorithm i.e route all the demands of the provided demand graphs.
       *
       */
      template < typename DemandGraph, typename DemandValueMap >
      int run(const DemandGraph& dg, const DemandValueMap& dvm) noexcept {
        ZoneScoped;
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
       */
      template < typename DemandGraph, typename DemandValueMap, typename DemandsPerArc >
      int run(const DemandGraph& dg, const DemandValueMap& dvm, DemandsPerArc& dpa) noexcept {
        ZoneScoped;
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
          this->_addFlow(arc, f_split);
        });
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
             this->_addFlow(arc, f_split);
             dpa[arc].add(demand_arc);
           });
        return r;
      }

      /**
       * @brief Add a 'static' amount of flow on an arc. This flow is not routed through shortest path and thus will not
       * be impacted by length changes.
       *
       */
      constexpr void routeStaticFlow(const Arc arc, const FlowValue flow) noexcept {
        // ZoneScoped; // Generate compile time error when building with tracy : error: ‘__tracy_source_location448’
        // declared ‘static’ in ‘constexpr’ function
        this->_addFlow(arc, flow);
      }

      /**
       * @brief Remove a given amount of flow from source to target
       *
       */
      bool removeFlow(const Node source, const Node target, const FlowValue flow) noexcept {
        ZoneScoped;
        return routeFlow(source, target, -flow);
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
       */
      template < typename MAP >
      bool computeRatios(const Node source, const Node target, MAP& ratio_map) noexcept {
        ratio_map.setBitsToZero();
        return _ecmp.run(_dijkstra,
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
        return _ecmp.run(_dijkstra,
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
        return _ecmp.run(_dijkstra, source, target, flow, _node_flows, doVisitArc) != Ecmp::E_NO_PATH;
      }

      /**
       * @brief Check whether there exists a unique shortest path toward a given node
       *
       * @param v Node toward which we search for a unique path
       *
       * @return true if there is a unique shortest path toward node v, false otherwise
       */
      constexpr bool hasUniqueShortestPath(Node u, Node v) const noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u);
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
      constexpr bool hasUniqueShortestPath(Node u, Node v, TrivialDynamicArray< Arc >& path) const noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u, [&](const Arc a) { path.add(a); });
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
      constexpr bool hasUniqueShortestPath(Node u, Node v, F doVisitArc) const noexcept {
        return _runDijkstraIfNecessary(v).hasUniqueShortestPath(u, doVisitArc);
      }

      /**
       *  @brief Returns the digraph
       *
       */
      constexpr const Digraph& digraph() const noexcept { return _dijkstra.digraph(); }

      /**
       * @brief Returns the maximum link utilization (MLU) value
       *
       * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
       *
       */
      template < typename CM >
      constexpr Vector maxSaturation(const CM& cm) const noexcept {
        Vector f_mlu = 0.;
        for (ArcIt arc(_dijkstra.digraph()); arc != nt::INVALID; ++arc)
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
        for (ArcIt arc(_dijkstra.digraph()); arc != nt::INVALID; ++arc) {
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
      constexpr const typename Digraph::template StaticArcMap< FlowValue >& flows() const noexcept {
        return _arc_flows;
      }

      /**
       *  @brief Returns the saturations
       *
       */
      template < typename CM >
      constexpr typename Digraph::template StaticArcMap< FlowValue > saturations(const CM& cm) const noexcept {
        typename Digraph::template StaticArcMap< FlowValue > saturations(_dijkstra.digraph());
        for (ArcIt arc(_dijkstra.digraph()); arc != nt::INVALID; ++arc)
          saturations[arc] = saturation(arc, cm[arc]);
        return saturations;
      }

      /**
       *  @brief Returns the flow value of a given arc.
       *
       */
      constexpr FlowValue flow(Arc arc) const noexcept { return _flow(arc); }

      /**
       *  @brief
       *
       */
      constexpr const DistMap& distMap(Node t) const noexcept { return _dijkstra.distMap(); }

      /**
       * @brief Sets the metric map.
       *
       * @param m The metric map to set.
       * @return Reference to this DijkstraBase instance (for method chaining).
       */
      constexpr ShortestPathRouting& metricMap(const MetricMap& mm) noexcept {
        _dijkstra.metricMap(mm);
        return *this;
      }

      /**
       * @brief Get a reference to the metric map used for arc metrics.
       *
       * @return A reference to the metric map.
       */
      constexpr const MetricMap& metricMap() const noexcept { return _dijkstra.lengthMap(); }

      /**
       * @brief
       *
       */
      void clear() noexcept {
        ZoneScoped;
        _arc_flows.setBitsToZero();
        _node_flows.setBitsToZero();
      }

      // Ensures that the Dijkstra toward 'target' has been run and returns it
      Dijkstra& _runDijkstraIfNecessary(const Node target) noexcept {
        if (_dijkstra.source() != target) _dijkstra.run(target);
        return _dijkstra;
      }

      // Methods for setting/getting flows
      constexpr void      _addFlow(Arc arc, FlowValue f_flow) noexcept { _arc_flows[arc] += f_flow; }
      constexpr FlowValue _flow(Arc arc) const noexcept { return _arc_flows[arc]; }
    };

  }   // namespace te
}   // namespace nt

#endif