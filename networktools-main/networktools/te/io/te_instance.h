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
 * @brief Traffic Engineering instance container with network topology and traffic matrix.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_TE_INSTANCE_H_
#define _NT_TE_INSTANCE_H_

#include "../../graphs/smart_graph.h"
#include "../../graphs/algorithms/connectivity.h"
#include "../../graphs/io/sndlib_reader.h"
#include "../../graphs/io/networkx_io.h"
#include "../../graphs/io/repetita_io.h"

#include "../../core/algebra.h"

namespace nt {
  namespace te {

    /**
     * @class TeInstance
     * @headerfile te_instance.h
     * @brief Container for traffic engineering instances combining network topology and traffic matrix.
     *
     * This class provides a unified structure for loading traffic engineering instances from several file formats
     *
     * The class supports multiple input formats:
     * - **SNDLib format**: XML-based format from the Survivable Network Design Library
     * - **NetworkX JSON format**: JSON format compatible with NetworkX Python library
     * - **Repetita format**: Text-based format used by the Repetita TE framework
     *
     * Multi-period support: The STEP_NUM template parameter allows storing traffic demands across
     * multiple time periods (e.g., hourly traffic patterns). Use STEP_NUM=1 for single-period.
     *
     * Example usage:
     * @code
     * // Single-period instance with float precision
     * using Instance = nt::te::TeInstance<nt::graphs::SmartDigraph, float, 1>;
     * Instance instance;
     *
     * // Load from SNDLib XML file
     * if (instance.readSndlib("network.xml")) {
     *   std::cout << "Network nodes: " << instance.digraph().nodeNum() << std::endl;
     *   std::cout << "Network arcs: " << instance.digraph().arcNum() << std::endl;
     *   std::cout << "Demands: " << instance.demandGraph().arcNum() << std::endl;
     *
     *   // Access network properties
     *   for (auto arc : instance.digraph().arcs()) {
     *     double capacity = instance.capacity(arc);
     *     double metric = instance.metric(arc);
     *   }
     *
     *   // Access demands
     *   for (auto demand : instance.demandGraph().arcs()) {
     *     double volume = instance.demandValueAt(demand, 0);  // time period 0
     *   }
     * }
     *
     *
     * // Ensure bidirectional network (useful for OSPF/IS-IS routing)
     * instance.makeBidirected();
     * @endcode
     *
     * @tparam GR The digraph type for the network topology (e.g., SmartDigraph, ListDigraph).
     * @tparam FPTYPE Floating-point type for numerical values (float, double, long double).
     * @tparam STEP_NUM Number of time periods for multi-period traffic matrices (default: 1).
     * @tparam DG The demand graph type (default: graphs::DemandGraph<GR>).
     */
    template < typename GR, typename FPTYPE = float, int STEP_NUM = 1, typename DG = graphs::DemandGraph< GR > >
    struct TeInstance {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
      using Scalar = FPTYPE;
      using CapacityMap = typename Digraph::template ArcMap< Scalar >;
      using MetricMap = typename Digraph::template ArcMap< Scalar >;
      using DelayMap = typename Digraph::template ArcMap< Scalar >;
      using NameMap = typename Digraph::template NodeMap< nt::String >;
      using GeoPosMap = typename Digraph::template NodeMap< nt::DoubleDynamicArray >;

      using DemandGraph = DG;
      using DemandValueMap = typename DemandGraph::template ArcMap< nt::Vec< Scalar, STEP_NUM > >;
      using DemandArc = typename DemandGraph::Arc;

      Digraph     _digraph;
      DemandGraph _demand_graph;

      CapacityMap _capacities;
      MetricMap   _metrics;
      DelayMap    _delays;
      NameMap     _names;
      GeoPosMap   _geopos;

      DemandValueMap _dvm;

      TeInstance() :
          _demand_graph(_digraph), _capacities(_digraph), _metrics(_digraph), _delays(_digraph), _names(_digraph),
          _geopos(_digraph), _dvm(_demand_graph) {}
      TeInstance(const char* sz_filename) :
          _demand_graph(_digraph), _capacities(_digraph), _metrics(_digraph), _delays(_digraph), _names(_digraph),
          _geopos(_digraph), _dvm(_demand_graph) {
        readSndlib(sz_filename);
      }

      constexpr Scalar             capacity(Arc arc) const noexcept { return _capacities[arc]; }
      constexpr const CapacityMap& capacityMap() const noexcept { return _capacities; }
      constexpr CapacityMap&       capacityMap() noexcept { return _capacities; }
      constexpr Scalar             metric(Arc arc) const noexcept { return _metrics[arc]; }
      constexpr const MetricMap&   metricMap() const noexcept { return _metrics; }
      constexpr MetricMap&         metricMap() noexcept { return _metrics; }
      constexpr Scalar             delay(Arc arc) const noexcept { return _delays[arc]; }
      constexpr const DelayMap&    delayMap() const noexcept { return _delays; }
      constexpr DelayMap&          delayMap() noexcept { return _delays; }
      constexpr const nt::String&  name(Node n) const noexcept { return _names[n]; }
      constexpr const NameMap&     nameMap() const noexcept { return _names; }
      constexpr NameMap&           nameMap() noexcept { return _names; }
      constexpr const nt::Vec2Dd   geopos(Node n) const noexcept { return nt::Vec2Dd(_geopos[n]); }
      constexpr const GeoPosMap&   geoposMap() const noexcept { return _geopos; }
      constexpr GeoPosMap&         geoposMap() noexcept { return _geopos; }
      constexpr Scalar demandValueAt(DemandArc demand_arc, int t) const noexcept { return _dvm[demand_arc][t]; }
      constexpr nt::Vec< Scalar, STEP_NUM > demandValues(DemandArc demand_arc) const noexcept {
        return _dvm[demand_arc];
      }
      constexpr const DemandValueMap& demandValueMap() const noexcept { return _dvm; }
      constexpr DemandValueMap&       demandValueMap() noexcept { return _dvm; }

      static constexpr int stepNum() noexcept { return STEP_NUM; }

      /**
       * @brief Makes the network bidirected by adding reverse arcs where missing.
       *
       * For each arc (u,v) in the network, ensures a corresponding arc (v,u) exists.
       * Newly created reverse arcs inherit the capacity, metric, and delay of their
       * forward counterparts. Useful for protocols like OSPF/IS-IS that require
       * bidirectional links.
       *
       * @return The number of arcs added to make the graph bidirected.
       */
      int makeBidirected() noexcept { return nt::graphs::makeBidirected(_digraph, _capacities, _metrics, _delays); }


      /**
       * @brief Returns a reference to the network topology graph.
       *
       * The digraph contains the network nodes and arcs. Use this to access graph structure
       * and iterate over nodes/arcs. Compatible with all networktools graph algorithms.
       *
       * @return Reference to the network digraph.
       */
      constexpr Digraph& digraph() noexcept { return _digraph; }

      /**
       * @brief Returns a const reference to the network topology graph.
       *
       * @return Const reference to the network digraph.
       */
      constexpr const Digraph& digraph() const noexcept { return _digraph; }


      /**
       * @brief Returns a reference to the traffic matrix (demand graph).
       *
       * The demand graph represents the traffic matrix as a graph where:
       * - Nodes correspond to network nodes
       * - Arcs represent demands from source to destination
       * - Arc values (accessed via demandValueAt) contain traffic volumes
       *
       * @return Reference to the demand graph.
       */
      constexpr DemandGraph& demandGraph() noexcept { return _demand_graph; }

      /**
       * @brief Returns a const reference to the traffic matrix (demand graph).
       *
       * @return Const reference to the demand graph.
       */
      constexpr const DemandGraph& demandGraph() const noexcept { return _demand_graph; }

      /**
       * @brief Load the topology and traffic matrix from an SNDLib file (see graphs/sndlib_reader.h)
       *
       * @param sz_filename Name of the XML file in SNDlib format
       * @param b_ensure_simply_bidirected Ensure that the graph has no duplicated arcs and is bidirected.
       * Each arc uv added to make the graph bidirected has the same capacity as the opposite arc vu.
       *
       * @return true if the file is successfully loaded, false otherwise
       */
      bool readSndlib(const char* sz_filename, bool b_ensure_simply_bidirected = true) noexcept {
        using SndlibReader = graphs::SndlibReader< Digraph >;

        SndlibReader reader;
        if (!reader.loadFile(sz_filename)) return false;
        if (!reader.readNetwork(_digraph, _capacities, b_ensure_simply_bidirected)) return false;
        if (!reader.readDemandGraph(_demand_graph, _dvm)) return false;

        for(ArcIt arc(_digraph); arc != nt::INVALID; ++arc)
          _metrics[arc] = 1;   // Default metric = 1 for all arcs

        _delays.setBitsToZero();    // Default delay = 0 for all arcs
        _geopos.setBitsToZero();    // Default geopos = (0,0) for all nodes

        return true;
      }

      /**
       * @brief Load the topology and traffic matrix from json files in NetworkX format (see graphs/networkx_io.h)
       *
       * @param sz_topo_filename Name of the json file containing the topology data
       * @param sz_tm_filename Name of the json file containing the traffic matrix data
       * @param sz_topo_src Attribute name of an arc source in the topology json file
       * @param sz_topo_dst Attribute name of an arc target in the topology json file
       * @param sz_tm_src Attribute name of an arc source in the traffic matrix json file
       * @param sz_tm_dst Attribute name of an arc target in the traffic matrix json file
       *
       * @return true if the file is successfully loaded, false otherwise
       *
       */
      bool readJson(const char* sz_topo_filename,
                    const char* sz_tm_filename,
                    const char* sz_topo_src = "from",
                    const char* sz_topo_dst = "to",
                    const char* sz_tm_src = "source",
                    const char* sz_tm_dst = "target",
                    const char* sz_weight = "weight",
                    const char* sz_capacity = "capa",
                    const char* sz_delay = "delay",
                    const char* sz_name = "name",
                    const char* sz_geopos = "geopos") noexcept {
        using NetworkxIo = graphs::NetworkxIo< Digraph >;

        NetworkxIo reader(sz_topo_src, sz_topo_dst);

        graphs::GraphProperties< Digraph > network_props;
        network_props.arcMap(sz_weight, _metrics);
        network_props.arcMap(sz_capacity, _capacities);
        network_props.arcMap(sz_delay, _delays);
        network_props.nodeMap(sz_name, _names);
        network_props.nodeMap(sz_geopos, _geopos);

        if (!reader.template readFile< false >(sz_topo_filename, _digraph, &network_props)) return false;

        graphs::GraphProperties< DemandGraph > tm_props;
        tm_props.arcMap("demand", _dvm);

        reader.setSourceAttr(sz_tm_src);
        reader.setTargetAttr(sz_tm_dst);

        if (!reader.readLinksFromFile(sz_tm_filename, _demand_graph, &tm_props)) return false;

        return true;
      }

      /**
       * @brief Load the topology and traffic matrix from json files in NetworkX format (see graphs/networkx_io.h)
       *
       * @param sz_topo_filename Name of the json file containing the topology data
       * @param sz_tm_filename Name of the json file containing the traffic matrix data
       *
       * @return true if the file is successfully loaded, false otherwise
       *
       */
      bool readRepetita(const char* sz_topo_filename, const char* sz_tm_filename) noexcept {
        using RepetitaIo = graphs::RepetitaIo< Digraph >;

        RepetitaIo reader;

        reader.readNetworkFile(sz_topo_filename, _digraph, _names, _metrics, _capacities, _delays);
        reader.readDemandGraphFile(sz_tm_filename, _demand_graph, _dvm);

        _geopos.setBitsToZero();    // Default geopos = (0,0) for all nodes

        return true;
      }

      /**
       * @brief Removes all nodes, arcs, and associated data from the instance.
       *
       * Clears both the network topology and the traffic matrix, resetting the instance
       * to an empty state. All maps (capacities, metrics, demands, etc.) are also cleared.
       */
      void clear() noexcept {
        _digraph.clear();
        _demand_graph.clear();
      }
    };

  }   // namespace te
}   // namespace nt

#endif