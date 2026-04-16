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
 * @author Qiao Zhang (qiao.zhang@orange.com)
 */

#ifndef _NT_TM_GENERATORS_H_
#define _NT_TM_GENERATORS_H_

#include "../../core/random.h"
#include "../../core/tolerance.h"
#include "../../graphs/algorithms/dijkstra.h"
#include "../../graphs/algorithms/betweenness_centrality.h"
#include "../../graphs/demand_graph.h"
#include "segment_routing.h"

namespace nt {
  namespace te {

    /**
     * @class TrafficMatrixGenerator
     * @headerfile tm_generators.h
     * @brief This class implements a synthetic traffic matrix generator based on [1] and [2]
     *
     * @tparam GR The type of the digraph the algorithm runs on.
     * @tparam CM A concepts::ReadMap "readable" arc map storing the capacities of the arcs.
     * @tparam WM A concepts::ReadMap "readable" arc map storing the weights of the arcs.
     *
     * [1] Antonio Nucci, Ashwin Sridharan, Nina Taft:
     *     The problem of synthetically generating IP traffic matrices: initial
     *     recommendations. Comput. Commun. Rev. 35(3): 19-32 (2005)
     *     https://dl.acm.org/doi/pdf/10.1145/1070873.1070876
     *
     * [2] Matthew Roughan:
     *     Simplifying the synthesis of internet traffic matrices.
     *     Comput. Commun. Rev. 35(5): 93-96 (2005)
     *     https://roughan.info/papers/ccr_2005.pdf
     */
    template < typename GR,
               typename CM = typename GR::template ArcMap< int >,
               typename WM = typename GR::template ArcMap< double > >
    struct TrafficMatrixGenerator {
      using Digraph = GR;

      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using CapacityMap = CM;
      using WeightMap = WM;

      struct Parameters {
        int                     seed;
        nt::Tolerance< double > tolerance;
        WeightMap&              weight_map;

        // parameters for betweeness centrality
        bool use_betweenness_centrality;   // if use_betweenness_centrality
        bool normalized;
        bool endpoints;

        Parameters(WeightMap&              wm,
                   int                     seed = 0,
                   nt::Tolerance< double > tolerance = nt::Tolerance< double >(1e-5),
                   bool                    use_betweenness_centrality = false,
                   bool                    normalized = true,
                   bool                    endpoints = false) :
            seed(seed),
            tolerance(tolerance), weight_map(wm), use_betweenness_centrality(use_betweenness_centrality),
            normalized(normalized), endpoints(endpoints) {}
      };

      const Digraph&     _graph;
      const CapacityMap& _cm;

      nt::Random                                          _rnd;
      nt::DoubleDynamicArray                              _Tin;
      nt::DoubleDynamicArray                              _Tout;
      nt::DoubleMultiDimArray< 2 >                        _T;
      DoubleArcMap                                        _load;
      WeightMap                                           _default_weight_map;
      graphs::BetweennessCentrality< Digraph, WeightMap > _betweenness_centrality;
      Parameters                                          _params;

      // Store the final demand graph and its demand values
      using DemandGraph = nt::graphs::DemandGraph< Digraph >;
      mutable DemandGraph                                     _demand_graph;
      mutable typename DemandGraph::template ArcMap< double > _demand_values;


      struct OdPair {
        Node src;
        Node des;

        OdPair(Node src, Node des) : src(src), des(des) {}

        constexpr bool operator==(const OdPair& other) const { return src == other.src && des == other.des; }
        constexpr bool operator!=(const OdPair& other) const { return !(*this == other); }
        constexpr bool operator<(const OdPair& other) const {
          return src < other.src || (src == other.src && des < other.des);
        }
      };

      struct OdPathsInfo {
        nt::DynamicArray< nt::TrivialDynamicArray< Arc > > paths;
        nt::DoubleDynamicArray                             paths_weight;
      };

      struct NodeInfo {
        double fanin;
        double fanout;
        int    connectivity;
        int    nb_paths;
      };

      struct OdPairInfo {
        OdPair od_pair;
        double m1;
        double m2;
        double m3;

        OdPairInfo(OdPair od_pair, double m1, double m2, double m3) : od_pair(od_pair), m1(m1), m2(m2), m3(m3) {}
        OdPairInfo(OdPairInfo&& other) noexcept :
            od_pair(std::move(other.od_pair)), m1(std::move(other.m1)), m2(std::move(other.m2)),
            m3(std::move(other.m3)) {}
        OdPairInfo(const OdPairInfo&) = delete;
        OdPairInfo& operator=(const OdPairInfo&) = delete;

        OdPairInfo& operator=(OdPairInfo&& other) noexcept {
          if (this != &other) {
            od_pair = std::move(other.od_pair);
            m1 = std::move(other.m1);
            m2 = std::move(other.m2);
            m3 = std::move(other.m3);
          }
          return *this;
        }

        static constexpr bool compare(const OdPairInfo& a, const OdPairInfo& b) noexcept {
          if (a.m1 == b.m1) {
            if (a.m2 == b.m2) {
              return a.m3 > b.m3;
            } else {
              return a.m2 > b.m2;
            }
          } else {
            return a.m1 < b.m1;
          }
        }
      };

      explicit TrafficMatrixGenerator(const Digraph& g, const CapacityMap& cm, const Parameters& params) :
          _graph(g), _cm(cm), _rnd(params.seed), _load(g), _default_weight_map(g, 1),
          _betweenness_centrality(g, params.weight_map), _params(params), _demand_graph(g),
          _demand_values(_demand_graph) {
        if (_params.use_betweenness_centrality) { _betweenness_centrality.run(_params.normalized, _params.endpoints); }
      }

      // Constructor avec paramètres individuels pour compatibilité
      explicit TrafficMatrixGenerator(const Digraph&     g,
                                      const CapacityMap& cm,
                                      int                seed = 0,
                                      bool               use_betweenness_centrality = false) :
          _graph(g),
          _cm(cm), _rnd(seed), _load(g), _default_weight_map(g, 1), _betweenness_centrality(g, _default_weight_map),
          _params(_default_weight_map, seed, nt::Tolerance< double >(1e-5), use_betweenness_centrality),
          _demand_graph(g), _demand_values(_demand_graph) {
        if (_params.use_betweenness_centrality) { _betweenness_centrality.run(_params.normalized, _params.endpoints); }
      }

      // Constructor with custom weight map
      explicit TrafficMatrixGenerator(const Digraph&     g,
                                      const CapacityMap& cm,
                                      WeightMap&         weight_map,
                                      int                seed = 0,
                                      bool               use_betweenness_centrality = false) :
          _graph(g),
          _cm(cm), _rnd(seed), _load(g), _default_weight_map(g, 1), _betweenness_centrality(g, weight_map),
          _params(weight_map, seed, nt::Tolerance< double >(1e-5), use_betweenness_centrality), _demand_graph(g),
          _demand_values(_demand_graph) {
        if (_params.use_betweenness_centrality) { _betweenness_centrality.run(_params.normalized, _params.endpoints); }
      }
      ~TrafficMatrixGenerator() = default;

      /**
       * @brief Enable or disable the use of betweenness centrality in traffic generation
       * @param use_centrality Whether to use betweenness centrality
       * @param normalized Whether to apply normalization (default: true)
       * @param endpoints Whether to include endpoints in the calculation (default: false)
       */
      void useBetweennessCentrality(bool use_centrality, bool normalized = true, bool endpoints = false) noexcept {
        _params.use_betweenness_centrality = use_centrality;
        _params.normalized = normalized;
        _params.endpoints = endpoints;
        if (use_centrality) { _betweenness_centrality.run(normalized, endpoints); }
      }

      /**
       * @brief Check if betweenness centrality is currently being used in traffic generation
       * @return True if betweenness centrality is enabled, false otherwise
       */
      bool isUsingBetweennessCentrality() const noexcept { return _params.use_betweenness_centrality; }

      /**
       * @brief Get the betweenness centrality value for a node
       * @param node The node
       * @return The betweenness centrality value
       */
      double getBetweennessCentrality(Node node) const noexcept { return _betweenness_centrality.centrality(node); }

      /**
       * @brief Get the current parameters
       * @return Reference to the current parameters
       */
      const Parameters& getParameters() const noexcept { return _params; }

      /**
       * @brief Set new parameters (note: weight_map reference cannot be changed)
       * @param params New parameters to set
       */
      void setParameters(const Parameters& params) noexcept {
        // Note: We cannot change the weight_map reference, only copy other parameters
        _params.seed = params.seed;
        _params.tolerance = params.tolerance;
        _params.use_betweenness_centrality = params.use_betweenness_centrality;
        _params.normalized = params.normalized;
        _params.endpoints = params.endpoints;
        _rnd.seed(_params.seed);
        if (_params.use_betweenness_centrality) { _betweenness_centrality.run(_params.normalized, _params.endpoints); }
      }

      /**
       * @brief Set the random seed
       * @param seed New random seed
       */
      void setSeed(int seed) noexcept {
        _params.seed = seed;
        _rnd.seed(seed);
      }

      /**
       * @brief Set the tolerance
       * @param tolerance New tolerance
       */
      void setTolerance(const nt::Tolerance< double >& tolerance) noexcept { _params.tolerance = tolerance; }

      /**
       * @brief
       *
       * @param f_total_traffic Total traffic going through the network, in Mbps.
       *
       */
      void run(double f_total_traffic) noexcept {
        _generateFlowRate(f_total_traffic);
        _assign_flows(f_total_traffic);
      }

      /**
       * @brief
       *
       * @param i
       * @param j
       * @return constexpr double
       */
      constexpr double flowRate(int i, int j) const noexcept { return _T(i, j); }

      /**
       * @brief
       *
       * @param arc
       * @return constexpr double
       */
      constexpr double getLoad(Arc arc) noexcept { return _load[arc]; }

      /**
       * @brief Get the Traffic Matrix object
       *
       * @return constexpr const nt::DoubleMultiDimArray< 2 >&
       */
      constexpr const nt::DoubleMultiDimArray< 2 >& getTrafficMatrix() const noexcept { return _T; }

      /**
       * @brief Get the final demand graph with scaled demand values
       *
       * @return const DemandGraph& The demand graph containing all demands after traffic matrix generation and scaling
       */
      const DemandGraph& getDemandGraph() const noexcept { return _demand_graph; }


      /**
       * @brief Get the demand values map for the final demand graph
       *
       * @return const typename DemandGraph::template ArcMap< double >& The demand values after scaling
       */
      const typename DemandGraph::template ArcMap< double >& getDemandValues() const noexcept { return _demand_values; }

      /**
       * @brief
       *
       * @param total_traffic
       * @return true
       * @return false
       */
      constexpr bool checkLoad(int total_traffic) noexcept {
        double total_load = 0.;
        for (ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
          if (_params.tolerance.less((double)_cm[arc], _load[arc])) {
            LOG_F(INFO, "capacity arc {}: load arc {}", _cm[arc], _load[arc]);
            return false;
          }
          total_load += _load[arc];
        }
        return _params.tolerance.positiveOrZero((double)total_traffic - total_load);
      }

      /**
       * @brief
       *
       * @param f_total_traffic
       * @return true
       * @return false
       */
      constexpr bool checkInputTrafficTotal(int f_total_traffic) noexcept {
        if (f_total_traffic <= 0) {
          LOG_F(ERROR, "Total traffic must be positive");
          return false;
        }
        return true;
      }

      /**
       * @brief
       *
       * @return true
       * @return false
       */
      constexpr bool checkInputGraphNodeArc() noexcept {
        if (_graph.nodeNum() == 0 || _graph.arcNum() == 0) {
          LOG_F(ERROR, "Graph must have at least one node and one arc");
          return false;
        }

        if (_cm.size() != _graph.arcNum()) {
          LOG_F(ERROR, "Capacity arc map size must be equal to the number of arcs in the graph");
          return false;
        }

        for (int i = 0; i < _graph.arcNum(); ++i) {
          if (_cm[_graph.arcFromId(i)] <= 0) {
            LOG_F(ERROR, "Arc weights must be positive");
            return false;
          }
        }

        return true;
      }

      // Private

      /**
       * @brief Assign traffic generated for nodes to arcs with the heuristic method.
       *
       * @param f_total_traffic Total traffic going through the network.
       *
       */
      void _assign_flows(double f_total_traffic) noexcept {
        const int i_num_nodes = _graph.nodeNum();
        const int i_num_arcs = _graph.arcNum();

        // Handle empty graph - nothing to assign
        if (i_num_nodes == 0 || i_num_arcs == 0) { return; }

        // Clear the demand graph and recreate it from the traffic matrix
        _updateDemandGraph();

        // Initialize loads to zero
        for (ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
          _load[arc] = 0.;
        }

        // Create a double metric map from the weight map
        typename Digraph::template ArcMap< double > metric_map(_graph);
        for (typename Digraph::ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
          metric_map[arc] = static_cast< double >(_params.weight_map[arc]);
        }

        // Use DynamicShortestPathRouting to route all demands
        DynamicShortestPathRouting< Digraph, double > spr(_graph, metric_map);

        const int i_num_routed = spr.run(_demand_graph, _demand_values);
        LOG_F(INFO, "Number of demands routed: {}", i_num_routed);

        // Copy flows from the SPR algorithm to our load map
        for (ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
          _load[arc] = spr.flow(arc)[0];
        }

        // Compute objective value (maximum link utilization)
        double objective = spr.maxSaturation(_cm).max();
        LOG_F(INFO, "Max saturation : {}", objective);


        if (_params.tolerance.less(1., objective)) {
          LOG_F(INFO, "Scaling down TM by {} to reach feasible routing.", objective);
          // scale down the traffic matrix
          _scaleDownTrafficMatrix(objective);

          // Update the demand graph to reflect the scaled traffic matrix
          _updateDemandGraph();

          double total_link_load = 0.;
          for (ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
            _load[arc] /= objective;
            total_link_load += _load[arc];
          }
          double target_mean_link_load = f_total_traffic / i_num_arcs;
          double mean_link_load = total_link_load / i_num_arcs;
          double factor = mean_link_load / target_mean_link_load;

          if (_params.tolerance.less(1., factor)) {
            LOG_F(INFO, "Scaling down TM by {} to reach {} mean link load.", factor, target_mean_link_load);
            _scaleDownTrafficMatrix(factor);

            // Update the demand graph again and re-run SPR
            _updateDemandGraph();

            for (ArcIt arc(_graph); arc != nt::INVALID; ++arc) {
              _load[arc] /= factor;
            }
          } else {
            LOG_F(INFO, "Mean link load is at {}", mean_link_load);
          }

        } else {
          LOG_F(INFO, "The network is feasible with objective value {}", objective);
        }
      }

      void _scaleDownTrafficMatrix(double factor) noexcept {
        const int i_num_nodes = _graph.nodeNum();
        for (int i = 0; i < i_num_nodes; ++i) {
          for (int j = 0; j < i_num_nodes; ++j) {
            _T(i, j) /= factor;
          }
        }
      }

      /**
       * @brief Update the stored demand graph with current traffic matrix values
       * This function recreates the demand graph based on the current traffic matrix,
       * typically called after scaling down the traffic matrix.
       */
      void _updateDemandGraph() noexcept {
        // Clear existing demands
        _demand_graph.clear();

        // Add demands to the demand graph based on the current traffic matrix
        for (NodeIt u(_graph); u != nt::INVALID; ++u) {
          for (NodeIt v(_graph); v != nt::INVALID; ++v) {
            if (u == v) continue;

            const int    u_id = _graph.id(u);
            const int    v_id = _graph.id(v);
            const double flow_rate = _T(u_id, v_id);


            auto demand_arc = _demand_graph.addArc(u, v);
            _demand_values[demand_arc] = flow_rate;
          }
        }
      }

      /**
       * @brief Generates |V| * (|V| - 1) flow rates according to the Gravity model.
       * If betweenness centrality is enabled, adjusts flow generation probability
       * based on node centrality (higher centrality = lower probability).
       */
      void _generateFlowRate(double f_total_traffic) noexcept {
        const int        i_num_nodes = _graph.nodeNum();
        constexpr double f_mean_base = 0.086;

        _Tin.clear();
        _Tout.clear();
        if (i_num_nodes <= 0) return;

        // Generate Tout (outgoing traffic)
        double f_sum_Tout = 0.;
        _Tout.ensureAndFill(i_num_nodes);
        int k = 0;
        for (NodeIt node(_graph); node != nt::INVALID; ++node, ++k) {
          double f_mean = f_mean_base;

          if (_params.use_betweenness_centrality) {
            // Adjust f_mean based on betweenness centrality
            // Higher centrality = lower probability = higher f_mean for exponential distribution
            double centrality = _betweenness_centrality.centrality(node);
            // Scale centrality influence: centrality factor ranges from 0.5 to 2.0
            double centrality_factor = 1.0 + centrality * 2.0;   // Higher centrality increases the factor
            f_mean = f_mean_base * centrality_factor;
          }

          const double v = _rnd.exponential(f_mean);
          _Tout[k] = v;
          f_sum_Tout += v;
        }

        // Generate Tin (incoming traffic)
        double f_sum_Tin = 0.;
        _Tin.ensureAndFill(i_num_nodes);
        k = 0;
        for (NodeIt node(_graph); node != nt::INVALID; ++node, ++k) {
          double f_mean = f_mean_base;

          if (_params.use_betweenness_centrality) {
            // Adjust f_mean based on betweenness centrality
            // Higher centrality = lower probability = higher f_mean
            // for exponential distribution
            double centrality = _betweenness_centrality.centrality(node);
            // Scale centrality influence: centrality factor ranges from 0.5 to 2.0
            // Higher centrality increases the factor
            double centrality_factor = 1.0 + centrality * 2.0;
            f_mean = f_mean_base * centrality_factor;
          }

          const double v = _rnd.exponential(f_mean);
          _Tin[k] = v;
          f_sum_Tin += v;
        }

        const double f_prod = f_sum_Tout * f_sum_Tin;
        assert(f_prod > 0);
        _T.setDimensions(i_num_nodes, i_num_nodes);
        _T.setBitsToZero();
        for (int i = 0; i < i_num_nodes; ++i) {
          for (int j = 0; j < i_num_nodes; ++j) {
            if (i == j) continue;
            _T(i, j) = f_total_traffic * (_Tin[i] * _Tout[j]) / f_prod;   // div 0
          }
        }
      }
    };
  }   // namespace te
}   // namespace nt

#endif
