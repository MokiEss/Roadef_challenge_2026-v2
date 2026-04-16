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
 * @author Pierrick Nolet (pierrick.nolet@orange.com)
 */

#ifndef _NT_COLD_H_
#define _NT_COLD_H_

#include "../../core/common.h"
#include "../../core/random.h"
#include "../../core/queue.h"
#include "../../core/maps.h"
#include "../../core/tuple.h"
#include "../../core/arrays.h"
#include "../../core/sort.h"
#include "../../core/common.h"
#include "../../graphs/tools.h"

namespace nt {
  namespace te {

    /**
     * @brief COLD (Complex Optimization of Link Design) topology generator based on [1]
     *
     *
     * [1] Rhys Alistair Bowden, Matthew Roughan, Nigel G. Bean:
     *     COLD: PoP-level Network Topology Synthesis. CoNEXT 2014: 173-184
     *
     * @tparam GR Graph type
     * @tparam FPTYPE Floating point type for computations
     * @tparam CM Capacity map type
     * @tparam MM Metric map type
     * @tparam PM Position map type
     */
    template < typename GR,
               typename FPTYPE = float,
               typename CM = typename GR::template ArcMap< FPTYPE >,
               typename MM = typename GR::template ArcMap< FPTYPE >,
               typename PM = typename GR::template NodeMap< nt::Vec< double, 2 > > >
    struct COLD {
      using Digraph = GR;
      TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

      using MetricMap = MM;
      using Metric = typename MetricMap::Value;

      using CapacityMap = CM;
      using Capacity = typename CapacityMap::Value;

      using PositionsMap = PM;
      using Position = typename PositionsMap::Value;

      /**
       * This class stores parameter settings for the COLD algorithm.
       */
      struct Parameters {
        // Enumeration of parameters that take continuous values.
        enum DoubleParam {
          WIDTH = 0,             // Geographic width
          HEIGHT = 1,            // Geographic height
          NODE_POPULATION = 2,   // Node population parameter
          K0 = 3,                // Link existence cost
          K1 = 4,                // Link length cost
          K2 = 5,                // Bandwidth cost
          K3 = 6,                // Hub cost
          CROSSOVER_RATE = 7,
          HUB_MUTATION_RATE = 8
        };

        // Enumeration of parameters that take integer values.
        enum IntegerParam {
          SEED = 1000,
          NUM_NODES = 1001,
          NUM_CHROMOSOMES = 1002,
          NUM_GENERATIONS = 1003,
          CROSSOVER_A = 1004,
          CROSSOVER_B = 1005,
          START_LINKS = 1006,
          NUM_SAVED_CHROMOSOMES = 1007
        };

        // Population distribution types
        enum PopulationDistribution { EXPONENTIAL = 20, PARETO = 21, UNIFORM = 22 };

        /**
         * @brief COLD Algorithm Parameters
         *
         * All parameters have sensible defaults - only customize what you need.
         *
         * **Geographic Parameters:**
         * - WIDTH (1.0): Geographic area width
         * - HEIGHT (1.0): Geographic area height
         * - NODE_POPULATION (30.0): Base population parameter for nodes
         *
         * **Cost Model Parameters:**
         * - K0 (10.0): Fixed cost per link existence
         * - K1 (1.0): Cost per unit distance
         * - K2 (0.001): Cost per unit bandwidth×distance
         * - K3 (10.0): Fixed cost per hub (node with >1 connections)
         *
         * **Genetic Algorithm Parameters:**
         * - NUM_NODES (50): Number of nodes to generate
         * - NUM_CHROMOSOMES (100): Population size
         * - NUM_GENERATIONS (100): Number of evolution cycles
         * - NUM_SAVED_CHROMOSOMES (10): Elitism - best solutions kept per generation
         * - CROSSOVER_RATE (0.7): Probability of crossover vs mutation
         * - HUB_MUTATION_RATE (0.2): Probability of hub-specific mutation
         * - CROSSOVER_A (2): Number of parents in crossover
         * - CROSSOVER_B (10): Candidate pool size for parent selection
         * - START_LINKS (3): Average links per node in random initialization
         * - SEED (2004): Random seed for reproducibility
         *
         * **Population Distribution:**
         * - EXPONENTIAL: λ = 1/NODE_POPULATION (default)
         * - PARETO: α = 1.5, scale = NODE_POPULATION
         * - UNIFORM: range [1, 2×NODE_POPULATION]
         *
         * **Quick Start:**
         * ```cpp
         * COLD<SmartDigraph> cold;
         * // Minimal config - just set network size
         * cold.getParameters().setIntegerParam(COLD::Parameters::NUM_NODES, 30);
         * double cost = cold.run();
         * const auto& graph = cold.getGraph();
         * ```
         */

        Parameters() :
            _f_width(1.0), _f_height(1.0), _f_node_population(30.0), _f_k0(10.0), _f_k1(1.0), _f_k2(0.001), _f_k3(10.0),
            _f_crossover_rate(0.7), _f_hub_mutation_rate(0.2), _i_seed(2004), _i_num_nodes(50), _i_num_chromosomes(100),
            _i_num_generations(100), _i_crossover_a(2), _i_crossover_b(10), _i_start_links(3),
            _i_num_saved_chromosomes(10), _population_dist(EXPONENTIAL) {}

        /**
         * @brief Sets a double parameter to a specific value.
         */
        constexpr void setDoubleParam(DoubleParam param, double value) noexcept {
          switch (param) {
            case WIDTH: _f_width = value; break;
            case HEIGHT: _f_height = value; break;
            case NODE_POPULATION: _f_node_population = value; break;
            case K0: _f_k0 = value; break;
            case K1: _f_k1 = value; break;
            case K2: _f_k2 = value; break;
            case K3: _f_k3 = value; break;
            case CROSSOVER_RATE: _f_crossover_rate = value; break;
            case HUB_MUTATION_RATE: _f_hub_mutation_rate = value; break;
            default: {
              LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
            }
          }
        }

        /**
         * @brief Returns the value of a double parameter.
         */
        constexpr double getDoubleParam(DoubleParam param) const noexcept {
          switch (param) {
            case WIDTH: return _f_width;
            case HEIGHT: return _f_height;
            case NODE_POPULATION: return _f_node_population;
            case K0: return _f_k0;
            case K1: return _f_k1;
            case K2: return _f_k2;
            case K3: return _f_k3;
            case CROSSOVER_RATE: return _f_crossover_rate;
            case HUB_MUTATION_RATE: return _f_hub_mutation_rate;
            default: {
              LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
              return NT_DOUBLE_INFINITY;
            }
          }
        }

        /**
         * @brief Sets an integer parameter to a specific value.
         */
        constexpr void setIntegerParam(IntegerParam param, int value) noexcept {
          switch (param) {
            case SEED: _i_seed = value; break;
            case NUM_NODES: _i_num_nodes = value; break;
            case NUM_CHROMOSOMES: _i_num_chromosomes = value; break;
            case NUM_GENERATIONS: _i_num_generations = value; break;
            case CROSSOVER_A: _i_crossover_a = value; break;
            case CROSSOVER_B: _i_crossover_b = value; break;
            case START_LINKS: _i_start_links = value; break;
            case NUM_SAVED_CHROMOSOMES: _i_num_saved_chromosomes = value; break;
            default: {
              LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
            }
          }
        }

        /**
         * @brief Returns the value of an integer parameter.
         */
        constexpr int getIntegerParam(IntegerParam param) const noexcept {
          switch (param) {
            case SEED: return _i_seed;
            case NUM_NODES: return _i_num_nodes;
            case NUM_CHROMOSOMES: return _i_num_chromosomes;
            case NUM_GENERATIONS: return _i_num_generations;
            case CROSSOVER_A: return _i_crossover_a;
            case CROSSOVER_B: return _i_crossover_b;
            case START_LINKS: return _i_start_links;
            case NUM_SAVED_CHROMOSOMES: return _i_num_saved_chromosomes;
            default: {
              LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
              return NT_INT32_MAX;
            }
          }
        }

        double                 _f_width;
        double                 _f_height;
        double                 _f_node_population;
        double                 _f_k0;
        double                 _f_k1;
        double                 _f_k2;
        double                 _f_k3;
        double                 _f_crossover_rate;
        double                 _f_hub_mutation_rate;
        int                    _i_seed;
        int                    _i_num_nodes;
        int                    _i_num_chromosomes;
        int                    _i_num_generations;
        int                    _i_crossover_a;
        int                    _i_crossover_b;
        int                    _i_start_links;
        int                    _i_num_saved_chromosomes;
        PopulationDistribution _population_dist;
      };

      Parameters   _params;
      Digraph      _graph;
      MetricMap    _weights;
      CapacityMap  _capacities;
      PositionsMap _positions;

      // Additional maps for COLD algorithm
      typename Digraph::template NodeMap< double >              _populations;
      typename Digraph::template NodeMap< nt::IntDynamicArray > _distance_order;

      // Backup vectors for node properties (needed when recreating graph)
      nt::DynamicArray< nt::Pair< double, double > > _node_positions_backup;
      nt::DoubleDynamicArray                         _node_populations_backup;

      COLD() :
          _weights(_graph), _capacities(_graph), _positions(_graph), _populations(_graph), _distance_order(_graph) {}

      /**
       * @brief Set the Parameters
       *
       * @param params
       */
      constexpr void setParameters(const Parameters& params) noexcept { _params = params; }
      constexpr void setParameters(Parameters&& params) noexcept { _params = std::move(params); }

      /**
       * @brief Get the Parameters
       *
       */
      constexpr const Parameters& getParameters() const noexcept { return _params; }

      /**
       * @brief Run the COLD algorithm
       * @return The total cost of the generated topology
       */
      double run() noexcept {
        // Validation des paramètres critiques
        if (_params._i_num_nodes <= 0) { return NT_DOUBLE_INFINITY; }

        if (_params._i_num_saved_chromosomes > _params._i_num_chromosomes) { return NT_DOUBLE_INFINITY; }

        // Initialize random generator
        _rng.seed(_params._i_seed);

        // Clear existing graph
        _graph.clear();

        // Generate context (positions, populations, distances)
        _generateContext();

        // Run genetic algorithm
        _runGeneticAlgorithm();

        return _total_cost;
      }

      /**
       * @brief Clear the graph
       */
      void clear() noexcept {
        _graph.clear();
        _node_positions_backup.clear();
        _node_populations_backup.clear();
      }

      private:
      nt::Random _rng;

      /**
       * @brief Fisher-Yates shuffle algorithm
       */
      template < typename Container >
      void _fisherYatesShuffle(Container& container) noexcept {
        for (int i = static_cast< int >(container.size()) - 1; i > 0; --i) {
          int j = _rng.integer(0, i + 1);
          nt::swap(container[i], container[j]);
        }
      }

      // Node distance matrix and demand matrix
      nt::DoubleMultiDimArray< 2 > _node_distances;
      nt::DoubleMultiDimArray< 2 > _demand;

      // Genetic algorithm data structures
      nt::TrivialMultiDimArray< bool, 3 > _chromosomes;
      nt::DoubleDynamicArray              _chromosome_costs;
      double                              _total_cost = 0.0;

      /**
       * @brief Generate node positions and populations
       */
      void _generateContext() noexcept {
        // Create nodes
        nt::DynamicArray< Node > nodes;
        nt::IntDynamicArray      node_ids;
        nodes.reserve(_params._i_num_nodes);
        node_ids.reserve(_params._i_num_nodes);

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          Node node = _graph.addNode();
          nodes.push_back(node);
          int node_id = _graph.id(node);
          node_ids.push_back(node_id);
        }

        // Clear and resize backup vectors
        _node_positions_backup = nt::DynamicArray< nt::Pair< double, double > >(_params._i_num_nodes);
        _node_populations_backup = nt::DoubleDynamicArray(_params._i_num_nodes);
        _node_populations_backup.fill(0.0);

        // Generate random positions
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          Position pos;
          pos[0] = _rng(0.0, _params._f_width);
          pos[1] = _rng(0.0, _params._f_height);

          _positions[nodes[i]] = pos;

          // Backup position using node ID as index
          int node_id = node_ids[i];
          if (node_id >= 0 && node_id < static_cast< int >(_node_positions_backup.size())) {
            _node_positions_backup[node_id] = nt::Pair< double, double >(pos[0], pos[1]);
          }
        }

        // Generate populations
        if (_params._population_dist == Parameters::EXPONENTIAL) {
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            double pop = _rng.exponential(1.0 / _params._f_node_population);
            _populations[nodes[i]] = pop;
            int node_id = node_ids[i];
            if (node_id >= 0 && node_id < static_cast< int >(_node_populations_backup.size())) {
              _node_populations_backup[node_id] = pop;
            }
          }
        } else if (_params._population_dist == Parameters::PARETO) {
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            double u = _rng() + 1e-15;
            double pop = _params._f_node_population / ::pow(u, 1.0 / 1.5);
            _populations[nodes[i]] = pop;
            int node_id = node_ids[i];
            if (node_id >= 0 && node_id < static_cast< int >(_node_populations_backup.size())) {
              _node_populations_backup[node_id] = pop;
            }
          }
        } else {   // UNIFORM
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            double pop = _rng(1.0, _params._f_node_population * 2);
            _populations[nodes[i]] = pop;
            int node_id = node_ids[i];
            if (node_id >= 0 && node_id < static_cast< int >(_node_populations_backup.size())) {
              _node_populations_backup[node_id] = pop;
            }
          }
        }

        // Initialize distance and demand matrices
        _node_distances.setDimensions(_params._i_num_nodes, _params._i_num_nodes);
        _node_distances.setBitsToZero();
        _demand.setDimensions(_params._i_num_nodes, _params._i_num_nodes);
        _demand.setBitsToZero();

        // Calculate distances and demands
        int i = 0;
        for (typename Digraph::NodeIt ni(_graph); ni != nt::INVALID; ++ni, ++i) {
          int j = 0;
          for (typename Digraph::NodeIt nj(_graph); nj != nt::INVALID; ++nj, ++j) {
            if (i != j) {
              double dx = _positions[ni][0] - _positions[nj][0];
              double dy = _positions[ni][1] - _positions[nj][1];
              double distance = ::sqrt(dx * dx + dy * dy);

              _node_distances(i, j) = distance;
              _demand(i, j) = _populations[ni] * _populations[nj];
            }
          }
        }

        // Initialize distance order matrix for connectivity repair
        _initializeDistanceOrder();
      }

      /**
       * @brief Initialize distance ordering for connectivity repair
       */
      void _initializeDistanceOrder() noexcept {
        nt::DynamicArray< nt::Triple< int, int, double > > distance_pairs;

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            if (i != j) { distance_pairs.push_back(nt::Triple< int, int, double >(i, j, _node_distances(i, j))); }
          }
        }

        nt::sort(distance_pairs, [](const auto& a, const auto& b) { return a.third < b.third; });

        // Store order for each node
        int idx = 0;
        for (typename Digraph::NodeIt node(_graph); node != nt::INVALID; ++node) {
          _distance_order[node] = nt::IntDynamicArray(_params._i_num_nodes);
          _distance_order[node].fill(0);
          idx++;
        }
      }

      /**
       * @brief Swap two chromosomes in the 3D array
       */
      void _swapChromosomes(int idx1, int idx2) noexcept {
        if (idx1 == idx2) return;

        nt::TrivialMultiDimArray< bool, 2 > temp;
        temp.setDimensions(_params._i_num_nodes, _params._i_num_nodes);

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            temp(i, j) = _chromosomes(idx1, i, j);
          }
        }

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            _chromosomes(idx1, i, j) = _chromosomes(idx2, i, j);
          }
        }

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            _chromosomes(idx2, i, j) = temp(i, j);
          }
        }

        std::swap(_chromosome_costs[idx1], _chromosome_costs[idx2]);
      }

      /**
       * @brief Run the genetic algorithm
       */
      void _runGeneticAlgorithm() noexcept {
        _initializeChromosomes();

        for (int gen = 0; gen < _params._i_num_generations; ++gen) {
          nt::DynamicArray< size_t > indices(_params._i_num_chromosomes);
          for (size_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
          }
          nt::sort(indices, [this](size_t a, size_t b) { return _chromosome_costs[a] < _chromosome_costs[b]; });

          for (int i = 0; i < _params._i_num_saved_chromosomes; ++i) {
            if (indices[i] != static_cast< size_t >(i)) {
              _swapChromosomes(i, indices[i]);

              for (size_t j = i + 1; j < indices.size(); ++j) {
                if (indices[j] == static_cast< size_t >(i)) {
                  indices[j] = indices[i];
                  break;
                }
              }
              indices[i] = i;
            }
          }

          for (int i = _params._i_num_saved_chromosomes; i < _params._i_num_chromosomes; ++i) {
            double r = _rng();
            if (r < _params._f_crossover_rate) {
              _performCrossover(i);
            } else {
              _performMutation(i);
            }
          }

          for (int i = _params._i_num_saved_chromosomes; i < _params._i_num_chromosomes; ++i) {
            bool needs_repair = false;
            _chromosome_costs[i] = _calculateChromosomeCost(i, needs_repair);
          }
        }

        int    best_index = 0;
        double best_cost = _chromosome_costs[0];
        for (int i = 1; i < _params._i_num_chromosomes; ++i) {
          if (_chromosome_costs[i] < best_cost) {
            best_cost = _chromosome_costs[i];
            best_index = i;
          }
        }
        _total_cost = _chromosome_costs[best_index];

        if (!_isConnected(best_index)) {
          _repairConnectivity(best_index);

          bool dummy;
          _total_cost = _calculateChromosomeCost(best_index, dummy);
        }

        _applyChromosomeToGraph(best_index);
      }

      /**
       * @brief Perform mutation operation
       */
      void _performMutation(int targetIndex) noexcept {
        nt::DoubleDynamicArray inverse_costs(_params._i_num_chromosomes);
        double                 sum_inverse = 0.0;

        for (int i = 0; i < _params._i_num_chromosomes; ++i) {
          inverse_costs[i] = 1.0 / _chromosome_costs[i];
          sum_inverse += inverse_costs[i];
        }

        for (int i = 0; i < _params._i_num_chromosomes; ++i) {
          inverse_costs[i] /= sum_inverse;
        }

        double r = _rng();
        double cumul = 0.0;
        int    chosen_index = 0;

        for (int i = 0; i < _params._i_num_chromosomes; ++i) {
          cumul += inverse_costs[i];
          if (r <= cumul) {
            chosen_index = i;
            break;
          }
        }

        if (chosen_index != targetIndex) {
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            for (int j = 0; j < _params._i_num_nodes; ++j) {
              _chromosomes(targetIndex, i, j) = _chromosomes(chosen_index, i, j);
            }
          }
          _chromosome_costs[targetIndex] = _chromosome_costs[chosen_index];
        }

        if (_rng() < _params._f_hub_mutation_rate) {
          nt::IntDynamicArray hubs;
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            int degree = 0;
            for (int j = 0; j < _params._i_num_nodes; ++j) {
              if (_chromosomes(targetIndex, i, j)) degree++;
            }
            if (degree > 1) hubs.push_back(i);
          }

          if (hubs.size() > 1) {
            int hub_to_remove = hubs[_rng.integer(0, hubs.size())];

            nt::IntDynamicArray remaining_hubs;
            for (int h: hubs) {
              if (h != hub_to_remove) remaining_hubs.push_back(h);
            }

            if (!remaining_hubs.empty()) {
              int    closest_hub = remaining_hubs[0];
              double min_dist = _node_distances(hub_to_remove, closest_hub);

              for (int h: remaining_hubs) {
                if (_node_distances(hub_to_remove, h) < min_dist) {
                  closest_hub = h;
                  min_dist = _node_distances(hub_to_remove, h);
                }
              }

              for (int j = 0; j < _params._i_num_nodes; ++j) {
                if (j != closest_hub) {
                  _chromosomes(targetIndex, hub_to_remove, j) = false;
                  _chromosomes(targetIndex, j, hub_to_remove) = false;
                }
              }

              _chromosomes(targetIndex, hub_to_remove, closest_hub) = true;
              _chromosomes(targetIndex, closest_hub, hub_to_remove) = true;
            }
          }
        } else {
          int m_plus = static_cast< int >(::floor(::log(nt::max(_rng(), 1e-15))) / ::log(0.5));
          int m_minus = static_cast< int >(::floor(::log(nt::max(_rng(), 1e-15))) / ::log(0.5));

          nt::DynamicArray< nt::Pair< int, int > > existing_links;
          nt::DynamicArray< nt::Pair< int, int > > nonexistent_links;

          for (int i = 0; i < _params._i_num_nodes; ++i) {
            for (int j = i + 1; j < _params._i_num_nodes; ++j) {
              if (_chromosomes(targetIndex, i, j)) {
                existing_links.push_back(nt::Pair< int, int >(i, j));
              } else {
                nonexistent_links.push_back(nt::Pair< int, int >(i, j));
              }
            }
          }

          m_plus = nt::min(m_plus, static_cast< int >(existing_links.size()));
          m_minus = nt::min(m_minus, static_cast< int >(nonexistent_links.size()));

          _fisherYatesShuffle(existing_links);
          for (int i = 0; i < m_plus; ++i) {
            int u = existing_links[i].first;
            int v = existing_links[i].second;
            _chromosomes(targetIndex, u, v) = false;
            _chromosomes(targetIndex, v, u) = false;
          }

          _fisherYatesShuffle(nonexistent_links);
          for (int i = 0; i < m_minus; ++i) {
            int u = nonexistent_links[i].first;
            int v = nonexistent_links[i].second;
            _chromosomes(targetIndex, u, v) = true;
            _chromosomes(targetIndex, v, u) = true;
          }
        }
      }

      /**
       * @brief Initialize chromosome population
       */
      void _initializeChromosomes() noexcept {
        _chromosomes.setDimensions(_params._i_num_chromosomes, _params._i_num_nodes, _params._i_num_nodes);
        _chromosomes.setBitsToZero();
        _chromosome_costs = nt::DoubleDynamicArray(_params._i_num_chromosomes, 0);

        // First chromosome: MST
        _createMST(0);

        // Second chromosome: fully connected
        if (_params._i_num_chromosomes >= 2) { _createFullyConnectedGraph(1); }

        // Remaining: random
        double link_probability = static_cast< double >(_params._i_start_links) / _params._i_num_nodes;
        for (int c = 2; c < _params._i_num_chromosomes; ++c) {
          _createRandomGraph(c, link_probability);
        }

        // Evaluate initial costs
        _evaluateChromosomeCosts();
      }

      /**
       * @brief Create minimum spanning tree
       */
      void _createMST(int chromosome_index) noexcept {
        if (_params._i_num_nodes == 0) { return; }

        nt::DynamicArray< bool > visited(_params._i_num_nodes);
        visited.fill(false);
        nt::IntDynamicArray min_edge_node(_params._i_num_nodes);
        min_edge_node.fill(0);
        nt::DoubleDynamicArray min_edge_weight(_params._i_num_nodes);
        min_edge_weight.fill(NT_DOUBLE_MAX);

        visited[0] = true;
        int edges_added = 0;

        for (int i = 1; i < _params._i_num_nodes; ++i) {
          min_edge_weight[i] = _node_distances(0, i);
          min_edge_node[i] = 0;
        }

        for (int i = 1; i < _params._i_num_nodes; ++i) {
          int    next_node = -1;
          double min_weight = NT_DOUBLE_MAX;

          for (int j = 0; j < _params._i_num_nodes; ++j) {
            if (!visited[j] && min_edge_weight[j] < min_weight) {
              next_node = j;
              min_weight = min_edge_weight[j];
            }
          }

          if (next_node == -1) { break; }

          visited[next_node] = true;
          _chromosomes(chromosome_index, next_node, min_edge_node[next_node]) = true;
          _chromosomes(chromosome_index, min_edge_node[next_node], next_node) = true;
          edges_added++;

          for (int j = 0; j < _params._i_num_nodes; ++j) {
            if (!visited[j] && _node_distances(next_node, j) < min_edge_weight[j]) {
              min_edge_weight[j] = _node_distances(next_node, j);
              min_edge_node[j] = next_node;
            }
          }
        }
      }

      /**
       * @brief Create fully connected graph
       */
      void _createFullyConnectedGraph(int chromosome_index) noexcept {
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            _chromosomes(chromosome_index, i, j) = (i != j);
          }
        }
      }

      /**
       * @brief Create random graph
       */
      void _createRandomGraph(int chromosome_index, double linkProbability) noexcept {
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          _chromosomes(chromosome_index, i, i) = false;
        }

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = i + 1; j < _params._i_num_nodes; ++j) {
            if (_rng() < linkProbability) {
              _chromosomes(chromosome_index, i, j) = true;
              _chromosomes(chromosome_index, j, i) = true;
            } else {
              _chromosomes(chromosome_index, i, j) = false;
              _chromosomes(chromosome_index, j, i) = false;
            }
          }
        }
      }

      /**
       * @brief Calculate chromosome cost and repair if needed (working directly on the chromosome)
       */
      double _calculateChromosomeCost(int chromosome_index, bool& needsRepair) noexcept {
        needsRepair = false;

        if (!_isConnected(chromosome_index)) {
          needsRepair = true;
          _repairConnectivity(chromosome_index);
        }

        nt::IntMultiDimArray< 2 > routing;
        _calculateShortestPaths(chromosome_index, routing);
        nt::DoubleMultiDimArray< 2 > capacities;
        _calculateLinkCapacities(chromosome_index, routing, capacities);

        double total_cost = 0.0;

        // Link existence and length costs
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = i + 1; j < _params._i_num_nodes; ++j) {
            if (_chromosomes(chromosome_index, i, j)) {
              total_cost += _params._f_k0 + _params._f_k1 * _node_distances(i, j);
            }
          }
        }

        // Bandwidth costs
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = i + 1; j < _params._i_num_nodes; ++j) {
            if (_chromosomes(chromosome_index, i, j)) {
              total_cost += _params._f_k2 * _node_distances(i, j) * capacities(i, j);
            }
          }
        }

        // Hub costs
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          int degree = 0;
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            if (_chromosomes(chromosome_index, i, j)) degree++;
          }
          if (degree > 1) { total_cost += _params._f_k3; }
        }

        return total_cost;
      }

      /**
       * @brief Evaluate all chromosome costs
       */
      void _evaluateChromosomeCosts() noexcept {
        for (int i = 0; i < _params._i_num_chromosomes; ++i) {
          bool needs_repair = false;
          _chromosome_costs[i] = _calculateChromosomeCost(i, needs_repair);
        }
      }

      /**
       * @brief Check if graph is connected
       */
      bool _isConnected(int chromosome_index) noexcept {
        if (_params._i_num_nodes == 0) { return true; }

        nt::DynamicArray< bool > visited(_params._i_num_nodes);
        visited.fill(false);

        nt::Queue< int > q;

        q.push(0);
        visited[0] = true;

        while (!q.empty()) {
          int node = q.front();
          q.pop();

          for (int i = 0; i < _params._i_num_nodes; ++i) {
            if (_chromosomes(chromosome_index, node, i) && !visited[i]) {
              visited[i] = true;
              q.push(i);
            }
          }
        }

        bool connected = true;
        for (size_t i = 0; i < visited.size(); ++i) {
          if (!visited[i]) {
            connected = false;
            break;
          }
        }
        return connected;
      }

      /**
       * @brief Find connected components
       */
      nt::DynamicArray< nt::IntDynamicArray > _findConnectedComponents(int chromosome_index) noexcept {
        nt::IntDynamicArray component(_params._i_num_nodes);
        component.fill(-1);
        int comp_count = 0;

        for (int start = 0; start < _params._i_num_nodes; ++start) {
          if (component[start] != -1) continue;

          nt::Queue< int > q;
          q.push(start);
          component[start] = comp_count;

          while (!q.empty()) {
            int node = q.front();
            q.pop();

            for (int i = 0; i < _params._i_num_nodes; ++i) {
              if (_chromosomes(chromosome_index, node, i) && component[i] == -1) {
                component[i] = comp_count;
                q.push(i);
              }
            }
          }

          comp_count++;
        }

        nt::DynamicArray< nt::IntDynamicArray > components(comp_count);
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          components[component[i]].push_back(i);
        }

        return components;
      }

      /**
       * @brief Repair connectivity of a disconnected graph
       */
      void _repairConnectivity(int chromosome_index) noexcept {
        if (_isConnected(chromosome_index)) { return; }

        auto components = _findConnectedComponents(chromosome_index);

        if (components.size() <= 1) { return; }

        for (size_t c = 1; c < components.size(); ++c) {
          int    min_i = -1, min_j = -1;
          double min_dist = NT_DOUBLE_MAX;

          for (size_t prev_c = 0; prev_c < c; ++prev_c) {
            for (int i: components[prev_c]) {
              for (int j: components[c]) {
                if (_node_distances(i, j) < min_dist) {
                  min_dist = _node_distances(i, j);
                  min_i = i;
                  min_j = j;
                }
              }
            }
          }

          if (min_i != -1 && min_j != -1) {
            _chromosomes(chromosome_index, min_i, min_j) = true;
            _chromosomes(chromosome_index, min_j, min_i) = true;
          }
        }
      }

      /**
       * @brief Calculate shortest paths using Floyd-Warshall
       */
      void _calculateShortestPaths(int chromosome_index, nt::IntMultiDimArray< 2 >& next) noexcept {
        // Initialize result directly
        next.setDimensions(_params._i_num_nodes, _params._i_num_nodes);
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            next(i, j) = -1;
          }
        }

        nt::DoubleMultiDimArray< 2 > dist;
        dist.setDimensions(_params._i_num_nodes, _params._i_num_nodes);
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            dist(i, j) = NT_DOUBLE_MAX;
          }
        }

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          dist(i, i) = 0;
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            if (_chromosomes(chromosome_index, i, j)) {
              dist(i, j) = _node_distances(i, j);
              next(i, j) = j;
            }
          }
        }

        for (int k = 0; k < _params._i_num_nodes; ++k) {
          for (int i = 0; i < _params._i_num_nodes; ++i) {
            for (int j = 0; j < _params._i_num_nodes; ++j) {
              if (dist(i, k) != NT_DOUBLE_MAX && dist(k, j) != NT_DOUBLE_MAX && dist(i, k) + dist(k, j) < dist(i, j)) {
                dist(i, j) = dist(i, k) + dist(k, j);
                next(i, j) = next(i, k);
              }
            }
          }
        }
      }

      /**
       * @brief Calculate link capacities based on routing
       */
      void _calculateLinkCapacities(int                              chromosome_index,
                                    const nt::IntMultiDimArray< 2 >& routing,
                                    nt::DoubleMultiDimArray< 2 >&    capacities) noexcept {
        // Initialize result directly
        capacities.setDimensions(_params._i_num_nodes, _params._i_num_nodes);
        capacities.setBitsToZero();

        for (int src = 0; src < _params._i_num_nodes; ++src) {
          for (int dst = 0; dst < _params._i_num_nodes; ++dst) {
            if (src == dst) continue;

            nt::IntDynamicArray path;
            int                 current = src;
            while (current != dst) {
              path.push_back(current);
              current = routing(current, dst);
              if (current == -1) break;
            }

            if (current == dst) {
              path.push_back(dst);

              for (size_t i = 0; i < path.size() - 1; ++i) {
                int u = path[i];
                int v = path[i + 1];
                capacities(u, v) += _demand(src, dst);
                capacities(v, u) += _demand(src, dst);
              }
            }
          }
        }
      }

      /**
       * @brief Perform crossover operation
       */
      void _performCrossover(int targetIndex) noexcept {
        // Select candidates
        nt::DynamicArray< int > candidates(_params._i_num_chromosomes);
        for (int i = 0; i < static_cast< int >(candidates.size()); ++i) {
          candidates[i] = i;
        }
        _fisherYatesShuffle(candidates);
        candidates.resize(_params._i_crossover_b);

        // Sort by cost
        nt::sort(candidates, [this](int a, int b) { return _chromosome_costs[a] < _chromosome_costs[b]; });

        candidates.resize(_params._i_crossover_a);

        // Calculate probabilities
        nt::DoubleDynamicArray inverse_costs(_params._i_crossover_a);
        double                 sum_inverse = 0.0;

        for (int i = 0; i < _params._i_crossover_a; ++i) {
          inverse_costs[i] = 1.0 / _chromosome_costs[candidates[i]];
          sum_inverse += inverse_costs[i];
        }

        for (int i = 0; i < _params._i_crossover_a; ++i) {
          inverse_costs[i] /= sum_inverse;
        }

        // Work directly on target chromosome
        // Initialize target chromosome
        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = 0; j < _params._i_num_nodes; ++j) {
            _chromosomes(targetIndex, i, j) = false;
          }
        }

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = i + 1; j < _params._i_num_nodes; ++j) {
            double r = _rng();
            double cumul = 0.0;

            for (int p = 0; p < _params._i_crossover_a; ++p) {
              cumul += inverse_costs[p];
              if (r <= cumul) {
                _chromosomes(targetIndex, i, j) = _chromosomes(candidates[p], i, j);
                _chromosomes(targetIndex, j, i) = _chromosomes(candidates[p], i, j);
                break;
              }
            }
          }
        }
      }


      /**
       * @brief Apply the best chromosome to the graph
       */
      void _applyChromosomeToGraph(int chromosome_index) noexcept {
        _graph.clear();

        nt::DynamicArray< Node >      nodes;
        nt::NumericalMap< int, Node > id_to_node;
        nodes.reserve(_params._i_num_nodes);

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          Node node = _graph.addNode();
          nodes.push_back(node);
          int node_id = _graph.id(node);
          id_to_node[node_id] = node;
        }

        for (const auto& [node_id, node]: id_to_node) {
          if (node_id >= 0 && node_id < static_cast< int >(_node_positions_backup.size())) {
            Position pos;
            pos[0] = _node_positions_backup[node_id].first;
            pos[1] = _node_positions_backup[node_id].second;
            _positions[node] = pos;

            _populations[node] = _node_populations_backup[node_id];
          }
        }

        nt::IntMultiDimArray< 2 > routing;
        _calculateShortestPaths(chromosome_index, routing);
        nt::DoubleMultiDimArray< 2 > capacities;
        _calculateLinkCapacities(chromosome_index, routing, capacities);

        for (int i = 0; i < _params._i_num_nodes; ++i) {
          for (int j = i + 1; j < _params._i_num_nodes; ++j) {
            if (_chromosomes(chromosome_index, i, j)) {
              typename Digraph::Arc arc1 = _graph.addArc(nodes[i], nodes[j]);
              typename Digraph::Arc arc2 = _graph.addArc(nodes[j], nodes[i]);

              _weights[arc1] = _node_distances(i, j);
              _weights[arc2] = _node_distances(i, j);

              _capacities[arc1] = capacities(i, j) * _params._f_k2;
              _capacities[arc2] = capacities(j, i) * _params._f_k2;
            }
          }
        }
      }

      public:
      /**
       * @brief Get the total cost of the generated topology
       */
      double getTotalCost() const noexcept { return _total_cost; }
      /**
       * @brief Get the generated graph
       */
      const Digraph& getGraph() const noexcept { return _graph; }
      Digraph&       getGraph() noexcept { return _graph; }

      /**
       * @brief Get positions map
       */
      const PositionsMap& getPositions() const noexcept { return _positions; }

      /**
       * @brief Get weights map
       */
      const MetricMap& getWeights() const noexcept { return _weights; }

      /**
       * @brief Get capacities map
       */
      const CapacityMap& getCapacities() const noexcept { return _capacities; }
    };

  }   // namespace te
}   // namespace nt

#endif