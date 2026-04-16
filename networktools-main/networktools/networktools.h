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
 * @file networktools.h
 * @brief Main header file for the Networktools library.
 *
 * This file includes all core modules and components of the Networktools library.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_NETWORKTOOLS_H_
#define _NT_NETWORKTOOLS_H_

#include "ntconfig.h"

//-----------------------------------------------------------------------------
// [MODULE] Core
//-----------------------------------------------------------------------------

#include "core/common.h"
#include "core/logging.h"
#include "core/arrays.h"
#include "core/algebra.h"
#include "core/any.h"
#include "core/color.h"
#include "core/counter.h"
#include "core/csv.h"
#include "core/dim2.h"
#include "core/format.h"
#include "core/json.h"
#include "core/maps.h"
#include "core/random.h"
#include "core/sets.h"
#include "core/sort.h"
#include "core/string.h"
#include "core/time_measure.h"
#include "core/tolerance.h"
#include "core/traits.h"
#include "core/tuple.h"
#include "core/typeid.h"
#include "core/unionfind.h"
#include "core/variant.h"
#include "core/xml.h"

// Heaps
#include "core/bin_heap.h"
#include "core/binomial_heap.h"
#include "core/bucket_heap.h"
#include "core/dheap.h"
#include "core/fib_heap.h"
#include "core/pairing_heap.h"
#include "core/quad_heap.h"
#include "core/quick_heap.h"
#include "core/radix_heap.h"

// Containers
#include "core/queue.h"
#include "core/stack.h"

// Concepts
#include "core/concepts/bpgraph.h"
#include "core/concepts/digraph.h"
#include "core/concepts/graph.h"
#include "core/concepts/graph_components.h"
#include "core/concepts/heap.h"
#include "core/concepts/maps.h"
#include "core/concepts/matrix_maps.h"
#include "core/concepts/path.h"
#include "core/concepts/tags.h"

//-----------------------------------------------------------------------------
// [MODULE] Graphs
//-----------------------------------------------------------------------------

// Graph structures
#include "graphs/tools.h"
#include "graphs/adaptors.h"
#include "graphs/compact_graph.h"
#include "graphs/csr_graph.h"
#include "graphs/demand_graph.h"
#include "graphs/edge_graph.h"
#include "graphs/elevator.h"
#include "graphs/full_graph.h"
#include "graphs/graph_element_set.h"
#include "graphs/graph_maps.h"
#include "graphs/graph_properties.h"
#include "graphs/grid_graph.h"
#include "graphs/hypercube_graph.h"
#include "graphs/list_graph.h"
#include "graphs/matrix_maps.h"
#include "graphs/mutable_graph.h"
#include "graphs/path.h"
#include "graphs/smart_graph.h"
#include "graphs/static_graph.h"

// Graph algorithms
#include "graphs/algorithms/bellman_ford.h"
#include "graphs/algorithms/betweenness_centrality.h"
#include "graphs/algorithms/bfs.h"
#include "graphs/algorithms/capacity_scaling.h"
#include "graphs/algorithms/chordality.h"
#include "graphs/algorithms/christofides_tsp.h"
#include "graphs/algorithms/circulation.h"
#include "graphs/algorithms/connectivity.h"
#include "graphs/algorithms/cost_scaling.h"
#include "graphs/algorithms/csp.h"
#include "graphs/algorithms/cycle_canceling.h"
#include "graphs/algorithms/dfs.h"
#include "graphs/algorithms/dijkstra.h"
#include "graphs/algorithms/dynamic_dijkstra.h"
#include "graphs/algorithms/edmonds_karp.h"
#include "graphs/algorithms/euler.h"
#include "graphs/algorithms/floyd_warshall.h"
#include "graphs/algorithms/fractional_matching.h"
#include "graphs/algorithms/gomory_hu.h"
#include "graphs/algorithms/graph_generators.h"
#include "graphs/algorithms/graph_layout.h"
#include "graphs/algorithms/greedy_tsp.h"
#include "graphs/algorithms/grosso_locatelli_pullan_mc.h"
#include "graphs/algorithms/hao_orlin.h"
#include "graphs/algorithms/hartmann_orlin_mmc.h"
#include "graphs/algorithms/howard_mmc.h"
#include "graphs/algorithms/insertion_tsp.h"
#include "graphs/algorithms/isp.h"
#include "graphs/algorithms/johnson.h"
#include "graphs/algorithms/karp_mmc.h"
#include "graphs/algorithms/kruskal.h"
#include "graphs/algorithms/matching.h"
#include "graphs/algorithms/max_cardinality_search.h"
#include "graphs/algorithms/mcf.h"
#include "graphs/algorithms/min_cost_arborescence.h"
#include "graphs/algorithms/nagamochi_ibaraki.h"
#include "graphs/algorithms/nearest_neighbor_tsp.h"
#include "graphs/algorithms/network_simplex.h"
#include "graphs/algorithms/opt2_tsp.h"
#include "graphs/algorithms/planarity.h"
#include "graphs/algorithms/preflow.h"
#include "graphs/algorithms/suurballe.h"
#include "graphs/algorithms/tree_decomposition.h"
#include "graphs/algorithms/vclp.h"
#include "graphs/algorithms/vertex_cover.h"
#include "graphs/algorithms/vf2.h"
#include "graphs/algorithms/vf2pp.h"

// Graph I/O
#include "graphs/io/binary_io.h"
#include "graphs/io/io.h"
#include "graphs/io/jgf_io.h"
#include "graphs/io/lgf_reader.h"
#include "graphs/io/lgf_writer.h"
#include "graphs/io/networkx_io.h"
#include "graphs/io/rapid_networkx_io.h"
#include "graphs/io/pace_io.h"
#include "graphs/io/repetita_io.h"
#include "graphs/io/sndlib_reader.h"

//-----------------------------------------------------------------------------
// [MODULE] Traffic Engineering (TE)
//-----------------------------------------------------------------------------

#include "te/io/te_instance.h"
#include "te/algorithms/bg_weights.h"
#include "te/algorithms/bley_branch_and_cut.h"
#include "te/algorithms/cg4sr.h"
#include "te/algorithms/cold.h"
#include "te/algorithms/cost_functions.h"
#include "te/algorithms/ecmp.h"
#include "te/algorithms/inet.h"
#include "te/algorithms/segment_routing.h"
#include "te/algorithms/spr.h"
#include "te/algorithms/dynamic_spr.h"
#include "te/algorithms/srte.h"
#include "te/algorithms/tm_generators.h"

//-----------------------------------------------------------------------------
// [MODULE] Mathematical Programming (MP)
//-----------------------------------------------------------------------------

#include "mp/mp.h"

#endif