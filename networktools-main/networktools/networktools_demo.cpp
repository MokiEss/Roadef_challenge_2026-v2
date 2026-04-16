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
 * @file networktools_demo.cpp
 * @brief Networktools library usage examples and programming guide.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

//-----------------------------------------------------------------------------
// [SECTION] Introduction
//-----------------------------------------------------------------------------

// This file serves as a programming guide and a collection of examples for using the Networktools library.
// It demonstrates key features and addons of the library through small, self-contained code snippets.
// These examples are designed to be easily copy-pasted into your projects.

// [Compilation Instructions]
// - Linux:   g++ -O3 -std=c++20 networktools_demo.cpp
// - Windows: cl /O2 /std:c++20 networktools_demo.cpp

// [Documentation References]
// As networktools follows the same API design as the LEMON library, it is also a good idea to look at LEMON documentation
//  - Slides :   https://gitlab.tech.orange/more/networktools/-/blob/master/doc/lemon-intro-presentation.pdf
//  - Tutorials: http://lemon.cs.elte.hu/pub/tutorial/index.html

// [How to Use]
// - Each example is marked with one or more hashtags (e.g., #dijkstra, #mip, #graphs).
// - Use your IDE's search functionality (e.g., Ctrl+F) to locate examples by keyword.
// - Examples are grouped into sections for easier navigation.

// [File Organization]

// [SECTION] Mathematical Programming (MP) - Solver Configuration
// [SECTION] Forward Declarations, Helpers
// [SECTION] Core
// [SECTION] Graphs
// [SECTION] Traffic Engineering (TE)
// [SECTION] Mathematical Programming (MP) - Usage Examples

#include "networktools.h"

// If you intend to build this file outside of the networktools folder,
// you will need to update the DATA_DIR value accordingly
#define DATA_DIR "../data"
#define DATA(path) DATA_DIR "/" path

//-----------------------------------------------------------------------------
// [SECTION] Mathematical Programming (MP) - Solver Configuration
//-----------------------------------------------------------------------------

// The MP module provides a unified interface for linear and mixed-integer programming.
// To use the MP module, you must select a solver backend by including the appropriate
// solver header from the mp/ directory.
//
// See mp/README.md for detailed documentation.

// DUMMY solver (default)
// ======================
// #dummy

// By default, this demo file uses the DUMMY solver to allow building without any
// solver installation. Note: This solver does not perform any actual optimization.

// If you have a solver installed (e.g., CPLEX, HIGHS), include the appropriate
// header (e.g., mp/mp_cplex.h or mp/mp_highs.h) to enable real optimization.
// See the solver-specific sections below for installation and build instructions.

#include "mp/mp_dummy.h"
using LPModel = nt::mp::dummy::LPModel;
using MIPModel = nt::mp::dummy::MIPModel;
using MPEnv = nt::mp::dummy::MPEnv;

// CPLEX solver
// ============
// #cplex

// 1 - Uncomment the following lines
// #include "mp/mp_cplex.h"
// using LPModel = nt::mp::cplex::LPModel;
// using MIPModel = nt::mp::cplex::MIPModel;
// using MPEnv = nt::mp::cplex::MPEnv;

// 2 - Install CPLEX on your system (here we take version 12.10)
// Ubuntu  : https://www.ibm.com/support/pages/installation-ibm-ilog-cplex-optimization-studio-linux-platforms
// Windows : https://www.ibm.com/docs/en/icos/12.10.0?topic=v12100-installing-cplex-optimization-studio

// 3 - Build
// Ubuntu  : g++ -O3 -std=c++20 networktools_demo.cpp -lcplex -I/opt/ibm/ILOG/CPLEX_Studio1210/cplex/include -L/opt/ibm/ILOG/CPLEX_Studio1210/cplex/lib/x86-64_linux/static_pic
// Windows : cl /O2 /std:c++20 networktools_demo.cpp cplex12100.lib /I "C:\Program Files\IBM\ILOG\CPLEX_Studio1210\cplex\include" /link /LIBPATH:"C:\Program
// Files\IBM\ILOG\CPLEX_Studio1210\cplex\lib\x64_windows_msvc14\stat_mda"

// GUROBI solver
// =============
// #gurobi

// 1 - Uncomment the following lines
// #include "mp/mp_gurobi.h"
// using LPModel = nt::mp::gurobi::LPModel;
// using MIPModel = nt::mp::gurobi::MIPModel;
// using MPEnv = nt::mp::gurobi::MPEnv;

// 2 - Install GUROBI on your system (here we take version 1103)
//     https://support.gurobi.com/hc/en-us/articles/4534161999889-How-do-I-install-Gurobi-Optimizer

// 3 - Build
// Ubuntu  : g++ -std=c++20 -O3 networktools_demo.cpp -lgurobi_c++ -lgurobi110 -L/opt/gurobi1103/linux64/lib/ -I/opt/gurobi1103/linux64/include/
// Windows : TODO but should be similar to cplex

// HIGHS solver
// ============
// #highs

// 1 - Uncomment the following lines
// #include "mp/mp_highs.h"
// using LPModel = nt::mp::highs::LPModel;
// using MIPModel = nt::mp::highs::MIPModel;
// using MPEnv = nt::mp::highs::MPEnv;

// 2 - Install HIGHS on your system
//     https://ergo-code.github.io/HiGHS/dev/installation/

// 3 - Build
// Ubuntu  : g++ -std=c++20 -O3 networktools_demo.cpp -lhighs -L/usr/local/lib/ -I/usr/local/include/highs
// Windows : TODO

// GLPK solver
// ===========
// #glpk

// 1 - Uncomment the following lines
// #include "mp/mp_glpk.h"
// using LPModel = nt::mp::glpk::LPModel;
// using MIPModel = nt::mp::glpk::MIPModel;
// using MPEnv = nt::mp::glpk::MPEnv;

// 2 - Install GLPK on your system (here we take version 4.65)
// Ubuntu  : install GLPK on your system by running 'sudo apt-get install libglpk-dev'
// Windows : download the latest version of GLPK from sourceforge (https://sourceforge.net/projects/winglpk), then extract the zip archive to your C: drive.

// 3 - Build
// Ubuntu  : g++ -O3 -std=c++20 networktools_demo.cpp -lglpk
// Windows : cl /O2 /std:c++20 networktools_demo.cpp glpk_4_65.lib /I "C:\glpk-4.65\src" /link /LIBPATH:"C:\glpk-4.65\w64"

// CBC solver
// ==========
// #cbc

// 1 - Uncomment the following lines
// #include "mp/mp_cbc.h"
// using MIPModel = nt::mp::cbc::MIPModel;

// 2 - Install cbc on your system (here we take version 2.10.7)
// Ubuntu  : install cbc on your system by running 'sudo apt-get install coinor-libcbc-dev'
// Windows : TODO

// 3 - Build
// Ubuntu  : g++ -O3 -std=c++20 networktools_demo.cpp -lOsiClp -lClp -lCoinUtils -lCbc -lCbcSolver
// Windows : TODO

// CLP solver
// ==========
// #clp

// 1 - Uncomment the following lines
// #include "mp/mp_clp.h"
// using LPModel = nt::mp::clp::LPModel;
// using MPEnv = nt::mp::clp::MPEnv;

// 2 - Install cbc on your system (here we take version 1.17.5)
// Ubuntu  : install cbc on your system by running 'sudo apt-get install coinor-libclp-dev'
// Windows : TODO

// 3 - Build
// Ubuntu  : g++ -O3 -std=c++20 networktools_demo.cpp -lOsiClp -lClp -lCoinUtils
// Windows : TODO


//-----------------------------------------------------------------------------
// [SECTION] Forward Declarations, Helpers
//-----------------------------------------------------------------------------

//
void printValues(nt::ArrayView< int > a) {
  for (int i = 0; i < a.size(); ++i)
    LOG_F(INFO, "{}", a[i]);
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(int argc, char** argv) {
  nt::Logging::init(argc, argv);
  nt::Logging::addFile("networktools_demo.log", nt::Logging::FileMode::Truncate, nt::Logging::NamedVerbosity::Verbosity_MAX);

  //-----------------------------------------------------------------------------
  // [SECTION] Core
  //-----------------------------------------------------------------------------

  // Demonstrates core functionalities such as arrays, JSON parsing, and logging.

  // #array

  // #ArrayView
  {
    LOG_SCOPE_F(INFO, "ArrayView");

    nt::IntDynamicArray arr1 = {1, 2, 3};
    std::vector< int >  arr2 = {1, 2, 3};
    int                 arr3[] = {1, 2, 3};

    printValues(arr1);
    printValues(arr2);
    printValues(arr3);
  }

  // #FixedSizeArray
  {
    nt::FixedSizeArray< const char* > arr(argc);

    for (int i = 0; i < arr.size(); ++i)
      arr[i] = argv[i];
  }

  // #StaticArray
  {
    // Create a static array of ints of size 3
    nt::StaticArray< int, 3 > arr;
    arr.add(1000);
    arr.add(1001);
    arr.add(1002);

    // Create and fill a static array of ints of size 3
    nt::StaticArray< int, 3 > arr_filled(true);
    arr_filled[0] = 1000;
    arr_filled[1] = 1001;
    arr_filled[2] = 1002;
  }

  // #file #json #load
  {
    LOG_SCOPE_F(INFO, "Parse JSON file");

    // Load the content of a JSON file passed in argument
    nt::ByteBuffer file;
    if (argc > 1 && file.load(argv[1])) {   // If the load succeeds
      nt::JSONDocument document;
      document.Parse(file.data());   // Parse the file's content
      if (document.HasParseError()) {
        LOG_F(ERROR, "Parse Error: Document is not a valid JSON file");
      } else
        LOG_F(INFO, "JSON file {} parsed successfully", argv[1]);
    } else {
      LOG_F(INFO, "No JSON filename passed in argument");
    }
  }

  //-----------------------------------------------------------------------------
  // [SECTION] Graphs
  //-----------------------------------------------------------------------------
  // #graphs

  // Demonstrates graph-related functionalities, including:
  // - Node and arc iteration
  // - Graph readers and writers
  // - Random graph generation
  // - Graph algorithms (e.g., shortest paths, connected components)

  // Node and arc iteration
  // #digraph #nodeit #arcit #outarcit #inarcit #iteration #degree #indegree #outdegree
  {
    LOG_SCOPE_F(INFO, "Node and Arc iteration");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Iterate over the nodes
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} has out-degree {} and in-degree {}", digraph.id(node), nt::graphs::countOutArcs(digraph, node), nt::graphs::countInArcs(digraph, node));
    }

    // Iterate over the arcs
    for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
      const Digraph::Node s = digraph.source(arc);
      const Digraph::Node t = digraph.target(arc);
      LOG_F(INFO, "Arc {} has source = {} and target = {}", digraph.id(arc), digraph.id(s), digraph.id(t));
    }

    // Iterate over the out arcs of node v0
    for (Digraph::OutArcIt out_arc(digraph, v0); out_arc != nt::INVALID; ++out_arc) {
      const Digraph::Node s = digraph.source(out_arc);
      const Digraph::Node t = digraph.target(out_arc);
      LOG_F(INFO, "Node {} has out-arc {} ({} -> {})", digraph.id(v0), digraph.id(out_arc), digraph.id(s), digraph.id(t));
    }

    // Iterate over the in arcs of node v2
    for (Digraph::InArcIt in_arc(digraph, v2); in_arc != nt::INVALID; ++in_arc) {
      const Digraph::Node s = digraph.source(in_arc);
      const Digraph::Node t = digraph.target(in_arc);
      LOG_F(INFO, "Node {} has in-arc {} ({} -> {})", digraph.id(v2), digraph.id(in_arc), digraph.id(s), digraph.id(t));
    }
  }

  // Node and edge iteration
  // #graph #nodeit #edgeit #iteration #undirected
  {
    LOG_SCOPE_F(INFO, "Node and Edge iteration");

    // Let's define a graph
    using Graph = nt::graphs::SmartGraph;

    // Build the following  graph :
    //        v0 -- v1 -- v2
    //        |           |
    //        +---- v3 ---+

    Graph graph;

    Graph::Node v0 = graph.addNode();
    Graph::Node v1 = graph.addNode();
    Graph::Node v2 = graph.addNode();
    Graph::Node v3 = graph.addNode();

    graph.addEdge(v0, v1);
    graph.addEdge(v1, v2);
    graph.addEdge(v3, v2);
    graph.addEdge(v0, v3);

    LOG_F(INFO, "Number of nodes {}", graph.nodeNum());
    LOG_F(INFO, "Number of edges {}", graph.edgeNum());

    // An undirected graph G=(V,E) is internally represented as a bidirected graph and thus contains 2*|E| arcs
    LOG_F(INFO, "(Number of arcs {})", graph.arcNum());

    // Iterate over the nodes
    for (Graph::NodeIt node(graph); node != nt::INVALID; ++node) {
      LOG_F(INFO,
            "Node {} has degree {} (out-degree {} and in-degree {})",
            graph.id(node),
            nt::graphs::countIncEdges(graph, node),
            nt::graphs::countOutArcs(graph, node),
            nt::graphs::countInArcs(graph, node));
    }

    // Iterate over the edges
    for (Graph::EdgeIt edge(graph); edge != nt::INVALID; ++edge) {
      const Graph::Node u = graph.u(edge);
      const Graph::Node v = graph.v(edge);
      LOG_F(INFO, "Edge {} has endpoints u = {} and v = {}", graph.id(edge), graph.id(u), graph.id(v));
    }

    // Iterate over the incident edges of node v2
    for (Graph::IncEdgeIt inc_edge(graph, v2); inc_edge != nt::INVALID; ++inc_edge) {
      const Graph::Node u = graph.u(inc_edge);
      const Graph::Node v = graph.v(inc_edge);
      LOG_F(INFO, "Node {} is incident to edge {} ({} -- {})", graph.id(v2), graph.id(inc_edge), graph.id(u), graph.id(v));
    }

    // Since the graph is bidirected all the methods for iterating over the arcs are available.

    // Iterate over the arcs
    for (Graph::ArcIt arc(graph); arc != nt::INVALID; ++arc) {
      const Graph::Node s = graph.source(arc);
      const Graph::Node t = graph.target(arc);
      LOG_F(INFO, "Arc {} has source = {} and target = {}", graph.id(arc), graph.id(s), graph.id(t));
    }

    // Iterate over the out arcs of node v0
    for (Graph::OutArcIt out_arc(graph, v0); out_arc != nt::INVALID; ++out_arc) {
      const Graph::Node s = graph.source(out_arc);
      const Graph::Node t = graph.target(out_arc);
      LOG_F(INFO, "Node {} has out-arc {} ({} -> {})", graph.id(v0), graph.id(out_arc), graph.id(s), graph.id(t));
    }

    // Iterate over the in arcs of node v2
    for (Graph::InArcIt in_arc(graph, v2); in_arc != nt::INVALID; ++in_arc) {
      const Graph::Node s = graph.source(in_arc);
      const Graph::Node t = graph.target(in_arc);
      LOG_F(INFO, "Node {} has in-arc {} ({} -> {})", graph.id(v2), graph.id(in_arc), graph.id(s), graph.id(t));
    }
  }

  // Random graph generation
  // #digraph #random #erdosrenyi #connectedcomponents
  {
    LOG_SCOPE_F(INFO, "Random graph generation");

    // Let's define the type of the generated graph
    using Graph = nt::graphs::SmartGraph;

    Graph graph;

    // Run Erdos-Renyi algorithm to generate a graph of 12 vertices with an arc probability of 0.3
    nt::graphs::generateErdosRenyiGraph(graph, 12, 0.1);

    // Compute the connected components of the generated graph
    Graph::NodeMap< int > components_map(graph);
    const int             i_num_components = nt::graphs::connectedComponents(graph, components_map);
    // Display some infos
    LOG_F(INFO, "Number of nodes {}", graph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", graph.arcNum());
    LOG_F(INFO, "Number of connected components {}", i_num_components);

    for (Graph::NodeIt node(graph); node != nt::INVALID; ++node)
      LOG_F(INFO, "Node {} belongs to component {}", graph.id(node), components_map[node]);
  }

  // Graph copy
  // #copy #DigraphCopy
  {
    LOG_SCOPE_F(INFO, "Graph copy");

    // Let's define the type of the original digraph i.e. the graph we want to copy from.
    using OrigGraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+
    OrigGraph orig_graph;

    OrigGraph::Node v0 = orig_graph.addNode();
    OrigGraph::Node v1 = orig_graph.addNode();
    OrigGraph::Node v2 = orig_graph.addNode();
    OrigGraph::Node v3 = orig_graph.addNode();

    orig_graph.addArc(v0, v1);
    orig_graph.addArc(v1, v2);
    orig_graph.addArc(v3, v2);
    orig_graph.addArc(v0, v3);

    // Let's define the type of the new digraph i.e. the graph we want to copy to.
    using NewGraph = nt::graphs::StaticDigraph;

    NewGraph new_graph;

    nt::graphs::DigraphCopy< OrigGraph, NewGraph > cg(orig_graph, new_graph);

    // Create references for the nodes
    // This map allows to retrieve an OrigGraph's node from a NewGraph's node
    OrigGraph::NodeMap< NewGraph::Node > nr(orig_graph);
    cg.nodeRef(nr);

    // Create cross references (inverse) for the arcs
    // This map allows to retrieve a NewGraph's arc from an OrigGraph's arc
    NewGraph::ArcMap< OrigGraph::Arc > acr(new_graph);
    cg.arcCrossRef(acr);

    // Copy an arc map
    OrigGraph::ArcMap< double > oamap(orig_graph);
    NewGraph::ArcMap< double >  namap(new_graph);
    cg.arcMap(oamap, namap);

    // Execute copying
    cg.run();

    // Display graph infos
    LOG_F(INFO, "OrigGraph : Number of nodes {}", orig_graph.nodeNum());
    LOG_F(INFO, "OrigGraph : Number of arcs {}", orig_graph.arcNum());

    LOG_F(INFO, "NewGraph : Number of nodes {}", orig_graph.nodeNum());
    LOG_F(INFO, "NewGraph : Number of arcs {}", orig_graph.arcNum());
  }

  // Read Sndlib file from stream
  // #sndlib #xml #SmartDigraph #demandgraph #trafficmatrix
  {
    LOG_SCOPE_F(INFO, "Read Sndlib file");

    // We assume that the following SNDLib XML file is present.
    // You probably need to change this to load your own file.
    const char* sz_filename = DATA("abilene.xml");

    // Define the type of graph that represents the network topology  (https://en.wikipedia.org/wiki/Flow_network)
    using Digraph = nt::graphs::SmartDigraph;

    // Define a demand graph : a digraph sharing the same node set as a Digraph.
    using DemandGraph = nt::graphs::DemandGraph< Digraph >;

    nt::graphs::SndlibReader< Digraph, DemandGraph > reader;

    // Load the data containing the network and demand graph
    if (reader.loadFile(sz_filename)) {
      // Step 1 - Load the flow network with the arc capacities
      Digraph                   network;
      Digraph::ArcMap< double > capacity_map(network);

      reader.readNetwork(network, capacity_map);

      // Step 2 - Load the associated demand graph and the demand values.
      DemandGraph                   demand_graph(network);
      DemandGraph::ArcMap< double > demand_values(demand_graph);

      reader.readDemandGraph(demand_graph, demand_values);

      // Display some infos
      LOG_F(INFO, "Number of nodes {}", network.nodeNum());
      LOG_F(INFO, "Number of arcs {}", network.arcNum());
      LOG_F(INFO, "Number of demands {}", demand_graph.arcNum());

      LOG_F(INFO, "Arc with id {} has capacity {}", 0, capacity_map[network.arcFromId(0)]);
      LOG_F(INFO, "Demand with id {} has value {}", 0, demand_values[demand_graph.arcFromId(0)]);
    } else {
      LOG_F(ERROR, "Cannot open file '{}'", sz_filename);
    }
  }

  // Graph generation with Cold
  {
    using Digraph = nt::graphs::SmartDigraph;
    nt::te::COLD< Digraph > cold;
    // Minimal config - just set network size: 30 nodes
    nt::te::COLD< Digraph >::Parameters params;
    params.setIntegerParam(nt::te::COLD< Digraph >::Parameters::NUM_NODES, 20);
    cold.setParameters(params);
    double      cost = cold.run();
    const auto& graph = cold.getGraph();

    LOG_F(INFO, "Cold generated graph with {} nodes and {} arcs (cost={})", graph.nodeNum(), graph.arcNum(), cost);
  }

  // Traffic generation for simple graph
  // #digraph #trafficgeneration
  {
    LOG_SCOPE_F(INFO, "Traffic generation for a simple graph");

    // Define aliases for convenience
    using Digraph = nt::graphs::SmartDigraph;

    // Define the basic graph types: Node, Arc, ...
    DIGRAPH_TYPEDEFS(Digraph);

    // Build a very simple digraph with one arc
    Digraph       digraph;
    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Arc           arc = digraph.addArc(v0, v1);

    // Create an arc map to store the arc capacity and set them to 1
    DoubleArcMap capacity_map(digraph, 1.);

    // Basic usage with default weight map
    nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg(digraph, capacity_map);
    LOG_F(INFO, "Check validity of input graph and capacity map: {}", tmg.checkInputGraphNodeArc());

    const int total_traffic = 1000;
    tmg.run(total_traffic);

    LOG_F(INFO, "Load check arc load after run: {}", tmg.checkLoad(total_traffic));
    LOG_F(INFO, "Final traffic matrice T (0, 1) = {}", tmg.getTrafficMatrix()(0, 1));
    LOG_F(INFO, "Load arc (0, 1) = {}", tmg.getLoad(arc));

    // Example with custom weight map
    DoubleArcMap                                            custom_weights(digraph, 2.5);   // Higher weights for all arcs
    nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg_custom(digraph, capacity_map, custom_weights, 12345);
    tmg_custom.run(total_traffic);
    LOG_F(INFO, "Final traffic matrice with custom weights T (0, 1) = {}", tmg_custom.getTrafficMatrix()(0, 1));
  }

  // Traffic generation for Abilene network
  // #digraph #trafficgeneration #abilene #sndlib
  {
    LOG_SCOPE_F(INFO, "Traffic generation for Abilene");

    // Define aliases for convenience
    using Digraph = nt::graphs::SmartDigraph;
    using DemandGraph = nt::graphs::DemandGraph< Digraph >;

    // Define the basic graph types: Node, Arc, ...
    DIGRAPH_TYPEDEFS(Digraph);

    const char*               sz_filename = DATA("abilene.xml");
    Digraph                   digraph;
    Digraph::ArcMap< double > capacity_map(digraph);
    DemandGraph               demand_graph(digraph);

    // Read the file
    nt::graphs::SndlibReader< Digraph, DemandGraph > reader;
    if (reader.loadFile(sz_filename)) {
      reader.readNetwork(digraph, capacity_map);

      // Basic usage - backward compatibility maintained
      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg(digraph, capacity_map);
      const int                                               total_traffic = 1000;
      tmg.run(total_traffic);

      LOG_F(INFO, "Load check arc load after run: {}", tmg.checkLoad(total_traffic));
      LOG_F(INFO, "Final traffic matrice T (1, 2) = {}", tmg.getTrafficMatrix()(1, 2));

      // Example with Parameters structure and custom weight map
      DoubleArcMap                                                        custom_weights(digraph, 1.5);           // Custom weights for routing
      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap >::Parameters params(custom_weights,                  // custom weight map
                                                                                 54321,                           // seed
                                                                                 nt::Tolerance< double >(1e-6),   // tolerance
                                                                                 false,                           // use_betweenness_centrality
                                                                                 true,                            // normalized
                                                                                 false                            // endpoints
      );

      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg_with_params(digraph, capacity_map, params);
      tmg_with_params.run(total_traffic);
      LOG_F(INFO, "Traffic generation with Parameters structure completed");
      LOG_F(INFO, "Parameters seed: {}", tmg_with_params.getParameters().seed);
    } else {
      LOG_F(ERROR, "Cannot open file {}", sz_filename);
    }
  }

  // Graph & traffic generation with Cold and Traffic generator: 30 nodes
  {
    using Digraph = nt::graphs::SmartDigraph;
    // Define the basic graph types: Node, Arc, ...
    DIGRAPH_TYPEDEFS(Digraph);

    nt::te::COLD< Digraph, double > cold;
    // Minimal config - just set network size: 30 nodes
    nt::te::COLD< Digraph, double >::Parameters params;
    params.setIntegerParam(nt::te::COLD< Digraph, double >::Parameters::NUM_NODES, 30);
    params.setIntegerParam(nt::te::COLD< Digraph, double >::Parameters::SEED, 42);
    cold.setParameters(params);
    double      cost = cold.run();
    const auto& digraph = cold.getGraph();

    LOG_F(INFO, "Cold generated graph with {} nodes and {} arcs (cost={})", digraph.nodeNum(), digraph.arcNum(), cost);

    DoubleArcMap capacity_map(digraph);
    DoubleArcMap weights_map(digraph);
    nt::graphs::mapCopy(digraph, cold.getCapacities(), capacity_map);
    nt::graphs::mapCopy(digraph, cold.getWeights(), weights_map);

    nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap >::Parameters tm_params(weights_map,                     // weight_map
                                                                                  98765,                           // seed
                                                                                  nt::Tolerance< double >(1e-6),   // tolerance
                                                                                  true,                            // use_betweenness_centrality
                                                                                  false,                           // normalized
                                                                                  true                             // endpoints
    );
    nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap >             tmg(digraph, capacity_map, tm_params);
    const double                                                        total_traffic = 1000.;
    tmg.run(total_traffic);

    LOG_F(INFO, "Load check arc load after run: {}", tmg.checkLoad(total_traffic));
    LOG_F(INFO, "Final traffic matrice T (1, 2) = {}", tmg.getTrafficMatrix()(1, 2));

    // export digraph only
    nt::graphs::NetworkxIo< Digraph > digraph_reader_writer("from", "to");
    digraph_reader_writer.writeFile("./cold_generated_graph.json", digraph);

    // export digraph and traffic matrix
    nt::graphs::GraphProperties< Digraph > digraph_props;
    digraph_props.arcMap("capa", capacity_map);

    Digraph::ArcMap< int > arc_load(digraph);   // routing
    const auto&            traffic_matrix = tmg.getTrafficMatrix();

    const nt::graphs::DemandGraph< Digraph >&            demand_graph = tmg.getDemandGraph();
    nt::graphs::DemandGraph< Digraph >::ArcMap< double > demand_values(demand_graph);
    nt::graphs::mapCopy(demand_graph, tmg.getDemandValues(), demand_values);

    nt::graphs::NetworkxIo< nt::graphs::DemandGraph< Digraph > >      digraph_reader_writer2("from", "to");
    nt::graphs::GraphProperties< nt::graphs::DemandGraph< Digraph > > digraph_props_demand;
    digraph_props_demand.arcMap("demand", demand_values);
    digraph_reader_writer2.writeFile("./cold_generated_graph_with_traffic_demand_graph.json", demand_graph, &digraph_props_demand);


    // Create a double metric map from the weight map
    typename Digraph::template ArcMap< double > metric_map(digraph);
    for (typename Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
      metric_map[arc] = static_cast< double >(weights_map[arc]);
    }

    // spr run
    nt::te::ShortestPathRouting< Digraph, double > spr(digraph, metric_map);
    const int                                      i_num_routed = spr.run(demand_graph, demand_values);
    DoubleArcMap                                   spr_saturations(digraph);
    for (typename Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
      spr_saturations[arc] = spr.saturation(arc, capacity_map[arc]).max();
    }
    digraph_props.arcMap("saturation", spr_saturations);

    digraph_reader_writer.writeFile("./cold_generated_graph_with_traffic_routing.json", digraph, &digraph_props);
  }

  // Traffic generation for Abilene network using betweenness centrality
  // #digraph #trafficgeneration #abilene #sndlib #betweennesscentral
  {
    LOG_SCOPE_F(INFO, "Traffic generation for Abilene using betweenness centrality");

    // Define aliases for convenience
    using Digraph = nt::graphs::SmartDigraph;
    using DemandGraph = nt::graphs::DemandGraph< Digraph >;

    // Define the basic graph types: Node, Arc, ...
    DIGRAPH_TYPEDEFS(Digraph);

    const char*               sz_filename = DATA("abilene.xml");
    Digraph                   digraph;
    Digraph::ArcMap< double > capacity_map(digraph);
    DemandGraph               demand_graph(digraph);

    // Read the file
    nt::graphs::SndlibReader< Digraph, DemandGraph > reader;
    if (reader.loadFile(sz_filename)) {
      reader.readNetwork(digraph, capacity_map);

      // Generate the traffic with betweenness centrality (backward compatibility)
      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg(digraph, capacity_map, 0, true);   // Use betweenness centrality
      // Note: the TrafficMatrixGenerator constructor has been modified to accept a boolean parameter
      // indicating whether to use betweenness centrality or not. If set to true,
      // it will compute the betweenness centrality of the nodes and use it to generate
      // the traffic matrix.
      // This is useful for networks where the traffic is expected to be concentrated on the most
      // central nodes, which is often the case in real-world networks.
      // If you want to use the default behavior (without betweenness centrality), simply
      // call the constructor without the last parameter or set it to false.
      // For example:
      // nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg(digraph, capacity_map);
      const int total_traffic = 1000;
      tmg.run(total_traffic);

      // Or Personnalisation des paramètres de centralité après création
      tmg.useBetweennessCentrality(true, false, true);   // use betweenness Centrality, normalized=false, endpoints=true
      tmg.run(total_traffic);

      LOG_F(INFO, "Use Betweenness Centrality: {}", tmg.isUsingBetweennessCentrality());
      LOG_F(INFO, "Load check arc load after run: {}", tmg.checkLoad(total_traffic));
      LOG_F(INFO, "Final traffic matrice T (1, 2) = {}", tmg.getTrafficMatrix()(1, 2));

      // Advanced example: Using Parameters with custom weight map and betweenness centrality
      DoubleArcMap high_capacity_weights(digraph);
      // Set weights inversely proportional to capacity (higher capacity = lower weight = preferred path)
      for (ArcIt arc_it(digraph); arc_it != nt::INVALID; ++arc_it) {
        Arc arc = arc_it;
        high_capacity_weights[arc] = 1.0 / (capacity_map[arc] + 1.0);   // Avoid division by zero
      }

      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap >::Parameters advanced_params(high_capacity_weights,           // weight map that prefers high-capacity arcs
                                                                                          98765,                           // different seed
                                                                                          nt::Tolerance< double >(1e-8),   // tighter tolerance
                                                                                          true,                            // enable betweenness centrality
                                                                                          true,                            // normalized centrality
                                                                                          false                            // don't include endpoints
      );

      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap > tmg_advanced(digraph, capacity_map, advanced_params);
      tmg_advanced.run(total_traffic);

      LOG_F(INFO, "Advanced traffic generation completed");
      LOG_F(INFO, "Using betweenness centrality: {}", tmg_advanced.isUsingBetweennessCentrality());
      LOG_F(INFO, "Advanced traffic matrice T (1, 2) = {}", tmg_advanced.getTrafficMatrix()(1, 2));

      // Demonstrate parameter modification (weight_map reference cannot be changed)
      nt::te::TrafficMatrixGenerator< Digraph, DoubleArcMap >::Parameters new_params(high_capacity_weights,   // same weight map reference
                                                                                     11111,                   // new seed
                                                                                     nt::Tolerance< double >(1e-5),
                                                                                     false,   // disable centrality
                                                                                     true,
                                                                                     true);
      tmg_advanced.setParameters(new_params);
      tmg_advanced.run(total_traffic);
      LOG_F(INFO, "Parameters updated - new seed: {}", tmg_advanced.getParameters().seed);
    } else {
      LOG_F(ERROR, "Cannot open file {}", sz_filename);
    }
  }

  // Read a digraph and its attributes from a NetworkX JSON file
  // #NetworkX #json #SmartDigraph #graphproperties #file
  {
    LOG_SCOPE_F(INFO, "Read a digraph from NetworkX JSON file");

    // We assume that the following NetworkX JSON file is present.
    // You probably need to change this to load your own file.
    const char* sz_digraph_file = DATA("simple.json");

    // Let's define a type Digraph to be a SmartDigraph
    using Digraph = nt::graphs::SmartDigraph;
    Digraph digraph;

    // Declare a GraphProperties instance that will hold the attributes and node/arc maps of the
    // graph
    nt::graphs::GraphProperties< Digraph > digraph_props;

    // Declare the variables that will store the graph attributes 'capacity_unit' and 'multigraph'
    int capacity_unit = 0;
    digraph_props.attribute("capacity_unit", capacity_unit);

    bool is_multigraph = false;
    digraph_props.attribute("multigraph", is_multigraph);

    // Declare the node maps 'node_id' and 'node_name' that will store the node attributes 'id' and
    // 'name', respectively
    Digraph::NodeMap< int > node_id(digraph);
    digraph_props.nodeMap("id", node_id);

    Digraph::NodeMap< nt::String > node_name(digraph);
    digraph_props.nodeMap("name", node_name);

    // Finally, declare the arc maps 'arc_weight' and 'arc_capa' that will store the arc attributes
    // 'weight' and 'capa', respectively
    Digraph::ArcMap< float > arc_weight(digraph);
    digraph_props.arcMap("weight", arc_weight);

    Digraph::ArcMap< int > arc_capa(digraph);
    digraph_props.arcMap("capa", arc_capa);

    // Declare a NetworkxIo object to read/write the graph data from/to a NetworkX JSON file.
    // By default, the attributes that store the enpoints of an arc are named 'source' and 'target'.
    // If, for some reason, other names are used then you can specify them in the
    // constructor of NetworkxIo. In this example, the names 'from' and 'to' are used.
    // You may also change their name later on by calling 'setSourceAttr' and 'setTargetAttr'.
    nt::graphs::NetworkxIo< Digraph > networkx_reader("from", "to");

    // Read the network and all its attribute from the file
    // Passing 'digraph_props' to 'read()' function is optional, if the graph has no properties or
    // you only need to load the topology then simply call 'networkx_reader.read(sz_digraph_file, digraph)'
    networkx_reader.readFile(sz_digraph_file, digraph, &digraph_props);

    // Display some graph infos
    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Display the graph attributes
    LOG_F(INFO, "Capacity unit {}", capacity_unit);
    LOG_F(INFO, "Multigraph {}", is_multigraph);

    // Display the node attributes
    LOG_SCOPE_F(INFO, "Node attributes");
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} has id = {} and name = {}", digraph.id(node), node_id[node], node_name[node]);
    }

    // Display the arc attributes
    LOG_SCOPE_F(INFO, "Arc attributes");
    for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
      LOG_F(INFO, "Arc {} has weight = {} and capacity = {}", digraph.id(arc), arc_weight[arc], arc_capa[arc]);
    }
  }

  // Read a digraph and its attributes from a NetworkX JSON file using RapidNetworkxIo
  // #NetworkX #json #CompactDigraph #rapidnetworkxio #file
  {
    LOG_SCOPE_F(INFO, "Read a digraph from NetworkX JSON file using RapidNetworkxIo");

    // We assume that the following NetworkX JSON file is present.
    // You probably need to change this to load your own file.
    const char* sz_digraph_file = DATA("simple.json");

    // Let's define a type Digraph to be a CompactDigraph
    using Digraph = nt::graphs::CompactDigraph;
    Digraph digraph;

    using RapidNetworkxIo = nt::graphs::RapidNetworkxIo<Digraph>;
    RapidNetworkxIo reader("from", "to");

    if (!reader.readFile(sz_digraph_file, digraph))
      LOG_F(ERROR, "Cannot open file '{}'", sz_digraph_file);

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Display the graph attributes
    LOG_F(INFO, "Capacity unit {}", reader.graphProperties().getInt64("capacity_unit", -1));
    LOG_F(INFO, "Multigraph {}", reader.graphProperties().getBool("multigraph", false));

    // Display the node attributes
    LOG_SCOPE_F(INFO, "Node attributes");
    const RapidNetworkxIo::NodePropMap& node_props = reader.nodeProperties();
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      const RapidNetworkxIo::NodePropBag& props = node_props[node];
      LOG_F(INFO, "Node {} has id = {} and name = {}", digraph.id(node), props.getInt64("id", -1), props.getString("name", ""));
    }

    // Display the arc attributes
    LOG_SCOPE_F(INFO, "Arc attributes");
    const RapidNetworkxIo::ArcPropMap& arc_props = reader.arcProperties();
    for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
      const RapidNetworkxIo::ArcPropBag& props = arc_props[arc];
      LOG_F(INFO, "Arc {} has weight = {} and capacity = {}", digraph.id(arc), props.getDouble("weight", 1.0), props.getInt64("capa", -1));
    }
  }

  // Read a graph and an associated demand graph from NetworkX JSON files
  // #NetworkX #json #SmartDigraph #demandgraph #trafficmatrix #graphproperties
  {
    LOG_SCOPE_F(INFO, "Read a digraph and an associated demand graph from NetworkX JSON files");

    // We assume that these two NetworkX JSON files are present.
    // You probably need to change these values to load your own files.
    const char* sz_digraph_file = DATA("simple.json");
    const char* sz_demand_graph_file = DATA("tm-simple.json");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;

    // Start by reading the digraph and its node/arc attributes (weight, capacities, ...)
    Digraph digraph;

    // Declare a GraphProperties instance that will hold the attributes and node/arc maps of the graph
    nt::graphs::GraphProperties< Digraph > digraph_props;

    // Declare a node map 'node_name' that will store the name of each node.
    Digraph::NodeMap< nt::String > node_name(digraph);
    digraph_props.nodeMap("name", node_name);

    // Finally, declare the arc maps 'arc_weight' and 'arc_capa' that will store the arc attributes
    // 'weight' and 'capa', respectively
    Digraph::ArcMap< float > arc_weight(digraph);
    digraph_props.arcMap("weight", arc_weight);

    Digraph::ArcMap< int > arc_capa(digraph);
    digraph_props.arcMap("capa", arc_capa);

    // Declare a NetworkxIo object to read the graph data from a NetworkX JSON file.
    nt::graphs::NetworkxIo< Digraph > digraph_reader("from", "to");
    if (!digraph_reader.readFile(sz_digraph_file, digraph, &digraph_props)) { LOG_F(ERROR, "Cannot read file '{}'", sz_digraph_file); }

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Define a demand graph : a digraph sharing the same node set as another digraph.
    // Each arc of this graph represent a traffic demand. Notice that a DemandGraph is
    // necessarily associated to a Digraph. This is why its constructor takes 'digraph'
    // as an argument
    using DemandGraph = nt::graphs::DemandGraph< Digraph >;
    DemandGraph demand_graph(digraph);

    // Declare a GraphProperties instance to store arc/node maps of the demand graph
    nt::graphs::GraphProperties< DemandGraph > dg_props;

    // Declare an arc map 'demands' that will store the values of each demand.
    // In this example, the json file contains 24 values for each demand (one for
    // each hour of the day), hence we use a FloatDynamicArray to store those values.
    DemandGraph::ArcMap< nt::FloatDynamicArray > demands(demand_graph);
    dg_props.arcMap("demand", demands);

    // The arc endpoints in the json file storing the demand graph are
    // called 'source'/'target' and not 'from'/'to' as in the previous json file.
    // Thus we must indicate this to our reader.
    digraph_reader.setSourceAttr("source");
    digraph_reader.setTargetAttr("target");

    // Finally read the demand graph !
    // Notice that we call readLinksFromFile() instead of readFile() since the demand graph has the
    // same node set as its associated digrah. Consequently, we only need to read the arc
    // infos of the file.
    if (digraph_reader.readLinksFromFile(sz_demand_graph_file, demand_graph, &dg_props)) {
      LOG_F(INFO, "Number of demands {}", demand_graph.arcNum());

      // Let's display the demand value of demand 0 at hour 0
      const int              i_demand = 0;
      const int              i_time = 0;
      const DemandGraph::Arc demand_arc = demand_graph.arcFromId(i_demand);
      LOG_F(INFO, "Demand with id {} has value {} at time step {}", i_demand, demands[demand_arc][i_time], i_time);
    } else {
      LOG_F(ERROR, "Cannot read file '{}'", sz_demand_graph_file);
    }
  }

  // Read gt binary file format (see https://graph-tool.skewed.de/static/doc/gt_format.html)
  // #gt #binary
  {
    LOG_SCOPE_F(INFO, "Read GT file format");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;
    Digraph digraph;

    if (argc > 1) {
      nt::graphs::BinaryIo< Digraph > bin_io;
      if (!bin_io.readFile(argv[1], digraph)) LOG_F(ERROR, "Cannot open file '{}'", argv[1]);

      LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
      LOG_F(INFO, "Number of arcs {}", digraph.arcNum());
    } else {
      LOG_F(ERROR, "No input file");
    }
  }


  // Write and read gt binary file format (see https://graph-tool.skewed.de/static/doc/gt_format.html)
  // #gt #binary
  {
    LOG_SCOPE_F(INFO, "Read/write digraph in binary");

    // Let's define some digraph types
    using Digraph = nt::graphs::SmartDigraph;
    using GraphProperties = nt::graphs::GraphProperties< Digraph >;

    // Create a BinaryIo object that export/import digraph in binary format
    nt::graphs::BinaryIo< Digraph > bin_io;

    // Name of the binary file
    const char* sz_filename = "digraph.bin";

    // Construct and export a digraph and its properties to the binary file
    {
      Digraph digraph;

      Digraph::Node v0 = digraph.addNode();
      Digraph::Node v1 = digraph.addNode();
      Digraph::Node v2 = digraph.addNode();
      Digraph::Node v3 = digraph.addNode();

      Digraph::Arc a0 = digraph.addArc(v0, v1);
      Digraph::Arc a1 = digraph.addArc(v1, v2);
      Digraph::Arc a2 = digraph.addArc(v3, v2);
      Digraph::Arc a3 = digraph.addArc(v0, v3);

      // Let's define the graph properties
      GraphProperties digraph_props;

      // Graph properties
      nt::String type("optical");
      digraph_props.attribute("type", type);

      // Node properties
      Digraph::NodeMap< nt::String > node_name(digraph);
      digraph_props.nodeMap("name", node_name);

      node_name[v0] = "v0";
      node_name[v1] = "v1";
      node_name[v2] = "v2";
      node_name[v3] = "v3";

      // Arc properties
      Digraph::ArcMap< float > arc_weight(digraph);
      digraph_props.arcMap("weight", arc_weight);

      arc_weight[a0] = 10.f;
      arc_weight[a1] = 100.f;
      arc_weight[a2] = 1.f;
      arc_weight[a3] = 123.f;

      Digraph::ArcMap< nt::FloatDynamicArray > arc_hourly_traffic(digraph);
      digraph_props.arcMap("hourly_traffic", arc_hourly_traffic);

      arc_hourly_traffic[a0] = {10.f, 20.f, 13.f, 18.f, 19.f};
      arc_hourly_traffic[a1] = {4572.f, 454568.f, 331.f, 783.f, 43.f};
      arc_hourly_traffic[a2] = {713.f, -7856.f, -75.f, 783.f, 0.f};
      arc_hourly_traffic[a3] = {537.f, 122.f, 327.f, 43.f, 79.f};

      // Write the graph in the file
      if (!bin_io.writeFile(sz_filename, digraph, &digraph_props)) LOG_F(ERROR, "Error while writing to '{}'.", sz_filename);
    }

    // Read the previously exported digraph
    {
      Digraph         digraph;
      GraphProperties digraph_props;

      nt::String type;
      digraph_props.attribute("type", type);

      Digraph::NodeMap< nt::String > node_name(digraph);
      digraph_props.nodeMap("name", node_name);

      Digraph::ArcMap< float > arc_weight(digraph);
      digraph_props.arcMap("weight", arc_weight);

      Digraph::ArcMap< nt::FloatDynamicArray > arc_hourly_traffic(digraph);
      digraph_props.arcMap("hourly_traffic", arc_hourly_traffic);

      if (!bin_io.readFile(sz_filename, digraph, &digraph_props)) LOG_F(ERROR, "Error while reading from '{}'.", sz_filename);

      LOG_F(INFO, "Graph type : {}", type.c_str());

      LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
      LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

      LOG_F(INFO, "Number of graph attributes {}", digraph_props.attributes().size());

      LOG_F(INFO, "Number of node properties {}", digraph_props.nodeMaps().size());
      for (Digraph::NodeIt u(digraph); u != nt::INVALID; ++u) {
        LOG_F(INFO, "Node {} has name '{}'", digraph.id(u), node_name[u]);
      }

      LOG_F(INFO, "Number of arc properties {}", digraph_props.arcMaps().size());
      for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
        LOG_F(INFO, "Arc {} has weight {}", digraph.id(arc), arc_weight[arc]);

        const nt::FloatDynamicArray& traffics = arc_hourly_traffic[arc];
        for (int t = 0; t < traffics.size(); ++t)
          LOG_F(INFO, " traffic flow at {} : {}", t, traffics[t]);
      }
    }
  }

  // Dijkstra
  // #Dijkstra #ShortestPath #GraphTraversal #PropertyGraph #graph #digraph
  {
    LOG_SCOPE_F(INFO, "Dijkstra");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Declare an arc maps 'weight_map' store the arc weights.
    // Here we set all weights to 1
    Digraph::ArcMap< float > weight_map(digraph, 1.f);

    // Declare Dijkstra object
    using Dijkstra = nt::graphs::Dijkstra< Digraph, float >;
    Dijkstra dijkstra(digraph, weight_map);

    // Run Dijkstra's algorithm to compute the shortest paths from v0 to every other vertex with respect to the weight provided by 'weight_map'.
    dijkstra.run(v0);

    // Display all shortest distances
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), dijkstra.dist(node), digraph.id(v0));
    }

    // Display predecessor map
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      if (!dijkstra.numPred(node))
        LOG_F(INFO, "Node {} has no predecessor arc", digraph.id(node));
      else {
        for (int k = 0; k < dijkstra.numPred(node); ++k) {
          Digraph::Arc pred_arc = dijkstra.predArc(node, k);
          LOG_F(INFO, "Predecessor #{} arc of node {} is arc {} ({} -> {})", k, digraph.id(node), digraph.id(pred_arc), digraph.id(digraph.source(pred_arc)), digraph.id(digraph.target(pred_arc)));
        }
      }
    }

    // Loop over each shortest path joining v0 and v2
    int k = 1;
    for (nt::ArrayView< Digraph::Arc > path: dijkstra.paths(v2)) {
      LOG_F(INFO, "List of arcs in the shortest path #{} between {} and {}", k++, digraph.id(v0), digraph.id(v2));
      for (const Digraph::Arc& arc: path) {
        LOG_F(INFO, " {} -> {} (id {})", digraph.id(digraph.source(arc)), digraph.id(digraph.target(arc)), digraph.id(arc));
      }
    }

    // Export and print the result in JSON
    const nt::JSONDocument dijkstra_result_json = dijkstra.exportSolutionToJSON();
    LOG_F(INFO, "Dijkstra JSON : {}", nt::JSONDocument::toString(dijkstra_result_json).c_str());

    // By default, the algorithm follows the arc direction to reach the neighbors of each node. If you want the
    // algorithm to follow the reverse direction instead, you can call `run()` with the following template parameters:
    //  - Forward (Default): The algorithm uses the direction of the arcs to traverse the graph.
    //  - Backward: The algorithm uses the reverse direction of the arcs for traversal.

    // By default, the `run()` method computes and stores all the predecessors of each node, allowing you to retrieve the shortest paths afterward.
    // If you only need to find at least one path or are only concerned about the shortest distances, you can call `run()` with the following template parameters:
    //  - AllPreds (Default): Stores all predecessors at each node (provides complete path information).
    //  - SinglePred: Stores at most one predecessor (runs faster than AllPreds and is suitable for finding a single path).
    //  - NoPred: Stores no predecessors at all (fastest option when you don't need predecessor information).
  }

  // #Dijkstra16bits #ShortestPath #GraphTraversal #PropertyGraph #graph #digraph
  {
    LOG_SCOPE_F(INFO, "Dijkstra16bits");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Declare an arc maps 'weight_map' store the arc weights.
    // Here we set all weights to 1
    Digraph::ArcMap< std::uint16_t > weight_map(digraph, 1);

    // Declare Dijkstra16bits object
    using Dijkstra16bits = nt::graphs::Dijkstra16Bits< Digraph >;
    Dijkstra16bits dijkstra(digraph, weight_map);

    // Run Dijkstra's algorithm to compute the shortest paths from v0 to every other vertex with respect to the weight provided by 'weight_map'.
    dijkstra.run(v0);

    // Display all shortest distances
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), dijkstra.dist(node), digraph.id(v0));
    }

    // Loop over each shortest path joining v0 and v2
    int k = 1;
    for (nt::ArrayView< Digraph::Arc > path: dijkstra.paths(v2)) {
      LOG_F(INFO, "List of arcs in the shortest path #{} between {} and {}", k++, digraph.id(v0), digraph.id(v2));
      for (const Digraph::Arc& arc: path) {
        LOG_F(INFO, " {} -> {} (id {})", digraph.id(digraph.source(arc)), digraph.id(digraph.target(arc)), digraph.id(arc));
      }
    }

    // Export and print the result in JSON
    const nt::JSONDocument dijkstra_result_json = dijkstra.exportSolutionToJSON();
    LOG_F(INFO, "Dijkstra JSON : {}", nt::JSONDocument::toString(dijkstra_result_json).c_str());
  }

  // #DynamicDijkstra #ShortestPath #GraphTraversal #PropertyGraph #graph #digraph
  {
    LOG_SCOPE_F(INFO, "DynamicDijkstra");

    // Let's define a digraph
    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    Digraph::Arc a01 = digraph.addArc(v0, v1);
    Digraph::Arc a12 = digraph.addArc(v1, v2);
    Digraph::Arc a32 = digraph.addArc(v3, v2);
    Digraph::Arc a03 = digraph.addArc(v0, v3);

    LOG_F(INFO, "Number of nodes {}", digraph.nodeNum());
    LOG_F(INFO, "Number of arcs {}", digraph.arcNum());

    // Declare an arc maps 'weight_map' store the arc weights.
    Digraph::ArcMap< double > weight_map(digraph);
    weight_map[a01] = 2.0;
    weight_map[a12] = 2.0;
    weight_map[a32] = 4.0;
    weight_map[a03] = 1.0;

    // Declare DynamicDijkstra object
    using DynamicDijkstra = nt::graphs::DynamicDijkstraForward< Digraph >;
    DynamicDijkstra dijkstra(digraph, weight_map);

    // Run Dijkstra's algorithm to compute the shortest paths from v0 to every other vertex with respect to the weight provided by 'weight_map'.
    dijkstra.run(v0);

    // Display all shortest distances
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), dijkstra.dist(node), digraph.id(v0));
    }

    // Display predecessor map
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      if (!dijkstra.numPred(node))
        LOG_F(INFO, "Node {} has no predecessor arc", digraph.id(node));
      else {
        for (int k = 0; k < dijkstra.numPred(node); ++k) {
          Digraph::Arc pred_arc = dijkstra.predArc(node, k);
          LOG_F(INFO, "Predecessor #{} of node {} is arc {} ({} -> {})", k, digraph.id(node), digraph.id(pred_arc), digraph.id(digraph.source(pred_arc)), digraph.id(digraph.target(pred_arc)));
        }
      }
    }

    // Loop over each shortest path joining v0 and v2
    int k = 1;
    for (nt::ArrayView< Digraph::Arc > path: dijkstra.paths(v2)) {
      LOG_F(INFO, "List of arcs in the shortest path #{} between {} and {}", k++, digraph.id(v0), digraph.id(v2));
      for (const Digraph::Arc& arc: path) {
        LOG_F(INFO, " {} -> {} (id {})", digraph.id(digraph.source(arc)), digraph.id(digraph.target(arc)), digraph.id(arc));
      }
    }

    LOG_F(INFO, "Set new weight 2 to arc {} ({} -> {})", digraph.id(a32), digraph.id(digraph.source(a32)), digraph.id(digraph.target(a32)));
    dijkstra.updateArcLength(a32, 2.0);

    // Display all shortest distances
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), dijkstra.dist(node), digraph.id(v0));
    }

    // Loop over each shortest path joining v0 and v2
    k = 1;
    for (nt::ArrayView< Digraph::Arc > path: dijkstra.paths(v2)) {
      LOG_F(INFO, "List of arcs in the shortest path #{} between {} and {}", k++, digraph.id(v0), digraph.id(v2));
      for (const Digraph::Arc& arc: path) {
        LOG_F(INFO, " {} -> {} (id {})", digraph.id(digraph.source(arc)), digraph.id(digraph.target(arc)), digraph.id(arc));
      }
    }


    LOG_F(INFO, "Set new weight 5 to arc {} ({} -> {})", digraph.id(a32), digraph.id(digraph.source(a32)), digraph.id(digraph.target(a32)));
    dijkstra.updateArcLength(a32, 5.0);

    // Display all shortest distances
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), dijkstra.dist(node), digraph.id(v0));
    }

    // Loop over each shortest path joining v0 and v2
    k = 1;
    for (nt::ArrayView< Digraph::Arc > path: dijkstra.paths(v2)) {
      LOG_F(INFO, "List of arcs in the shortest path #{} between {} and {}", k++, digraph.id(v0), digraph.id(v2));
      for (const Digraph::Arc& arc: path) {
        LOG_F(INFO, " {} -> {} (id {})", digraph.id(digraph.source(arc)), digraph.id(digraph.target(arc)), digraph.id(arc));
      }
    }
  }

  // Breadth first search
  // #bfs #GraphTraversal
  {
    LOG_SCOPE_F(INFO, "BFS");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // Run BFS with default visitor
    nt::graphs::BfsDistanceVisit< Digraph > bfs(digraph);
    bfs.run(v0);

    // Display the distances from v0 of each node
    for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
      LOG_F(INFO, "Node {} is at distance {} from {}", digraph.id(node), bfs.dist(node), digraph.id(v0));
    }
  }

  // Breadth first search with customized visitor
  // #bfs #GraphTraversal #bfsvisitor
  {
    LOG_SCOPE_F(INFO, "BFS with customized visitor");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // Declaration of a BFS visitor to customize the behavior of BFS algorithm
    // In this example, we simply print the name of the current traversal step
    struct MyBfsVisitor {
      using Arc = typename Digraph::Arc;
      using Node = typename Digraph::Node;

      const Digraph& _digraph;

      MyBfsVisitor(const Digraph& digraph) : _digraph(digraph) {}

      /// \brief Called for the source node(s) of the BFS.
      ///
      /// This function is called for the source node(s) of the BFS.
      void start(const Node& node) noexcept { LOG_F(INFO, "Start: {}", _digraph.id(node)); }
      /// \brief Called when a node is reached first time.
      ///
      /// This function is called when a node is reached first time.
      void reach(const Node& node) noexcept { LOG_F(INFO, "Reached: {}", _digraph.id(node)); }
      /// \brief Called when a node is processed.
      ///
      /// This function is called when a node is processed.
      void process(const Node& node) noexcept { LOG_F(INFO, "Processed: {}", _digraph.id(node)); }
      /// \brief Called when an arc reaches a new node.
      ///
      /// This function is called when the BFS finds an arc whose target node
      /// is not reached yet.
      void discover(const Arc& arc) noexcept { LOG_F(INFO, "Discovered: {}", _digraph.id(arc)); }
      /// \brief Called when an arc is examined but its target node is
      /// already discovered.
      ///
      /// This function is called when an arc is examined but its target node is
      /// already discovered.
      void examine(const Arc& arc) noexcept { LOG_F(INFO, "Examined: {}", _digraph.id(arc)); }
    };

    // Run BFS with MyBfsVisitor.
    MyBfsVisitor                                          myvisitor(digraph);
    nt::graphs::BfsDistanceVisit< Digraph, MyBfsVisitor > my_bfs(digraph, myvisitor);
    my_bfs.run(v0);
  }

  // #DijkstraLegacy #ShortestPath #GraphTraversal #PropertyGraph #graph #digraph
  {
    LOG_SCOPE_F(INFO, "DijkstraLegacy");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // Run dijkstra to compute the shortest paths from v0 to every other vertex.
    Digraph::ArcMap< int >                weight_map(digraph, 1);
    nt::graphs::DijkstraLegacy< Digraph > dijkstra(digraph, weight_map);
    dijkstra.run(v0);

    LOG_F(INFO, "Distances :");
    for (Digraph::NodeIt v(digraph); v != nt::INVALID; ++v)
      LOG_F(INFO, "{} : {} ", digraph.id(v), dijkstra.dist(v));
  }

  // #edmondskarp
  {
    LOG_SCOPE_F(INFO, "EdmondsKarp");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // Define an arc map that holds the capacities of the arcs.
    // In this example, each arc has capacity 1
    Digraph::ArcMap< int > capacity_map(digraph, 1);

    // Run Edmonds-Karp algorithm
    nt::graphs::EdmondsKarp< Digraph > ek(digraph, capacity_map, v0, v2);
    ek.run();

    LOG_F(INFO, "Max flow value : {}", ek.flowValue());
    LOG_F(INFO, "Flows :");
    for (Digraph::ArcIt a(digraph); a != nt::INVALID; ++a)
      LOG_F(INFO, "{} : {} ", digraph.id(a), ek.flow(a));

    // Export the flow in a CSV file that can be imported into NtEditor
    nt::graphs::writeArcMapFile("ek_flow.csv", digraph, ek.flowMap());
  }

  // #GomoryHu #undirector
  {
    LOG_SCOPE_F(INFO, "GomoryHu");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // GomoryHu requires an undirected graph as input, so we apply nt::graphs::undirector to our digraph
    // to get the underlying undirected graph.
    using Undigraph = nt::graphs::Undirector< const Digraph >;
    Undigraph undigraph = nt::graphs::undirector(digraph);

    // Define an arc map that holds the capacities of the arcs.
    // In this example, each arc has capacity 1
    Undigraph::EdgeMap< int > capacity_map(undigraph, 1);

    // Run Gomory-Hu algorithm
    nt::graphs::GomoryHu< Undigraph > ght(undigraph, capacity_map);
    ght.run();

    LOG_F(INFO, "Cut values :");
    for (Undigraph::NodeIt u(undigraph); u != nt::INVALID; ++u) {
      for (Undigraph::NodeIt v(undigraph); v != nt::INVALID; ++v) {
        if (u == v) continue;
        LOG_F(INFO, "s={} t={} : {} ", undigraph.id(u), undigraph.id(v), ght.minCutValue(u, v));
      }
    }
  }

  // #clique #maxclique
  {
    LOG_SCOPE_F(INFO, "GrossoLocatelliPullanMc");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // GrossoLocatelliPullanMc requires an undirected graph as input, so we apply nt::graphs::undirector
    // to our digraph to get the underlying undirected graph.
    using Undigraph = nt::graphs::Undirector< const Digraph >;
    Undigraph undigraph = nt::graphs::undirector(digraph);

    nt::graphs::GrossoLocatelliPullanMc< Undigraph > mc(undigraph);
    mc.run();

    LOG_F(INFO, "Found clique size : {}", mc.cliqueSize());
  }

  // #kruskal #spanningtree
  {
    LOG_SCOPE_F(INFO, "kruskal");

    using Digraph = nt::graphs::SmartDigraph;

    // Build the following directed graph :
    //        v0 --> v1 --> v2
    //        |             ^
    //        +----> v3 ----+

    Digraph digraph;

    Digraph::Node v0 = digraph.addNode();
    Digraph::Node v1 = digraph.addNode();
    Digraph::Node v2 = digraph.addNode();
    Digraph::Node v3 = digraph.addNode();

    digraph.addArc(v0, v1);
    digraph.addArc(v1, v2);
    digraph.addArc(v3, v2);
    digraph.addArc(v0, v3);

    // Define an arc map that holds the costs of the arcs.
    // In this example, each arc has cost 1
    Digraph::ArcMap< int > cost_map(digraph, 1);

    // Run kruskal algorithm using the costs provided by 'cost_map'.
    // The arcs of the tree will be stored in the array 'tree_arcs'.
    nt::TrivialDynamicArray< Digraph::Arc > tree_arcs;
    nt::graphs::kruskal(digraph, cost_map, std::back_inserter(tree_arcs));

    // Print the arcs of the found minimum cost spanning tree
    LOG_F(INFO, "Num arcs: {}", tree_arcs.size());
    for (const Digraph::Arc& a: tree_arcs)
      LOG_F(INFO, "Arc id {} : {} -> {}", digraph.id(a), digraph.id(digraph.source(a)), digraph.id(digraph.target(a)));
  }

  //-----------------------------------------------------------------------------
  // [SECTION] Traffic Engineering (TE)
  //-----------------------------------------------------------------------------
  // #te

  // Demonstrates traffic engineering functionalities, such as:
  // - Segment Routing
  // - Traffic matrix generation

  // Running Segment Routing
  // #sr #segmentrouting
  {
    LOG_SCOPE_F(INFO, "Running Segment Routing");

    // Define aliases for convenience
    using Digraph = nt::graphs::SmartDigraph;
    using DemandGraph = nt::graphs::DemandGraph< Digraph >;
    using DemandArc = DemandGraph::Arc;
    using SrPath = nt::te::SrPath< Digraph >;
    using SegmentRouting = nt::te::SegmentRouting< Digraph >;
    using DemandArray = nt::TrivialDynamicArray< DemandArc >;

    // Define the basic graph types: Node, Arc, ...
    DIGRAPH_TYPEDEFS(Digraph);

    // Build a simple network
    Digraph digraph;

    Node v0 = digraph.addNode();
    Node v1 = digraph.addNode();
    Node v2 = digraph.addNode();
    Node v3 = digraph.addNode();
    Arc  a0 = digraph.addArc(v1, v0);
    Arc  a1 = digraph.addArc(v1, v2);
    Arc  a2 = digraph.addArc(v2, v3);
    Arc  a3 = digraph.addArc(v0, v3);

    // Create an arc map to store the arc capacity and set them to 1
    Digraph::ArcMap< double > capacities(digraph, 1);

    // Create an arc map to store the arc metric and set them to 1
    Digraph::ArcMap< double > metrics(digraph, 1);

    // Build the demand graph
    DemandGraph demand_graph(digraph);

    const DemandArc darc0 = demand_graph.addArc(v0, v3);
    const DemandArc darc1 = demand_graph.addArc(v1, v3);

    // DemandGraph::ArcMap< float > demand_values(demand_graph, 1);
    DemandGraph::ArcMap< double > demand_values(demand_graph);
    demand_values[darc0] = 1.0;
    demand_values[darc1] = 1.0;

    // Create an SrPath for each demand
    DemandGraph::ArcMap< SrPath > sr_paths(demand_graph);

    SrPath& sr_path_darc0 = sr_paths[darc0];
    sr_path_darc0.addSegment(v1);
    sr_path_darc0.addSegment(v2);
    sr_path_darc0.addSegment(v3);

    SrPath& sr_path_darc1 = sr_paths[darc1];
    sr_path_darc1.addSegment(v1);
    sr_path_darc1.addSegment(v2);
    sr_path_darc1.addSegment(v3);

    // Print the SR paths
    LOG_F(INFO, "SR Path for demand 0: {}", sr_path_darc0.toString(digraph));
    LOG_F(INFO, "SR Path for demand 1: {}", sr_path_darc1.toString(digraph));

    // Create an instance of SegmentRouting
    SegmentRouting sr(digraph, metrics);

    // Run the segment routing algorithm with demand graph and volume map
    // furthermore we store the demands routed on each arc in the map 'dpa' (Demands Per Arc)
    // Notice that the arc weights are equal to 1 by default. If you need to change the weights
    // simply call 'sr.lengthMap(m)' where m is an ArcMap that stores the weights.
    Digraph::ArcMap< DemandArray > dpa(digraph);

    const int i_num_routed = sr.run(demand_graph, demand_values, sr_paths, dpa);
    LOG_F(INFO, "Number of demands routed: {}", i_num_routed);
    LOG_F(INFO, "Number of demands routed on arc '{}' : {}", Digraph::id(a1), dpa[a1].size());
    LOG_F(INFO, "Max saturation: {}", sr.maxSaturation(capacities).max());

    // Reset previous routing
    sr.clear();

    // Change the SR path for demand 1
    sr_path_darc1.clear();
    sr_path_darc1.addSegment(v1);
    sr_path_darc1.addSegment(v0);
    sr_path_darc1.addSegment(v3);

    LOG_F(INFO, "SR Path for demand 0: {}", sr_path_darc0.toString(digraph));
    LOG_F(INFO, "SR Path for demand 1: {}", sr_path_darc1.toString(digraph));

    LOG_F(INFO, "Number of demands routed: {}", sr.run(demand_graph, demand_values, sr_paths));
    LOG_F(INFO, "Max saturation: {}", sr.maxSaturation(capacities).max());

    sr.removeFlow(sr_paths[darc0], demand_values[darc0]);

    LOG_F(INFO, "Max saturation: {}", sr.maxSaturation(capacities).max());

    sr.removeFlow(sr_paths[darc1], demand_values[darc1]);

    LOG_F(INFO, "Max saturation: {}", sr.maxSaturation(capacities).max());
  }

  // ShortestPathRouting & TeInstance
  // #shortestpathrouting #teinstance
  {
    LOG_SCOPE_F(INFO, "Shortest Path Routing & TeInstance");

    // Define aliases for convenience
    using TeInstance = nt::te::TeInstance< nt::graphs::SmartDigraph, double >;
    using Digraph = TeInstance::Digraph;
    using CapacityMap = TeInstance::CapacityMap;
    using MetricMap = TeInstance::MetricMap;
    using DemandGraph = TeInstance::DemandGraph;
    using DemandValueMap = TeInstance::DemandValueMap;

    const char* sz_filename = DATA("abilene.xml");

    TeInstance te_instance;

    if (te_instance.readSndlib(sz_filename)) {
      const Digraph&     g = te_instance.digraph();        // Get the network topology
      const CapacityMap& cm = te_instance.capacityMap();   // Get the network arc capacities
      const MetricMap&   mm = te_instance.metricMap();     // Get the network arc metrics

      const DemandGraph&    dg = te_instance.demandGraph();       // Get the demand graph
      const DemandValueMap& dvm = te_instance.demandValueMap();   // Get the demand values

      nt::te::ShortestPathRouting< Digraph, double > spr(g, mm);

      const int i_num_routed = spr.run(dg, dvm);
      LOG_F(INFO, "Demands routed : {}", i_num_routed);
      LOG_F(INFO, "MLU : {}", spr.maxSaturation(cm).max());
    } else {
      LOG_F(ERROR, "Cannot open file '{}'", sz_filename);
    }
  }

  //-----------------------------------------------------------------------------
  // [SECTION] Mathematical Programming (MP)
  //-----------------------------------------------------------------------------
  // #lp #mp #mip #mathematicalprogramming #linearprogramming #mixedintegerprogram

  // Demonstrates mathematical programming functionalities, including:
  // - Linear Programming (LP)
  // - Mixed Integer Programming (MIP)
  // - Column Generation

  // Building a simple LP
  // #LP
  {
    LOG_SCOPE_F(INFO, "Building & solving a simple LP");

    // Create the solver environment
    MPEnv env;

    // Model creation
    LPModel solver("Simple LP", env);

    // x and y are continuous non-negative variables
    LPModel::MPVariable* const x = solver.makeNumVar(0.0, nt::mp::infinity(), "x");
    LPModel::MPVariable* const y = solver.makeNumVar(0.0, nt::mp::infinity(), "y");

    // Objectif function: Maximize x + 2y
    LPModel::MPObjective* const objective = solver.mutableObjective();
    objective->setCoefficient(x, 1);
    objective->setCoefficient(y, 2);
    objective->setMaximization();

    // -x + y <= 1
    LPModel::MPConstraint* const c0 = solver.makeRowConstraint(-nt::mp::infinity(), 1.0, "c0");
    c0->setCoefficient(x, -1);
    c0->setCoefficient(y, 1);

    // 2x + 3y <= 12
    LPModel::MPConstraint* const c1 = solver.makeRowConstraint(-nt::mp::infinity(), 12.0, "c1");
    c1->setCoefficient(x, 2);
    c1->setCoefficient(y, 3);

    // 3x + 2y <= 12
    LPModel::MPConstraint* const c2 = solver.makeRowConstraint(-nt::mp::infinity(), 12.0, "c2");
    c2->setCoefficient(x, 3);
    c2->setCoefficient(y, 2);

    const nt::mp::ResultStatus result_status = solver.solve();

    LOG_F(INFO, " Solver version = {}", solver.solverVersion());

    LOG_F(INFO, " Model:\n{}", solver.print());

    LOG_F(INFO, " Number of variables = {}", solver.numVariables());
    LOG_F(INFO, " Number of constraints = {}", solver.numConstraints());

    LOG_F(INFO, " Obj = {}", solver.objective().value());
    LOG_F(INFO, " x = {}", x->solution_value());
    LOG_F(INFO, " y = {}", y->solution_value());

    LOG_F(INFO, " reduced cost x = {}", x->reduced_cost());
    LOG_F(INFO, " reduced cost y = {}", y->reduced_cost());

    LOG_F(INFO, " Dual c0 = {}", c0->dual_value());
    LOG_F(INFO, " Dual c1 = {}", c1->dual_value());
    LOG_F(INFO, " Dual c2 = {}", c2->dual_value());

    const nt::DoubleDynamicArray activities = solver.computeConstraintActivities();

    if (!activities.empty()) {
      LOG_F(INFO, " Value of c0 = {}", activities[c0->index()]);
      LOG_F(INFO, " Value of c1 = {}", activities[c1->index()]);
      LOG_F(INFO, " Value of c2 = {}", activities[c2->index()]);
    }

    // Export and print the result in JSON
    const nt::JSONDocument ResultJson = solver.exportSolutionToJSON();
    LOG_F(INFO, "JSON Solution : {}", nt::JSONDocument::toString(ResultJson).c_str());
  }

  // Building a simple MIP
  // #mip
  {
    LOG_SCOPE_F(INFO, "Building & solving a simple MIP");

    // Create the solver environment
    MPEnv env;

    // Model creation
    MIPModel solver("Simple MIP", env);

    // x and y are non-negative integer variables
    MIPModel::MPVariable* const x = solver.makeIntVar(0.0, nt::mp::infinity(), "x");
    MIPModel::MPVariable* const y = solver.makeIntVar(0.0, nt::mp::infinity(), "y");

    // Objectif function: Maximize x + 2y
    MIPModel::MPObjective* const objective = solver.mutableObjective();
    objective->setCoefficient(x, 1);
    objective->setCoefficient(y, 2);
    objective->setMaximization();

    // -x + y <= 1
    MIPModel::MPConstraint* const c0 = solver.makeRowConstraint(-nt::mp::infinity(), 1.0, "c0");
    c0->setCoefficient(x, -1);
    c0->setCoefficient(y, 1);

    // 2x + 3y <= 12
    MIPModel::MPConstraint* const c1 = solver.makeRowConstraint(-nt::mp::infinity(), 12.0, "c1");
    c1->setCoefficient(x, 2);
    c1->setCoefficient(y, 3);

    // 3x + 2y <= 12
    MIPModel::MPConstraint* const c2 = solver.makeRowConstraint(-nt::mp::infinity(), 12.0, "c2");
    c2->setCoefficient(x, 3);
    c2->setCoefficient(y, 2);

    const nt::mp::ResultStatus result_status = solver.solve();

    LOG_F(INFO, " Solver version = {}", solver.solverVersion());

    LOG_F(INFO, " Model:\n{}", solver.print());

    LOG_F(INFO, " Number of variables = {}", solver.numVariables());
    LOG_F(INFO, " Number of constraints = {}", solver.numConstraints());

    LOG_F(INFO, " Obj = {}", solver.objective().value());
    LOG_F(INFO, " x = {}", x->solution_value());
    LOG_F(INFO, " y = {}", y->solution_value());

    const nt::DoubleDynamicArray activities = solver.computeConstraintActivities();

    if (!activities.empty()) {
      LOG_F(INFO, " Value of c0 = {}", activities[c0->index()]);
      LOG_F(INFO, " Value of c1 = {}", activities[c1->index()]);
      LOG_F(INFO, " Value of c2 = {}", activities[c2->index()]);
    }

    // Export and print the result in JSON
    const nt::JSONDocument ResultJson = solver.exportSolutionToJSON();
    LOG_F(INFO, "JSON Solution : {}", nt::JSONDocument::toString(ResultJson).c_str());
  }

  // The cutting stock problem : compact formulation
  // #cuttingstock #multidimvar #mip
  {
    LOG_SCOPE_F(INFO, "The cutting stock problem : compact formulation");

    // Data for the cutting stock instance
    constexpr double f_width = 100.0;   // Width of the rolls
    constexpr int    I = 20;            // Number of possible pieces
    constexpr int    J = 500;           // Number of available rolls

    // Set of possible pieces : for each piece i has two entries {w_i, d_i} where w_i is the width
    // of i and d_i the corresponding demand
    constexpr double pieces[I][2] = {{75.0, 38}, {75.0, 44}, {75.0, 30}, {75.0, 41}, {75.0, 36}, {53.8, 33}, {53.0, 36}, {51.0, 41}, {50.2, 35}, {32.2, 37},
                                     {30.8, 44}, {29.8, 49}, {20.1, 37}, {16.2, 36}, {14.5, 42}, {11.0, 33}, {8.6, 47},  {8.2, 35},  {6.6, 49},  {5.1, 42}};

    // Create the solver environment
    MPEnv env;

    // Model creation
    MIPModel solver("cutting_stock_compact", env);

    // Create X_i_j variables with i in [1, I] and j in [1, J]
    // X_i_j is an integer variable that indicates how many pieces of i we cut from roll j
    const MIPModel::MPMultiDimVariable< 2 > x = solver.makeMultiDimIntVar(0.0, nt::mp::infinity(), "x", I, J);

    // Create Y_j variables with j in [1, J]
    // Y_j is a binary variable that indicates whether we use the roll j
    const MIPModel::MPMultiDimVariable< 1 > y = solver.makeMultiDimBoolVar("y", J);

    // The objective is to minimize the number of rolls that we use.
    MIPModel::MPObjective* const p_objective = solver.mutableObjective();
    p_objective->setMinimization();
    for (int j = 0; j < J; ++j) {
      p_objective->setCoefficient(y(j), 1);
    }

    // The following two families of constraints ensure that we respect the width of each
    // roll (c1) and that we satisfy the demands (c2)

    // c1
    for (int j = 0; j < J; ++j) {
      MIPModel::MPConstraint* const c1 = solver.makeRowConstraint(-nt::mp::infinity(), 0., nt::format("roll-{}", j));
      for (int i = 0; i < I; ++i) {
        c1->setCoefficient(x(i, j), pieces[i][0]);
      }
      c1->setCoefficient(y(j), -f_width);
    }

    // c2
    for (int i = 0; i < I; ++i) {
      MIPModel::MPConstraint* const c2 = solver.makeRowConstraint(pieces[i][1], nt::mp::infinity(), nt::format("demand-{}", i));
      for (int j = 0; j < J; ++j) {
        c2->setCoefficient(x(i, j), 1);
      }
    }

    // Run the solver
    const nt::mp::ResultStatus result_status = solver.solve();

    // Print some stats
    LOG_F(INFO, " Solver version = {}", solver.solverVersion());

    LOG_F(INFO, " Number of variables = {}", solver.numVariables());
    LOG_F(INFO, " Number of constraints = {}", solver.numConstraints());

    // Print the found optimal value
    LOG_F(INFO, " Obj = {}", solver.objective().value());
  }

  // The cutting stock problem : column generation
  // #cuttingstock #multidim #columngeneration #cg #lp
  {
    LOG_SCOPE_F(INFO, "The cutting stock problem : column generation");

    // Data for the cutting stock instance
    constexpr double f_width = 100.0;   // Width of the rolls
    constexpr int    I = 20;            // Number of possible pieces
    constexpr int    J = 500;           // Number of available rolls

    // Set of possible pieces : for each piece i has two entries {w_i, d_i} where w_i is the width
    // of i and d_i the corresponding demand
    constexpr double pieces[I][2] = {{75.0, 38}, {75.0, 44}, {75.0, 30}, {75.0, 41}, {75.0, 36}, {53.8, 33}, {53.0, 36}, {51.0, 41}, {50.2, 35}, {32.2, 37},
                                     {30.8, 44}, {29.8, 49}, {20.1, 37}, {16.2, 36}, {14.5, 42}, {11.0, 33}, {8.6, 47},  {8.2, 35},  {6.6, 49},  {5.1, 42}};

    // Step 1 - Create an initial set of patterns
    nt::IntDynamicArray patterns(I);
    for (int i = 0; i < I; ++i) {
      patterns[i] = static_cast< int >(std::min(f_width / pieces[i][0], pieces[i][1]));
    }
    const int P = patterns.size();   // Number of initial pattern = Number of possible pieces

    // Step 2 - Build the initial master model
    MPEnv   env;
    LPModel master("cutting_stock_cg", env);

    // Variables declaration
    const LPModel::MPMultiDimVariable< 1 > x = master.makeMultiDimNumVar(0.0, nt::mp::infinity(), "x", P);

    // Objective definition
    LPModel::MPObjective* const p_master_obj = master.mutableObjective();
    p_master_obj->setMinimization();
    for (int p = 0; p < P; ++p) {
      p_master_obj->setCoefficient(x(p), 1.0);
    }

    // Constraint
    for (int i = 0; i < I; ++i) {
      LPModel::MPConstraint* const p_master_cons = master.makeRowConstraint(pieces[i][1], nt::mp::infinity(), nt::format("master-{}", i));
      p_master_cons->setCoefficient(x(i), patterns[i]);
    }

    // Print some stats
    LOG_F(INFO, " Solver version = {}", master.solverVersion());

    LOG_F(INFO, " Number of variables = {}", master.numVariables());
    LOG_F(INFO, " Number of constraints = {}", master.numConstraints());

    // Step 3 - Run the iterative column generation loop until no improving columns are found
    bool b_new_col = true;
    int  i_num_cols = 0;
    while (b_new_col) {
      const nt::mp::ResultStatus result_status = master.solve();

      LOG_F(INFO, "Current objective value = {}", p_master_obj->value());

      // Solve the pricing problem (knapsack)
      {
        LOG_SCOPE_F(INFO, "Solve pricing problem");

        b_new_col = false;

        // Build an ILP to solve the knapsack problem
        // In practice, it would be a better idea to create the model outside of the loop and only
        // update the constraints/objective here. This is to avoid recreating the model from scratch
        // at every iteration which can be time consuming.
        MIPModel knapsack("knapsack", env);
        LOG_F(INFO, " Solver version = {}", knapsack.solverVersion());

        const MIPModel::MPMultiDimVariable< 1 > y = knapsack.makeMultiDimIntVar(0.0, nt::mp::infinity(), "y", I);

        MIPModel::MPObjective* const p_ks_obj = knapsack.mutableObjective();
        p_ks_obj->setMaximization();
        for (int i = 0; i < I; ++i) {
          const double dual = master.constraints()[i]->dual_value();
          p_ks_obj->setCoefficient(y(i), dual);
        }

        MIPModel::MPConstraint* const p_ks_cons = knapsack.makeRowConstraint(0., f_width, "ks_cons");
        for (int i = 0; i < I; ++i) {
          p_ks_cons->setCoefficient(y(i), pieces[i][0]);
        }

        const nt::mp::ResultStatus result_status = knapsack.solve();

        // If a new column is found then add it to the master problem
        if ((result_status == nt::mp::ResultStatus::OPTIMAL || result_status == nt::mp::ResultStatus::FEASIBLE) && p_ks_obj->value() > 1.000001) {
          LOG_F(INFO, " Dual obj = {}", p_ks_obj->value());
          b_new_col = true;
          ++i_num_cols;

          // Create the new column
          LPModel::MPVariable* const new_x = master.makeNumVar(0.0, nt::mp::infinity(), "new_x");

          // Update the master objective
          p_master_obj->setCoefficient(new_x, 1.0);

          // Update the master constraint
          for (int i = 0; i < I; ++i) {
            if (y(i)->solution_value() > 0.5) { master.constraints()[i]->setCoefficient(new_x, y(i)->solution_value()); }
          }
        }
      }
    }

    LOG_F(INFO, " No more improving columns found");

    LOG_F(INFO, " Number of added columns = {}", i_num_cols);

    // Print the found optimal value
    LOG_F(INFO, " Obj = {}", master.objective().value());
  }

  // The Multi-Commodity Flow problem
  // #mcf #columngeneration #cg #lp
  {
    LOG_SCOPE_F(INFO, "Multi-Commodity Flow (Path Formulation) Example");

    // 1. Create network topology
    std::cout << "1. Creating network topology...\n";

    using Digraph = nt::graphs::SmartDigraph;
    Digraph g;

    // Create nodes (6-node network)
    auto n1 = g.addNode();
    auto n2 = g.addNode();
    auto n3 = g.addNode();
    auto n4 = g.addNode();
    auto n5 = g.addNode();
    auto n6 = g.addNode();

    std::cout << "  Created " << g.nodeNum() << " nodes\n";

    // Create arcs (links) with topology:
    //     n1 ---> n2 ---> n3
    //     |       |       |
    //     v       v       v
    //     n4 ---> n5 ---> n6

    auto a12 = g.addArc(n1, n2);
    auto a14 = g.addArc(n1, n4);
    auto a23 = g.addArc(n2, n3);
    auto a25 = g.addArc(n2, n5);
    auto a36 = g.addArc(n3, n6);
    auto a45 = g.addArc(n4, n5);
    auto a56 = g.addArc(n5, n6);
    auto a52 = g.addArc(n5, n2);

    std::cout << "  Created " << g.arcNum() << " arcs\n\n";

    // 2. Set up capacity map (link capacities in Mbps)
    std::cout << "2. Setting up link capacities...\n";

    Digraph::ArcMap< double > capacity(g);
    capacity[a12] = 100.0;   // 100 Mbps
    capacity[a14] = 80.0;    // 80 Mbps
    capacity[a23] = 120.0;   // 120 Mbps
    capacity[a25] = 90.0;    // 90 Mbps
    capacity[a36] = 100.0;   // 100 Mbps
    capacity[a45] = 70.0;    // 70 Mbps
    capacity[a56] = 110.0;   // 110 Mbps
    capacity[a52] = 90.0;    // 90 Mbps

    std::cout << "  Link capacities set\n\n";

    // 3. Create demand graph (traffic demands between OD pairs)
    std::cout << "3. Creating demand graph...\n";

    using DemandGraph = nt::graphs::DemandGraph< Digraph >;
    DemandGraph demand_graph(g);

    // Add traffic demands (origin -> destination)
    // Demand 1: n1 -> n3 (50 Mbps)
    auto d1 = demand_graph.addArc(n1, n3);

    // Demand 2: n1 -> n6 (60 Mbps)
    auto d2 = demand_graph.addArc(n1, n6);

    // Demand 3: n4 -> n3 (40 Mbps)
    auto d3 = demand_graph.addArc(n4, n3);

    // Demand 4: n2 -> n6 (45 Mbps)
    auto d4 = demand_graph.addArc(n2, n6);

    std::cout << "  Created " << demand_graph.arcNum() << " demands\n";

    // Set demand values
    DemandGraph::ArcMap< double > demand_values(demand_graph);
    demand_values[d1] = 50.0;   // 50 Mbps
    demand_values[d2] = 60.0;   // 60 Mbps
    demand_values[d3] = 40.0;   // 40 Mbps
    demand_values[d4] = 45.0;   // 45 Mbps

    std::cout << "  Demands:\n";
    for (DemandGraph::ArcIt d(demand_graph); d != nt::INVALID; ++d) {
      std::cout << "    n" << (g.id(demand_graph.source(d)) + 1) << " -> n" << (g.id(demand_graph.target(d)) + 1)
                << ": " << demand_values[d] << " Mbps\n";
    }
    std::cout << "\n";

    // 4. Create LP solver and McfPath instance
    std::cout << "4. Setting up McfPath solver...\n";

    MPEnv   env;
    LPModel solver("mcf-path", env);

    // Create McfPath instance
    using McfPathSolver = nt::graphs::McfPath< Digraph,                        // Network graph type
                                               LPModel,                        // LP solver type
                                               double,                         // Floating point type
                                               double,                         // Demand vector type
                                               DemandGraph,                    // Demand graph type
                                               Digraph::ArcMap< double >,      // Capacity map type
                                               DemandGraph::ArcMap< double >   // Demand value map type
                                               >;

    McfPathSolver mcf(g, demand_graph, solver);

    std::cout << "  McfPath solver created\n\n";

    // 5. Build the optimization model
    std::cout << "5. Building optimization model...\n";

    if (!mcf.build(&capacity, &demand_values)) {
      std::cerr << "Error: Failed to build model!\n";
    }

    std::cout << "  Model built successfully\n";
    std::cout << "  Initial columns: " << demand_graph.arcNum() << " (one per demand)\n\n";

    // 6. Solve using column generation
    std::cout << "6. Solving with column generation...\n";

    nt::mp::ResultStatus status = mcf.solve();

    if (status != nt::mp::ResultStatus::OPTIMAL) {
      std::cerr << "Error: Could not find optimal solution!\n";
    }

    std::cout << "  ✓ Optimal solution found!\n\n";

    // 7. Display results
    std::cout << "=== RESULTS ===\n\n";

    double max_load = mcf.getLoad();
    std::cout << "Maximum Link Utilization (MLU): " << max_load << "\n\n";

    // Show link utilizations
    std::cout << "Link Utilizations:\n";
    std::cout << "  Link    | Capacity | Load      | Utilization\n";
    std::cout << "  --------|----------|-----------|------------\n";

    auto printLink = [&](Digraph::Arc arc, const char* sz_name) {
      const double cap = capacity[arc];
      double       load = 0.0;

      // Calculate load on this arc from all columns (paths)
      for (const McfPathSolver::Column& col: mcf.columns()) {
        if (col.hasArc(arc)) {
          const double flow = col.value();
          load += flow * demand_values[col.demand()];
        }
      }

      double util = (cap > 0) ? (load / cap) * 100.0 : 0.0;

      printf("  %-7s | %8.1f | %9.2f | %6.2f%%\n", sz_name, cap, load, util);
    };

    printLink(a12, "n1->n2");
    printLink(a14, "n1->n4");
    printLink(a23, "n2->n3");
    printLink(a25, "n2->n5");
    printLink(a36, "n3->n6");
    printLink(a45, "n4->n5");
    printLink(a56, "n5->n6");

    std::cout << "\n";

    // Show routing paths found by column generation
    std::cout << "Routing Paths (generated by column generation):\n";
    std::cout << "  Total columns generated: " << mcf.genColNum() << "\n";
    std::cout << "  Total columns in solution: " << mcf.columns().size() << "\n\n";

    int path_num = 0;
    for (const auto& col: mcf.columns()) {
      double flow = col.value();

      // Only show paths with non-zero flow
      if (flow > 1e-6) {
        ++path_num;

        auto   src = demand_graph.source(col.demand());
        auto   tgt = demand_graph.target(col.demand());
        double demand = demand_values[col.demand()];

        std::cout << "  Path " << path_num << " (Demand: n" << (g.id(src) + 1) << " -> n" << (g.id(tgt) + 1) << "):\n";
        std::cout << "    Flow fraction: " << flow << " (" << (flow * demand) << " Mbps)\n";
        std::cout << "    Route: n" << (g.id(src) + 1);

        const nt::ArrayView< Digraph::Arc > path = col.path();
        for (const auto& arc: path) {
          std::cout << " -> n" << (g.id(g.target(arc)) + 1);
        }
        std::cout << "\n\n";
      }
    }

    // 8. Additional analysis
    std::cout << "=== ANALYSIS ===\n\n";

    // Check which demands use multiple paths (load balancing)
    std::map< DemandGraph::Arc, int > demand_path_counts;
    for (const auto& col: mcf.columns()) {
      double flow = col.value();
      if (flow > 1e-6) { demand_path_counts[col.demand()]++; }
    }

    std::cout << "Load Balancing Analysis:\n";
    for (DemandGraph::ArcIt d(demand_graph); d != nt::INVALID; ++d) {
      int  num_paths = demand_path_counts[d];
      auto src = demand_graph.source(d);
      auto tgt = demand_graph.target(d);

      std::cout << "  Demand n" << (g.id(src) + 1) << " -> n" << (g.id(tgt) + 1);
      if (num_paths > 1) {
        std::cout << ": Split across " << num_paths << " paths ✓\n";
      } else {
        std::cout << ": Single path\n";
      }
    }

    std::cout << "\n";

    // Show congestion statistics
    std::cout << "Congestion Statistics:\n";
    nt::DoubleDynamicArray utilizations;

    for (Digraph::ArcIt arc(g); arc != nt::INVALID; ++arc) {
      double cap = capacity[arc];
      double load = 0.0;

      for (const auto& col: mcf.columns()) {
        if (col.hasArc(arc)) {
          double flow = col.value();
          load += flow * demand_values[col.demand()];
        }
      }

      if (cap > 0) { utilizations.push_back((load / cap) * 100.0); }
    }

    std::sort(utilizations.begin(), utilizations.end());

    double avg_util = 0.0;
    for (double u: utilizations)
      avg_util += u;
    avg_util /= utilizations.size();

    std::cout << "  Average utilization: " << avg_util << "%\n";
    std::cout << "  Maximum utilization: " << utilizations.back() << "%\n";
    std::cout << "  Minimum utilization: " << utilizations.front() << "%\n";

    if (!utilizations.empty()) {
      size_t median_idx = utilizations.size() / 2;
      std::cout << "  Median utilization: " << utilizations[median_idx] << "%\n";
    }
  }

  // Inverse Shortest Path problem (ISP)
  // #isp #shortestpath #lp
  {
    LOG_SCOPE_F(INFO, "Inverse Shortest Path problem (ISP)");

    // Create a digraph
    using Digraph = nt::graphs::SmartDigraph;
    Digraph digraph;
    auto    n1 = digraph.addNode();
    auto    n2 = digraph.addNode();
    auto    n3 = digraph.addNode();
    auto    a12 = digraph.addArc(n1, n2);
    auto    a23 = digraph.addArc(n2, n3);
    auto    a13 = digraph.addArc(n1, n3);

    // Create LP solver and ISP instance
    // using MPEnv = nt::mp::glpk::MPEnv;
    // using LPModel = nt::mp::glpk::LPModel;
    MPEnv                                               env;
    LPModel                                             lp_solver("lp_solver", env);
    nt::graphs::InverseShortestPath< Digraph, LPModel > isp(lp_solver);

    // Specify that path n1->n2->n3 should be shortest for terminal n3
    isp.addForwardingArc(a12, n3);
    isp.addForwardingArc(a23, n3);

    // Build and solve the model
    isp.build(digraph);
    auto status = isp.solve();

    if (status == nt::mp::ResultStatus::OPTIMAL) {
      // Retrieve computed weights
      for (Digraph::ArcIt arc(digraph); arc != nt::INVALID; ++arc) {
        double weight = isp.W(arc)->solution_value();
        std::cout << "Arc weight: " << weight << std::endl;
      }
    }
  }

  // Vertex cover problem (VCLP)
  // #vclp #vertexcover #lp
  {
    LOG_SCOPE_F(INFO, "Vertex Cover problem (VCLP)");

    // Create a bidirected graph
    using Digraph = nt::graphs::SmartDigraph;
    Digraph digraph;
    auto n1 = digraph.addNode();
    auto n2 = digraph.addNode();
    auto n3 = digraph.addNode();
    auto n4 = digraph.addNode();
    
    // Add bidirectional edges
    digraph.addArc(n1, n2);
    digraph.addArc(n2, n1);
    digraph.addArc(n2, n3);
    digraph.addArc(n3, n2);
    digraph.addArc(n3, n4);
    digraph.addArc(n4, n3);
    
    // Create MIP solver and Vclp instance for exact integer solution
    MPEnv                                               env;
    LPModel                                             mip_solver("lp_solver", env);
    nt::graphs::Vclp<Digraph, LPModel> vclp(mip_solver);
    
    // Build and solve the model
    vclp.build(digraph);
    auto status = vclp.solve();
    
    if (status == nt::mp::ResultStatus::OPTIMAL) {
      // Get the minimum vertex cover size
      double cover_size = vclp.getVertexCoverNumber();
      std::cout << "Minimum vertex cover size: " << cover_size << std::endl;
    
      // Check which vertices are in the cover
      for (Digraph::NodeIt node(digraph); node != nt::INVALID; ++node) {
        double val = vclp.X(node)->solution_value();
        if (val > 0.5) {  // For MIP, will be 0 or 1
          std::cout << "Node " << digraph.id(node) << " is in the cover" << std::endl;
        }
      }
    }
  }

  // Shortest path routing 
  {
    using Digraph = nt::graphs::SmartDigraph;
    using DemandGraph = nt::graphs::DemandGraph<Digraph>;

     // Setup network
     Digraph network;
     Digraph::ArcMap<double> metrics(network);
     Digraph::ArcMap<double> capacities(network);
     Digraph::Node n1 = network.addNode();
     Digraph::Node n2 = network.addNode();
     Digraph::Node n3 = network.addNode();
     Digraph::Arc a12 = network.addArc(n1, n2); metrics[a12] = 10; capacities[a12] = 100;
     Digraph::Arc a23 = network.addArc(n2, n3); metrics[a23] = 10; capacities[a23] = 100;
     Digraph::Arc a13 = network.addArc(n1, n3); metrics[a13] = 20; capacities[a13] = 100;
    
     // Create demand graph
     DemandGraph demands(network);
     DemandGraph::ArcMap<double> volumes(demands);
     DemandGraph::Arc d = demands.addArc(n1, n3); volumes[d] = 50.0;
    
     // Initialize dynamic routing
     using DynamicShortestPathRouting = nt::te::DynamicShortestPathRouting<Digraph, double, double>;
     DynamicShortestPathRouting dspr(network, metrics);
     dspr.run(demands, volumes);
    
     std::cout << "Initial MLU: " << dspr.maxSaturation(capacities) << std::endl;
    
     // Save current state using snapshot
     DynamicShortestPathRouting::Snapshot snapshot(dspr);
     snapshot.save();
  
     // Simulate link failure by increasing metric to infinity
     dspr.updateArcWeight(a23, DynamicShortestPathRouting::infinity);
    
     std::cout << "After failure MLU: " << dspr.maxSaturation(capacities) << std::endl;
    
     // Rollback changes using snapshot
     snapshot.restore();
     std::cout << "After restore MLU: " << dspr.maxSaturation(capacities) << std::endl;
  }

  // Underlying solver
  // #cplex #glpk #scip #cbc
  {
    LOG_SCOPE_F(INFO, "Underlying solver");

    MPEnv   env;
    LPModel model("my_model", env);

    //  underlying_solver() returns the underlying solver so that the user can use solver-specific
    //  features or features that are not exposed in networktools API.

    //  This method is for advanced users, use at your own risk! In particular, if
    //  you modify the model or the solution by accessing the underlying solver
    //  directly, then the underlying solver will be out of sync with the
    //  information kept in the wrapper (MPSolver, MPVariable, MPConstraint,
    //  MPObjective).

    // You need to cast the void* returned back to its original type that depends on the interface:
    //  CBC: OsiClpSolverInterface*
    //  CLP: ClpSimplex*
    //  GLPK: glp_prob*
    //  SCIP: SCIP*
    //  CPLEX: CPXLPptr

#if 0
    {
      // underlying_solver() returns only a pointer to CPXLPptr
      CPXLPptr lp = static_cast< CPXLPptr >(model.underlying_solver());
    }
    {
      // If you need both the pointer to CPXENVptr and CPXLPptr, you may get them like so
      CPXENVptr env = model.get_env();
      CPXLPptr  lp = model.get_lp();
      char      buff[32];
      CPXSIZE   surplus;
      CPXXgetprobname(env, lp, buff, 32, &surplus);
      LOG_F(INFO, " CPXXgetprobname = {}", buff);
    }
#endif

#if 0
    glp_prob* solver = static_cast< glp_prob* >(model.underlying_solver());
    LOG_F(INFO, " glp_get_prob_name = {}", glp_get_prob_name(solver));
#endif

#if 0
    OsiClpSolverInterface* solver = static_cast< OsiClpSolverInterface* >(model.underlying_solver());
#endif
  }

  return 0;
}
