<div align="center">
<img src="doc/nt_banner.svg" height="100" /><br>
<i> A fast header-only C++20 library for modeling and solving network optimization problems.</i>
</div>

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)

</div>

<br>

**Networktools** is a header-only C++20 library developed at **Orange Research** for modeling and solving **network optimization problems**. Built on the foundation of the [LEMON](http://lemon.cs.elte.hu/pub/doc/lemon-intro-presentation.pdf) library, it brings modern C++20 features and optimizations to graph-based network algorithms.

- **Fast**: the library is optimized for speed using modern C++20 features.
- **Header-only** and **self-contained**, easy to integrate and extend.
- **Cache efficient** data structures and algorithms, **SIMD** optimizations.
- Multiple graph file formats are supported: [**NetworkX JSON**](https://networkx.org/documentation/stable/reference/readwrite/json_graph.html), [**JSON Graph Format v2**](https://jsongraphformat.info/), [**SNDLib XML**](http://sndlib.zib.de/home.action), [**Binary**](https://graph-tool.skewed.de/static/doc/gt_format.html), [**REPETITA**](https://github.com/svissicchio/Repetita)
- Abstract API to several **LP/MIP solvers** (CPLEX, GUROBI, HIGHS, CBC, GLPK, ...) with **callback support**. Based on a heavily modified version of [Google's OR-tools linear solver module](https://github.com/google/or-tools/tree/stable/ortools/linear_solver).
- Code design and API based on the **[C++ LEMON library](http://lemon.cs.elte.hu/pub/doc/lemon-intro-presentation.pdf)**. All original LEMON data structures and algorithms are integrated directly in networktools to benefit from modern C++ features and optimized memory management.
- **Networking capabilities**: routing protocols (OSPF/ISIS, Segment Routing) and network failure simulation algorithms.

# Use Cases

Networktools is designed for researchers and engineers working on:
- **Traffic Engineering**: Optimize network flows and minimize congestion
- **Routing Protocol Simulation**: Shortest path routing OSPF/ISIS, Segment Routing
- **Network Planning**: Capacity planning and resilience analysis
- **Shortest Path Problems**: With constraints, multiple metrics, and custom objectives
- **Graph Optimization**: Maximum flow, minimum cut, vertex cover, and more

# Quick Start

## Requirements

- **C++20 compiler**: GCC 11.0+ or Clang 14.0+
- **Optional**: LP/MIP solver (CPLEX, GUROBI, HIGHS, GLPK, CBC) if using the mathematical programming module

## Installation

To start using networktools in your project, simply download the [networktools](networktools) folder into your project directory (or project's include path). Then include `networktools.h` where needed.

## Basic Example

This example showcases networktools' routing capabilities, including shortest path routing with ECMP and efficient failure scenario analysis:

```cpp
// main.cpp
#include "networktools.h"

using Digraph = nt::graphs::SmartDigraph;
using DemandGraph = nt::graphs::DemandGraph<Digraph>;
using DynamicShortestPathRouting = nt::te::DynamicShortestPathRouting<Digraph, double, double>;

int main() {
    // Create a network topology with metrics and capacities
    Digraph network;
    Digraph::ArcMap<double> metrics(network);
    Digraph::ArcMap<double> capacities(network);

    // Add nodes (routers)
    Digraph::Node n1 = network.addNode();
    Digraph::Node n2 = network.addNode();
    Digraph::Node n3 = network.addNode();

    // Add arcs (IP links) with the associated ISIS metrics and capacities
    Digraph::Arc a12 = network.addArc(n1, n2); metrics[a12] = 10; capacities[a12] = 100;
    Digraph::Arc a23 = network.addArc(n2, n3); metrics[a23] = 10; capacities[a23] = 100;
    Digraph::Arc a13 = network.addArc(n1, n3); metrics[a13] = 20; capacities[a13] = 100;

    // Create demand graph
    DemandGraph demands(network);
    DemandGraph::ArcMap<double> volumes(demands);

    // Add a demand from n1 to n3 with volume 50
    DemandGraph::Arc d = demands.addArc(n1, n3); volumes[d] = 50.0;

    // Route the demand in the network according to the shortest path routing protocol with ECMP rule
    DynamicShortestPathRouting dspr(network, metrics);
    dspr.run(demands, volumes);

    // Display the maximum link utilization (MLU)
    // Initial MLU: 0.25 (traffic via n1→n2→n3: 25/100 and n1→n3: 25/100, ECMP applies)
    std::cout << "Initial MLU: " << dspr.maxSaturation(capacities) << std::endl;

    // Save current network state using snapshot
    DynamicShortestPathRouting::Snapshot snapshot(dspr);
    snapshot.save();

    // Simulate link failure by increasing metric to infinity
    dspr.updateArcWeight(a23, DynamicShortestPathRouting::infinity);

    // After failure MLU: 0.5 (traffic rerouted via n1→n3: 50/100, no ECMP)
    std::cout << "After failure MLU: " << dspr.maxSaturation(capacities) << std::endl;

    // Rollback changes using snapshot
    snapshot.restore();
    // After restore MLU: 0.25 (back to original routing)
    std::cout << "After restore MLU: " << dspr.maxSaturation(capacities) << std::endl;

    return 0;
}
```

Compile with:
```bash
g++ -O3 -std=c++20 main.cpp -o main
```

# Learn More

Since networktools is based on LEMON, we recommend reviewing LEMON's [tutorials](http://lemon.cs.elte.hu/pub/tutorial/index.html) and presentation [slides](doc/lemon-intro-presentation.pdf).

Additionally, the library documentation and tutorials are provided in a single C++ file: [networktools_demo.cpp](networktools/networktools_demo.cpp). 
**Building** and **reading** this file is the **recommended starting point** for familiarizing yourself with the library.

Follow these steps to build the file on **Unix/Linux** based systems:

1. Make sure that GCC 11.0+ (or Clang 14.0+) is installed
2. Step into the `networktools` folder and run:
```bash
g++ -O3 -std=c++20 networktools_demo.cpp -o networktools_demo
./networktools_demo
```

# Contributing

We welcome contributions! To contribute:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Merge Request

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

# Contributors

See [CONTRIBUTORS.md](CONTRIBUTORS.md).

# License

Networktools is released under the **MIT License**. See [LICENSE](LICENSE) for full details.


