# Networktools MP (Mathematical Programming) Module

A header-only C++20 library for Linear Programming (LP) and Mixed-Integer Programming (MIP) that provides a unified interface to multiple solver backends.

## Directory Structure

```
mp/
├── mp.h                    # Main header: enums, parameters, and API documentation
├── bits/mp_solver_base.h   # Base implementation (internal - do not include directly)
└── mp_<solver>.h           # Solver-specific headers (CPLEX, Gurobi, GLPK, etc.)
```

### Header Inclusion Order

1. **User code**: Include only `mp/mp_<solver>.h` (e.g., `mp/mp_cplex.h`)
2. **Automatic inclusion**: `mp_<solver>.h` includes `bits/mp_solver_base.h` internally
3. **Never include directly**: `bits/mp_solver_base.h`

## Overview

This module is based on the `linear_solver` component from Google's [OR-Tools framework](https://github.com/google/or-tools/tree/stable/ortools/linear_solver), extensively modified and enhanced for integration with networktools.

### Key Differences from OR-Tools

- **Header-only**: No separate compilation required
- **Zero external dependencies**: Removed Abseil, Protocol Buffers, etc.
- **Modern logging**: Uses [loguru](https://github.com/emilk/loguru) + [fmtlib](https://github.com/fmtlib/fmt)
- **Native integration**: Uses networktools containers instead of STL
- **Enhanced features**:
  - Farkas dual values for infeasibility analysis
  - Multi-dimensional variable arrays
  - Callback support for branch-and-cut algorithms (CPLEX)
  - Graph-based variable creation utilities
- **Consistent naming**: camelCase methods following networktools/LEMON conventions

## Supported Solvers

| Solver | Header | Type Aliases | License |
|--------|--------|--------------|---------|
| CPLEX | `mp/mp_cplex.h` | `nt::mp::cplex::LPModel`, `MIPModel` | Commercial |
| Gurobi | `mp/mp_gurobi.h` | `nt::mp::gurobi::LPModel`, `MIPModel` | Commercial |
| GLPK | `mp/mp_glpk.h` | `nt::mp::glpk::LPModel`, `MIPModel` | Open Source  |
| CBC | `mp/mp_cbc.h` | `nt::mp::cbc::MIPModel` | Open Source  |
| HiGHS | `mp/mp_highs.h` | `nt::mp::highs::LPModel`, `MIPModel` | Open Source  |
| CLP | `mp/mp_clp.h` | `nt::mp::clp::LPModel` | Open Source  |
| Dummy | `mp/mp_dummy.h` | `nt::mp::dummy::LPModel`, `MIPModel` | Testing only |

## Quick Start

### Basic Example

Include the solver header you need.

```cpp
// main.cpp
#include "networktools.h"
#include "mp/mp_cplex.h"

int main() {
    // Create a CPLEX-based MIP model
    nt::mp::cplex::MIPModel model("MyModel");
    
    // Create variables using camelCase methods
    auto* x = model.makeNumVar(0.0, nt::mp::infinity(), "x");
    auto* y = model.makeIntVar(0.0, 10.0, "y");
    
    // Add constraints
    auto* c1 = model.makeRowConstraint(0, 100, "constraint_1");
    c1->setCoefficient(x, 2.0);
    c1->setCoefficient(y, 1.0);
    
    // Set objective
    auto* objective = model.mutableObjective();
    objective->setCoefficient(x, 3.0);
    objective->setCoefficient(y, 2.0);
    objective->setMaximization();
    
    // Solve
    const auto status = model.solve();
    if (status == nt::mp::ResultStatus::OPTIMAL) {
        std::cout << "Optimal value: " << objective->value() << std::endl;
        std::cout << "x = " << x->solutionValue() << std::endl;
        std::cout << "y = " << y->solutionValue() << std::endl;
    }
    
    return 0;
}
```

> **Note**: See [networktools_demo.cpp](https://gitlab.tech.orange/more/networktools/-/raw/master/src/networktools/networktools_demo.cpp) for more examples. Search for `#mip` or `#lp` tags.

## Compilation

### Requirements
- C++20 compatible compiler (GCC 10+, Clang 12+, MSVC 2019+)
- Solver library installed (e.g., CPLEX, Gurobi, GLPK)

**CPLEX Example** (assuming CPLEX 12.10):
```bash
g++ -O3 -std=c++20 main.cpp -o main \
    -lcplex \
    -I/opt/ibm/ILOG/CPLEX_Studio1210/cplex/include \
    -L/opt/ibm/ILOG/CPLEX_Studio1210/cplex/lib/x86-64_linux/static_pic
```

**GLPK Example** (open-source alternative):
```bash
g++ -O3 -std=c++20 main.cpp -o main -lglpk
```

**Gurobi Example**:
```bash
g++ -O3 -std=c++20 main.cpp -o main \
    -lgurobi_c++ -lgurobi110 \
    -I/opt/gurobi1100/linux64/include \
    -L/opt/gurobi1100/linux64/lib
```

## API Reference

### Core Classes

#### MPSolver
The main solver interface. Type aliases available:
- `nt::mp::cplex::LPModel` / `MIPModel`
- `nt::mp::gurobi::LPModel` / `MIPModel`
- `nt::mp::glpk::LPModel` / `MIPModel`
- etc.

#### Key Methods

**Variable Creation:**
```cpp
MPVariable* makeVar(double lb, double ub, bool integer, const string& name);
MPVariable* makeNumVar(double lb, double ub, const string& name);
MPVariable* makeIntVar(double lb, double ub, const string& name);
MPVariable* makeBoolVar(const string& name);
```

**Constraint Creation:**
```cpp
MPConstraint* makeRowConstraint(double lb, double ub, const string& name);
```

**Objective:**
```cpp
MPObjective* mutableObjective();  // Get mutable objective
void setMaximization();
void setMinimization();
```

**Solving:**
```cpp
ResultStatus solve();  // Solve the model
ResultStatus solve(const MPSolverParameters& params);
```

**Solution Queries:**
```cpp
double objective().value();           // Objective value
double variable->solutionValue();     // Variable value
int numVariables();                   // Number of variables
int numConstraints();                 // Number of constraints
```

### Advanced Features

**Multi-dimensional Variables:**
```cpp
// Create 3x2 array of positive variables X_ij: X_00, X_01, X_10, X_11, X_20 and X_21
MPMultiDimVariable< 2 > vars = model.makeMultiDimNumVar(0., nt::mp::infinity(), "X", 3, 2);
```

**Graph-based Variables:**
```cpp
// Create variables indexed by graph nodes
MPGraphVariable< Digraph, Node > nodeVars = model.makeGraphNumVar< Digraph, Node >(0., mp::infinity(), "X", digraph);
for (Node node : nodeVars) {
    // Use nodeVars(v)
}
```

**Callbacks (CPLEX example):**
```cpp
struct MyCutCallback : nt::cplex::MIPModel::MPCallback {
    void runCallback(MPCallbackContext* p_context) override {
        // Add custom cuts during branch-and-cut
    }
};
MyCutCallback callback;
model.setCallback(&callback);
```
