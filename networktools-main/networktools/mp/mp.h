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
 * @file mp.h
 * @brief Mathematical Programming (MP) module - Core definitions and API
 *
 * This is the **main header** for the MP module, providing:
 * - Common enums (ResultStatus, MPCallbackEvent, BasisStatus, etc.)
 * - Solver parameter configuration (MPSolverParameters)
 * - Type definition macros (MODEL_TYPEDEFS)
 * - API documentation
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Module Organization:
 * - **mp.h** (this file): Public API definitions and documentation
 * - **bits/mp_solver_base.h**: Base implementation (internal, auto-included)
 * - **mp_<solver>.h**: Solver-specific headers (CPLEX, Gurobi, GLPK, etc.)
 *
 * @par Usage:
 * Users should only include solver-specific headers (e.g., `mp_cplex.h`).
 * This file and `bits/mp_solver_base.h` are included automatically.
 *
 * @section mp_origins Origins
 * Based on the `linear_solver` component from Google's
 * [OR-Tools framework](https://github.com/google/or-tools).
 *
 * @par Key Modifications from OR-Tools:
 * - Header-only implementation (no separate compilation)
 * - Zero external dependencies (removed Abseil, Protocol Buffers)
 * - Uses networktools containers instead of STL
 * - Added Farkas dual values for infeasibility analysis
 * - Multi-dimensional and graph-based variable arrays
 * - Cut callback support for branch-and-cut algorithms (CPLEX)
 * - SoPlex solver integration
 * - Modern logging (loguru + fmtlib)
 * - camelCase naming following networktools/LEMON conventions
 *
 * @par Original Copyright:
 * Copyright 2010-2018 Google LLC
 * Licensed under the Apache License, Version 2.0
 *
 * @section lp_intro What is Linear Programming?
 *
 * Linear Programming (LP) is an optimization technique for linear objective
 * functions subject to linear equality and inequality constraints. It determines
 * the optimal solution (maximum or minimum) for a mathematical model with linear
 * requirements.
 *
 * The Simplex algorithm (Dantzig, 1947) is the most widely used method, with
 * polynomial average-case performance. Interior point methods (Karmarkar, 1984)
 * are also efficient for very large problems.
 *
 * @par Example Linear Program:
 * @code
 * maximize:    3x + y
 * subject to:  1.5x + 2y <= 12
 *              0 <= x <= 3
 *              0 <= y <= 5
 * @endcode
 *
 * @section mip_intro What is Mixed Integer Programming?
 *
 * Mixed Integer Programming (MIP) extends LP by requiring some or all variables
 * to take integer values. MIPs are NP-hard but powerful for modeling:
 * - Discrete decisions (binary choices)
 * - Logical relationships (if-then constraints)
 * - Piecewise linear approximations
 *
 * @section usage_guide Usage Guide
 *
 * Build models using MPSolver, then query solutions via MPSolver, MPVariable,
 * and MPConstraint classes.
 *
 * @par Basic Workflow:
 * 1. Create solver instance
 * 2. Add variables with bounds and integrality
 * 3. Add constraints
 * 4. Set objective function
 * 5. Call solve()
 * 6. Query solution values
 *
 * @par Important:
 * - Solution queries require a successful solve() call
 * - Model must not be modified between solve() and solution queries
 * - Some queries (e.g., reduced costs) only apply to continuous problems
 *
 * @see networktools_demo.cpp for usage examples
 *
 * @section arch_notes Architecture Notes
 *
 * MPSolver maintains the model in its own data structures while wrapping
 * underlying solver interfaces (GLPK, CLP, CBC, SCIP, CPLEX, Gurobi, etc.).
 * Synchronization between MPSolver and the underlying solver occurs via an
 * extraction mechanism (synchronous for some operations, asynchronous for others).
 */

#ifndef _NT_MP_H_
#define _NT_MP_H_

#include "../core/common.h"
#include "../core/logging.h"

/**
 * @brief Convenience macros for MIP/LP model type definitions
 *
 * These macros create typedefs for MP model-related types (MPObjective, MPConstraint,
 * MPVariable, etc.). They work similarly to the XXX_TYPEDEFS() macros in the graphs module.
 *
 * @par Usage:
 * - Use TEMPLATE_MODEL_TYPEDEFS() or MODEL_TYPEDEFS() at namespace/class scope
 * - Use TEMPLATE_MODEL_TYPEDEFS_BLOCKSCOPE() or MODEL_TYPEDEFS_BLOCKSCOPE() inside functions
 *
 * @note The TEMPLATE_ variants are for dependent types in templates
 */

#define TEMPLATE_MODEL_TYPEDEFS(Model)                                            \
  using MPObjective = typename Model::MPObjective;                                \
  using MPConstraint = typename Model::MPConstraint;                              \
  using MPVariable = typename Model::MPVariable;                                  \
  using MPCallback = typename Model::MPCallback;                                  \
  using MPCallbackContext = typename Model::MPCallbackContext;                    \
  using LinearExpr = typename Model::LinearExpr;                                  \
  using LinearRange = typename Model::LinearRange;                                \
  template < int _DIM >                                                           \
  using MPMultiDimVariable = typename Model::template MPMultiDimVariable< _DIM >; \
  template < class _GR, class... _ELEMENTS >                                      \
  using MPGraphVariable = typename Model::template MPGraphVariable< _GR, _ELEMENTS... >;

#define MODEL_TYPEDEFS(Model)                                   \
  using MPObjective = Model::MPObjective;                       \
  using MPConstraint = Model::MPConstraint;                     \
  using MPVariable = Model::MPVariable;                         \
  using MPCallback = Model::MPCallback;                         \
  using MPCallbackContext = Model::MPCallbackContext;           \
  using LinearExpr = Model::LinearExpr;                         \
  using LinearRange = Model::LinearRange;                       \
  template < int _DIM >                                         \
  using MPMultiDimVariable = Model::MPMultiDimVariable< _DIM >; \
  template < class _GR, class... _ELEMENTS >                    \
  using MPGraphVariable = Model::MPGraphVariable< _GR, _ELEMENTS... >;

#define TEMPLATE_MODEL_TYPEDEFS_BLOCKSCOPE(Model)              \
  using MPObjective = typename Model::MPObjective;             \
  using MPConstraint = typename Model::MPConstraint;           \
  using MPVariable = typename Model::MPVariable;               \
  using MPCallback = typename Model::MPCallback;               \
  using MPCallbackContext = typename Model::MPCallbackContext; \
  using LinearExpr = typename Model::LinearExpr;               \
  using LinearRange = typename Model::LinearRange;

#define MODEL_TYPEDEFS_BLOCKSCOPE(Model)              \
  using MPObjective = Model::MPObjective;             \
  using MPConstraint = Model::MPConstraint;           \
  using MPVariable = Model::MPVariable;               \
  using MPCallback = Model::MPCallback;               \
  using MPCallbackContext = Model::MPCallbackContext; \
  using LinearExpr = Model::LinearExpr;               \
  using LinearRange = Model::LinearRange;

namespace nt {
  namespace mp {

    /**
     * Infinity.
     *
     * You can use -infinity() for negative infinity.
     */
    constexpr inline double infinity() { return NT_DOUBLE_INFINITY; }

    /**
     * @brief Solution status after solving
     *
     * Indicates the result of the optimization process.
     */
    enum class ResultStatus {
      OPTIMAL,         ///< Optimal solution found
      FEASIBLE,        ///< Feasible solution found (may not be optimal, e.g., stopped by time limit)
      INFEASIBLE,      ///< Problem proven infeasible
      UNBOUNDED,       ///< Problem proven unbounded
      ABNORMAL,        ///< Solver error occurred
      MODEL_INVALID,   ///< Model has invalid coefficients (NaN, etc.)
      NOT_SOLVED = 6   ///< Model not yet solved
    };

    /**
     * @brief Solver callback event types
     *
     * Indicates when a callback is invoked during the solving process.
     * For Gurobi, corresponds to the 'where' parameter in callback API.
     *
     * @see http://www.gurobi.com/documentation/refman/callback_codes.html
     */
    enum class MPCallbackEvent {
      kUnknown,       ///< Unknown event
      kPolling,       ///< Polling event for main thread control (not for solver interaction)
      kPresolve,      ///< During presolve phase
      kSimplex,       ///< During simplex method
      kMip,           ///< MIP loop iteration (before starting new node)
      kMipSolution,   ///< New MIP incumbent found
      kMipNode,       ///< Cut loop pass within MIP node
      kBarrier,       ///< Interior point/barrier method iteration
      kMessage,       ///< Solver message logging
      kMultiObj,      ///< Multi-objective optimization
    };

    /**
     * @brief Synchronization state between MPSolver and underlying solver
     */
    enum class SynchronizationStatus {
      MUST_RELOAD,            ///< Model and solution out of sync, reload required
      MODEL_SYNCHRONIZED,     ///< Model in sync, solution outdated
      SOLUTION_SYNCHRONIZED   ///< Both model and solution in sync
    };

    /**
     * @brief Basis status for simplex-based solvers
     *
     * Advanced usage: Indicates the basis status of a variable or constraint
     * slack variable in simplex algorithms.
     */
    enum class BasisStatus {
      FREE = 0,         ///< Free variable (no active bound)
      AT_LOWER_BOUND,   ///< At lower bound
      AT_UPPER_BOUND,   ///< At upper bound
      FIXED_VALUE,      ///< Fixed to a specific value
      BASIC             ///< Basic variable in simplex basis
    };

    namespace details {
      /** @brief Default primal feasibility tolerance for basic solutions */
      constexpr double kDefaultPrimalTolerance = 1e-07;

      /** @brief Sentinel value when solver doesn't report simplex iteration count */
      constexpr int64_t kUnknownNumberOfIterations = -1;

      /** @brief Sentinel value when solver doesn't report branch-and-bound node count */
      constexpr int64_t kUnknownNumberOfNodes = -1;

      /** @brief Index of dummy variable for empty constraints or objective offset */
      constexpr int kDummyVariableIndex = 0;
    }   // namespace details

    /**
     * @brief Solver parameter configuration
     *
     * Stores parameter settings for LP and MIP solvers. Parameters controlling
     * solution properties (e.g., tolerances) or consistently improving performance
     * have wrapper-defined defaults.
     *
     * @warning Advanced parameters: Modify with caution!
     *
     * @par Adding New Parameters (for developers):
     * 1. Add to DoubleParam or IntegerParam enum
     * 2. Create FooValues enum if categorical
     * 3. Define kDefaultFoo if wrapper should set default
     * 4. Add foo_value_ member (and foo_is_default_ if no default)
     * 5. Update Set/Reset/Get methods and constructor
     * 6. Implement in MPSolverInterface subclasses
     * 7. Add tests
     */
    struct MPSolverParameters {
      /**
       * @brief Floating-point solver parameters
       */
      enum DoubleParam {
        RELATIVE_MIP_GAP = 0,   ///< Relative MIP optimality gap tolerance
        PRIMAL_TOLERANCE = 1,   ///< Primal feasibility tolerance (advanced, doesn't affect MIP integrality or presolve)
        DUAL_TOLERANCE = 2      ///< Dual feasibility tolerance (advanced)
      };

      /**
       * @brief Integer and categorical solver parameters
       */
      enum IntegerParam {
        PRESOLVE = 1000,         ///< Presolve mode (advanced)
        LP_ALGORITHM = 1001,     ///< LP algorithm selection (simplex, barrier, etc.)
        INCREMENTALITY = 1002,   ///< Warm start mode between solves (advanced)
        SCALING = 1003,          ///< Matrix scaling (advanced)
        MIP_STRATEGY = 1004      ///< MIP search strategy (optimality vs feasibility)
      };

      /**
       * @brief Presolve parameter values
       */
      enum PresolveValues {
        PRESOLVE_OFF = 0,   ///< Disable presolve
        PRESOLVE_ON = 1     ///< Enable presolve
      };

      /**
       * @brief LP algorithm selection values
       */
      enum LpAlgorithmValues {
        DUAL = 10,     ///< Dual simplex method
        PRIMAL = 11,   ///< Primal simplex method
        BARRIER = 12   ///< Interior point/barrier method
      };

      /**
       * @brief Warm start parameter values (advanced)
       */
      enum IncrementalityValues {
        INCREMENTALITY_OFF = 0,   ///< Cold start (solve from scratch)
        INCREMENTALITY_ON = 1     ///< Warm start (reuse previous solution)
      };

      /**
       * @brief Matrix scaling parameter values (advanced)
       */
      enum ScalingValues {
        SCALING_OFF = 0,   ///< Disable matrix scaling
        SCALING_ON = 1     ///< Enable matrix scaling
      };

      /**
       * @brief MIP search strategy values
       */
      enum MipStrategyValues {
        OPTIMALITY = 0,   ///< Focus on proving optimality
        FEASABILITY = 1   ///< Focus on finding feasible solutions quickly
      };

      /** @brief Sentinel: parameter uses wrapper default */
      static constexpr double kDefaultDoubleParamValue = -1.0;
      /** @brief Sentinel: parameter uses wrapper default */
      static constexpr int kDefaultIntegerParamValue = -1;

      /** @brief Sentinel: parameter value unknown/unset */
      static constexpr double kUnknownDoubleParamValue = -2.0;
      /** @brief Sentinel: parameter value unknown/unset */
      static constexpr int kUnknownIntegerParamValue = -2;

      /** @brief Default relative MIP gap tolerance */
      static constexpr double kDefaultRelativeMipGap = 1e-4;
      /** @brief Default primal feasibility tolerance (matches CLP/GLPK) */
      static constexpr double kDefaultPrimalTolerance = nt::mp::details::kDefaultPrimalTolerance;
      /** @brief Default dual feasibility tolerance (matches CLP/GLPK) */
      static constexpr double kDefaultDualTolerance = 1e-7;
      /** @brief Default presolve setting */
      static constexpr PresolveValues kDefaultPresolve = MPSolverParameters::PRESOLVE_ON;
      /** @brief Default incrementality setting */
      static constexpr IncrementalityValues kDefaultIncrementality = MPSolverParameters::INCREMENTALITY_ON;
      /** @brief Default MIP strategy */
      static constexpr MipStrategyValues kDefaultMipStrategy = MPSolverParameters::OPTIMALITY;


      /**
       * @brief Constructor initializes all parameters to defaults
       */
      MPSolverParameters() :
          _relative_mip_gap_value(kDefaultRelativeMipGap), _primal_tolerance_value(kDefaultPrimalTolerance),
          _dual_tolerance_value(kDefaultDualTolerance), _presolve_value(kDefaultPresolve),
          _scaling_value(kDefaultIntegerParamValue), _lp_algorithm_value(kDefaultIntegerParamValue),
          _incrementality_value(kDefaultIncrementality), _mip_strategy_value(kDefaultMipStrategy),
          _lp_algorithm_is_default(true) {}
      MPSolverParameters(const MPSolverParameters&) = delete;
      MPSolverParameters& operator=(const MPSolverParameters&) = delete;

      /**
       * @brief Set a floating-point parameter
       * @param param Parameter to set
       * @param value New value
       */
      void setDoubleParam(MPSolverParameters::DoubleParam param, double value) {
        switch (param) {
          case RELATIVE_MIP_GAP: {
            _relative_mip_gap_value = value;
            break;
          }
          case PRIMAL_TOLERANCE: {
            _primal_tolerance_value = value;
            break;
          }
          case DUAL_TOLERANCE: {
            _dual_tolerance_value = value;
            break;
          }
          default: {
            LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
          }
        }
      }

      /**
       * @brief Set an integer parameter
       * @param param Parameter to set
       * @param value New value
       */
      void setIntegerParam(MPSolverParameters::IntegerParam param, int value) {
        switch (param) {
          case PRESOLVE: {
            if (value != PRESOLVE_OFF && value != PRESOLVE_ON) {
              LOG_F(ERROR,
                    "Trying to set a supported parameter: {} to an unknown value: {}",
                    static_cast< int >(param),
                    value);
            }
            _presolve_value = value;
            break;
          }
          case SCALING: {
            if (value != SCALING_OFF && value != SCALING_ON) {
              LOG_F(ERROR,
                    "Trying to set a supported parameter: {} to an unknown value: {}",
                    static_cast< int >(param),
                    value);
            }
            _scaling_value = value;
            break;
          }
          case LP_ALGORITHM: {
            if (value != DUAL && value != PRIMAL && value != BARRIER) {
              LOG_F(ERROR,
                    "Trying to set a supported parameter: {} to an unknown value: {}",
                    static_cast< int >(param),
                    value);
            }
            _lp_algorithm_value = value;
            _lp_algorithm_is_default = false;
            break;
          }
          case INCREMENTALITY: {
            if (value != INCREMENTALITY_OFF && value != INCREMENTALITY_ON) {
              LOG_F(ERROR,
                    "Trying to set a supported parameter: {} to an unknown value: {}",
                    static_cast< int >(param),
                    value);
            }
            _incrementality_value = value;
            break;
          }
          case MIP_STRATEGY: {
            if (value != OPTIMALITY && value != FEASABILITY) {
              LOG_F(ERROR,
                    "Trying to set a supported parameter: {} to an unknown value: {}",
                    static_cast< int >(param),
                    value);
            }
            _mip_strategy_value = value;
            break;
          }
          default: {
            LOG_F(ERROR, "Trying to set an unknown parameter: {}.", static_cast< int >(param));
          }
        }
      }

      /**
       * @brief Reset floating-point parameter to default
       * @param param Parameter to reset
       *
       * Uses wrapper default if defined, otherwise solver default.
       */
      void resetDoubleParam(MPSolverParameters::DoubleParam param) {
        switch (param) {
          case RELATIVE_MIP_GAP: {
            _relative_mip_gap_value = kDefaultRelativeMipGap;
            break;
          }
          case PRIMAL_TOLERANCE: {
            _primal_tolerance_value = kDefaultPrimalTolerance;
            break;
          }
          case DUAL_TOLERANCE: {
            _dual_tolerance_value = kDefaultDualTolerance;
            break;
          }
          default: {
            LOG_F(ERROR, "Trying to reset an unknown parameter: {}.", static_cast< int >(param));
          }
        }
      }

      /**
       * @brief Reset integer parameter to default
       * @param param Parameter to reset
       *
       * Uses wrapper default if defined, otherwise solver default.
       */
      void resetIntegerParam(MPSolverParameters::IntegerParam param) {
        switch (param) {
          case PRESOLVE: {
            _presolve_value = kDefaultPresolve;
            break;
          }
          case SCALING: {
            _scaling_value = kDefaultIntegerParamValue;
            break;
          }
          case LP_ALGORITHM: {
            _lp_algorithm_is_default = true;
            break;
          }
          case INCREMENTALITY: {
            _incrementality_value = kDefaultIncrementality;
            break;
          }
          case MIP_STRATEGY: {
            _mip_strategy_value = kDefaultMipStrategy;
            break;
          }
          default: {
            LOG_F(ERROR, "Trying to reset an unknown parameter: {}.", static_cast< int >(param));
          }
        }
      }

      /**
       * @brief Reset all parameters to defaults
       */
      void reset() {
        resetDoubleParam(RELATIVE_MIP_GAP);
        resetDoubleParam(PRIMAL_TOLERANCE);
        resetDoubleParam(DUAL_TOLERANCE);
        resetIntegerParam(PRESOLVE);
        resetIntegerParam(SCALING);
        resetIntegerParam(LP_ALGORITHM);
        resetIntegerParam(INCREMENTALITY);
        resetIntegerParam(MIP_STRATEGY);
      }

      /**
       * @brief Get floating-point parameter value
       * @param param Parameter to query
       * @return Current value
       */
      double getDoubleParam(MPSolverParameters::DoubleParam param) const {
        switch (param) {
          case RELATIVE_MIP_GAP: {
            return _relative_mip_gap_value;
          }
          case PRIMAL_TOLERANCE: {
            return _primal_tolerance_value;
          }
          case DUAL_TOLERANCE: {
            return _dual_tolerance_value;
          }
          default: {
            LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
            return kUnknownDoubleParamValue;
          }
        }
      }

      /**
       * @brief Get integer parameter value
       * @param param Parameter to query
       * @return Current value
       */
      int getIntegerParam(MPSolverParameters::IntegerParam param) const {
        switch (param) {
          case PRESOLVE: {
            return _presolve_value;
          }
          case LP_ALGORITHM: {
            if (_lp_algorithm_is_default) return kDefaultIntegerParamValue;
            return _lp_algorithm_value;
          }
          case INCREMENTALITY: {
            return _incrementality_value;
          }
          case SCALING: {
            return _scaling_value;
          }
          case MIP_STRATEGY: {
            return _mip_strategy_value;
          }
          default: {
            LOG_F(ERROR, "Trying to get an unknown parameter: {}.", static_cast< int >(param));
            return kUnknownIntegerParamValue;
          }
        }
      }

      private:
      /**
       * @name Parameter Storage
       * @{
       */
      double _relative_mip_gap_value;
      double _primal_tolerance_value;
      double _dual_tolerance_value;
      int    _presolve_value;
      int    _scaling_value;
      int    _lp_algorithm_value;
      int    _incrementality_value;
      int    _mip_strategy_value;

      /** @brief Indicates if LP algorithm uses solver default */
      bool _lp_algorithm_is_default;
      /** @} */
    };

  }   // namespace mp
}   // namespace nt

/**
 * @note For implementation details, see bits/mp_solver_base.h (internal use only).
 * @note To use MP solvers, include mp_<solver>.h headers (e.g., mp_cplex.h).
 * @note This file (mp.h) is included automatically by solver headers.
 */

#endif   // _NT_COMMON_H_