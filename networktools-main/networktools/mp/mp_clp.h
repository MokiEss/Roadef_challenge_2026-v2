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
 * @file mp_clp.h
 * @brief CLP (COIN-OR Linear Programming) solver integration for networktools MP module
 *
 * This header enables COIN-OR CLP support for LP problems.
 * Include this file after networktools.h to use CLP solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_clp.h"
 *
 * nt::mp::clp::LPModel model;
 * @endcode
 *
 * @par Features:
 * - LP solver only (continuous variables)
 * - Open-source simplex and interior point methods
 * - Efficient for large-scale LP problems
 * - Part of the COIN-OR optimization suite
 *
 * @par Requirements:
 * - CLP library from COIN-OR
 * - Link with CLP library (-lClp -lCoinUtils)
 *
 * @note Use CBC for MIP problems (CBC internally uses CLP)
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

// This file contains a modified version of the source code from Google OR-Tools linear programming
// module. See the appropriate copyright notice below.

// Copyright 2010-2021 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef _NT_CLP_INTERFACE_H_
#define _NT_CLP_INTERFACE_H_

#include "bits/mp_solver_base.h"

#undef PACKAGE
#undef VERSION

#include <coin/ClpConfig.h>
#include <coin/ClpMessage.hpp>
#include <coin/ClpSimplex.hpp>
#include <coin/CoinBuild.hpp>

namespace nt {
  namespace mp {
    namespace clp {

      struct MPEnv {
        constexpr bool isValid() const noexcept { return true; }
      };

      namespace details {

        // Variable indices are shifted by 1 internally because of the dummy "objective
        // offset" variable (with internal index 0).
        constexpr int MPSolverVarIndexToClpVarIndex(int var_index) noexcept { return var_index + 1; }

        constexpr BasisStatus TransformCLPBasisStatus(ClpSimplex::Status clp_basis_status) noexcept {
          switch (clp_basis_status) {
            case ClpSimplex::isFree: return BasisStatus::FREE;
            case ClpSimplex::basic: return BasisStatus::BASIC;
            case ClpSimplex::atUpperBound: return BasisStatus::AT_UPPER_BOUND;
            case ClpSimplex::atLowerBound: return BasisStatus::AT_LOWER_BOUND;
            case ClpSimplex::superBasic: return BasisStatus::FREE;
            case ClpSimplex::isFixed: return BasisStatus::FIXED_VALUE;
            default: LOG_F(FATAL, "Unknown CLP basis status"); return BasisStatus::FREE;
          }
        }

        struct CLPSolver: nt::mp::details::MPSolver< CLPSolver > {
          using Parent = nt::mp::details::MPSolver< CLPSolver >;
          using MPObjective = Parent::MPObjective;
          using MPConstraint = Parent::MPConstraint;
          using MPVariable = Parent::MPVariable;
          using LinearRange = Parent::LinearRange;

          using Parent::Parent;

          struct MPCallbackContext: Parent::template MPCallbackContextInterface< MPCallbackContext > {
            MPCallbackEvent event() { return MPCallbackEvent::kUnknown; }
            bool            canQueryVariableValues() { return false; }
            double          variableValue(const MPVariable* variable) { return 0.; }
            void            addCut(const LinearRange& cutting_plane) {}
            void            addLazyConstraint(const LinearRange& lazy_constraint) {}
            void            addPricedVariable(const MPVariable* variable) {}
            double          suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) { return 0.; }
            int64_t         numExploredNodes() { return 0; }
          };

          using MPCallback = Parent::template MPCallback< MPCallbackContext >;
          using MPCallbackList = Parent::template MPCallbackList< MPCallbackContext >;

          // Creates a LP/MIP instance with the specified name and minimization objective.
          CLPSolver(const std::string& name, const MPEnv& e) :
              Parent(name), clp_(new ClpSimplex), options_(new ClpSolve) {
            clp_->setStrParam(ClpProbName, name);
            clp_->setOptimizationDirection(1);
          }

          ~CLPSolver() {}

          void _impl_SetOptimizationDirection(bool maximize) noexcept {
            Parent::invalidateSolutionSynchronization();
            clp_->setOptimizationDirection(maximize ? -1 : 1);
          }

          // Solve the LP/MIP. Returns true only if the optimal solution was revealed.
          // Returns the status of the search.
          ResultStatus _impl_Solve(const MPSolverParameters& param) noexcept {
            try {
              nt::WallTimer timer;
              timer.start();

              if (param.getIntegerParam(MPSolverParameters::INCREMENTALITY) == MPSolverParameters::INCREMENTALITY_OFF) {
                Reset();
              }

              // Set log level.
              CoinMessageHandler message_handler;
              clp_->passInMessageHandler(&message_handler);
              if (_quiet) {
                message_handler.setLogLevel(1, 0);
                clp_->setLogLevel(0);
              } else {
                message_handler.setLogLevel(1, 1);
                clp_->setLogLevel(1);
              }

              // Special case if the model is empty since CLP is not able to
              // handle this special case by itself.
              if (Parent::_variables.empty() && Parent::_constraints.empty()) {
                _sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
                _result_status = ResultStatus::OPTIMAL;
                _objective_value = Parent::Objective().offset();
                return _result_status;
              }

              extractModel();
              VLOG_F(1, "Model built in {} seconds.", timer.get());

              // Time limit.
              if (Parent::time_limit() != 0) {
                VLOG_F(1, "Setting time limit = {} ms.", Parent::time_limit());
                clp_->setMaximumSeconds(Parent::time_limit_in_secs());
              } else {
                clp_->setMaximumSeconds(-1.0);
              }

              // Start from a fresh set of default parameters and set them to
              // specified values.
              options_ = std::make_unique< ClpSolve >();
              setParameters(param);

              // Solve
              timer.restart();
              clp_->initialSolve(*options_);
              VLOG_F(1, "Solved in {} seconds.", timer.get());

              // Check the status: optimal, infeasible, etc.
              int tmp_status = clp_->status();
              VLOG_F(1, "clp result status: {}", tmp_status);
              switch (tmp_status) {
                case CLP_SIMPLEX_FINISHED: _result_status = ResultStatus::OPTIMAL; break;
                case CLP_SIMPLEX_INFEASIBLE: _result_status = ResultStatus::INFEASIBLE; break;
                case CLP_SIMPLEX_UNBOUNDED: _result_status = ResultStatus::UNBOUNDED; break;
                case CLP_SIMPLEX_STOPPED: _result_status = ResultStatus::FEASIBLE; break;
                default: _result_status = ResultStatus::ABNORMAL; break;
              }

              if (_result_status == ResultStatus::OPTIMAL || _result_status == ResultStatus::FEASIBLE) {
                // Get the results
                _objective_value = clp_->objectiveValue();
                VLOG_F(1, "objective= {}", _objective_value);
                const double* const values = clp_->getColSolution();
                const double* const reduced_costs = clp_->getReducedCost();
                for (int i = 0; i < Parent::_variables.size(); ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  const int         clp_var_index = MPSolverVarIndexToClpVarIndex(var->index());
                  const double      val = values[clp_var_index];
                  var->set_solution_value(val);
                  VLOG_F(3, "{}: value = {}", var->name(), val);
                  double reduced_cost = reduced_costs[clp_var_index];
                  var->set_reduced_cost(reduced_cost);
                  VLOG_F(4, "{}: reduced cost = {}", var->name(), reduced_cost);
                }
                const double* const dual_values = clp_->getRowPrice();
                for (int i = 0; i < Parent::_constraints.size(); ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  const int           constraint_index = ct->index();
                  const double        dual_value = dual_values[constraint_index];
                  ct->set_dual_value(dual_value);
                  VLOG_F(4, "row {} dual value = {}", ct->index(), dual_value);
                }
              }

              ResetParameters();
              _sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
              return _result_status;
            } catch (CoinError& e) {
              LOG_F(WARNING, "Caught exception in Coin LP: {}", e.message());
              _result_status = ResultStatus::ABNORMAL;
              return _result_status;
            }
          }

          void _impl_Reset() noexcept {
            clp_ = std::make_unique< ClpSimplex >();
            clp_->setOptimizationDirection(Parent::_maximize ? -1 : 1);
            Parent::resetExtractionInformation();
          }

          void _impl_SetVariableBounds(int var_index, double lb, double ub) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::variable_is_extracted(var_index)) {
              // Not cached if the variable has been extracted
              DCHECK_LT_F(var_index, _last_variable_index);
              clp_->setColumnBounds(MPSolverVarIndexToClpVarIndex(var_index), lb, ub);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetVariableInteger(int var_index, bool integer) noexcept {}
          void _impl_SetConstraintBounds(int index, double lb, double ub) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::constraint_is_extracted(index)) {
              // Not cached if the row has been extracted
              DCHECK_LT_F(index, _last_constraint_index);
              clp_->setRowBounds(index, lb, ub);
            } else {
              _sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }

          void _impl_AddRowConstraint(MPConstraint* ct) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_AddVariable(MPVariable* var) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetCoefficient(MPConstraint*     constraint,
                                    const MPVariable* variable,
                                    double            new_value,
                                    double            old_value) noexcept {
            invalidateSolutionSynchronization();
            if (Parent::constraint_is_extracted(constraint->index())
                && Parent::variable_is_extracted(variable->index())) {
              // The modification of the coefficient for an extracted row and
              // variable is not cached.
              DCHECK_LE_F(constraint->index(), _last_constraint_index);
              DCHECK_LE_F(variable->index(), _last_variable_index);
              clp_->modifyCoefficient(constraint->index(), MPSolverVarIndexToClpVarIndex(variable->index()), new_value);
            } else {
              // The modification of an unextracted row or variable is cached
              // and handled in ExtractModel.
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_ClearConstraint(MPConstraint* constraint) noexcept {
            Parent::invalidateSolutionSynchronization();
            // Constraint may not have been extracted yet.
            if (!Parent::constraint_is_extracted(constraint->index())) return;
            for (const auto& entry: constraint->_coefficients) {
              DCHECK_F(Parent::variable_is_extracted(entry.first->index()));
              clp_->modifyCoefficient(constraint->index(), MPSolverVarIndexToClpVarIndex(entry.first->index()), 0.0);
            }
          }
          void _impl_SetObjectiveCoefficient(const MPVariable* variable, double coefficient) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::variable_is_extracted(variable->index())) {
              clp_->setObjectiveCoefficient(MPSolverVarIndexToClpVarIndex(variable->index()), coefficient);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetObjectiveOffset(double offset) noexcept {
            // Constant term. Use -offset instead of +offset because CLP does
            // not follow conventions.
            Parent::invalidateSolutionSynchronization();
            clp_->setObjectiveOffset(-offset);
          }
          void _impl_ClearObjective() noexcept {
            Parent::invalidateSolutionSynchronization();
            // Clear linear terms
            for (const auto& entry: Parent::_objective._coefficients) {
              const int mpsolver_var_index = entry.first->index();
              // Variable may have not been extracted yet.
              if (!Parent::variable_is_extracted(mpsolver_var_index)) {
                DCHECK_NE_F(SynchronizationStatus::MODEL_SYNCHRONIZED, _sync_status);
              } else {
                clp_->setObjectiveCoefficient(MPSolverVarIndexToClpVarIndex(mpsolver_var_index), 0.0);
              }
            }
            // Clear constant term.
            clp_->setObjectiveOffset(0.0);
          }

          int64_t _impl_iterations() const noexcept {
            if (!checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfIterations;
            return clp_->getIterationCount();
          }
          int64_t _impl_nodes() const noexcept {
            LOG_F(FATAL, "Number of nodes only available for discrete problems");
            return nt::mp::details::kUnknownNumberOfNodes;
          }
          BasisStatus _impl_row_status(int constraint_index) const noexcept {
            DCHECK_LE_F(0, constraint_index);
            DCHECK_GT_F(Parent::_last_constraint_index, constraint_index);
            const ClpSimplex::Status clp_basis_status = clp_->getRowStatus(constraint_index);
            return TransformCLPBasisStatus(clp_basis_status);
          }
          BasisStatus _impl_column_status(int variable_index) const noexcept {
            DCHECK_LE_F(0, variable_index);
            DCHECK_GT_F(Parent::_last_variable_index, variable_index);
            const ClpSimplex::Status clp_basis_status =
               clp_->getColumnStatus(MPSolverVarIndexToClpVarIndex(variable_index));
            return TransformCLPBasisStatus(clp_basis_status);
          }

          bool _impl_HasDualFarkas() const noexcept { return false; }

          bool _impl_IsContinuous() const noexcept { return _impl_IsLP(); }
          bool _impl_IsLP() const noexcept { return true; }
          bool _impl_IsMIP() const noexcept { return false; }

          void _impl_ExtractNewVariables() noexcept {
            // Define new variables
            int total_num_vars = Parent::_variables.size();
            if (total_num_vars > _last_variable_index) {
              if (_last_variable_index == 0 && _last_constraint_index == 0) {
                // Faster extraction when nothing has been extracted yet.
                clp_->resize(0, total_num_vars + 1);
                CreateDummyVariableForEmptyConstraints();
                for (int i = 0; i < total_num_vars; ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  set_variable_as_extracted(i, true);
                  if (!var->name().empty()) {
                    std::string name = var->name();
                    clp_->setColumnName(MPSolverVarIndexToClpVarIndex(i), name);
                  }
                  clp_->setColumnBounds(MPSolverVarIndexToClpVarIndex(i), var->lb(), var->ub());
                }
              } else {
                // TODO(user): This could perhaps be made slightly faster by
                // iterating through old constraints, constructing by hand the
                // column-major representation of the addition to them and call
                // clp_->addColumns. But this is good enough for now.
                // Create new variables.
                for (int j = _last_variable_index; j < total_num_vars; ++j) {
                  MPVariable* const var = Parent::_variables[j];
                  DCHECK_F(!variable_is_extracted(j));
                  set_variable_as_extracted(j, true);
                  // The true objective coefficient will be set later in ExtractObjective.
                  double tmp_obj_coef = 0.0;
                  clp_->addColumn(0, nullptr, nullptr, var->lb(), var->ub(), tmp_obj_coef);
                  if (!var->name().empty()) {
                    std::string name = var->name();
                    clp_->setColumnName(MPSolverVarIndexToClpVarIndex(j), name);
                  }
                }
                // Add new variables to existing constraints.
                for (int i = 0; i < _last_constraint_index; i++) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  const int           ct_index = ct->index();
                  for (const auto& entry: ct->_coefficients) {
                    const int mpsolver_var_index = entry.first->index();
                    DCHECK_F(variable_is_extracted(mpsolver_var_index));
                    if (mpsolver_var_index >= _last_variable_index) {
                      clp_->modifyCoefficient(
                         ct_index, MPSolverVarIndexToClpVarIndex(mpsolver_var_index), entry.second);
                    }
                  }
                }
              }
            }
          }
          void _impl_ExtractNewConstraints() noexcept {
            int total_num_rows = Parent::_constraints.size();
            if (_last_constraint_index < total_num_rows) {
              // Find the length of the longest row.
              int max_row_length = 0;
              for (int i = _last_constraint_index; i < total_num_rows; ++i) {
                MPConstraint* const ct = Parent::_constraints[i];
                DCHECK_F(!constraint_is_extracted(ct->index()));
                set_constraint_as_extracted(ct->index(), true);
                if (ct->_coefficients.size() > max_row_length) { max_row_length = ct->_coefficients.size(); }
              }
              // Make space for dummy variable.
              max_row_length = std::max(1, max_row_length);
              std::unique_ptr< int[] >    indices(new int[max_row_length]);
              std::unique_ptr< double[] > coefs(new double[max_row_length]);
              CoinBuild                   build_object;
              // Add each new constraint.
              for (int i = _last_constraint_index; i < total_num_rows; ++i) {
                MPConstraint* const ct = Parent::_constraints[i];
                DCHECK_F(constraint_is_extracted(ct->index()));
                int size = ct->_coefficients.size();
                if (size == 0) {
                  // Add dummy variable to be able to build the constraint.
                  indices[0] = nt::mp::details::kDummyVariableIndex;
                  coefs[0] = 1.0;
                  size = 1;
                }
                int j = 0;
                for (const auto& entry: ct->_coefficients) {
                  const int mpsolver_var_index = entry.first->index();
                  DCHECK_F(variable_is_extracted(mpsolver_var_index));
                  indices[j] = MPSolverVarIndexToClpVarIndex(mpsolver_var_index);
                  coefs[j] = entry.second;
                  j++;
                }
                build_object.addRow(size, indices.get(), coefs.get(), ct->lb(), ct->ub());
              }
              // Add and name the rows.
              try {
                clp_->addRows(build_object);
              } catch (CoinError& e) {
                // Note: Exception might be a bug if using Clp 1.17.5 see
                // https://github.com/coin-or/Clp/issues/130
                LOG_F(FATAL,
                      "Caught exception in Coin LP: {} (if you are using version 1.17.5, this "
                      "might be "
                      "a bug "
                      "(see https://github.com/coin-or/Clp/issues/130)",
                      e.message());
                _result_status = ResultStatus::ABNORMAL;
              }
              for (int i = _last_constraint_index; i < total_num_rows; ++i) {
                MPConstraint* const ct = Parent::_constraints[i];
                if (!ct->name().empty()) {
                  std::string name = ct->name();
                  clp_->setRowName(ct->index(), name);
                }
              }
            }
          }
          void _impl_ExtractObjective() noexcept {
            // Linear objective: set objective coefficients for all variables
            // (some might have been modified)
            for (const auto& entry: Parent::_objective._coefficients) {
              clp_->setObjectiveCoefficient(MPSolverVarIndexToClpVarIndex(entry.first->index()), entry.second);
            }

            // Constant term. Use -offset instead of +offset because CLP does
            // not follow conventions.
            clp_->setObjectiveOffset(-Parent::Objective().offset());
          }

          std::string _impl_SolverVersion() const noexcept { return "Clp " CLP_VERSION; }

          // TODO(user): Maybe we should expose the CbcModel build from osi_
          // instead, but a new CbcModel is built every time Solve is called,
          // so it is not possible right now.
          void* _impl_underlying_solver() noexcept { return reinterpret_cast< void* >(&clp_); }

          // See MPSolver::SetCallback() for details.
          void _impl_SetCallback(MPCallback* mp_callback) noexcept {}
          bool _impl_SupportsCallbacks() const noexcept { return true; }

          // private:
          void _impl_SetParameters(const MPSolverParameters& param) noexcept { Parent::setCommonParameters(param); }
          void _impl_SetRelativeMipGap(double value) noexcept {
            LOG_F(WARNING, "The relative MIP gap is only available for discrete problems.");
          }
          void _impl_SetPrimalTolerance(double value) noexcept { clp_->setPrimalTolerance(value); }
          void _impl_SetDualTolerance(double value) noexcept { clp_->setDualTolerance(value); }
          void _impl_SetPresolveMode(int value) noexcept {
            switch (value) {
              case MPSolverParameters::PRESOLVE_OFF: {
                options_->setPresolveType(ClpSolve::presolveOff);
                break;
              }
              case MPSolverParameters::PRESOLVE_ON: {
                options_->setPresolveType(ClpSolve::presolveOn);
                break;
              }
              default: {
                setIntegerParamToUnsupportedValue(MPSolverParameters::PRESOLVE, value);
              }
            }
          }
          void _impl_SetScalingMode(int scaling) noexcept {
            Parent::SetUnsupportedIntegerParam(MPSolverParameters::SCALING);
          }
          void _impl_SetLpAlgorithm(int value) noexcept {
            switch (value) {
              case MPSolverParameters::DUAL: {
                options_->setSolveType(ClpSolve::useDual);
                break;
              }
              case MPSolverParameters::PRIMAL: {
                options_->setSolveType(ClpSolve::usePrimal);
                break;
              }
              case MPSolverParameters::BARRIER: {
                options_->setSolveType(ClpSolve::useBarrier);
                break;
              }
              default: {
                setIntegerParamToUnsupportedValue(MPSolverParameters::LP_ALGORITHM, value);
              }
            }
          }

          bool _impl_SetNumThreads(int num_threads) noexcept {
            CHECK_GE_F(num_threads, 1);
            num_threads_ = num_threads;
            return true;
          }
          bool _impl_SetSolverSpecificParametersAsString(const std::string& parameters) noexcept {
            LOG_F(ERROR, "SetSolverSpecificParametersAsString() not currently supported for Clp.");
            return false;
          }

          void CreateDummyVariableForEmptyConstraints() noexcept {
            clp_->setColumnBounds(nt::mp::details::kDummyVariableIndex, 0.0, 0.0);
            clp_->setObjectiveCoefficient(nt::mp::details::kDummyVariableIndex, 0.0);
            // Workaround for peculiar signature of setColumnName. Note that we do need
            // std::string here, and not 'string', which aren't the same as of 2013-12
            // (this will change later).
            std::string dummy = "dummy";   // We do need to create this temporary variable.
            clp_->setColumnName(nt::mp::details::kDummyVariableIndex, dummy);
          }

          // Reset to their default value the parameters for which CLP has a
          // stateful API. To be called after the solve so that the next solve
          // starts from a clean parameter state.
          void ResetParameters() noexcept {
            clp_->setPrimalTolerance(MPSolverParameters::kDefaultPrimalTolerance);
            clp_->setDualTolerance(MPSolverParameters::kDefaultDualTolerance);
          }

          std::unique_ptr< ClpSimplex > clp_;       // TODO(user) : remove pointer.
          std::unique_ptr< ClpSolve >   options_;   // For parameter setting.
        };


      }   // namespace details

      using LPModel = details::CLPSolver;

      inline constexpr bool hasMIP() noexcept { return false; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace clp

  }   // namespace mp
}   // namespace nt


#endif