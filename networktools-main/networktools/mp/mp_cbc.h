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
 * @file mp_cbc.h
 * @brief CBC (COIN-OR Branch and Cut) solver integration for networktools MP module
 *
 * This header enables COIN-OR CBC support for MIP problems.
 * Include this file after networktools.h to use CBC solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_cbc.h"
 *
 * nt::mp::cbc::MIPModel model;
 * @endcode
 *
 * @par Features:
 * - MIP solver (no LP-only mode)
 * - Open-source alternative to commercial solvers
 * - Good performance on medium-sized MIP problems
 * - Part of the COIN-OR optimization suite
 *
 * @par Requirements:
 * - CBC library from COIN-OR
 * - Link with CBC and CLP libraries (-lCbc -lCgl -lOsi -lClp -lCoinUtils)
 *
 * @note CBC uses CLP as its LP solver internally
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


#ifndef _NT_CBC_INTERFACE_H_
#define _NT_CBC_INTERFACE_H_

#include "bits/mp_solver_base.h"

#undef PACKAGE
#undef VERSION

#include <coin/CbcConfig.h>
#include <coin/CbcMessage.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CoinModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

namespace nt {
  namespace mp {
    namespace cbc {

      struct MPEnv {
        constexpr bool isValid() const noexcept { return true; }
      };

      namespace details {

        // CBC adds a "dummy" variable with index 0 to represent the objective offset.
        constexpr int MPSolverVarIndexToCbcVarIndex(int var_index) noexcept { return var_index + 1; }

        struct CBCSolver: nt::mp::details::MPSolver< CBCSolver > {
          using Parent = nt::mp::details::MPSolver< CBCSolver >;
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

          CBCSolver(const std::string& name, const MPEnv& e) :
              Parent(name), iterations_(0), nodes_(0), relative_mip_gap_(MPSolverParameters::kDefaultRelativeMipGap) {
            osi_.setStrParam(OsiProbName, name);
            osi_.setObjSense(1);
          }

          ~CBCSolver() {}

          void _impl_SetOptimizationDirection(bool maximize) noexcept {
            Parent::invalidateSolutionSynchronization();
            if (Parent::_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              osi_.setObjSense(maximize ? -1 : 1);
            } else {
              Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }

          // Solve the LP/MIP. Returns true only if the optimal solution was revealed.
          // Returns the status of the search.
          ResultStatus _impl_Solve(const MPSolverParameters& param) noexcept {
            // CBC requires unique variable and constraint names. By using Lookup*, we
            // generate variable and constraint indices and ensure the duplicate name
            // crash will happen here with a readable error message.
            if (!Parent::_variables.empty()) { Parent::lookupVariableOrNull(Parent::_variables[0]->name()); }
            if (!Parent::_constraints.empty()) { Parent::lookupConstraintOrNull(Parent::_constraints[0]->name()); }

            nt::WallTimer timer;
            timer.start();

            // Note that CBC does not provide any incrementality.
            if (param.getIntegerParam(MPSolverParameters::INCREMENTALITY) == MPSolverParameters::INCREMENTALITY_OFF) {
              Parent::Reset();
            }

            // Special case if the model is empty since CBC is not able to
            // handle this special case by itself.
            if (Parent::_variables.empty() && Parent::_constraints.empty()) {
              Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
              Parent::_result_status = ResultStatus::OPTIMAL;
              Parent::_objective_value = Parent::Objective().offset();
              Parent::_best_objective_bound = Parent::Objective().offset();
              return Parent::_result_status;
            }

            // Finish preparing the problem.
            // Define variables.
            switch (Parent::_sync_status) {
              case SynchronizationStatus::MUST_RELOAD: {
                Parent::Reset();
                CoinModel build;
                // Create dummy variable for objective offset.
                build.addColumn(0, nullptr, nullptr, 1.0, 1.0, Parent::Objective().offset(), "dummy", false);
                const int nb_vars = Parent::_variables.size();
                for (int i = 0; i < nb_vars; ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  Parent::set_variable_as_extracted(i, true);
                  const double obj_coeff = Parent::Objective().getCoefficient(var);
                  if (var->name().empty()) {
                    build.addColumn(0, nullptr, nullptr, var->lb(), var->ub(), obj_coeff, nullptr, var->integer());
                  } else {
                    build.addColumn(
                       0, nullptr, nullptr, var->lb(), var->ub(), obj_coeff, var->name().c_str(), var->integer());
                  }
                }

                // Define constraints.
                int max_row_length = 0;
                for (int i = 0; i < Parent::_constraints.size(); ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  Parent::set_constraint_as_extracted(i, true);
                  if (ct->_coefficients.size() > max_row_length) { max_row_length = ct->_coefficients.size(); }
                }
                std::unique_ptr< int[] >    indices(new int[max_row_length]);
                std::unique_ptr< double[] > coefs(new double[max_row_length]);

                for (int i = 0; i < Parent::_constraints.size(); ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  const int           size = ct->_coefficients.size();
                  int                 j = 0;
                  for (const auto& entry: ct->_coefficients) {
                    const int index = MPSolverVarIndexToCbcVarIndex(entry.first->index());
                    indices[j] = index;
                    coefs[j] = entry.second;
                    j++;
                  }
                  if (ct->name().empty()) {
                    build.addRow(size, indices.get(), coefs.get(), ct->lb(), ct->ub());
                  } else {
                    build.addRow(size, indices.get(), coefs.get(), ct->lb(), ct->ub(), ct->name().c_str());
                  }
                }
                osi_.loadFromCoinModel(build);
                break;
              }
              case SynchronizationStatus::MODEL_SYNCHRONIZED: {
                break;
              }
              case SynchronizationStatus::SOLUTION_SYNCHRONIZED: {
                break;
              }
            }

            // Changing optimization direction through OSI so that the model file
            // (written through OSI) has the correct optimization duration.
            osi_.setObjSense(Parent::_maximize ? -1 : 1);

            Parent::_sync_status = SynchronizationStatus::MODEL_SYNCHRONIZED;
            VLOG_F(1, "Model built in {} seconds.", timer.get());

            ResetBestObjectiveBound();

            // Solve
            CbcModel model(osi_);

            // Set log level.
            CoinMessageHandler message_handler;
            model.passInMessageHandler(&message_handler);
            if (Parent::quiet()) {
              message_handler.setLogLevel(0, 0);   // Coin messages
              message_handler.setLogLevel(1, 0);   // Clp messages
              message_handler.setLogLevel(2, 0);   // Presolve messages
              message_handler.setLogLevel(3, 0);   // Cgl messages
            } else {
              message_handler.setLogLevel(0, 1);   // Coin messages
              message_handler.setLogLevel(1, 1);   // Clp messages
              message_handler.setLogLevel(2, 1);   // Presolve messages
              message_handler.setLogLevel(3, 1);   // Cgl messages
            }

            // Time limit.
            if (Parent::time_limit() != 0) {
              VLOG_F(1, "Setting time limit = {} ms", Parent::time_limit());
              model.setMaximumSeconds(Parent::time_limit_in_secs());
            }

            // And solve.
            timer.restart();

            // Here we use the default function from the command-line CBC solver.
            // This enables to activate all the features and get the same performance
            // as the CBC stand-alone executable. The syntax is ugly, however.
            Parent::setParameters(param);
            // Always turn presolve on (it's the CBC default and it consistently
            // improves performance).
            model.setTypePresolve(0);
            // Special way to set the relative MIP gap parameter as it cannot be set
            // through callCbc.
            model.setAllowableFractionGap(relative_mip_gap_);
            // NOTE: Trailing space is required to avoid buffer overflow in cbc.
            nt::MemoryBuffer buffer;
            nt::format_to(std::back_inserter(buffer), "-threads {} -solve ", num_threads_);
            int return_status = num_threads_ == 1 ? callCbc("-solve ", model) : callCbc(nt::to_string(buffer), model);
            const int kBadReturnStatus = 777;
            CHECK_NE_F(kBadReturnStatus, return_status);   // Should never happen according
                                                           // to the CBC source

            VLOG_F(1, "Solved in {} seconds.", timer.get());

            // Check the status: optimal, infeasible, etc.
            int tmp_status = model.status();

            VLOG_F(1, "cbc result status: {}", tmp_status);
            /* Final status of problem
               (info from cbc/.../CbcSolver.cpp,
                See http://cs?q="cbc+status"+file:CbcSolver.cpp)
               Some of these can be found out by is...... functions
               -1 before branchAndBound
               0 finished - check isProvenOptimal or isProvenInfeasible to see
               if solution found
               (or check value of best solution)
               1 stopped - on maxnodes, maxsols, maxtime
               2 difficulties so run was abandoned
               (5 event user programmed event occurred)
            */
            switch (tmp_status) {
              case 0:
                // Order of tests counts; if model.isContinuousUnbounded() returns true,
                // then so does model.isProvenInfeasible()!
                if (model.isProvenOptimal()) {
                  Parent::_result_status = ResultStatus::OPTIMAL;
                } else if (model.isContinuousUnbounded()) {
                  Parent::_result_status = ResultStatus::UNBOUNDED;
                } else if (model.isProvenInfeasible()) {
                  Parent::_result_status = ResultStatus::INFEASIBLE;
                } else if (model.isAbandoned()) {
                  Parent::_result_status = ResultStatus::ABNORMAL;
                } else {
                  Parent::_result_status = ResultStatus::ABNORMAL;
                }
                break;
              case 1:
                if (model.bestSolution() != nullptr) {
                  Parent::_result_status = ResultStatus::FEASIBLE;
                } else {
                  Parent::_result_status = ResultStatus::NOT_SOLVED;
                }
                break;
              default: Parent::_result_status = ResultStatus::ABNORMAL; break;
            }

            if (Parent::_result_status == ResultStatus::OPTIMAL || Parent::_result_status == ResultStatus::FEASIBLE) {
              // Get the results
              Parent::_objective_value = model.getObjValue();
              VLOG_F(1, "objective={}", Parent::_objective_value);
              const double* const values = model.bestSolution();
              if (values != nullptr) {
                // if optimal or feasible solution is found.
                for (int i = 0; i < Parent::_variables.size(); ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  const int         var_index = MPSolverVarIndexToCbcVarIndex(var->index());
                  const double      val = values[var_index];
                  var->set_solution_value(val);
                  VLOG_F(3, "{}={}", var->name(), val);
                }
              } else {
                VLOG_F(1, "No feasible solution found.");
              }
            }

            iterations_ = model.getIterationCount();
            nodes_ = model.getNodeCount();
            Parent::_best_objective_bound = model.getBestPossibleObjValue();
            VLOG_F(1, "best objective bound={}", Parent::_best_objective_bound);

            Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;

            return Parent::_result_status;
          }

          void _impl_Reset() noexcept {
            osi_.reset();
            osi_.setObjSense(Parent::_maximize ? -1 : 1);
            osi_.setStrParam(OsiProbName, Parent::_name);
            Parent::resetExtractionInformation();
          }

          void _impl_SetVariableBounds(int var_index, double lb, double ub) noexcept {
            invalidateSolutionSynchronization();
            if (_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              osi_.setColBounds(MPSolverVarIndexToCbcVarIndex(var_index), lb, ub);
            } else {
              _sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetVariableInteger(int var_index, bool integer) noexcept {
            invalidateSolutionSynchronization();
            // TODO(user) : Check if this is actually a change.
            if (_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              if (integer) {
                osi_.setInteger(MPSolverVarIndexToCbcVarIndex(var_index));
              } else {
                osi_.setContinuous(MPSolverVarIndexToCbcVarIndex(var_index));
              }
            } else {
              _sync_status = SynchronizationStatus::MUST_RELOAD;
            }
          }
          void _impl_SetConstraintBounds(int row_index, double lb, double ub) noexcept {
            invalidateSolutionSynchronization();
            if (_sync_status == SynchronizationStatus::MODEL_SYNCHRONIZED) {
              osi_.setRowBounds(row_index, lb, ub);
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
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_ClearConstraint(MPConstraint* constraint) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetObjectiveCoefficient(const MPVariable* variable, double coefficient) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_SetObjectiveOffset(double value) noexcept {
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }
          void _impl_ClearObjective() noexcept { Parent::_sync_status = SynchronizationStatus::MUST_RELOAD; }

          int64_t _impl_iterations() const noexcept {
            if (!checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfIterations;
            return iterations_;
          }
          int64_t _impl_nodes() const noexcept {
            if (!checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfNodes;
            return nodes_;
          }
          BasisStatus _impl_row_status(int constraint_index) const noexcept {
            LOG_F(FATAL, "Basis status only available for continuous problems");
            return BasisStatus::FREE;
          }
          BasisStatus _impl_column_status(int variable_index) const noexcept {
            LOG_F(FATAL, "Basis status only available for continuous problems");
            return BasisStatus::FREE;
          }

          bool _impl_HasDualFarkas() const noexcept { return false; }

          bool _impl_IsContinuous() const noexcept { return _impl_IsLP(); }
          bool _impl_IsLP() const noexcept { return false; }
          bool _impl_IsMIP() const noexcept { return true; }

          void _impl_ExtractNewVariables() noexcept {}
          void _impl_ExtractNewConstraints() noexcept {}
          void _impl_ExtractObjective() noexcept {}

          std::string _impl_SolverVersion() const noexcept { return "Cbc " CBC_VERSION; }

          // TODO(user): Maybe we should expose the CbcModel build from osi_
          // instead, but a new CbcModel is built every time Solve is called,
          // so it is not possible right now.
          void* _impl_underlying_solver() noexcept { return reinterpret_cast< void* >(&osi_); }

          // See MPSolver::SetCallback() for details.
          void _impl_SetCallback(MPCallback* mp_callback) noexcept {}
          bool _impl_SupportsCallbacks() const noexcept { return true; }

          // private:
          void _impl_SetParameters(const MPSolverParameters& param) noexcept {
            Parent::setCommonParameters(param);
            Parent::setMIPParameters(param);
          }
          void _impl_SetRelativeMipGap(double value) noexcept { relative_mip_gap_ = value; }
          void _impl_SetPrimalTolerance(double value) noexcept {
            // Skip the warning for the default value as it coincides with
            // the default value in CBC.
            if (value != MPSolverParameters::kDefaultPrimalTolerance) {
              Parent::setUnsupportedDoubleParam(MPSolverParameters::PRIMAL_TOLERANCE);
            }
          }
          void _impl_SetDualTolerance(double value) noexcept {
            // Skip the warning for the default value as it coincides with
            // the default value in CBC.
            if (value != MPSolverParameters::kDefaultDualTolerance) {
              Parent::setUnsupportedDoubleParam(MPSolverParameters::DUAL_TOLERANCE);
            }
          }
          void _impl_SetPresolveMode(int value) noexcept {
            switch (value) {
              case MPSolverParameters::PRESOLVE_ON: {
                // CBC presolve is always on.
                break;
              }
              default: {
                Parent::setUnsupportedIntegerParam(MPSolverParameters::PRESOLVE);
              }
            }
          }
          void _impl_SetScalingMode(int scaling) noexcept {
            Parent::setUnsupportedIntegerParam(MPSolverParameters::SCALING);
          }
          void _impl_SetLpAlgorithm(int lp_algorithm) noexcept {
            Parent::setUnsupportedIntegerParam(MPSolverParameters::LP_ALGORITHM);
          }

          bool _impl_SetNumThreads(int num_threads) noexcept {
            CHECK_GE_F(num_threads, 1);
            num_threads_ = num_threads;
            return true;
          }
          bool _impl_SetSolverSpecificParametersAsString(const std::string& parameters) noexcept {
            LOG_F(ERROR, "SetSolverSpecificParametersAsString() not currently supported for CBC.");
            return false;
          }

          void ResetBestObjectiveBound() noexcept {
            if (Parent::_maximize) {
              Parent::_best_objective_bound = NT_DOUBLE_INFINITY;
            } else {
              Parent::_best_objective_bound = -NT_DOUBLE_INFINITY;
            }
          }

          OsiClpSolverInterface osi_;
          // TODO(user): remove and query number of iterations directly from CbcModel
          int64_t iterations_;
          int64_t nodes_;
          // Special way to handle the relative MIP gap parameter.
          double relative_mip_gap_;
          int    num_threads_ = 1;
        };


      }   // namespace details

      using MIPModel = details::CBCSolver;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return false; }

    }   // namespace cbc

  }   // namespace mp
}   // namespace nt


#endif