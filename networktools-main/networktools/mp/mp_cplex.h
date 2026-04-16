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
 * @file mp_cplex.h
 * @brief CPLEX solver integration for networktools MP module
 *
 * This header enables IBM ILOG CPLEX support for LP and MIP problems.
 * Include this file after networktools.h to use CPLEX solver.
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 *
 * @par Usage:
 * @code
 * #include "networktools.h"
 * #include "mp/mp_cplex.h"
 *
 * // Use type aliases
 * nt::mp::cplex::MIPModel model;
 * // or
 * nt::mp::cplex::LPModel lpModel;
 * @endcode
 *
 * @par Features:
 * - Full LP and MIP support with advanced CPLEX features
 * - Incremental extraction for faster repeated solves
 * - Cut callbacks for branch-and-cut algorithms
 * - Warm start and basis management
 * - Farkas dual values for infeasibility analysis
 *
 * @par Requirements:
 * - CPLEX 12.8 or newer
 * - Link with CPLEX library (-lcplex)
 * - Include CPLEX headers in compiler path
 *
 * @see mp.h for API documentation
 * @see README.md for compilation examples
 */

// This file contains a modified version of the source code from Google OR-Tools linear programming module.
// See the appropriate copyright notice below.

// Copyright 2014 IBM Corporation
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

// Initial version of this code was written by Daniel Junglas (IBM)

#ifndef _NT_CPLEX_INTERFACE_H_
#define _NT_CPLEX_INTERFACE_H_

#include "bits/mp_solver_base.h"

extern "C" {
#include <ilcplex/cplexx.h>
// This is an undocumented function, setting the objective offset
// is not supported everywhere (for example it may not be exported if a
// model is written to a file), but it works in the cases we need here.
CPXLIBAPI int CPXPUBLIC CPXEsetobjoffset(CPXCENVptr, CPXLPptr, double);
}

// In case we need to return a double but don't have a value for that
// we just return a NaN.
#if !defined(CPX_NAN)
#  define CPX_NAN NT_DOUBLE_NAN
#endif

// The argument to this macro is the invocation of a CPXX function that
// returns a status. If the function returns non-zero the macro aborts
// the program with an appropriate error message.
#define CHECK_STATUS(s)     \
  do {                      \
    int const status_ = s;  \
    CHECK_EQ_F(0, status_); \
  } while (0)

namespace nt {
  namespace mp {
    namespace cplex {

      struct MPEnv {
        CPXENVptr _env;

        MPEnv() noexcept {
          int status;
          _env = CPXXopenCPLEX(&status);
          if (status) {
            LOG_F(ERROR, "Could not create Cplex environment");
            _env = nullptr;
          }
          DCHECK_F(_env != nullptr);   // should not be NULL if status=0
        }

        ~MPEnv() noexcept { CPXXcloseCPLEX(&_env); }

        constexpr bool isValid() const noexcept { return _env != nullptr; }
      };

      namespace details {

        using std::unique_ptr;

        // For a model that is extracted to an instance of this class there is a
        // 1:1 corresponence between MPVariable instances and CPLEX columns: the
        // index of an extracted variable is the column index in the CPLEX model.
        // Similiar for instances of MPConstraint: the index of the constraint in
        // the model is the row index in the CPLEX model.
        template < bool MIP >
        struct CplexInterface: public nt::mp::details::MPSolver< CplexInterface< MIP > > {
          using Parent = nt::mp::details::MPSolver< CplexInterface< MIP > >;
          using MPObjective = Parent::MPObjective;
          using MPConstraint = Parent::MPConstraint;
          using MPVariable = Parent::MPVariable;
          // using MPCallback = Parent::MPCallback;
          using LinearRange = Parent::LinearRange;

          using Parent::Parent;

          // public:
          struct CplexMPCallbackContext: Parent::template MPCallbackContextInterface< CplexMPCallbackContext > {
            CPXCENVptr CpxEnv;
            void*      pCbData;
            int        iWhereFrom;
            int*       pUserAction;
            bool       bConstraintsAdded;

            nt::DoubleDynamicArray Values;
            nt::IntDynamicArray    CutInd;
            nt::DoubleDynamicArray CutVal;

            CplexMPCallbackContext(CPXCENVptr CpxEnv, void* pCbData, int iWhereFrom, int* pUserAction) :
                CpxEnv(CpxEnv), pCbData(pCbData), iWhereFrom(iWhereFrom), pUserAction(pUserAction),
                bConstraintsAdded(false) {}

            MPCallbackEvent event() {
              switch (iWhereFrom) {
                // CPLEX has found an integer feasible solution, and the user has the
                // option to generate a lazy constraint not satisfied by that solution.
                case CPX_CALLBACK_MIP_CUT_FEAS: return MPCallbackEvent::kMipSolution;

                // CPLEX has found that the LP relaxation at the current node is primal
                // unbounded, and the user can generate a lazy constraint to cut off the
                // primal ray.
                case CPX_CALLBACK_MIP_CUT_UNBD: return MPCallbackEvent::kUnknown;
              }

              return MPCallbackEvent::kUnknown;
            }

            bool canQueryVariableValues() {
              switch (iWhereFrom) {
                case CPX_CALLBACK_MIP:
                case CPX_CALLBACK_MIP_BRANCH:
                case CPX_CALLBACK_MIP_INCUMBENT_NODESOLN:
                case CPX_CALLBACK_MIP_INCUMBENT_HEURSOLN:
                case CPX_CALLBACK_MIP_INCUMBENT_USERSOLN:
                case CPX_CALLBACK_MIP_NODE:
                case CPX_CALLBACK_MIP_SOLVE:
                case CPX_CALLBACK_MIP_HEURISTIC:
                case CPX_CALLBACK_MIP_CUT_FEAS:
                case CPX_CALLBACK_MIP_CUT_UNBD:
                case CPX_CALLBACK_MIP_CUT_LOOP:
                case CPX_CALLBACK_MIP_CUT_LAST: return true;
              }
              return false;
            }

            double variableValue(const MPVariable* pVariable) {
              CHECK_F(canQueryVariableValues());
              if (!pVariable) return CPX_NAN;

              if (!Values._i_count) {
                CPXCLPptr Lp;
                if (CPXXgetcallbacklp(CpxEnv, pCbData, iWhereFrom, &Lp)) return CPX_NAN;
                const int iNumCols = CPXXgetnumcols(CpxEnv, Lp);
                if (iNumCols <= 0) return CPX_NAN;
                Values.ensureAndFillBytes(iNumCols, 0);
                if (CPXXgetcallbacknodex(CpxEnv, pCbData, iWhereFrom, Values._p_data, 0, Values._i_count - 1)) {
                  Values.removeAll();
                  return CPX_NAN;
                }
              }

              double fValue = CPX_NAN;
              if (pVariable->index() >= 0 || pVariable->index() < Values._i_count) fValue = Values[pVariable->index()];

              return fValue;
            }

            void addCut(const LinearRange& cutting_plane) { LOG_F(FATAL, "addCut() not yet implemented for CPLEX."); }

            void addPricedVariable(const MPVariable* pVariable) {
              LOG_F(FATAL, "addPricedVariable() not yet implemented for CPLEX.");
            }

            void addLazyConstraint(const LinearRange& lazy_constraint) {
              const nt::PointerMap< const MPVariable*, double >& Terms = lazy_constraint.linear_expr().terms();
              const int                                          iNumTerms = Terms.size();
              CutInd.ensureAndFillBytes(iNumTerms, 0);
              CutVal.ensureAndFillBytes(iNumTerms, 0);
              int k = 0;
              for (const auto& e: Terms) {
                CutInd[k] = e.first->index();
                CutVal[k] = e.second;
                ++k;
              }
              if (lazy_constraint.upper_bound() < infinity()) {
                const int iStatus = CPXXcutcallbackadd(CpxEnv,
                                                       pCbData,
                                                       iWhereFrom,
                                                       iNumTerms,
                                                       lazy_constraint.upper_bound(),
                                                       'L',
                                                       CutInd._p_data,
                                                       CutVal._p_data,
                                                       0);
                if (iStatus) VLOG_F(1, "Failed to call CPXXcutcallbackadd (upper_bound). Error {}", iStatus);
              }
              if (lazy_constraint.lower_bound() > -infinity()) {
                const int iStatus = CPXXcutcallbackadd(CpxEnv,
                                                       pCbData,
                                                       iWhereFrom,
                                                       iNumTerms,
                                                       lazy_constraint.lower_bound(),
                                                       'G',
                                                       CutInd._p_data,
                                                       CutVal._p_data,
                                                       0);
                if (iStatus) VLOG_F(1, "Failed to call CPXXcutcallbackadd (lower_bound). Error {}", iStatus);
              }
              bConstraintsAdded = true;
            }

            double suggestSolution(const nt::PointerMap< const MPVariable*, double >& solution) {
              LOG_F(FATAL, "suggestSolution() not currently supported for CPLEX.");
              return CPX_NAN;
            }

            int64_t numExploredNodes() {
              CPXLONG   iNodeCount;
              const int iStatus =
                 CPXXgetcallbackinfo(CpxEnv, pCbData, iWhereFrom, CPX_CALLBACK_INFO_NODE_COUNT_LONG, &iNodeCount);
              if (iStatus) iNodeCount = -1;
              return iNodeCount;
            }
          };

          using MPCallbackContext = CplexMPCallbackContext;
          using MPCallback = Parent::template MPCallback< MPCallbackContext >;
          using MPCallbackList = Parent::template MPCallbackList< MPCallbackContext >;

          // NOTE: 'mip' specifies the type of the problem (either continuous or
          //       mixed integer. This type is fixed for the lifetime of the
          //       instance. There are no dynamic changes to the model type.
          // Creates a LP/MIP instance.
          CplexInterface(const std::string& name, const MPEnv& e) :
              Parent(name), mEnv(0), mLp(0),
              slowUpdates(static_cast< SlowUpdates >(SlowSetObjectiveCoefficient | SlowClearObjective)),
              supportIncrementalExtraction(false), mCstat(), mRstat() {
            mEnv = e._env;
            if (!mEnv) LOG_F(ERROR, "Cplex environment not initialized.");

            int status;
            mLp = CPXXcreateprob(mEnv, &status, name.c_str());
            CHECK_STATUS(status);
            DCHECK_F(mLp != nullptr);   // should not be NULL if status=0

            CHECK_STATUS(CPXXchgobjsen(mEnv, mLp, Parent::_maximize ? CPX_MAX : CPX_MIN));
            if constexpr (_impl_IsMIP()) CHECK_STATUS(CPXXchgprobtype(mEnv, mLp, CPXPROB_MILP));
          }

          ~CplexInterface() {
            CHECK_STATUS(CPXXfreeprob(mEnv, &mLp));
            // CHECK_STATUS(CPXXcloseCPLEX(&mEnv));
          }

          // Sets the optimization direction (min/max).
          void _impl_SetOptimizationDirection(bool maximize) {
            Parent::invalidateSolutionSynchronization();
            CPXXchgobjsen(mEnv, mLp, maximize ? CPX_MAX : CPX_MIN);
          }

          // ----- Solve -----
          // Solve the problem using the parameter values specified.
          ResultStatus _impl_Solve(MPSolverParameters const& param) {
            int status;

            // Delete cached information
            mCstat = 0;
            mRstat = 0;
            has_dual_farkas_ = false;

            nt::WallTimer timer;
            timer.start();

            // Set incrementality
            MPSolverParameters::IncrementalityValues const inc =
               static_cast< MPSolverParameters::IncrementalityValues >(
                  param.getIntegerParam(MPSolverParameters::INCREMENTALITY));
            switch (inc) {
              case MPSolverParameters::INCREMENTALITY_OFF:
                _impl_Reset(); /* This should not be required but re-extracting everything
                                * may be faster, so we do it. */
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_ADVIND, 0));
                break;
              case MPSolverParameters::INCREMENTALITY_ON:
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_ADVIND, 2));
                break;
            }

            // Set MIP Strategy
            MPSolverParameters::MipStrategyValues const mipstrat = static_cast< MPSolverParameters::MipStrategyValues >(
               param.getIntegerParam(MPSolverParameters::MIP_STRATEGY));
            switch (mipstrat) {
              case MPSolverParameters::OPTIMALITY:
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_MIPEMPHASIS, CPX_MIPEMPHASIS_BALANCED));
                CHECK_STATUS(CPXXsetlongparam(mEnv, CPX_PARAM_INTSOLLIM, 9223372036800000000));
                break;
              case MPSolverParameters::FEASABILITY:
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_MIPEMPHASIS, CPX_MIPEMPHASIS_FEASIBILITY));
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_INTSOLLIM, 1));
                break;
            }

            // Extract the model to be solved.
            // If we don't support incremental extraction and the low-level modeling
            // is out of sync then we have to re-extract everything. Note that this
            // will lose MIP starts or advanced basis information from a previous
            // solve.
            if (!supportIncrementalExtraction && Parent::_sync_status == SynchronizationStatus::MUST_RELOAD)
              _impl_Reset();
            Parent::extractModel();
            VLOG_F(1, "Model built in {} seconds.", timer.get());

            // Set log level.
            CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_SCRIND, Parent::quiet() ? CPX_OFF : CPX_ON));

            // Set parameters.
            // NOTE: We must invoke SetSolverSpecificParametersAsString() _first_.
            //       Its current implementation invokes ReadParameterFile() which in
            //       turn invokes CPXXreadcopyparam(). The latter will _overwrite_
            //       all current parameter settings in the environment.
            // solver_->SetSolverSpecificParametersAsString(solver_->solver_specific_parameter_string_);
            _impl_SetParameters(param);
            if (Parent::time_limit()) {
              VLOG_F(1, "Setting time limit = {} ms.", Parent::time_limit());
              CHECK_STATUS(CPXXsetdblparam(mEnv, CPX_PARAM_TILIM, Parent::time_limit() * 1e-3));
            }

            // Add callback
            if (callback_) {
              CPXXsetintparam(mEnv, CPX_PARAM_MIPCBREDLP, CPX_OFF);
              if (callback_->might_add_lazy_constraints())
                CPXXsetlazyconstraintcallbackfunc(mEnv, CplexLazyCutCallback, callback_);
            }

            // Warm start if any
            if (_impl_IsMIP() && !Parent::_solution_hint.empty()) {
              hint_ind_.removeAll();
              hint_val_.removeAll();
              bool      is_solution_partial = false;
              const int num_vars = Parent::_variables.size();
              if (Parent::_solution_hint.size() != num_vars) {
                // We start by creating an empty partial solution.
                is_solution_partial = true;
              } else {
                // We start by creating the all-zero solution.
                hint_ind_.ensureAndFillBytes(Parent::_solution_hint.size(), 0);
                hint_val_.ensureAndFillBytes(Parent::_solution_hint.size(), 0);

                int k = 0;
                for (const auto& var_value_pair: Parent::_solution_hint) {
                  hint_ind_[k] = var_value_pair.first->index();
                  hint_val_[k] = var_value_pair.second;
                  ++k;
                }

                CPXNNZ beg[] = {0};
                CHECK_STATUS(CPXXaddmipstarts(mEnv, mLp, 1, num_vars, beg, hint_ind_._p_data, hint_val_._p_data, 0, 0));
              }
            }

            // Set variables priority
            if constexpr (_impl_IsMIP()) {
              if (branching_priority_reset_) {
                const CPXDIM           cols = CPXXgetnumcols(mEnv, mLp);
                unique_ptr< CPXDIM[] > priority(new CPXDIM[cols]);
                unique_ptr< CPXDIM[] > indices(new CPXDIM[cols]);
                DCHECK_EQ_F(cols, Parent::_variables.size());
                for (int j = 0; j < cols; ++j) {
                  MPVariable const* const var = Parent::_variables[j];
                  priority[j] = var->branching_priority();
                  indices[j] = j;
                }
                CHECK_STATUS(CPXXcopyorder(mEnv, mLp, cols, indices.get(), priority.get(), 0));
                branching_priority_reset_ = false;
              }
            }

            if (Parent::time_limit()) {
              VLOG_F(1, "Setting time limit = {} ms", Parent::time_limit());
              CHECK_STATUS(CPXXsetdblparam(mEnv, CPX_PARAM_TILIM, Parent::time_limit() * 0.001));
            }

            // Solve.
            // Do not CHECK_STATUS here since some errors (for example CPXERR_NO_MEMORY)
            // still allow us to query useful information.
            timer.restart();
            if constexpr (_impl_IsMIP()) {
              status = CPXXmipopt(mEnv, mLp);
            } else {
              status = CPXXlpopt(mEnv, mLp);
            }

            // Disable screen output right after solve
            (void)CPXXsetintparam(mEnv, CPX_PARAM_SCRIND, CPX_OFF);

            if (status) {
              VLOG_F(1, "Failed to optimize MIP. Error {}", status);
              // NOTE: We do not return immediately since there may be information
              //       to grab (for example an incumbent)
            } else {
              VLOG_F(1, "Solved in {} seconds.", timer.get());
            }

            int const cpxstat = CPXXgetstat(mEnv, mLp);
            VLOG_F(1, "CPLEX solution status {}.", cpxstat);

            // Figure out what solution we have.
            int solnmethod, solntype, pfeas, dfeas;
            CHECK_STATUS(CPXXsolninfo(mEnv, mLp, &solnmethod, &solntype, &pfeas, &dfeas));
            bool const feasible = pfeas != 0;

            // Get problem dimensions for solution queries below.
            CPXDIM const rows = CPXXgetnumrows(mEnv, mLp);
            CPXDIM const cols = CPXXgetnumcols(mEnv, mLp);
            DCHECK_EQ_F(rows, Parent::_constraints.size());
            DCHECK_EQ_F(cols, Parent::_variables.size());

            // Capture objective function value.
            Parent::_objective_value = CPX_NAN;
            Parent::_best_objective_bound = CPX_NAN;
            if (feasible) {
              CHECK_STATUS(CPXXgetobjval(mEnv, mLp, &(Parent::_objective_value)));
              if constexpr (_impl_IsMIP()) {
                CHECK_STATUS(CPXXgetbestobjval(mEnv, mLp, &(Parent::_best_objective_bound)));
              }
            }
            VLOG_F(1, "objective={}, bound={}", Parent::_objective_value, Parent::_best_objective_bound);

            // Capture primal and dual solutions
            if constexpr (_impl_IsMIP()) {
              // If there is a primal feasible solution then capture it.
              if (feasible) {
                if (cols > 0) {
                  unique_ptr< double[] > x(new double[cols]);
                  CHECK_STATUS(CPXXgetx(mEnv, mLp, x.get(), 0, cols - 1));
                  for (int i = 0; i < Parent::_variables.size(); ++i) {
                    MPVariable* const var = Parent::_variables[i];
                    var->set_solution_value(x[i]);
                    VLOG_F(3, "{}: value ={}", var->name(), x[i]);
                  }
                }
              } else {
                for (int i = 0; i < Parent::_variables.size(); ++i)
                  Parent::_variables[i]->set_solution_value(CPX_NAN);
              }

              // MIP does not have duals
              for (int i = 0; i < Parent::_variables.size(); ++i)
                Parent::_variables[i]->set_reduced_cost(CPX_NAN);
              for (int i = 0; i < Parent::_constraints.size(); ++i) {
                Parent::_constraints[i]->set_dual_value(CPX_NAN);
                Parent::_constraints[i]->set_dual_farkas_value(CPX_NAN);
              }
            } else {
              // Continuous problem.
              if (cols > 0) {
                unique_ptr< double[] > x(new double[cols]);
                unique_ptr< double[] > dj(new double[cols]);
                if (feasible) CHECK_STATUS(CPXXgetx(mEnv, mLp, x.get(), 0, cols - 1));
                if (dfeas) CHECK_STATUS(CPXXgetdj(mEnv, mLp, dj.get(), 0, cols - 1));
                for (int i = 0; i < Parent::_variables.size(); ++i) {
                  MPVariable* const var = Parent::_variables[i];
                  var->set_solution_value(x[i]);
                  bool value = false, dual = false;

                  if (feasible) {
                    var->set_solution_value(x[i]);
                    value = true;
                  } else
                    var->set_solution_value(CPX_NAN);
                  if (dfeas) {
                    var->set_reduced_cost(dj[i]);
                    dual = true;
                  } else
                    var->set_reduced_cost(CPX_NAN);
                  VLOG_F(3,
                         "{}: {} {}",
                         var->name(),
                         (value ? nt::format("  value = {}", x[i]) : ""),
                         (dual ? nt::format("  reduced cost = {}", dj[i]) : ""));
                }
              }

              if (rows > 0) {
                unique_ptr< double[] > pi(new double[rows]);
                unique_ptr< double[] > dualfarkas(new double[rows]);
                if (dfeas) CHECK_STATUS(CPXXgetpi(mEnv, mLp, pi.get(), 0, rows - 1));
                if (!feasible) {
                  // Attempt to retrieve dual farkas values
                  if (CPXXdualfarkas(mEnv, mLp, dualfarkas.get(), nullptr) == CPXERR_NOT_DUAL_UNBOUNDED)
                    VLOG_F(0,
                           "Cannot retrieve dual farkas values : use dual simplex and desactivate "
                           "presolve to get dual farkas values");
                  else
                    has_dual_farkas_ = true;
                }
                for (int i = 0; i < Parent::_constraints.size(); ++i) {
                  MPConstraint* const ct = Parent::_constraints[i];
                  bool                dual = false;
                  if (dfeas) {
                    ct->set_dual_value(pi[i]);
                    dual = true;
                  } else {
                    ct->set_dual_value(CPX_NAN);
                  }
                  if (has_dual_farkas_) ct->set_dual_farkas_value(dualfarkas[i]);
                  VLOG_F(4,
                         "row {}: {}",
                         ct->index(),
                         (dual ? nt::format("  dual = {}", pi[i])
                               : (has_dual_farkas_ ? nt::format("  dual farkas = {}", dualfarkas[i]) : "")));
                }
              }
            }

            // Map CPLEX status to more generic solution status in MPSolver
            switch (cpxstat) {
              case CPX_STAT_OPTIMAL:
              case CPXMIP_OPTIMAL: Parent::_result_status = ResultStatus::OPTIMAL; break;
              case CPXMIP_OPTIMAL_TOL:
                // To be consistent with the other solvers.
                Parent::_result_status = ResultStatus::OPTIMAL;
                break;
              case CPX_STAT_INFEASIBLE:
              case CPXMIP_INFEASIBLE: Parent::_result_status = ResultStatus::INFEASIBLE; break;
              case CPX_STAT_UNBOUNDED:
              case CPXMIP_UNBOUNDED: Parent::_result_status = ResultStatus::UNBOUNDED; break;
              case CPX_STAT_INForUNBD:
              case CPXMIP_INForUNBD: Parent::_result_status = ResultStatus::INFEASIBLE; break;
              default: Parent::_result_status = feasible ? ResultStatus::FEASIBLE : ResultStatus::ABNORMAL; break;
            }

            Parent::_sync_status = SynchronizationStatus::SOLUTION_SYNCHRONIZED;
            return Parent::_result_status;
          }

          // ----- Model modifications and extraction -----
          // Resets extracted model
          void _impl_Reset() {
            // Instead of explicitly clearing all modeling objects we
            // just delete the problem object and allocate a new one.
            CHECK_STATUS(CPXXfreeprob(mEnv, &mLp));

            int               status;
            const char* const name = Parent::_name.c_str();
            mLp = CPXXcreateprob(mEnv, &status, name);
            CHECK_STATUS(status);
            DCHECK_F(mLp != nullptr);   // should not be NULL if status=0

            CHECK_STATUS(CPXXchgobjsen(mEnv, mLp, Parent::_maximize ? CPX_MAX : CPX_MIN));
            if constexpr (_impl_IsMIP()) CHECK_STATUS(CPXXchgprobtype(mEnv, mLp, CPXPROB_MILP));

            Parent::resetExtractionInformation();
            mCstat = 0;
            mRstat = 0;
            has_dual_farkas_ = false;
          }

          void _impl_SetVariableBounds(int var_index, double lb, double ub) {
            Parent::invalidateSolutionSynchronization();

            // Changing the bounds of a variable is fast. However, doing this for
            // many variables may still be slow. So we don't perform the update by
            // default. However, if we support incremental extraction
            // (supportIncrementalExtraction is true) then we MUST perform the
            // update here or we will lose it.

            if (!supportIncrementalExtraction && !(slowUpdates & SlowSetVariableBounds)) {
              _impl_InvalidateModelSynchronization();
            } else {
              if (Parent::variable_is_extracted(var_index)) {
                // Variable has already been extracted, so we must modify the
                // modeling object.
                DCHECK_LT_F(var_index, Parent::_last_variable_index);
                char const   lu[2] = {'L', 'U'};
                double const bd[2] = {lb, ub};
                CPXDIM const idx[2] = {var_index, var_index};
                CHECK_STATUS(CPXXchgbds(mEnv, mLp, 2, idx, lu, bd));
              } else {
                // Variable is not yet extracted. It is sufficient to just mark
                // the modeling object "out of sync"
                _impl_InvalidateModelSynchronization();
              }
            }
          }

          // Modifies integrality of an extracted variable.
          void _impl_SetVariableInteger(int var_index, bool integer) {
            Parent::invalidateSolutionSynchronization();

            // NOTE: The type of the model (continuous or mixed integer) is
            //       defined once and for all in the constructor. There are no
            //       dynamic changes to the model type.

            // Changing the type of a variable should be fast. Still, doing all
            // updates in one big chunk right before solve() is usually faster.
            // However, if we support incremental extraction
            // (supportIncrementalExtraction is true) then we MUST change the
            // type of extracted variables here.

            if (!supportIncrementalExtraction && !(slowUpdates && SlowSetVariableInteger)) {
              _impl_InvalidateModelSynchronization();
            } else {
              if constexpr (_impl_IsMIP()) {
                if (Parent::variable_is_extracted(var_index)) {
                  // Variable is extracted. Change the type immediately.
                  // TODO: Should we check the current type and don't do anything
                  //       in case the type does not change?
                  DCHECK_LE_F(var_index, CPXXgetnumcols(mEnv, mLp));
                  char const type = integer ? CPX_INTEGER : CPX_CONTINUOUS;
                  CHECK_STATUS(CPXXchgctype(mEnv, mLp, 1, &var_index, &type));
                } else
                  _impl_InvalidateModelSynchronization();
              } else {
                LOG_F(FATAL, "Attempt to change variable to integer in non-MIP problem!");
              }
            }
          }

          void _impl_SetConstraintBounds(int row_index, double lb, double ub) {
            Parent::invalidateSolutionSynchronization();

            // Changing rhs, sense, or range of a constraint is not too slow.
            // Still, doing all the updates in one large operation is faster.
            // Note however that if we do not want to re-extract the full model
            // for each solve (supportIncrementalExtraction is true) then we MUST
            // update the constraint here, otherwise we lose this update information.

            if (!supportIncrementalExtraction && !(slowUpdates & SlowSetConstraintBounds)) {
              _impl_InvalidateModelSynchronization();
            } else {
              if (Parent::constraint_is_extracted(row_index)) {
                // Constraint is already extracted, so we must update its bounds
                // and its type.
                DCHECK_F(mLp != NULL);
                char   sense;
                double range, rhs;
                MakeRhs(lb, ub, rhs, sense, range);
                CHECK_STATUS(CPXXchgrhs(mEnv, mLp, 1, &row_index, &lb));
                CHECK_STATUS(CPXXchgsense(mEnv, mLp, 1, &row_index, &sense));
                CHECK_STATUS(CPXXchgrngval(mEnv, mLp, 1, &row_index, &range));
              } else {
                // Constraint is not yet extracted. It is sufficient to mark the
                // modeling object as "out of sync"
                _impl_InvalidateModelSynchronization();
              }
            }
          }

          void _impl_AddRowConstraint(MPConstraint* const ct) {
            // This is currently only invoked when a new constraint is created,
            // see MPSolver::makeRowConstraint().
            // At this point we only have the lower and upper bounds of the
            // constraint. We could immediately call CPXXaddrows() here but it is
            // usually much faster to handle the fully populated constraint in
            // extractNewConstraints() right before the solve.
            _impl_InvalidateModelSynchronization();
          }

          void _impl_AddVariable(MPVariable* const var) {
            // This is currently only invoked when a new variable is created,
            // see MPSolver::makeVar().
            // At this point the variable does not appear in any constraints or
            // the objective function. We could invoke CPXXaddcols() to immediately
            // create the variable here but it is usually much faster to handle the
            // fully setup variable in extractNewVariables() right before the solve.
            _impl_InvalidateModelSynchronization();
          }

          void _impl_BranchingPriorityChangedForVariable(int var_index) {
            // We force reset the model when setting the priority on an already extracted variable.
            // Note that this is a more drastic step than merely changing the sync_status.
            // This may be slightly conservative, as it is technically possible that
            // the extraction has occurred without a call to Solve().
            if (Parent::variable_is_extracted(var_index)) { branching_priority_reset_ = true; }
          }

          void _impl_SetCoefficient(MPConstraint* const     constraint,
                                    MPVariable const* const variable,
                                    double                  new_value,
                                    double                  old_value) {
            Parent::invalidateSolutionSynchronization();

            // Changing a single coefficient in the matrix is potentially pretty
            // slow since that coefficient has to be found in the sparse matrix
            // representation. So by default we don't perform this update immediately
            // but instead mark the low-level modeling object "out of sync".
            // If we want to support incremental extraction then we MUST perform
            // the modification immediately or we will lose it.

            if (!supportIncrementalExtraction && !(slowUpdates & SlowSetCoefficient)) {
              _impl_InvalidateModelSynchronization();
            } else {
              int const row = constraint->index();
              int const col = variable->index();
              if (Parent::constraint_is_extracted(row) && Parent::variable_is_extracted(col)) {
                // If row and column are both extracted then we can directly
                // update the modeling object
                DCHECK_LE_F(row, Parent::_last_constraint_index);
                DCHECK_LE_F(col, Parent::_last_variable_index);
                CHECK_STATUS(CPXXchgcoef(mEnv, mLp, row, col, new_value));
              } else {
                // If either row or column is not yet extracted then we can
                // defer the update to extractModel()
                _impl_InvalidateModelSynchronization();
              }
            }
          }

          // Clear a constraint from all its terms.
          void _impl_ClearConstraint(MPConstraint* const constraint) {
            CPXDIM const row = constraint->index();
            if (!Parent::constraint_is_extracted(row))
              // There is nothing to do if the constraint was not even extracted.
              return;

            // Clearing a constraint means setting all coefficients in the corresponding
            // row to 0 (we cannot just delete the row since that would renumber all
            // the constraints/rows after it).
            // Modifying coefficients in the matrix is potentially pretty expensive
            // since they must be found in the sparse matrix representation. That is
            // why by default we do not modify the coefficients here but only mark
            // the low-level modeling object "out of sync".

            if (!(slowUpdates & SlowClearConstraint)) {
              _impl_InvalidateModelSynchronization();
            } else {
              Parent::invalidateSolutionSynchronization();

              CPXDIM const           len = constraint->_coefficients.size();
              unique_ptr< CPXDIM[] > rowind(new CPXDIM[len]);
              unique_ptr< CPXDIM[] > colind(new CPXDIM[len]);
              unique_ptr< double[] > val(new double[len]);
              CPXDIM                 j = 0;
              const auto&            coeffs = constraint->_coefficients;
              for (auto it(coeffs.begin()); it != coeffs.end(); ++it) {
                CPXDIM const col = it->first->index();
                if (Parent::variable_is_extracted(col)) {
                  rowind[j] = row;
                  colind[j] = col;
                  val[j] = 0.0;
                  ++j;
                }
              }
              if (j) CHECK_STATUS(CPXXchgcoeflist(mEnv, mLp, j, rowind.get(), colind.get(), val.get()));
            }
          }

          // Change a coefficient in the linear objective
          void _impl_SetObjectiveCoefficient(MPVariable const* const variable, double coefficient) {
            CPXDIM const col = variable->index();
            if (!Parent::variable_is_extracted(col))
              // Nothing to do if variable was not even extracted
              return;

            Parent::invalidateSolutionSynchronization();

            // The objective function is stored as a dense vector, so updating a
            // single coefficient is O(1). So by default we update the low-level
            // modeling object here.
            // If we support incremental extraction then we have no choice but to
            // perform the update immediately.

            if (supportIncrementalExtraction || (slowUpdates & SlowSetObjectiveCoefficient)) {
              CHECK_STATUS(CPXXchgobj(mEnv, mLp, 1, &col, &coefficient));
            } else
              _impl_InvalidateModelSynchronization();
          }

          // Change the constant term in the linear objective.
          void _impl_SetObjectiveOffset(double value) {
            // Changing the objective offset is O(1), so we always do it immediately.
            Parent::invalidateSolutionSynchronization();
            CHECK_STATUS(CPXEsetobjoffset(mEnv, mLp, value));
          }

          // Clear the objective from all its terms.
          void _impl_ClearObjective() {
            Parent::invalidateSolutionSynchronization();

            // Since the objective function is stored as a dense vector updating
            // it is O(n), so we usually perform the update immediately.
            // If we want to support incremental extraction then we have no choice
            // but to perform the update immediately.

            if (supportIncrementalExtraction || (slowUpdates & SlowClearObjective)) {
              CPXDIM const           cols = CPXXgetnumcols(mEnv, mLp);
              unique_ptr< CPXDIM[] > ind(new CPXDIM[cols]);
              unique_ptr< double[] > zero(new double[cols]);
              CPXDIM                 j = 0;
              const auto&            coeffs = Parent::_objective._coefficients;
              for (auto it(coeffs.begin()); it != coeffs.end(); ++it) {
                CPXDIM const idx = it->first->index();
                // We only need to reset variables that have been extracted.
                if (Parent::variable_is_extracted(idx)) {
                  DCHECK_LT_F(idx, cols);
                  ind[j] = idx;
                  zero[j] = 0.0;
                  ++j;
                }
              }
              if (j > 0) CHECK_STATUS(CPXXchgobj(mEnv, mLp, j, ind.get(), zero.get()));
              CHECK_STATUS(CPXEsetobjoffset(mEnv, mLp, 0.0));
            } else
              _impl_InvalidateModelSynchronization();
          }

          // Writes the model.
          void _impl_Write(const std::string& filename) {
            if (Parent::_sync_status == SynchronizationStatus::MUST_RELOAD) { _impl_Reset(); }
            Parent::extractModel();
            VLOG_F(1, "Writing CPLEX model file \"{}\".", filename.c_str());
            CHECK_STATUS(CPXXwriteprob(mEnv, mLp, filename.c_str(), NULL));
          }

          // ------ Query statistics on the solution and the solve ------

          // Number of simplex iterations
          int64_t _impl_iterations() const {
            int iter;
            if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfIterations;
            if constexpr (_impl_IsMIP())
              return static_cast< int64_t >(CPXXgetmipitcnt(mEnv, mLp));
            else
              return static_cast< int64_t >(CPXXgetitcnt(mEnv, mLp));
          }

          // Number of branch-and-bound nodes. Only available for discrete problems.
          int64_t _impl_nodes() const {
            if constexpr (_impl_IsMIP()) {
              if (!Parent::checkSolutionIsSynchronized()) return nt::mp::details::kUnknownNumberOfNodes;
              return static_cast< int64_t >(CPXXgetnodecnt(mEnv, mLp));
            } else {
              LOG_F(FATAL, "Number of nodes only available for discrete problems");
              return Parent::kUnknownNumberOfNodes;
            }
          }

          // Returns the basis status of a row.
          BasisStatus _impl_row_status(int constraint_index) const {
            if constexpr (_impl_IsMIP()) {
              LOG_F(FATAL, "Basis status only available for continuous problems");
              return BasisStatus::FREE;
            }

            if (Parent::checkSolutionIsSynchronized()) {
              if (!mRstat) {
                CPXDIM const        rows = CPXXgetnumrows(mEnv, mLp);
                unique_ptr< int[] > data(new int[rows]);
                mRstat.swap(data);
                CHECK_STATUS(CPXXgetbase(mEnv, mLp, 0, mRstat.get()));
              }
            } else
              mRstat = 0;

            if (mRstat)
              return xformBasisStatus(mRstat[constraint_index]);
            else {
              LOG_F(FATAL, "Row basis status not available");
              return BasisStatus::FREE;
            }
          }

          // Returns the basis status of a column.
          BasisStatus _impl_column_status(int variable_index) const {
            if constexpr (_impl_IsMIP()) {
              LOG_F(FATAL, "Basis status only available for continuous problems");
              return BasisStatus::FREE;
            }

            if (Parent::checkSolutionIsSynchronized()) {
              if (!mCstat) {
                CPXDIM const        cols = CPXXgetnumcols(mEnv, mLp);
                unique_ptr< int[] > data(new int[cols]);
                mCstat.swap(data);
                CHECK_STATUS(CPXXgetbase(mEnv, mLp, mCstat.get(), 0));
              }
            } else
              mCstat = 0;

            if (mCstat)
              return xformBasisStatus(mCstat[variable_index]);
            else {
              LOG_F(FATAL, "Column basis status not available");
              return BasisStatus::FREE;
            }
          }

          bool _impl_HasDualFarkas() const { return has_dual_farkas_; }

          // ----- Misc -----

          // Query problem type.
          // Remember that problem type is a static property that is set
          // in the template parameter and never changed.
          constexpr static bool _impl_IsContinuous() { return _impl_IsLP(); }
          constexpr static bool _impl_IsLP() { return !MIP; }
          constexpr static bool _impl_IsMIP() { return MIP; }

          // Extract all variables that have not yet been extracted.
          void _impl_ExtractNewVariables() {
            // NOTE: The code assumes that a linear expression can never contain
            //       non-zero duplicates.

            Parent::invalidateSolutionSynchronization();

            if (!supportIncrementalExtraction) {
              // Without incremental extraction extractModel() is always called
              // to extract the full model.
              CHECK_F(Parent::_last_variable_index == 0 || Parent::_last_variable_index == Parent::_variables.size());
              CHECK_F(Parent::_last_constraint_index == 0
                      || Parent::_last_constraint_index == Parent::_constraints.size());
            }

            int const last_extracted = Parent::_last_variable_index;
            int const var_count = Parent::_variables.size();
            CPXDIM    newcols = var_count - last_extracted;
            if (newcols > 0) {
              // There are non-extracted variables. Extract them now.

              unique_ptr< double[] >      obj(new double[newcols]);
              unique_ptr< double[] >      lb(new double[newcols]);
              unique_ptr< double[] >      ub(new double[newcols]);
              unique_ptr< char[] >        ctype(new char[newcols]);
              unique_ptr< const char*[] > colname(new const char*[newcols]);

              bool have_names = false;
              for (int j = 0, varidx = last_extracted; j < newcols; ++j, ++varidx) {
                MPVariable const* const var = Parent::_variables[varidx];
                lb[j] = var->lb();
                ub[j] = var->ub();
                ctype[j] = var->integer() ? CPX_INTEGER : CPX_CONTINUOUS;
                colname[j] = var->name().empty() ? 0 : var->name().c_str();
                have_names = have_names || var->name().empty();
                obj[j] = Parent::_objective.getCoefficient(var);
              }

              // Arrays for modifying the problem are setup. Update the index
              // of variables that will get extracted now. Updating indices
              // _before_ the actual extraction makes things much simpler in
              // case we support incremental extraction.
              // In case of error we just reset the indeces.
              nt::TrivialDynamicArray< MPVariable* > const& variables = Parent::variables();
              for (int j = last_extracted; j < var_count; ++j) {
                CHECK_F(!Parent::variable_is_extracted(variables[j]->index()));
                Parent::set_variable_as_extracted(variables[j]->index(), true);
              }

              try {
                bool use_newcols = true;

                if (supportIncrementalExtraction) {
                  // If we support incremental extraction then we must
                  // update existing constraints with the new variables.
                  // To do that we use CPXXaddcols() to actually create the
                  // variables. This is supposed to be faster than combining
                  // CPXXnewcols() and CPXXchgcoeflist().

                  // For each column count the size of the intersection with
                  // existing constraints.
                  unique_ptr< CPXDIM[] > collen(new CPXDIM[newcols]);
                  for (CPXDIM j = 0; j < newcols; ++j)
                    collen[j] = 0;
                  CPXNNZ nonzeros = 0;
                  // TODO: Use a bitarray to flag the constraints that actually
                  //       intersect new variables?
                  for (int i = 0; i < Parent::_last_constraint_index; ++i) {
                    MPConstraint const* const ct = Parent::_constraints[i];
                    CHECK_F(Parent::constraint_is_extracted(ct->index()));
                    const auto& coeffs = ct->_coefficients;
                    for (auto it(coeffs.begin()); it != coeffs.end(); ++it) {
                      int const idx = it->first->index();
                      if (Parent::variable_is_extracted(idx) && idx > Parent::_last_variable_index) {
                        collen[idx - Parent::_last_variable_index]++;
                        ++nonzeros;
                      }
                    }
                  }

                  if (nonzeros > 0) {
                    // At least one of the new variables did intersect with an
                    // old constraint. We have to create the new columns via
                    // CPXXaddcols().
                    use_newcols = false;
                    unique_ptr< CPXNNZ[] > begin(new CPXNNZ[newcols + 2]);
                    unique_ptr< CPXDIM[] > cmatind(new CPXDIM[nonzeros]);
                    unique_ptr< double[] > cmatval(new double[nonzeros]);

                    // Here is how cmatbeg[] is setup:
                    // - it is initialized as
                    //     [ 0, 0, collen[0], collen[0]+collen[1], ... ]
                    //   so that cmatbeg[j+1] tells us where in cmatind[] and
                    //   cmatval[] we need to put the next nonzero for column
                    //   j
                    // - after nonzeros have been setup the array looks like
                    //     [ 0, collen[0], collen[0]+collen[1], ... ]
                    //   so that it is the correct input argument for CPXXaddcols
                    CPXNNZ* cmatbeg = begin.get();
                    cmatbeg[0] = 0;
                    cmatbeg[1] = 0;
                    ++cmatbeg;
                    for (CPXDIM j = 0; j < newcols; ++j)
                      cmatbeg[j + 1] = cmatbeg[j] + collen[j];

                    for (int i = 0; i < Parent::_last_constraint_index; ++i) {
                      MPConstraint const* const ct = Parent::_constraints[i];
                      CPXDIM const              row = ct->index();
                      const auto&               coeffs = ct->_coefficients;
                      for (auto it(coeffs.begin()); it != coeffs.end(); ++it) {
                        int const idx = it->first->index();
                        if (Parent::variable_is_extracted(idx) && idx > Parent::_last_variable_index) {
                          CPXNNZ const nz = cmatbeg[idx]++;
                          cmatind[nz] = idx;
                          cmatval[nz] = it->second;
                        }
                      }
                    }
                    --cmatbeg;
                    CHECK_STATUS(CPXXaddcols(mEnv,
                                             mLp,
                                             newcols,
                                             nonzeros,
                                             obj.get(),
                                             cmatbeg,
                                             cmatind.get(),
                                             cmatval.get(),
                                             lb.get(),
                                             ub.get(),
                                             have_names ? colname.get() : 0));
                  }
                }
                if (use_newcols) {
                  // Either incremental extraction is not supported or none of
                  // the new variables did intersect an existing constraint.
                  // We can just use CPXXnewcols() to create the new variables.
                  CHECK_STATUS(CPXXnewcols(mEnv,
                                           mLp,
                                           newcols,
                                           obj.get(),
                                           lb.get(),
                                           ub.get(),
                                           _impl_IsMIP() ? ctype.get() : 0,
                                           have_names ? colname.get() : 0));
                } else {
                  // Incremental extraction: we must update the ctype of the
                  // newly created variables (CPXXaddcols() does not allow
                  // specifying the ctype)
                  if constexpr (_impl_IsMIP()) {
                    // Query the actual number of columns in case we did not
                    // manage to extract all columns.
                    int const              cols = CPXXgetnumcols(mEnv, mLp);
                    unique_ptr< CPXDIM[] > ind(new CPXDIM[newcols]);
                    for (int j = last_extracted; j < cols; ++j)
                      ind[j - last_extracted] = j;
                    CHECK_STATUS(CPXXchgctype(mEnv, mLp, cols - last_extracted, ind.get(), ctype.get()));
                  }
                }
              } catch (...) {
                // Undo all changes in case of error.
                CPXDIM const cols = CPXXgetnumcols(mEnv, mLp);
                if (cols > last_extracted) (void)CPXXdelcols(mEnv, mLp, last_extracted, cols - 1);
                nt::TrivialDynamicArray< MPVariable* > const& variables = Parent::variables();
                int const                                     size = variables.size();
                for (int j = last_extracted; j < size; ++j)
                  Parent::set_variable_as_extracted(j, false);
                throw;
              }
            }
          }

          // Extract constraints that have not yet been extracted.
          void _impl_ExtractNewConstraints() {
            // NOTE: The code assumes that a linear expression can never contain
            //       non-zero duplicates.

            if (!supportIncrementalExtraction) {
              // Without incremental extraction extractModel() is always called
              // to extract the full model.
              CHECK_F(Parent::_last_variable_index == 0 || Parent::_last_variable_index == Parent::_variables.size());
              CHECK_F(Parent::_last_constraint_index == 0
                      || Parent::_last_constraint_index == Parent::_constraints.size());
            }

            CPXDIM const offset = Parent::_last_constraint_index;
            CPXDIM const total = Parent::_constraints.size();

            if (total > offset) {
              // There are constraints that are not yet extracted.

              Parent::invalidateSolutionSynchronization();

              CPXDIM       newCons = total - offset;
              CPXDIM const cols = CPXXgetnumcols(mEnv, mLp);
              DCHECK_EQ_F(Parent::_last_variable_index, cols);
              CPXDIM const chunk = 10;   // max number of rows to add in one shot

              // Update indices of new constraints _before_ actually extracting
              // them. In case of error we will just reset the indices.
              for (CPXDIM c = offset; c < total; ++c)
                Parent::set_constraint_as_extracted(c, true);

              try {
                unique_ptr< CPXDIM[] >      rmatind(new CPXDIM[cols]);
                unique_ptr< double[] >      rmatval(new double[cols]);
                unique_ptr< CPXNNZ[] >      rmatbeg(new CPXNNZ[chunk]);
                unique_ptr< char[] >        sense(new char[chunk]);
                unique_ptr< double[] >      rhs(new double[chunk]);
                unique_ptr< char const*[] > name(new char const*[chunk]);
                unique_ptr< double[] >      rngval(new double[chunk]);
                unique_ptr< CPXDIM[] >      rngind(new CPXDIM[chunk]);
                bool                        haveRanges = false;

                // Loop over the new constraints, collecting rows for up to
                // CHUNK constraints into the arrays so that adding constraints
                // is faster.
                for (CPXDIM c = 0; c < newCons; /* nothing */) {
                  // Collect up to CHUNK constraints into the arrays.
                  CPXDIM nextRow = 0;
                  CPXNNZ nextNz = 0;
                  for (/* nothing */; c < newCons && nextRow < chunk; ++c, ++nextRow) {
                    MPConstraint const* const ct = Parent::_constraints[offset + c];

                    // Stop if there is not enough room in the arrays
                    // to add the current constraint.
                    if (nextNz + ct->_coefficients.size() > cols) {
                      DCHECK_GT_F(nextRow, 0);
                      break;
                    }

                    // Setup right-hand side of constraint.
                    MakeRhs(ct->lb(), ct->ub(), rhs[nextRow], sense[nextRow], rngval[nextRow]);
                    haveRanges = haveRanges || (rngval[nextRow] != 0.0);
                    rngind[nextRow] = offset + c;

                    // Setup left-hand side of constraint.
                    rmatbeg[nextRow] = nextNz;
                    const auto& coeffs = ct->_coefficients;
                    for (auto it(coeffs.begin()); it != coeffs.end(); ++it) {
                      CPXDIM const idx = it->first->index();
                      if (Parent::variable_is_extracted(idx)) {
                        DCHECK_LT_F(nextNz, cols);
                        DCHECK_LT_F(idx, cols);
                        rmatind[nextNz] = idx;
                        rmatval[nextNz] = it->second;
                        ++nextNz;
                      }
                    }

                    // Finally the name of the constraint.
                    name[nextRow] = ct->name().empty() ? 0 : ct->name().c_str();
                  }
                  if (nextRow > 0) {
                    CHECK_STATUS(CPXXaddrows(mEnv,
                                             mLp,
                                             0,
                                             nextRow,
                                             nextNz,
                                             rhs.get(),
                                             sense.get(),
                                             rmatbeg.get(),
                                             rmatind.get(),
                                             rmatval.get(),
                                             0,
                                             name.get()));
                    if (haveRanges) { CHECK_STATUS(CPXXchgrngval(mEnv, mLp, nextRow, rngind.get(), rngval.get())); }
                  }
                }
              } catch (...) {
                // Undo all changes in case of error.
                CPXDIM const rows = CPXXgetnumrows(mEnv, mLp);
                if (rows > offset) (void)CPXXdelrows(mEnv, mLp, offset, rows - 1);
                nt::TrivialDynamicArray< MPConstraint* > const& constraints = Parent::constraints();
                int const                                       size = constraints.size();
                for (int i = offset; i < size; ++i)
                  Parent::set_constraint_as_extracted(i, false);
                throw;
              }
            }
          }

          // Extract the objective function.
          void _impl_ExtractObjective() {
            // NOTE: The code assumes that the objective expression does not contain
            //       any non-zero duplicates.

            CPXDIM const cols = CPXXgetnumcols(mEnv, mLp);
            DCHECK_EQ_F(Parent::_last_variable_index, cols);

            unique_ptr< CPXDIM[] > ind(new CPXDIM[cols]);
            unique_ptr< double[] > val(new double[cols]);
            for (CPXDIM j = 0; j < cols; ++j) {
              ind[j] = j;
              val[j] = 0.0;
            }

            const auto& coeffs = Parent::_objective._coefficients;
            for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
              CPXDIM const idx = it->first->index();
              if (Parent::variable_is_extracted(idx)) {
                DCHECK_LT_F(idx, cols);
                val[idx] = it->second;
              }
            }

            CHECK_STATUS(CPXXchgobj(mEnv, mLp, cols, ind.get(), val.get()));
            CHECK_STATUS(CPXEsetobjoffset(mEnv, mLp, Parent::objective().offset()));
          }

          std::string _impl_SolverVersion() const {
            // We prefer CPXXversionnumber() over CPXXversion() since the
            // former will never pose any encoding issues.
            int version = 0;
            CHECK_STATUS(CPXXversionnumber(mEnv, &version));

            int const major = version / 1000000;
            version -= major * 1000000;
            int const release = version / 10000;
            version -= release * 10000;
            int const mod = version / 100;
            version -= mod * 100;
            int const fix = version;

            return nt::format("CPLEX library version {}.{}.{}.{}", major, release, mod, fix);
          }

          void* _impl_underlying_solver() { return reinterpret_cast< void* >(mLp); }

          CPXENVptr get_env() noexcept { return mEnv; }
          CPXLPptr  get_lp() noexcept { return mLp; }

          void _impl_SetCallback(MPCallback* mp_callback) {
            if (callback_ != nullptr) { callback_reset_ = true; }
            callback_ = mp_callback;
          };

          bool _impl_SupportsCallbacks() const { return true; }

          double _impl_ComputeExactConditionNumber() const {
            if (!_impl_IsContinuous()) {
              LOG_F(FATAL, "ComputeExactConditionNumber not implemented for CPLEX_MIXED_INTEGER_PROGRAMMING");
              return CPX_NAN;
            }

            if (Parent::checkSolutionIsSynchronized()) {
              double kappa = CPX_NAN;
              CHECK_STATUS(CPXXgetdblquality(mEnv, mLp, &kappa, CPX_EXACT_KAPPA));
              return kappa;
            }
            LOG_F(FATAL, "Cannot get exact condition number without solution");
            return CPX_NAN;
          }

          // protected:
          // Set all parameters in the underlying solver.
          void _impl_SetParameters(const MPSolverParameters& param) {
            Parent::setCommonParameters(param);
            if constexpr (_impl_IsMIP()) Parent::setMIPParameters(param);
          }
          // Set each parameter in the underlying solver.
          void _impl_SetRelativeMipGap(double value) {
            if constexpr (_impl_IsMIP()) {
              CHECK_STATUS(CPXXsetdblparam(mEnv, CPX_PARAM_EPGAP, value));
            } else {
              LOG_F(WARNING, "The relative MIP gap is only available for discrete problems.");
            }
          }
          void _impl_SetPrimalTolerance(double value) { CHECK_STATUS(CPXXsetdblparam(mEnv, CPX_PARAM_EPRHS, value)); }
          void _impl_SetDualTolerance(double value) { CHECK_STATUS(CPXXsetdblparam(mEnv, CPX_PARAM_EPOPT, value)); }
          void _impl_SetPresolveMode(int value) {
            MPSolverParameters::PresolveValues const presolve =
               static_cast< MPSolverParameters::PresolveValues >(value);

            switch (presolve) {
              case MPSolverParameters::PRESOLVE_OFF:
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_PREIND, CPX_OFF));
                return;
              case MPSolverParameters::PRESOLVE_ON:
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_PREIND, CPX_ON));
                return;
            }
            Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::PRESOLVE, value);
          }
          void _impl_SetScalingMode(int value) {
            MPSolverParameters::ScalingValues const scaling = static_cast< MPSolverParameters::ScalingValues >(value);

            switch (scaling) {
              case MPSolverParameters::SCALING_OFF: CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_SCAIND, -1)); break;
              case MPSolverParameters::SCALING_ON:
                // TODO: 0 is equilibrium scaling (the default), CPLEX also supports
                //       1 aggressive scaling
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_SCAIND, 0));
                break;
            }
          }

          // Sets the LP algorithm : primal, dual or barrier. Note that CPLEX offers other
          // LP algorithm (e.g. network) and automatic selection
          void _impl_SetLpAlgorithm(int value) {
            MPSolverParameters::LpAlgorithmValues const algorithm =
               static_cast< MPSolverParameters::LpAlgorithmValues >(value);

            int alg = CPX_ALG_NONE;

            switch (algorithm) {
              case MPSolverParameters::DUAL: alg = CPX_ALG_DUAL; break;
              case MPSolverParameters::PRIMAL: alg = CPX_ALG_PRIMAL; break;
              case MPSolverParameters::BARRIER: alg = CPX_ALG_BARRIER; break;
            }

            if (alg == CPX_ALG_NONE)
              Parent::setIntegerParamToUnsupportedValue(MPSolverParameters::LP_ALGORITHM, value);
            else {
              CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_LPMETHOD, alg));
              if constexpr (_impl_IsMIP()) {
                // For MIP we have to change two more parameters to specify the
                // algorithm that is used to solve LP relaxations.
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_STARTALG, alg));
                CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_SUBALG, alg));
              }
            }
          }
          bool _impl_SetNumThreads(int num_threads) {
            CHECK_STATUS(CPXXsetintparam(mEnv, CPX_PARAM_THREADS, num_threads));
            return true;
          }

          bool _impl_ReadParameterFile(std::string const& filename) {
            // Return true on success and false on error.
            return CPXXreadcopyparam(mEnv, filename.c_str()) == 0;
          }
          std::string _impl_ValidFileExtensionForParameterFile() const { return ".prm"; }

          // private:
          // Mark modeling object "out of sync". This implicitly invalidates
          // solution information as well. It is the counterpart of
          // MPSolverInterface::InvalidateSolutionSynchronization
          void _impl_InvalidateModelSynchronization() {
            mCstat = 0;
            mRstat = 0;
            Parent::_sync_status = SynchronizationStatus::MUST_RELOAD;
          }

          static int
             CplexLazyCutCallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p) {
            assert(cbhandle);
            CplexInterface::MPCallback*            p_callback = static_cast< CplexInterface::MPCallback* >(cbhandle);
            CplexInterface::CplexMPCallbackContext mpcontext(env, cbdata, wherefrom, useraction_p);
            p_callback->runCallback(&mpcontext);
            *useraction_p = CPX_CALLBACK_DEFAULT;
            if (mpcontext.bConstraintsAdded) { *useraction_p = CPX_CALLBACK_SET; }
            return 0;
          }

          static int
             CplexUserCutCallback(CPXCENVptr env, void* cbdata, int wherefrom, void* cbhandle, int* useraction_p) {
            return 0;
          }

          // Transform a CPLEX basis status to an MPSolver basis status.
          static BasisStatus xformBasisStatus(int cplex_basis_status) {
            switch (cplex_basis_status) {
              case CPX_AT_LOWER: return BasisStatus::AT_LOWER_BOUND;
              case CPX_BASIC: return BasisStatus::BASIC;
              case CPX_AT_UPPER: return BasisStatus::AT_UPPER_BOUND;
              case CPX_FREE_SUPER: return BasisStatus::FREE;
              default: LOG_F(FATAL, "Unknown CPLEX basis status"); return BasisStatus::FREE;
            }
          }

          // Setup the right-hand side of a constraint from its lower and upper bound.
          static void MakeRhs(double lb, double ub, double& rhs, char& sense, double& range) {
            if (lb == ub) {
              // Both bounds are equal -> this is an equality constraint
              rhs = lb;
              range = 0.0;
              sense = 'E';
            } else if (lb > -CPX_INFBOUND && ub < CPX_INFBOUND) {
              // Both bounds are finite -> this is a ranged constraint
              // The value of a ranged constraint is allowed to be in
              //   [ rhs[i], rhs[i]+rngval[i] ]
              // see also the reference documentation for CPXXnewrows()
              if (ub < lb) {
                // The bounds for the constraint are contradictory. CPLEX models
                // a range constraint l <= ax <= u as
                //    ax = l + v
                // where v is an auxiliary variable the range of which is controlled
                // by l and u: if l < u then v in [0, u-l]
                //             else          v in [u-l, 0]
                // (the range is specified as the rngval[] argument to CPXXnewrows).
                // Thus CPLEX cannot represent range constraints with contradictory
                // bounds and we must error out here.
                CHECK_STATUS(CPXERR_BAD_ARGUMENT);
              }
              rhs = lb;
              range = ub - lb;
              sense = 'R';
            } else if (ub < CPX_INFBOUND || (std::abs(ub) == CPX_INFBOUND && std::abs(lb) > CPX_INFBOUND)) {
              // Finite upper, infinite lower bound -> this is a <= constraint
              rhs = ub;
              range = 0.0;
              sense = 'L';
            } else if (lb > -CPX_INFBOUND || (std::abs(lb) == CPX_INFBOUND && std::abs(ub) > CPX_INFBOUND)) {
              // Finite lower, infinite upper bound -> this is a >= constraint
              rhs = lb;
              range = 0.0;
              sense = 'G';
            } else {
              // Lower and upper bound are both infinite.
              // This is used for example in .mps files to specify alternate
              // objective functions.
              // Note that the case lb==ub was already handled above, so we just
              // pick the bound with larger magnitude and create a constraint for it.
              // Note that we replace the infinite bound by CPX_INFBOUND since
              // bounds with larger magnitude may cause other CPLEX functions to
              // fail (for example the export to LP files).
              DCHECK_GT_F(std::abs(lb), CPX_INFBOUND);
              DCHECK_GT_F(std::abs(ub), CPX_INFBOUND);
              if (std::abs(lb) > std::abs(ub)) {
                rhs = (lb < 0) ? -CPX_INFBOUND : CPX_INFBOUND;
                sense = 'G';
              } else {
                rhs = (ub < 0) ? -CPX_INFBOUND : CPX_INFBOUND;
                sense = 'L';
              }
              range = 0.0;
            }
          }

          // private:
          CPXLPptr  mLp;
          CPXENVptr mEnv;

          bool has_dual_farkas_ = false;
          bool branching_priority_reset_ = false;

          nt::IntDynamicArray    hint_beg_;
          nt::IntDynamicArray    hint_ind_;
          nt::DoubleDynamicArray hint_val_;

          // Incremental extraction.
          // Without incremental extraction we have to re-extract the model every
          // time we perform a solve. Due to the way the Reset() function is
          // implemented, this will lose MIP start or basis information from a
          // previous solve. On the other hand, if there is a significant changes
          // to the model then just re-extracting everything is usually faster than
          // keeping the low-level modeling object in sync with the high-level
          // variables/constraints.
          // Note that incremental extraction is particularly expensive in function
          // extractNewVariables() since there we must scan _all_ old constraints
          // and update them with respect to the new variables.
          bool const supportIncrementalExtraction;

          // Use slow and immediate updates or try to do bulk updates.
          // For many updates to the model we have the option to either perform
          // the update immediately with a potentially slow operation or to
          // just mark the low-level modeling object out of sync and re-extract
          // the model later.
          enum SlowUpdates {
            SlowSetCoefficient = 0x0001,
            SlowClearConstraint = 0x0002,
            SlowSetObjectiveCoefficient = 0x0004,
            SlowClearObjective = 0x0008,
            SlowSetConstraintBounds = 0x0010,
            SlowSetVariableInteger = 0x0020,
            SlowSetVariableBounds = 0x0040,
            SlowUpdatesAll = 0xffff
          } const slowUpdates;
          // CPLEX has no method to query the basis status of a single variable.
          // Hence we query the status only once and cache the array. This is
          // much faster in case the basis status of more than one row/column
          // is required.
          unique_ptr< int[] > mutable mCstat;
          unique_ptr< int[] > mutable mRstat;

          MPCallback* callback_ = nullptr;
          bool        callback_reset_ = false;
        };

      }   // namespace details

      using LPModel = details::CplexInterface< false >;
      using MIPModel = details::CplexInterface< true >;

      inline constexpr bool hasMIP() noexcept { return true; }
      inline constexpr bool hasLP() noexcept { return true; }

    }   // namespace cplex
  }     // namespace mp
}   // namespace nt

#endif